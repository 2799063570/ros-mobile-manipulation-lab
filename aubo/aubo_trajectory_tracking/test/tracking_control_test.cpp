#include <gtest/gtest.h>
#include "tracking_control.h"
#include "tracking_curve.h"

TEST(TcpVelocityWindow, ConstantMotionAndOscillatingNoise)
{
  TcpVelocityWindow velocity, idle;
  KDL::Vector v, noisy;
  for(int i=0;i<100;++i) {
    v=velocity.add(1000000000.+i*.02,KDL::Vector(.02*i*.02,0,0),.08);
    noisy=idle.add(i*.02,KDL::Vector(i%2 ? .0005 : -.0005,0,0),.08);
  }
  EXPECT_NEAR(v.x(),.02,1e-7);
  EXPECT_LT(noisy.Norm(),.02);  // Raw two-sample speed is 0.05 m/s.
  v=velocity.add(0.,KDL::Vector::Zero(),.08);
  EXPECT_DOUBLE_EQ(v.Norm(),0.);  // Clock rewind discards the old window.
}

TEST(TrackingCorrection, AntiWindupAndSettlingHysteresis)
{
  TrackingCorrection controller;
  for(int i=0;i<1000;++i) {
    const auto v=controller.step(KDL::Vector::Zero(),KDL::Vector(1,0,0),.02,1.,.2,.05,.2,false,.002);
    EXPECT_LE(v.Norm(),.010000001);
  }
  auto v=controller.step(KDL::Vector::Zero(),KDL::Vector::Zero(),.02,1.,.2,.05,.2,false,.002);
  EXPECT_DOUBLE_EQ(v.Norm(),0.);  // Saturation must not charge the integrator.
  v=controller.step(KDL::Vector::Zero(),KDL::Vector(.0008,0,0),.02,1.,.2,.05,.2,true,.002);
  EXPECT_DOUBLE_EQ(v.Norm(),0.);
  v=controller.step(KDL::Vector::Zero(),KDL::Vector(.0015,0,0),.02,1.,.2,.05,.2,true,.002);
  EXPECT_DOUBLE_EQ(v.Norm(),0.);
  v=controller.step(KDL::Vector::Zero(),KDL::Vector(.0021,0,0),.02,1.,.2,.05,.2,true,.002);
  EXPECT_GT(v.x(),0.);
  controller.reset();
  v=controller.step(KDL::Vector::Zero(),KDL::Vector::Zero(),.02,1.,.2,.05,.2,false,.002);
  EXPECT_DOUBLE_EQ(v.Norm(),0.);
}

TEST(TrackingFeedforward, AnalyticAccelerationAndLimits)
{
  Curve c{}; c.duration=8.; c.radius=.03; c.amplitude=.02; c.length=.1; c.cycles=2; c.waves=3;
  for(const auto& shape: {"circle","sine"}) for(const auto& plane: {"xy","xz","yz"}) {
    c.shape=shape; c.plane=plane;
    KDL::Vector p,v,a,pm,vm,pp,vp;
    c.sample(3.1,p,v,&a); c.sample(3.1-1e-4,pm,vm); c.sample(3.1+1e-4,pp,vp);
    EXPECT_LT((a-(vp-vm)/2e-4).Norm(),1e-7);
    c.sample(0,p,v,&a); EXPECT_DOUBLE_EQ(a.Norm(),0.);
    c.sample(c.duration,p,v,&a); EXPECT_DOUBLE_EQ(a.Norm(),0.);
  }
  auto ff=accelerationFeedforward(KDL::Vector(.03,0,0),KDL::Vector(100,0,0),.25,.05);
  EXPECT_LE(ff.Norm(),.040000001);
  ff=accelerationFeedforward(KDL::Vector(.01,0,0),KDL::Vector(.1,0,0),0.,.05);
  EXPECT_DOUBLE_EQ(ff.x(),.01);
  TrackingCorrection controller;
  auto command=controller.step(KDL::Vector::Zero(),KDL::Vector::Zero(),.02,1.,0.,.05,.2,false,.002,KDL::Vector(.01,0,0),.5);
  EXPECT_NEAR(command.x(),.005,1e-12);
  command=controller.step(KDL::Vector::Zero(),KDL::Vector::Zero(),.02,1.,0.,.05,.2,false,.002,KDL::Vector(100,0,0),.5);
  EXPECT_LE(command.Norm(),.010000001);
}

int main(int argc,char** argv)
{
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
