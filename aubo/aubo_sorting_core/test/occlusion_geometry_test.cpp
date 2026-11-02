#include <aubo_sorting_core/occlusion_geometry.hpp>
#include <aubo_sorting_core/footprint.hpp>
#include <gtest/gtest.h>
using namespace aubo_sorting_core;

TEST(OcclusionGeometry, OnlyRobotBetweenCameraAndTargetMasksTheObservation)
{
  const Point3 camera{{0,0,1}},target{{0,0,0}};
  EXPECT_TRUE(robotOccludesTarget(camera,target,{{{-.1,0,.5}},{{.1,0,.5}}},.02));
  EXPECT_TRUE(robotOccludesTarget(camera,target,{{{0,0,.3}}},.02));
  EXPECT_FALSE(robotOccludesTarget(camera,target,{{{.2,0,.5}},{{.3,0,.5}}},.02));
  EXPECT_FALSE(robotOccludesTarget(camera,target,{{{0,0,-.2}},{{0,0,-.3}}},.02));
  EXPECT_FALSE(robotOccludesTarget(camera,target,{{{0,0,1.2}},{{0,0,1.3}}},.02));
  EXPECT_FALSE(robotOccludesTarget(camera,target,{{{0,0,.99}}},.02));
  EXPECT_FALSE(robotOccludesTarget(camera,target,{},.02));
}

TEST(FootprintGeometry, ClipsBothOrientationsAndContainedRectangles)
{
  const Footprint a{{{0,0}},{{2,0}},{{2,2}},{{0,2}}};
  Footprint b{{{1,0}},{{3,0}},{{3,2}},{{1,2}}};
  EXPECT_NEAR(footprintOverlap(a,b),.5,1e-12);
  std::reverse(b.begin(),b.end());
  EXPECT_NEAR(footprintOverlap(a,b),.5,1e-12);
  EXPECT_NEAR(footprintOverlap(a,{{{.5,.5}},{{1,.5}},{{1,1}},{{.5,1}}}),1.,1e-12);
  EXPECT_EQ(footprintOverlap(a,{{{3,0}},{{4,0}},{{4,1}},{{3,1}}}),0.);
  EXPECT_EQ(footprintOverlap(a,{}),0.);
  const Footprint diamond{{{1,0}},{{2,1}},{{1,2}},{{0,1}}};
  EXPECT_NEAR(footprintOverlap(a,diamond),1.,1e-12);
}

TEST(OcclusionGeometry, RecordedRealPregraspPoseCoversBothReportedClassPredictions)
{
  // Read-only TF/queue snapshot at the reported stopped pregrasp pose.
  const Point3 camera{{-.029238,-.652428,.909476}};
  const std::vector<Point3> links{{{-.121030,-.010677,.122000}},{{-.104071,-.202915,.481473}},
      {{-.071523,-.571904,.416946}},{{-.062517,-.674007,.416860}},
      {{-.062578,-.673933,.322860}},{{-.062656,-.673838,.202860}}};
  EXPECT_TRUE(robotOccludesTarget(camera,{{-.062616,-.673908,.034142}},links,.06));
  EXPECT_TRUE(robotOccludesTarget(camera,{{-.063039,-.667543,.034801}},links,.06));
}

int main(int argc,char** argv) {testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
