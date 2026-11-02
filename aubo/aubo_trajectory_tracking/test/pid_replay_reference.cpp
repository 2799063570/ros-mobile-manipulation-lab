// Offline numerical oracle using the actual production correction class.
// No ros::init, SDK connection, motion services, or hardware commands.
#include "tracking_control.h"
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc,char** argv)
{
  if(argc>1 && std::string(argv[1])=="--velocity")
  {
    double window,time,x,y,z;
    if(!(std::cin>>window)) return 1;
    TcpVelocityWindow estimator;
    std::cout<<std::setprecision(17);
    while(std::cin>>time>>x>>y>>z)
    {
      const auto v=estimator.add(time,KDL::Vector(x,y,z),window);
      std::cout<<v.x()<<' '<<v.y()<<' '<<v.z()<<'\n';
    }
    return 0;
  }
  double kp,ki,kd,speed,fraction,tolerance,duration;
  if(!(std::cin>>kp>>ki>>kd>>speed>>fraction>>tolerance>>duration)) return 1;
  TrackingCorrection controller;
  double time,previous=0.;
  double fx,fy,fz,ex,ey,ez,vx,vy,vz;
  std::cout<<std::setprecision(17);
  while(std::cin>>time>>fx>>fy>>fz>>ex>>ey>>ez>>vx>>vy>>vz)
  {
    const KDL::Vector ff(fx,fy,fz),error(ex,ey,ez),velocity_error(vx,vy,vz);
    const auto output=controller.step(ff,error,time-previous,kp,ki,speed,fraction,
                                     time>=duration,tolerance,velocity_error,kd);
    std::cout<<output.x()<<' '<<output.y()<<' '<<output.z()<<'\n';
    previous=time;
  }
  return 0;
}
