#include <aubo_mobile_nav_sorting/dock_control.h>
#include <cassert>
#include <cmath>
#include <iostream>
using aubo_mobile_nav_sorting::dockCommand;
int main() {
  assert(dockCommand(.4, 0, -.04, .04, .12, .06).angular < 0);
  assert(dockCommand(.4, .01, 0, .04, .12, .06).angular > 0);
  assert(dockCommand(-.3, .01, 0, .04, .12, .06).angular < 0);
  assert(std::abs(dockCommand(.005, 0, 0, .04, .12, .06).linear) < .005);
  for (double direction : {1., -1.}) {
    double x=0, y=.005, yaw=.01;
    bool reached=false;
    for (int i=0; i<300; ++i) {
      double dx=direction*.4-x, dy=-y;
      if (std::hypot(dx,dy)<=.015 && std::abs(yaw)<=.025) { reached=true; break; }
      auto u=dockCommand(dx,dy,-yaw,.04,.12,.06);
      x+=u.linear*std::cos(yaw)*.05;
      y+=u.linear*std::sin(yaw)*.05;
      yaw+=(u.angular+.008)*.05; // sustained drivetrain yaw disturbance
      assert(std::abs(yaw)<.06);
    }
    assert(reached);
  }
  std::cout << "Forward/reverse disturbed docking and command bounds passed\n";
}
