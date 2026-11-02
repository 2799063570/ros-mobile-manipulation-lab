"""Run the actual Inspire command method against fake services, without robot IO."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

CORE = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which('g++'), 'C++ compiler unavailable')
class InspireGripperTest(unittest.TestCase):
    def test_full_open_requires_position_readback_and_close_keeps_force_control(self):
        source = (CORE / 'src/motion_executor.cpp').read_text()
        body = source[source.index('bool SortingTask::commandInspireGripper('):
                      source.index('bool SortingTask::commandGripperImpl(')]
        fixture = r'''
#include <atomic>
#include <cassert>
#include <cmath>
#include <functional>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
namespace ros {
double clock_seconds=0;
struct WallDuration {double seconds; explicit WallDuration(double s):seconds(s) {}};
struct WallTime {
  double seconds;
  static WallTime now() {return {clock_seconds};}
  WallTime operator+(WallDuration d) const {return {seconds+d.seconds};}
  bool operator<(WallTime other) const {return seconds<other.seconds;}
};
}
bool wallSleep(double duration,const std::atomic<bool>& stopped) {
  ros::clock_seconds+=duration;return !stopped.load();
}
#define ROS_INFO_STREAM(message) do {std::ostringstream log;log << message;} while(false)
namespace inspire_gripper {
struct get_openlimit {struct {double openmax{1000},openmin{0};} response;};
struct get_copen {struct {double curopen{999};} response;};
struct move_max {struct {int speed{0};} request;struct {bool movemax_accepted{true};} response;};
struct move_min {struct {int speed{0},power{0};} request;struct {bool movemin_accepted{true};} response;};
struct get_state {struct {int error_code{0},motion_state{1};} response;};
struct set_es {};
}
template<class Service> struct Client {
  int calls{0};
  Service last;
  std::function<bool(Service&)> reply=[](Service&){return true;};
  bool call(Service& message) {++calls;bool result=reply(message);last=message;return result;}
};
class SortingTask {
public:
  std::atomic<bool> stop_requested_{false},grasp_secured_{true};
  double gripper_open_{0},gripper_closed_{.22},inspire_motion_timeout_{.6};
  int inspire_speed_{300},inspire_force_{100},inspire_open_tolerance_{5};
  Client<inspire_gripper::get_openlimit> inspire_open_limit_client_;
  Client<inspire_gripper::get_copen> inspire_current_open_client_;
  Client<inspire_gripper::move_max> inspire_open_client_;
  Client<inspire_gripper::move_min> inspire_close_client_;
  Client<inspire_gripper::get_state> inspire_state_client_;
  Client<inspire_gripper::set_es> inspire_stop_client_;
  std::string failure;
  void setFailure(const std::string& code,const std::string& reason) {failure=code+" | "+reason;}
  bool commandInspireGripper(double);
};
'''
        checks = r'''
int main() {
  SortingTask open;
  assert(open.commandInspireGripper(0));
  assert(open.inspire_open_client_.calls==1 && open.inspire_close_client_.calls==0);
  assert(open.inspire_open_client_.last.request.speed==300);
  assert(open.inspire_current_open_client_.calls==1 && !open.grasp_secured_.load());
  assert(open.inspire_stop_client_.calls==0 && open.failure.empty());

  SortingTask moving;
  const std::vector<double> positions{600,994,995};
  moving.inspire_current_open_client_.reply=[&](inspire_gripper::get_copen& m) {
    m.response.curopen=positions.at(moving.inspire_current_open_client_.calls-1);return true;
  };
  assert(moving.commandInspireGripper(0));
  assert(moving.inspire_current_open_client_.calls==3); // State 1 alone is insufficient.

  SortingTask blocked;
  blocked.inspire_current_open_client_.reply=[](inspire_gripper::get_copen& m) {
    m.response.curopen=600;return true;
  };
  assert(!blocked.commandInspireGripper(0));
  assert(blocked.inspire_stop_client_.calls==1 && blocked.grasp_secured_.load());
  assert(blocked.failure.find("current=600, maximum=1000")!=std::string::npos);

  SortingTask invalid_limits;
  invalid_limits.inspire_open_limit_client_.reply=[](inspire_gripper::get_openlimit& m) {
    m.response.openmax=1001;return true;
  };
  assert(!invalid_limits.commandInspireGripper(0));
  assert(invalid_limits.inspire_open_client_.calls==0);
  SortingTask missing_limits;
  missing_limits.inspire_open_limit_client_.reply=[](inspire_gripper::get_openlimit&) {return false;};
  assert(!missing_limits.commandInspireGripper(0));
  assert(missing_limits.inspire_open_client_.calls==0);

  SortingTask unreadable;
  unreadable.inspire_current_open_client_.reply=[](inspire_gripper::get_copen& m) {
    m.response.curopen=std::numeric_limits<double>::quiet_NaN();return true;
  };
  assert(!unreadable.commandInspireGripper(0));
  assert(unreadable.inspire_stop_client_.calls==1 && unreadable.grasp_secured_.load());
  SortingTask fault;
  fault.inspire_state_client_.reply=[](inspire_gripper::get_state& m) {m.response.error_code=1;return true;};
  assert(!fault.commandInspireGripper(0));
  assert(fault.inspire_stop_client_.calls==1 && fault.inspire_current_open_client_.calls==0);

  SortingTask close;
  close.inspire_state_client_.reply=[](inspire_gripper::get_state& m) {m.response.motion_state=6;return true;};
  assert(close.commandInspireGripper(.22));
  assert(close.inspire_close_client_.calls==1 && close.inspire_open_client_.calls==0);
  assert(close.inspire_close_client_.last.request.speed==300 && close.inspire_close_client_.last.request.power==100);
  assert(close.inspire_open_limit_client_.calls==0 && close.grasp_secured_.load());

  SortingTask unsupported;
  assert(!unsupported.commandInspireGripper(.1));
  assert(!unsupported.commandInspireGripper(std::numeric_limits<double>::quiet_NaN()));
  assert(unsupported.inspire_open_client_.calls==0 && unsupported.inspire_close_client_.calls==0);
  SortingTask stopped;
  stopped.stop_requested_.store(true);
  assert(!stopped.commandInspireGripper(0) && stopped.inspire_open_limit_client_.calls==0);
  SortingTask stop_during_query;
  stop_during_query.inspire_open_limit_client_.reply=[&](inspire_gripper::get_openlimit&) {
    stop_during_query.stop_requested_.store(true);return true;
  };
  assert(!stop_during_query.commandInspireGripper(0));
  assert(stop_during_query.inspire_open_client_.calls==0);

  SortingTask rejected;
  rejected.inspire_open_client_.reply=[](inspire_gripper::move_max& m) {m.response.movemax_accepted=false;return true;};
  assert(!rejected.commandInspireGripper(0) && rejected.inspire_state_client_.calls==0);
}
'''
        with tempfile.TemporaryDirectory(prefix='sorting_gripper_') as directory:
            path = Path(directory)
            (path / 'check.cpp').write_text(fixture + body + checks)
            result = subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra',
                                     str(path / 'check.cpp'), '-o', str(path / 'check')],
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(path / 'check')], check=True, timeout=5)


if __name__ == '__main__':
    unittest.main()
