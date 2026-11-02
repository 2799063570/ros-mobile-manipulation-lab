"""Exercise the actual C++ pick/place sequence with fake motion/gripper interfaces."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

CORE = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which('g++'), 'C++ compiler unavailable')
class PlaceOrientationTest(unittest.TestCase):
    def test_alignment_phase_symmetry_and_failure_cleanup(self):
        source = (CORE/'src/pick_place.cpp').read_text()
        method = source[source.index('bool SortingTask::pickAndPlaceImpl('):
                        source.index('\nbool SortingTask::sortingOperation()')]
        harness = r'''
#include <aubo_sorting_core/placement_height.hpp>
#include <aubo_sorting_core/gripper_height.hpp>
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>
#define ROS_INFO(...) ((void)0)
#define ROS_ERROR_STREAM(...) ((void)0)
namespace aubo_perception {
struct DetectedObject {
  struct { struct { double x=.5,y=0,z=.125; } position; } pose;
  double object_height=.04,grasp_angle=.7,grasp_width=.04;
  bool depth_valid=true,grasp_geometry_valid=true;
};
}
namespace aubo_sorting_core {
bool wallSleep(double,const std::atomic<bool>& stop) { return !stop.load(); }
struct Pose { double x,y,z,angle; };
struct SortingTask {
  double grasp_offset_x_=0,grasp_offset_y_=0,object_height_=.04,table_z_=.1;
  double gripper_closed_=.22,gripper_open_=0,height_tolerance_=.02;
  bool use_detected_angle_=true,use_detected_width_=false;
  double width_close_scale_=.9,gripper_width_open_=.08,gripper_width_closed_=0;
  double grasp_height_offset_=.01,minimum_gripper_height_=0,pregrasp_height_=.22;
  double lift_min_height_=.28,active_grasp_angle_=0,preplace_height_=.28;
  double place_clearance_=.02,minimum_place_z_=.15,place_object_yaw_=0;
  bool use_place_object_yaw_=false,continuous_sorting_=false,use_grasp_attachment_=false;
  std::string height_mode_="depth",target_frame_="base_link",place_frame_="base_link",attached_model_;
  std::map<std::string,std::string> grasp_model_names_;
  std::map<std::string,std::vector<double>> place_targets_{{"box",{.4,.2}}};
  std::atomic<bool> stop_requested_{false},grasp_secured_{false};
  std::mutex data_mutex_;
  enum class QueueVisionPhase { MASKED,COLLECT };
  bool lift_ok=true,preplace_ok=true;
  double lift_angle=0;
  std::vector<Pose> poses;
  std::vector<double> gripper;
  std::string objectCategory(const aubo_perception::DetectedObject&) { return "box"; }
  void releaseAttachedObjectNoWait() {}
  void setFailure(const std::string&,const std::string&) {}
  void publishProgress(const std::string&,const std::string&) {}
  void setQueueVisionPhase(QueueVisionPhase) {}
  bool validateReservedTarget() { return true; }
  bool commandGripper(double q) { gripper.push_back(q); return true; }
  bool setGraspAttachment(const std::string&,bool) { return true; }
  Pose makePose(double x,double y,double z) { return {x,y,z,active_grasp_angle_}; }
  bool moveToPose(Pose p,const std::string& stage,bool=false) {
    poses.push_back(p); return stage!="box pre-place" || preplace_ok;
  }
  bool cartesianTo(Pose p,const std::string&) { poses.push_back(p);return true; }
  bool liftWithRecovery(double,double,const std::string&) { lift_angle=active_grasp_angle_;return lift_ok; }
  bool xyInTargetFrame(const std::string&,const std::vector<double>& xy,double& x,double& y) {
    x=xy[0];y=xy[1];return true;
  }
  bool pickAndPlaceImpl(const aubo_perception::DetectedObject&);
};
'''
        # Include a real mutex for the scope in the actual implementation.
        harness = '#include <mutex>\n' + harness + method + r'''
}
int main() {
  using aubo_sorting_core::SortingTask;
  const double pi=std::acos(-1.0);
  aubo_perception::DetectedObject object;
  for (double pickup : {.7,-1.4,1.4}) {
    for (double target : {0.,pi/2,pi}) {
      SortingTask t;t.use_place_object_yaw_=true;t.place_object_yaw_=target;
      object.grasp_angle=pickup;
      assert(t.pickAndPlaceImpl(object));
      assert(t.poses.size()==5);
      assert(std::abs(t.poses[0].angle-pickup)<1e-10);
      assert(std::abs(t.poses[1].angle-pickup)<1e-10);
      assert(std::abs(t.lift_angle-pickup)<1e-10);
      for (unsigned i=2;i<5;++i) {
        double angle=t.poses[i].angle;
        assert(std::abs(std::remainder(angle-target-pi/2,pi))<1e-10);
        assert(std::abs(angle-pickup)<=pi/2+1e-10);
        // Real cell yaw offset is pi/2. TCP local X must align with object long axis.
        double tcp_yaw=pi/2+angle;
        assert(std::abs(std::sin(tcp_yaw-target))<1e-10);
      }
      assert(t.active_grasp_angle_==0.);
      assert(t.gripper.size()==3);
    }
  }
  object.grasp_angle=.7;
  SortingTask legacy;assert(legacy.pickAndPlaceImpl(object));
  for (auto p:legacy.poses) assert(std::abs(p.angle-.7)<1e-10);
  for (bool fail_lift : {true,false}) {
    SortingTask t;t.use_place_object_yaw_=true;
    t.lift_ok=!fail_lift;t.preplace_ok=fail_lift;
    assert(!t.pickAndPlaceImpl(object));
    assert(t.active_grasp_angle_==0.);
    assert(t.gripper.size()==2); // Failed transfer cannot open the gripper.
    assert(t.poses.size()==(fail_lift ? 2u : 3u));
  }
}
'''
        with tempfile.TemporaryDirectory(prefix='place_orientation_') as directory:
            path = Path(directory)
            (path/'check.cpp').write_text(harness)
            built = subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra',
                                    '-I'+str(CORE/'include'), str(path/'check.cpp'), '-o', str(path/'check')],
                                   capture_output=True, text=True, timeout=30)
            self.assertEqual(built.returncode, 0, built.stderr)
            ran = subprocess.run([str(path/'check')], capture_output=True, text=True, timeout=10)
            self.assertEqual(ran.returncode, 0, ran.stderr)


if __name__ == '__main__':
    unittest.main()
