"""Actual descent validation callback, fake task state, no ROS node or robot IO."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

CORE = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which('g++'), 'C++ compiler unavailable')
class ReservedObservationTest(unittest.TestCase):
    def test_actual_frame_gate_masks_only_known_occlusion_and_rejects_late_frames(self):
        source = (CORE/'src/instance_tracking.cpp').read_text()
        begin = source.index('void SortingTask::processInstanceFrame(')
        end = source.index('  tf::StampedTransform source_to_target', begin)
        body = source[begin:end] + '  ++processed;\n}\n'
        fixture = r'''
#include <ros/time.h>
#include <atomic>
#include <cassert>
#include <cmath>
#include <mutex>
#include <string>
#include <stdexcept>
namespace aubo_perception {
struct DetectedObjectArray {
  struct Header {ros::Time stamp;std::string frame_id;} header;
  std::string sensor_frame;
  bool observation_valid{false};
};
}
class SortingTask {
public:
  enum class QueueVisionPhase {DISABLED,COLLECT,APPROACH,MASKED};
  enum class State {OBSERVING,PICKING,DETECTING,READY,SORTING};
  std::atomic<QueueVisionPhase> queue_vision_phase_{QueueVisionPhase::APPROACH};
  std::atomic<bool> stop_requested_{false},observation_ready_{true};
  std::mutex data_mutex_;
  State state_{State::PICKING};
  ros::Time queue_epoch_,queue_last_stamp_;
  ros::WallTime queue_last_frame_,queue_last_source_frame_,queue_empty_since_,queue_occlusion_checked_at_;
  double queue_frame_max_age_{1};
  int queue_empty_frames_{7},processed{0};
  bool queue_target_occluded_{false},occludes{true},missing_tf{false};
  bool reservedTargetOccluded(const aubo_perception::DetectedObjectArray&) {
    if (missing_tf) throw std::runtime_error("TF unavailable");
    return occludes;
  }
  void processInstanceFrame(const aubo_perception::DetectedObjectArray&);
};
aubo_perception::DetectedObjectArray frame() {
  aubo_perception::DetectedObjectArray m;
  m.header.stamp=ros::Time::now()-ros::Duration(.1);
  m.header.frame_id="base_link";m.sensor_frame="camera";
  return m;
}
'''
        checks = r'''
int main() {
  ros::Time::init();
  SortingTask hidden;hidden.processInstanceFrame(frame());
  assert(hidden.processed==0 && hidden.queue_target_occluded_);
  assert(!hidden.queue_last_source_frame_.isZero() && hidden.queue_last_frame_.isZero());
  assert(hidden.queue_empty_frames_==0);
  SortingTask visible;visible.occludes=false;
  auto valid=frame();valid.observation_valid=true;
  visible.processInstanceFrame(valid);
  assert(visible.processed==1 && !visible.queue_target_occluded_);
  SortingTask invalid;invalid.occludes=false;invalid.processInstanceFrame(frame());
  assert(invalid.processed==0 && invalid.queue_last_frame_.isZero());
  SortingTask masked;masked.queue_vision_phase_.store(SortingTask::QueueVisionPhase::MASKED);
  masked.processInstanceFrame(frame());
  assert(masked.processed==0 && !masked.queue_last_source_frame_.isZero());
  SortingTask missing;missing.queue_vision_phase_.store(SortingTask::QueueVisionPhase::MASKED);
  auto no_source=frame();no_source.sensor_frame.clear();missing.processInstanceFrame(no_source);
  assert(missing.queue_last_source_frame_.isZero());
  SortingTask old;auto stale=frame();stale.header.stamp=ros::Time::now()-ros::Duration(2);
  old.processInstanceFrame(stale);assert(old.queue_last_source_frame_.isZero());
  SortingTask late;auto before_transition=frame();late.queue_epoch_=ros::Time::now();
  late.processInstanceFrame(before_transition);
  assert(late.queue_last_source_frame_.isZero() && !late.queue_target_occluded_);
  SortingTask tf_failure;tf_failure.missing_tf=true;
  try {tf_failure.processInstanceFrame(frame());assert(false);} catch (const std::runtime_error&) {}
  assert(tf_failure.queue_last_source_frame_.isZero() && !tf_failure.queue_target_occluded_);
}
'''
        with tempfile.TemporaryDirectory(prefix='sorting_frame_gate_') as directory:
            path = Path(directory)
            (path/'check.cpp').write_text(fixture+body+checks)
            result = subprocess.run(['g++', '-std=c++11', '-pthread', '-I', '/opt/ros/noetic/include',
                str(path/'check.cpp'), '-L', '/opt/ros/noetic/lib', '-Wl,-rpath,/opt/ros/noetic/lib',
                '-lrostime', '-lcpp_common', '-o', str(path/'check')],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(path/'check')],check=True,timeout=5)

    def test_occlusion_only_uses_live_source_and_cannot_erase_age_or_movement_faults(self):
        source = (CORE/'src/instance_tracking.cpp').read_text()
        body = source[source.index('bool SortingTask::validateReservedTarget()'):
                      source.index('void SortingTask::publishInstanceQueue()')]
        fixture = r'''
#include <aubo_sorting_core/instance_queue.hpp>
#include <ros/time.h>
#include <atomic>
#include <cassert>
#include <mutex>
#include <sstream>
#include <string>
using ObjectQueue=aubo_sorting_core::InstanceQueue<int>;
class SortingTask {
public:
  enum class QueueVisionPhase {DISABLED,COLLECT,APPROACH,MASKED};
  bool continuous_sorting_{true}, queue_target_occluded_{true};
  std::uint64_t active_instance_id_{1};
  std::mutex queue_mutex_;
  ObjectQueue instance_queue_;
  double queue_reserved_max_age_{120},queue_frame_max_age_{1};
  ros::WallTime queue_last_frame_,queue_last_source_frame_,queue_occlusion_checked_at_;
  std::atomic<QueueVisionPhase> queue_vision_phase_{QueueVisionPhase::APPROACH};
  std::atomic<bool> stop_requested_{false};
  std::string reason;
  void setFailure(const std::string&,const std::string& value) {reason=value;}
  bool validateReservedTarget();
};
void prepare(SortingTask& task,double age=0.) {
  const auto now=ros::WallTime::now();
  ObjectQueue::Sample sample; sample.x=.5; sample.category="can"; sample.payload=1;
  task.instance_queue_.min_observations=2; task.instance_queue_.max_age=200;
  task.instance_queue_.update({sample},now.toSec()-age-.1);
  task.instance_queue_.update({sample},now.toSec()-age);
  ObjectQueue::Track target;
  assert(task.instance_queue_.reserve({"can"},now.toSec(),target));
  task.active_instance_id_=target.id;
  task.queue_last_frame_=now-ros::WallDuration(3);
  task.queue_last_source_frame_=task.queue_occlusion_checked_at_=now;
}
'''
        checks = r'''
int main() {
  SortingTask t;prepare(t);
  assert(t.validateReservedTarget()); // Known occlusion + live camera, bounded cached target.
  t.queue_last_source_frame_=ros::WallTime::now()-ros::WallDuration(3);
  assert(!t.validateReservedTarget()); // A frozen camera cannot grant a mask exception.
  t.queue_last_source_frame_=ros::WallTime::now();
  t.queue_occlusion_checked_at_=ros::WallTime::now()-ros::WallDuration(3);
  assert(!t.validateReservedTarget()); // Old TF occlusion evidence cannot grant an exception.
  t.queue_occlusion_checked_at_=ros::WallTime::now();
  t.queue_vision_phase_.store(SortingTask::QueueVisionPhase::COLLECT);
  assert(!t.validateReservedTarget()); // Unmasked validation still requires valid geometry.
  t.queue_vision_phase_.store(SortingTask::QueueVisionPhase::APPROACH);
  ObjectQueue::Sample moved; moved.x=.538; moved.category="can";moved.payload=1;
  t.instance_queue_.update({moved},ros::WallTime::now().toSec());
  assert(!t.validateReservedTarget());
  assert(t.reason.find("association changed")!=std::string::npos);
  SortingTask old;prepare(old,130.);
  assert(!old.validateReservedTarget()); // Masking never extends the reserved age bound.
  assert(old.reason.find("last seen")!=std::string::npos);
  SortingTask stopped;prepare(stopped);stopped.stop_requested_.store(true);
  assert(!stopped.validateReservedTarget());
}
'''
        with tempfile.TemporaryDirectory(prefix='sorting_mask_') as directory:
            path = Path(directory)
            (path/'check.cpp').write_text(fixture+body+checks)
            result = subprocess.run(['g++', '-std=c++11', '-pthread', '-I', str(CORE/'include'),
                '-I', '/opt/ros/noetic/include', str(path/'check.cpp'), '-L', '/opt/ros/noetic/lib',
                '-Wl,-rpath,/opt/ros/noetic/lib', '-lrostime', '-lcpp_common', '-o', str(path/'check')],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(path/'check')],check=True,timeout=5)


if __name__ == '__main__':
    unittest.main()
