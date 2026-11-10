"""Exercise the actual C++ callback body with a fake arm, without ROS/hardware."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

CORE = Path(__file__).resolve().parents[2] / 'aubo_sorting_core'


@unittest.skipUnless(shutil.which('g++'), 'C++ compiler unavailable')
class BackendReconfigureTest(unittest.TestCase):
    def test_busy_invalid_values_and_queue_reconfirmation(self):
        source = (CORE / 'src/sorting_task.cpp').read_text()
        begin = source.index('void SortingTask::reconfigureCallback(')
        end = source.index('bool SortingTask::prepareWorkOperation()', begin)
        callback = source[begin:end]
        harness = r'''
#include <aubo_sorting_core/instance_queue.hpp>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#define ROS_WARN_THROTTLE(...) ((void)0)
namespace moveit_msgs {
struct PlannerInterfaceDescription {std::string pipeline_id; std::vector<std::string> planner_ids;};
}
struct SortingTaskConfig {
  std::string planning_pipeline, planner_id;
  double detection_timeout{15}, grasp_offset_x{0}, grasp_offset_y{0};
  double stacked_grasp_offset_z{0},stacked_support_min_height{.02};
  double velocity_scaling{.1}, acceleration_scaling{.1}, queue_max_age{3};
  double queue_merge_distance{.02}, queue_merge_overlap{.5};
  double minimum_gripper_height{.06};
  int detection_samples{8}, target_cache_min_observations{5};
};
struct FakeArm {
  int calls{0};
  int queries{0};
  bool available{true};
  std::string pipeline, planner;
  bool getInterfaceDescriptions(std::vector<moveit_msgs::PlannerInterfaceDescription>& result) {
    ++queries;
    result = {{"ompl", {"aubo_i5", "aubo_i5[RRTConnect]", "aubo_i5[RRTstar]", "gripper[RRT]"}},
              {"nsg", {"NSG"}}};
    return available;
  }
  std::string getDefaultPlanningPipelineId() {return "ompl";}
  void setPlanningPipelineId(const std::string& value) {pipeline=value;planner.clear();}
  void setPlannerId(const std::string& value) {planner=value;}
  void setMaxVelocityScalingFactor(double) { ++calls; }
  void setMaxAccelerationScalingFactor(double) { ++calls; }
};
class SortingTask {
public:
  std::string planning_pipeline_, planner_id_, planning_service_, group_name_{"aubo_i5"};
  std::mutex operation_mutex_, queue_mutex_;
  std::atomic<bool> busy_{false};
  double detection_timeout_{15}, grasp_offset_x_{0}, grasp_offset_y_{0};
  double stacked_grasp_offset_z_{0},stacked_support_min_height_{.02};
  double velocity_scaling_{.1}, acceleration_scaling_{.1};
  double minimum_gripper_height_{.06},pregrasp_height_{.2},lift_min_height_{.25},preplace_height_{.25};
  int detection_samples_{8}, target_cache_min_observations_{5};
  double queue_empty_confirmation_{2}, queue_reserved_max_age_{10};
  aubo_sorting_core::InstanceQueue<int> instance_queue_;
  std::unique_ptr<FakeArm> arm_{new FakeArm};
  void reconfigureCallback(SortingTaskConfig&, uint32_t);
};
'''
        checks = r'''
int main() {
  SortingTask task;
  using Queue = aubo_sorting_core::InstanceQueue<int>;
  Queue::Sample sample; sample.category="can"; sample.x=.2; sample.payload=1;
  for (int i=0;i<5;++i) task.instance_queue_.update({sample},1.+i*.1);
  assert(task.instance_queue_.tracks().begin()->second.status == Queue::Status::READY);
  SortingTaskConfig config; config.velocity_scaling=.2;
  config.target_cache_min_observations=8; config.queue_max_age=5.;
  task.reconfigureCallback(config,0);
  assert(task.velocity_scaling_==.2 && task.instance_queue_.min_observations==8);
  assert(task.instance_queue_.max_age==5.);
  assert(task.instance_queue_.tracks().begin()->second.status == Queue::Status::CANDIDATE);
  assert(task.instance_queue_.tracks().begin()->second.observations == 0);
  int calls = task.arm_->calls;
  task.busy_.store(true);
  config.velocity_scaling=.8; config.target_cache_min_observations=2;
  config.minimum_gripper_height=.1;
  config.stacked_grasp_offset_z=.02;config.stacked_support_min_height=.03;
  task.reconfigureCallback(config,0);
  assert(config.velocity_scaling==.2 && config.target_cache_min_observations==8);
  assert(config.minimum_gripper_height==.06 && task.minimum_gripper_height_==.06);
  assert(config.stacked_grasp_offset_z==0. && config.stacked_support_min_height==.02);
  assert(task.arm_->calls==calls && task.instance_queue_.min_observations==8);
  task.busy_.store(false);
  config.stacked_grasp_offset_z=.01;config.stacked_support_min_height=.025;
  task.reconfigureCallback(config,0);
  assert(task.stacked_grasp_offset_z_==.01 && task.stacked_support_min_height_==.025);
  const int offset_calls=task.arm_->calls;
  config.stacked_grasp_offset_z=.051;task.reconfigureCallback(config,0);
  assert(config.stacked_grasp_offset_z==.01 && task.arm_->calls==offset_calls);
  config.stacked_support_min_height=0;task.reconfigureCallback(config,0);
  assert(config.stacked_support_min_height==.025 && task.arm_->calls==offset_calls);
  config.stacked_grasp_offset_z=std::numeric_limits<double>::quiet_NaN();task.reconfigureCallback(config,0);
  assert(config.stacked_grasp_offset_z==.01 && task.arm_->calls==offset_calls);
  // Keep the existing call-count assertions focused on the following rejected edits.
  calls=offset_calls;
  config.detection_timeout=2.; task.reconfigureCallback(config,0);
  assert(config.detection_timeout==15. && task.arm_->calls==calls);
  config.queue_max_age=11.; task.reconfigureCallback(config,0);
  assert(config.queue_max_age==5. && task.arm_->calls==calls);
  config.grasp_offset_x=std::numeric_limits<double>::quiet_NaN();
  task.reconfigureCallback(config,0);
  assert(config.grasp_offset_x==0. && task.arm_->calls==calls);
  config.minimum_gripper_height=.2;
  task.reconfigureCallback(config,0);
  assert(config.minimum_gripper_height==.06 && task.arm_->calls==calls);
  config.minimum_gripper_height=.08;
  task.reconfigureCallback(config,0);
  assert(task.minimum_gripper_height_==.08 && task.arm_->calls==calls+2);
  config.planning_pipeline="nsg"; config.planner_id="NSG";
  task.reconfigureCallback(config,0);
  assert(task.planning_pipeline_=="nsg" && task.planner_id_=="NSG");
  assert(task.arm_->pipeline=="nsg" && task.arm_->planner=="NSG");
  task.busy_.store(true);
  const int queries=task.arm_->queries;
  config.planning_pipeline="ompl";config.planner_id="RRTConnect";
  task.reconfigureCallback(config,0);
  assert(config.planning_pipeline=="nsg" && config.planner_id=="NSG");
  assert(task.arm_->queries==queries);
  task.busy_.store(false);
  config.planning_pipeline="ompl";config.planner_id="unknown";
  task.reconfigureCallback(config,0);
  assert(config.planning_pipeline=="nsg" && config.planner_id=="NSG");
  config.planning_pipeline="missing";config.planner_id="";
  task.reconfigureCallback(config,0);
  assert(config.planning_pipeline=="nsg");
  task.planning_service_="/external";
  config.planning_pipeline="ompl";config.planner_id="RRTConnect";
  task.reconfigureCallback(config,0);
  assert(config.planning_pipeline=="nsg");
  task.planning_service_.clear();task.arm_->available=false;
  config.planning_pipeline="ompl";config.planner_id="RRTConnect";
  task.reconfigureCallback(config,0);
  assert(config.planning_pipeline=="nsg");
  task.arm_->available=true;
  config.planning_pipeline="ompl";config.planner_id="gripper[RRT]";
  task.reconfigureCallback(config,0);
  assert(config.planning_pipeline=="nsg");
  config.planning_pipeline="ompl";config.planner_id="RRTConnect";
  task.reconfigureCallback(config,0);
  assert(task.arm_->pipeline=="ompl" && task.arm_->planner=="RRTConnect");
  config.planning_pipeline="";config.planner_id="RRTstar";
  task.reconfigureCallback(config,0);
  assert(task.arm_->pipeline.empty() && task.arm_->planner=="RRTstar");
  config.planning_pipeline="nsg";config.planner_id="";
  task.reconfigureCallback(config,0);
  assert(task.arm_->pipeline=="nsg" && task.arm_->planner.empty());
}
'''
        with tempfile.TemporaryDirectory(prefix='sorting_reconfigure_') as folder:
            path = Path(folder)
            (path/'check.cpp').write_text(harness + callback + checks)
            subprocess.run(['g++', '-std=c++11', '-pthread', '-I', str(CORE/'include'),
                            str(path/'check.cpp'), '-o', str(path/'check')], check=True,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=30)
            subprocess.run([str(path/'check')], check=True, timeout=5)


if __name__ == '__main__':
    unittest.main()
