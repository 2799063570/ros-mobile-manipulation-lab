"""Actual queue publisher JSON: finite geometry and the reserved execution snapshot."""
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

CORE = Path(__file__).resolve().parents[2] / 'aubo_sorting_core'


@unittest.skipUnless(shutil.which('g++'), 'C++ compiler unavailable')
class BackendQueueGeometryTest(unittest.TestCase):
    def test_geometry_is_from_the_cached_snapshot_and_invalid_values_are_json_null(self):
        source = (CORE / 'src/instance_tracking.cpp').read_text()
        body = source[source.index('void SortingTask::publishInstanceQueue()'):
                      source.index('bool SortingTask::continuousSortingOperation()')]
        fixture = r'''
#include <aubo_sorting_core/instance_queue.hpp>
#include <atomic>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
namespace ros {struct WallTime {static WallTime now(){return {};} double toSec()const{return 11.;}};}
namespace std_msgs {struct String {std::string data;};}
namespace detail {std::string jsonEscape(const std::string& value){return value;}}
struct Object {bool grasp_geometry_valid{true};double grasp_angle{0},grasp_width{.064};};
using ObjectQueue=aubo_sorting_core::InstanceQueue<Object>;
class SortingTask {
public:
  enum class QueueVisionPhase {DISABLED,COLLECT,APPROACH,MASKED};
  std::atomic<QueueVisionPhase> queue_vision_phase_{QueueVisionPhase::COLLECT};
  std::mutex queue_mutex_;
  std::string target_frame_{"base_link"};
  bool queue_target_occluded_{false};
  ObjectQueue instance_queue_;
  struct {void publish(const std_msgs::String& message){std::cout<<message.data;}} target_cache_publisher_;
  void publishInstanceQueue();
};
'''
        checks = r'''
int main() {
  SortingTask task;task.instance_queue_.min_observations=2;
  std::vector<ObjectQueue::Sample> samples(4);
  const std::string categories[]{"can","zero","invalid","nan"};
  for(unsigned i=0;i<samples.size();++i) {
    samples[i].category=categories[i];samples[i].x=.1*i;samples[i].confidence=.9;
  }
  samples[0].payload.grasp_angle=std::acos(-1.)/4;
  samples[2].payload.grasp_geometry_valid=false;
  samples[3].payload.grasp_angle=std::numeric_limits<double>::quiet_NaN();
  samples[3].payload.grasp_width=std::numeric_limits<double>::infinity();
  task.instance_queue_.update(samples,10.);
  task.instance_queue_.update(samples,10.1);
  ObjectQueue::Track reserved;
  if(!task.instance_queue_.reserve({"can"},10.15,reserved))return 1;
  samples[0].payload.grasp_angle=-.3;samples[0].payload.grasp_width=.025;
  task.instance_queue_.update(samples,10.2);
  task.publishInstanceQueue();
}
'''
        with tempfile.TemporaryDirectory(prefix='sorting_queue_geometry_') as directory:
            path = Path(directory)
            (path / 'check.cpp').write_text(fixture + body + checks)
            result = subprocess.run(['g++', '-std=c++11', '-pthread', '-I', str(CORE / 'include'),
                                     str(path / 'check.cpp'), '-o', str(path / 'check')],
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr)
            result = subprocess.run([str(path / 'check')], check=True, stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE, text=True, timeout=5)
        def reject_non_json_number(token):
            raise ValueError('Non-JSON number: ' + token)
        document = json.loads(result.stdout, parse_constant=reject_non_json_number)
        rows = {row['category']: row for row in document['targets'].values()}
        self.assertEqual(document['schema_version'], 2)
        self.assertEqual(document['frame_id'], 'base_link')
        self.assertEqual(rows['can']['status'], 'reserved')
        self.assertAlmostEqual(rows['can']['grasp_angle'], math.pi/4)
        self.assertAlmostEqual(rows['can']['grasp_width'], .064)
        self.assertEqual(rows['zero']['grasp_angle'], 0.)
        for category in ('invalid', 'nan'):
            self.assertIsNone(rows[category]['grasp_angle'])
            self.assertIsNone(rows[category]['grasp_width'])
        self.assertFalse(rows['invalid']['grasp_geometry_valid'])


if __name__ == '__main__':
    unittest.main()
