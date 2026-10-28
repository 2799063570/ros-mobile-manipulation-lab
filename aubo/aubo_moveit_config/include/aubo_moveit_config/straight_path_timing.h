#pragma once

#include <array>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <vector>

namespace aubo_moveit_config {
// Optimize only a monotonic, rest-to-rest straight joint-space path. Every
// original waypoint is retained; additional points mark jerk switches so the
// controller reconstructs each cubic phase exactly with its P/V/A spline.
// Returns false without changing the input for curved/moving/invalid paths.
bool retimeStraightPath(robot_trajectory::RobotTrajectory &trajectory,
                        const std::vector<std::array<double, 3>> &limits,
                        std::vector<std::size_t> &added_path_index);
} // namespace aubo_moveit_config
