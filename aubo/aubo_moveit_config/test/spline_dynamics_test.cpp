#include <aubo_moveit_config/limit_spline_dynamics.h>
#include <aubo_moveit_config/spline_dynamics.h>
#include <aubo_moveit_config/straight_path_timing.h>
#include <gtest/gtest.h>
#include <limits>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <pluginlib/class_loader.hpp>
#include <srdfdom/model.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <urdf_parser/urdf_parser.h>

using namespace aubo_moveit_config;
using Segment = trajectory_interface::QuinticSplineSegment<double>;

namespace {
// Actual wrist1 goal from 21:14:31 bag, first execution. Peaks at the sparse
// knots are below 0.6 rad/s^2, but the controller's interpolant is not.
const std::array<double, 4> recorded[] = {
    {0, -0.8551640510559082, 0, 0},
    {0.492824866, -0.78375161785834291, 0.24796552113496723,
     0.59208174780506839},
    {0.696263582, -0.71233918466077761, 0.40183511903057101,
     0.56265208212979867},
    {0.854031089, -0.64092675146321221, 0.47632173867324584,
     0.31508798472275595},
    {0.996855955, -0.56951431826564691, 0.5, 0},
    {1.139680822, -0.49810188506808162, 0.47632173867324579,
     -0.31508798472275645},
    {1.297448329, -0.42668945187051627, 0.40183511903057106,
     -0.56265208212979778},
    {1.500887045, -0.35527701867295092, 0.24796552113496734,
     -0.59208174780506828},
    {2.134425851, -0.28386458547538557, 0, 0}};

Segment::State boundary(double p, double v, double a) {
  Segment::State state(1);
  state.position[0] = p;
  state.velocity[0] = v;
  state.acceleration[0] = a;
  return state;
}

moveit::core::RobotModelPtr robotModel(bool two_joints = false) {
  const std::string xml =
      R"(<robot name="spline_test"><link name="base"/><link name="tip"/>
    <joint name="wrist1_joint" type="revolute"><parent link="base"/><child link="tip"/>
    <axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="0.5"/></joint>
    <link name="tip2"/><joint name="test_joint" type="revolute"><parent link="tip"/><child link="tip2"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="1" velocity="0.5"/></joint></robot>)";
  auto urdf = urdf::parseURDF(xml);
  auto srdf = std::make_shared<srdf::Model>();
  if (!urdf ||
      !srdf->initString(
          *urdf,
          "<robot name='spline_test'><group name='arm'><joint "
          "name='wrist1_joint'/>" +
              std::string(two_joints ? "<joint name='test_joint'/>" : "") +
              "</group></robot>"))
    throw std::runtime_error("failed to construct in-memory model");
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  auto bounds = model->getVariableBounds("wrist1_joint");
  bounds.acceleration_bounded_ = true;
  bounds.max_acceleration_ = 0.6;
  bounds.min_acceleration_ = -0.6;
  model->getJointModel("wrist1_joint")
      ->setVariableBounds("wrist1_joint", bounds);
  auto second_bounds = model->getVariableBounds("test_joint");
  second_bounds.acceleration_bounded_ = true;
  second_bounds.max_acceleration_ = 0.4;
  second_bounds.min_acceleration_ = -0.4;
  model->getJointModel("test_joint")
      ->setVariableBounds("test_joint", second_bounds);
  return model;
}

robot_trajectory::RobotTrajectoryPtr
recordedTrajectory(bool two_joints = false) {
  auto model = robotModel(two_joints);
  auto trajectory =
      std::make_shared<robot_trajectory::RobotTrajectory>(model, "arm");
  double previous_time = 0;
  for (const auto &row : recorded) {
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    state.setVariablePosition("wrist1_joint", row[1]);
    state.setVariableVelocity("wrist1_joint", row[2]);
    state.setVariableAcceleration("wrist1_joint", row[3]);
    trajectory->addSuffixWayPoint(state, row[0] - previous_time);
    previous_time = row[0];
  }
  return trajectory;
}

std::array<double, 3>
trajectoryPeaks(const robot_trajectory::RobotTrajectory &trajectory) {
  std::array<double, 3> peak{};
  for (std::size_t i = 1; i < trajectory.getWayPointCount(); ++i) {
    const auto &first = trajectory.getWayPoint(i - 1);
    const auto &last = trajectory.getWayPoint(i);
    const double duration = trajectory.getWayPointDurationFromPrevious(i);
    auto polynomial = spline_dynamics::quintic(
        first.getVariablePosition(0), first.getVariableVelocity(0),
        first.getVariableAcceleration(0), last.getVariablePosition(0),
        last.getVariableVelocity(0), last.getVariableAcceleration(0), duration);
    auto segment_peaks = spline_dynamics::peaks(polynomial, duration);
    for (std::size_t order = 0; order < peak.size(); ++order)
      peak[order] = std::max(peak[order], segment_peaks[order]);
  }
  return peak;
}
} // namespace

TEST(SplineDynamics, RestToRestAnalyticExtremaAndRepeatedRoots) {
  auto peak =
      spline_dynamics::peaks(spline_dynamics::quintic(0, 0, 0, 1, 0, 0, 1), 1);
  EXPECT_NEAR(peak[0], 1.875, 1e-10);
  EXPECT_NEAR(peak[1], 10.0 / std::sqrt(3.0), 1e-10);
  EXPECT_NEAR(peak[2], 60.0, 1e-10);
  const auto roots = spline_dynamics::roots({0.25, -1.0, 1.0}); // (u-.5)^2
  ASSERT_FALSE(roots.empty());
  EXPECT_NEAR(roots.front(), 0.5, 1e-10);
  EXPECT_THROW(spline_dynamics::quintic(0, 0, 0, 1, 0, 0, 0),
               std::invalid_argument);
  EXPECT_THROW(spline_dynamics::quintic(
                   0, 0, 0, std::numeric_limits<double>::quiet_NaN(), 0, 0, 1),
               std::invalid_argument);
  EXPECT_THROW(spline_dynamics::requiredScale(peak, {{1, 0, 1}}),
               std::invalid_argument);
}

TEST(SplineDynamics, MatchesRealRosControlQuinticAndBoundsInteriorExtrema) {
  for (std::size_t i = 1; i < sizeof(recorded) / sizeof(recorded[0]); ++i) {
    const auto &first = recorded[i - 1];
    const auto &last = recorded[i];
    const double duration = last[0] - first[0];
    const auto polynomial = spline_dynamics::quintic(
        first[1], first[2], first[3], last[1], last[2], last[3], duration);
    const auto velocity = spline_dynamics::derivative(polynomial);
    const auto acceleration = spline_dynamics::derivative(velocity);
    const auto peak = spline_dynamics::peaks(polynomial, duration);
    Segment controller(0, boundary(first[1], first[2], first[3]), duration,
                       boundary(last[1], last[2], last[3]));
    for (int tick = 0; tick <= 1000; ++tick) {
      const double u = tick / 1000.0;
      Segment::State state;
      controller.sample(u * duration, state);
      EXPECT_NEAR(state.position[0], spline_dynamics::evaluate(polynomial, u),
                  1e-10);
      EXPECT_NEAR(state.velocity[0],
                  spline_dynamics::evaluate(velocity, u) / duration, 1e-10);
      EXPECT_NEAR(state.acceleration[0],
                  spline_dynamics::evaluate(acceleration, u) /
                      (duration * duration),
                  1e-9);
      EXPECT_LE(std::fabs(state.velocity[0]), peak[0] + 1e-9);
      EXPECT_LE(std::fabs(state.acceleration[0]), peak[1] + 1e-9);
    }
  }
}

TEST(SplineDynamics, RecordedGoalReproducesLargeInteriorAccelerationAndJerk) {
  auto peak = trajectoryPeaks(*recordedTrajectory());
  EXPECT_NEAR(peak[0], 0.50998785, 1e-7);
  EXPECT_NEAR(peak[1], 1.49843663, 1e-7);
  EXPECT_NEAR(peak[2], 42.28659434, 1e-6);
}

TEST(SplineDynamics,
     AdapterRespectsRequestScalingAndPreservesInterpolatedPath) {
  ros::param::set("/ruckig/wrist1_joint/jerk_limit", 2.5);
  for (double scaling : {1.0, 0.1}) {
    auto trajectory = recordedTrajectory();
    planning_interface::MotionPlanRequest request;
    request.max_velocity_scaling_factor =
        request.max_acceleration_scaling_factor = scaling;
    planning_interface::MotionPlanResponse response;
    const double original_duration = trajectory->getDuration();
    LimitSplineDynamics adapter;
    ros::NodeHandle options("/uniform_spline_test");
    options.setParam("optimize_straight_paths", false);
    options.setParam("stationary_start_transition_duration", 0.0);
    adapter.initialize(options);
    std::vector<std::size_t> added;
    auto planner = [&](const planning_scene::PlanningSceneConstPtr &,
                       const planning_interface::MotionPlanRequest &,
                       planning_interface::MotionPlanResponse &res) {
      res.trajectory_ = trajectory;
      return true;
    };
    ASSERT_TRUE(adapter.adaptAndPlan(planner, {}, request, response, added));
    auto peak = trajectoryPeaks(*trajectory);
    EXPECT_LE(peak[0], 0.5 * scaling);
    EXPECT_LE(peak[1], 0.6 * scaling);
    EXPECT_LE(peak[2], 2.5);
    const double ratio = trajectory->getDuration() / original_duration;
    EXPECT_GT(ratio, 2.5);
    for (std::size_t i = 1; i < trajectory->getWayPointCount(); ++i) {
      EXPECT_DOUBLE_EQ(trajectory->getWayPoint(i).getVariablePosition(0),
                       recorded[i][1]);
      EXPECT_NEAR(trajectory->getWayPointDurationFromStart(i),
                  recorded[i][0] * ratio, 1e-10);
      const double duration = recorded[i][0] - recorded[i - 1][0];
      Segment original(
          0,
          boundary(recorded[i - 1][1], recorded[i - 1][2], recorded[i - 1][3]),
          duration, boundary(recorded[i][1], recorded[i][2], recorded[i][3]));
      const auto &first = trajectory->getWayPoint(i - 1);
      const auto &last = trajectory->getWayPoint(i);
      Segment slowed(
          0,
          boundary(first.getVariablePosition(0), first.getVariableVelocity(0),
                   first.getVariableAcceleration(0)),
          duration * ratio,
          boundary(last.getVariablePosition(0), last.getVariableVelocity(0),
                   last.getVariableAcceleration(0)));
      for (int tick = 0; tick <= 20; ++tick) {
        Segment::State a, b;
        original.sample(duration * tick / 20., a);
        slowed.sample(duration * ratio * tick / 20., b);
        EXPECT_NEAR(a.position[0], b.position[0], 1e-10);
      }
    }
  }
}

TEST(SplineDynamics, AdapterRejectsMissingLimitsInvalidDurationAndMovingStart) {
  for (int mode = 0; mode < 3; ++mode) {
    ros::param::set("/ruckig/wrist1_joint/jerk_limit", 2.5);
    auto trajectory = recordedTrajectory();
    if (mode == 0)
      ros::param::del("/ruckig/wrist1_joint/jerk_limit");
    if (mode == 1)
      trajectory->setWayPointDurationFromPrevious(1, 0.0);
    if (mode == 2)
      trajectory->getWayPointPtr(0)->setVariableVelocity(0, 0.01);
    planning_interface::MotionPlanRequest request;
    planning_interface::MotionPlanResponse response;
    std::vector<std::size_t> added;
    auto planner = [&](const planning_scene::PlanningSceneConstPtr &,
                       const planning_interface::MotionPlanRequest &,
                       planning_interface::MotionPlanResponse &res) {
      res.trajectory_ = trajectory;
      return true;
    };
    LimitSplineDynamics adapter;
    EXPECT_FALSE(adapter.adaptAndPlan(planner, {}, request, response, added));
    EXPECT_EQ(response.error_code_.val,
              moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
  }
}

TEST(SplineDynamics, AdapterCanBeDiscoveredAndLoadedByMoveItPluginLoader) {
  pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter>
      loader("moveit_core", "planning_request_adapter::PlanningRequestAdapter");
  auto adapter =
      loader.createInstance("aubo_moveit_config/LimitSplineDynamics");
  ASSERT_TRUE(static_cast<bool>(adapter));
  EXPECT_NE(adapter->getDescription().find("quintic"), std::string::npos);
}

TEST(SplineDynamics,
     StraightSCurvePreservesAllWaypointsAndBoundsControllerSpline) {
  auto trajectory = recordedTrajectory();
  const std::vector<std::array<double, 3>> limits{{{0.25, 0.24, 2.5}}};
  std::vector<std::size_t> added;
  ASSERT_TRUE(retimeStraightPath(*trajectory, limits, added));
  EXPECT_LT(trajectory->getDuration(),
            4.0); // Uniform fallback for this goal was 5.48s.
  EXPECT_FALSE(added.empty());
  auto peak = trajectoryPeaks(*trajectory);
  EXPECT_LE(peak[0], 0.25);
  EXPECT_LE(peak[1], 0.24);
  EXPECT_LE(peak[2], 2.5);
  std::size_t original = 0;
  for (std::size_t i = 0; i < trajectory->getWayPointCount(); ++i) {
    if (std::find(added.begin(), added.end(), i) != added.end())
      continue;
    ASSERT_LT(original, sizeof(recorded) / sizeof(recorded[0]));
    EXPECT_DOUBLE_EQ(trajectory->getWayPoint(i).getVariablePosition(0),
                     recorded[original++][1]);
  }
  EXPECT_EQ(original, sizeof(recorded) / sizeof(recorded[0]));
  for (std::size_t i = 1; i < trajectory->getWayPointCount(); ++i) {
    const auto &a = trajectory->getWayPoint(i - 1);
    const auto &b = trajectory->getWayPoint(i);
    const double dt = trajectory->getWayPointDurationFromPrevious(i);
    Segment controller(
        0,
        boundary(a.getVariablePosition(0), a.getVariableVelocity(0),
                 a.getVariableAcceleration(0)),
        dt,
        boundary(b.getVariablePosition(0), b.getVariableVelocity(0),
                 b.getVariableAcceleration(0)));
    // Every phase has constant jerk. Independent controller samples must have
    // linear acceleration, without the old interior acceleration ripples.
    for (int tick = 0; tick <= 100; ++tick) {
      Segment::State state;
      const double u = tick / 100.;
      controller.sample(dt * u, state);
      EXPECT_LE(std::fabs(state.velocity[0]), 0.25 + 1e-8);
      EXPECT_LE(std::fabs(state.acceleration[0]), 0.24 + 1e-8);
      // ROS rounds absolute waypoint times to 1ns while original positions
      // remain exact. Bound the resulting Hermite perturbation separately;
      // the complete V/A/J extrema above must still obey the actual limits.
      const double rounding_bound = 1e-6 + 20 * 0.25 * 1e-9 / (dt * dt);
      EXPECT_NEAR(state.acceleration[0],
                  (1 - u) * a.getVariableAcceleration(0) +
                      u * b.getVariableAcceleration(0),
                  rounding_bound);
    }
  }
}

TEST(SplineDynamics,
     StraightSCurveSynchronizesTwoJointsOnTheSameGeometricLine) {
  auto trajectory = recordedTrajectory(true);
  const double origin = trajectory->getWayPoint(0).getVariablePosition(0);
  for (std::size_t i = 0; i < trajectory->getWayPointCount(); ++i) {
    auto state = trajectory->getWayPointPtr(i);
    state->setVariablePosition("test_joint",
                               (state->getVariablePosition(0) - origin) * 0.5);
    state->setVariableVelocity("test_joint",
                               state->getVariableVelocity(0) * 0.5);
    state->setVariableAcceleration("test_joint",
                                   state->getVariableAcceleration(0) * 0.5);
  }
  std::vector<std::size_t> added;
  ASSERT_TRUE(retimeStraightPath(
      *trajectory, {{{0.25, 0.24, 2.5}}, {{0.10, 0.08, 0.5}}}, added));
  for (std::size_t i = 1; i < trajectory->getWayPointCount(); ++i) {
    const auto &a = trajectory->getWayPoint(i - 1);
    const auto &b = trajectory->getWayPoint(i);
    const double dt = trajectory->getWayPointDurationFromPrevious(i);
    const auto polynomial = spline_dynamics::quintic(
        a.getVariablePosition(1), a.getVariableVelocity(1),
        a.getVariableAcceleration(1), b.getVariablePosition(1),
        b.getVariableVelocity(1), b.getVariableAcceleration(1), dt);
    const auto peak = spline_dynamics::peaks(polynomial, dt);
    EXPECT_LE(peak[0], 0.10);
    EXPECT_LE(peak[1], 0.08);
    EXPECT_LE(peak[2], 0.5);
    EXPECT_NEAR(b.getVariablePosition(1),
                (b.getVariablePosition(0) - origin) * 0.5, 1e-12);
    EXPECT_NEAR(b.getVariableVelocity(1), b.getVariableVelocity(0) * 0.5,
                1e-12);
    EXPECT_NEAR(b.getVariableAcceleration(1),
                b.getVariableAcceleration(0) * 0.5, 1e-12);
  }
}

TEST(SplineDynamics,
     StraightTimingRejectsCurvesHiddenInDerivativesAndIntentionalStops) {
  for (int mode = 0; mode < 9; ++mode) {
    auto trajectory = recordedTrajectory(mode == 0 || mode == 1);
    auto middle = trajectory->getWayPointPtr(3);
    if (mode == 0)
      middle->setVariablePosition("test_joint", 0.01);
    if (mode == 1)
      middle->setVariableVelocity("test_joint",
                                  0.01); // Knots alone are still collinear.
    if (mode == 2)
      middle->setVariableVelocity(0, 0.0); // Intermediate stop.
    if (mode == 3)
      middle->setVariablePosition(0, recorded[1][1]); // Backtracking.
    if (mode == 4)
      trajectory->getWayPointPtr(0)->setVariableVelocity(0, 0.01);
    if (mode == 5)
      trajectory->getWayPointPtr(8)->setVariableAcceleration(0, 0.01);
    if (mode == 6)
      middle->setVariablePosition("test_joint",
                                  0.01); // Outside the planning group.
    if (mode == 7)
      trajectory->setWayPointDurationFromPrevious(0,
                                                  0.2); // Preserve start delay.
    if (mode == 8)
      trajectory->setWayPointDurationFromPrevious(1, 0.0);
    const auto duration = trajectory->getDuration();
    const auto waypoint_count = trajectory->getWayPointCount();
    std::vector<std::array<double, 3>> limits(
        trajectory->getGroup()->getVariableCount(), {{0.25, 0.24, 2.5}});
    std::vector<std::size_t> added;
    EXPECT_FALSE(retimeStraightPath(*trajectory, limits, added))
        << "mode " << mode;
    EXPECT_DOUBLE_EQ(trajectory->getDuration(), duration);
    EXPECT_EQ(trajectory->getWayPointCount(), waypoint_count);
    EXPECT_TRUE(added.empty());
  }
}

TEST(SplineDynamics,
     AdapterSelectsFasterVerifiedSCurveAtFixedHalfSpeedProfile) {
  ros::param::set("/ruckig/wrist1_joint/jerk_limit", 2.5);
  auto trajectory = recordedTrajectory();
  planning_interface::MotionPlanRequest request;
  request.max_velocity_scaling_factor = 0.5;
  request.max_acceleration_scaling_factor = 0.4;
  planning_interface::MotionPlanResponse response;
  std::vector<std::size_t> added;
  auto planner = [&](const planning_scene::PlanningSceneConstPtr &,
                     const planning_interface::MotionPlanRequest &,
                     planning_interface::MotionPlanResponse &result) {
    result.trajectory_ = trajectory;
    return true;
  };
  LimitSplineDynamics adapter;
  ASSERT_TRUE(adapter.adaptAndPlan(planner, {}, request, response, added));
  EXPECT_FALSE(added.empty());
  EXPECT_LT(trajectory->getDuration(), 4.0);
  const auto peak = trajectoryPeaks(*trajectory);
  EXPECT_LE(peak[0], 0.25);
  EXPECT_LE(peak[1], 0.24);
  EXPECT_LE(peak[2], 2.5);
}

TEST(SplineDynamics,
     CurvedTrajectoryAutomaticallyKeepsVerifiedUniformFallback) {
  ros::param::set("/ruckig/wrist1_joint/jerk_limit", 2.5);
  ros::param::set("/ruckig/test_joint/jerk_limit", 2.0);
  auto trajectory = recordedTrajectory(true);
  trajectory->getWayPointPtr(3)->setVariablePosition("test_joint", 0.02);
  robot_trajectory::RobotTrajectory original(*trajectory, true);
  planning_interface::MotionPlanRequest request;
  request.max_velocity_scaling_factor = 0.5;
  request.max_acceleration_scaling_factor = 0.4;
  planning_interface::MotionPlanResponse response;
  std::vector<std::size_t> added;
  auto planner = [&](const planning_scene::PlanningSceneConstPtr &,
                     const planning_interface::MotionPlanRequest &,
                     planning_interface::MotionPlanResponse &result) {
    result.trajectory_ = trajectory;
    return true;
  };
  LimitSplineDynamics adapter;
  ros::NodeHandle options("/curved_spline_test");
  options.setParam("stationary_start_transition_duration", 0.0);
  adapter.initialize(options);
  ASSERT_TRUE(adapter.adaptAndPlan(planner, {}, request, response, added));
  EXPECT_TRUE(added.empty());
  EXPECT_EQ(trajectory->getWayPointCount(), original.getWayPointCount());
  const double scale = trajectory->getDuration() / original.getDuration();
  EXPECT_GT(scale, 1.0);
  for (std::size_t i = 0; i < trajectory->getWayPointCount(); ++i)
    for (int k : trajectory->getGroup()->getVariableIndexList()) {
      EXPECT_DOUBLE_EQ(trajectory->getWayPoint(i).getVariablePosition(k),
                       original.getWayPoint(i).getVariablePosition(k));
      EXPECT_NEAR(trajectory->getWayPoint(i).getVariableVelocity(k),
                  original.getWayPoint(i).getVariableVelocity(k) / scale,
                  1e-12);
      EXPECT_NEAR(trajectory->getWayPoint(i).getVariableAcceleration(k),
                  original.getWayPoint(i).getVariableAcceleration(k) /
                      (scale * scale),
                  1e-12);
    }
}

TEST(SplineDynamics, StationaryStartGivesControllerTimeToBridgeMicroradianFeedbackOffset) {
  ros::param::set("/ruckig/wrist1_joint/jerk_limit", 2.5);
  auto trajectory = recordedTrajectory();
  planning_interface::MotionPlanRequest request;
  request.max_velocity_scaling_factor = .5; request.max_acceleration_scaling_factor = .4;
  planning_interface::MotionPlanResponse response; std::vector<std::size_t> added;
  auto planner = [&](const planning_scene::PlanningSceneConstPtr&, const planning_interface::MotionPlanRequest&,
                     planning_interface::MotionPlanResponse& result) { result.trajectory_ = trajectory; return true; };
  LimitSplineDynamics adapter;
  ASSERT_TRUE(adapter.adaptAndPlan(planner, {}, request, response, added));
  const double transition = trajectory->getWayPointDurationFromPrevious(0);
  EXPECT_DOUBLE_EQ(transition, .2);
  const double target = trajectory->getWayPoint(0).getVariablePosition(0);
  const double offset = 1e-5;
  Segment bridge(0, boundary(target+offset,0,0), transition, boundary(target,0,0));
  const auto peak = spline_dynamics::peaks(spline_dynamics::quintic(target+offset,0,0,target,0,0,transition),transition);
  EXPECT_NEAR(peak[2], .075, 1e-6);
  for (int tick = 0; tick <= 100; ++tick) {
    Segment::State state; bridge.sample(transition*tick/100.,state);
    EXPECT_LE(std::fabs(state.velocity[0]), .0001);
    EXPECT_LE(std::fabs(state.acceleration[0]), .002);
  }
  EXPECT_DOUBLE_EQ(trajectory->getLastWayPoint().getVariablePosition(0), recorded[8][1]);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "spline_dynamics_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
