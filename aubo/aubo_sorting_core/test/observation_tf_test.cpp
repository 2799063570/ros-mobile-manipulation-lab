#include <aubo_sorting_core/observation_tf.hpp>
#include <gtest/gtest.h>
#include <atomic>
#include <thread>

TEST(ObservationTf, UsesOnlyVeryRecentWorldTransformForFutureTimestamp)
{
  tf::Transformer transformer;
  const tf::StampedTransform sample(
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 0, 0)),
      ros::Time(10, 0), "odom", "base_link");
  ASSERT_TRUE(transformer.setTransform(sample));

  tf::StampedTransform result;
  EXPECT_THROW(aubo_sorting_core::lookupObservationTransform(
      transformer, "odom", "base_link", ros::Time(10, 4000000), false, result),
      tf::ExtrapolationException);
  EXPECT_NO_THROW(aubo_sorting_core::lookupObservationTransform(
      transformer, "odom", "base_link", ros::Time(10, 4000000), true, result));
  EXPECT_NEAR(1.0, result.getOrigin().x(), 1e-9);
  EXPECT_THROW(aubo_sorting_core::lookupObservationTransform(
      transformer, "odom", "base_link", ros::Time(10, 40000000), true, result),
      tf::ExtrapolationException);
}

TEST(ObservationTf, WaitsForDelayedArmTfAndInterpolatesAtTheOriginalImageStamp)
{
  tf::Transformer transformer;
  ASSERT_TRUE(transformer.setTransform(tf::StampedTransform(
      tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,0)),ros::Time(10,0),"base_link","upperArm_Link")));
  const ros::Time image_stamp(10,28000000);
  std::thread publisher([&transformer] {
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    transformer.setTransform(tf::StampedTransform(
        tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(1,0,0)),ros::Time(10,60000000),"base_link","upperArm_Link"));
  });
  std::string error;
  const bool ready=aubo_sorting_core::waitForObservationTransforms(transformer,
      {{"base_link","upperArm_Link"}},image_stamp,.15,[]{return false;},error);
  publisher.join();
  ASSERT_TRUE(ready) << error;
  tf::StampedTransform result;
  aubo_sorting_core::lookupObservationTransform(transformer,"base_link","upperArm_Link",image_stamp,false,result);
  EXPECT_EQ(result.stamp_,image_stamp);
  EXPECT_NEAR(result.getOrigin().x(),28./60.,1e-8);
  EXPECT_EQ(image_stamp,ros::Time(10,28000000));
}

TEST(ObservationTf, RequiresAllMovingLinksAtTheSameImageTime)
{
  tf::Transformer transformer;
  const tf::Transform identity(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,0));
  ASSERT_TRUE(transformer.setTransform(tf::StampedTransform(identity,ros::Time(10,0),"base_link","upperArm_Link")));
  ASSERT_TRUE(transformer.setTransform(tf::StampedTransform(identity,ros::Time(10,60000000),"base_link","upperArm_Link")));
  ASSERT_TRUE(transformer.setTransform(tf::StampedTransform(identity,ros::Time(10,0),"base_link","tcp_link")));
  std::string error;
  EXPECT_FALSE(aubo_sorting_core::waitForObservationTransforms(transformer,
      {{"base_link","upperArm_Link"},{"base_link","tcp_link"}},ros::Time(10,29000000),.01,[]{return false;},error));
  EXPECT_FALSE(error.empty());
  ASSERT_TRUE(transformer.setTransform(tf::StampedTransform(identity,ros::Time(10,60000000),"base_link","tcp_link")));
  EXPECT_TRUE(aubo_sorting_core::waitForObservationTransforms(transformer,
      {{"base_link","upperArm_Link"},{"base_link","tcp_link"}},ros::Time(10,29000000),.01,[]{return false;},error));
}

TEST(ObservationTf, MissingOrTooOldTfFailsWithinABoundedWallTime)
{
  tf::Transformer transformer;
  std::string error;
  const auto begin=std::chrono::steady_clock::now();
  EXPECT_FALSE(aubo_sorting_core::waitForObservationTransforms(transformer,
      {{"base_link","missing"}},ros::Time(10,28000000),.03,[]{return false;},error));
  const double duration=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
  EXPECT_GE(duration,.025);
  EXPECT_LT(duration,.2);
  EXPECT_FALSE(error.empty());
  const tf::Transform identity(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,0));
  ASSERT_TRUE(transformer.setTransform(tf::StampedTransform(identity,ros::Time(10,0),"base_link","upperArm_Link")));
  EXPECT_FALSE(aubo_sorting_core::waitForObservationTransforms(transformer,
      {{"base_link","upperArm_Link"}},ros::Time(9,0),.01,[]{return false;},error));
}

TEST(ObservationTf, StopOrPhaseChangeCancelsTheWait)
{
  tf::Transformer transformer;
  std::atomic<bool> cancel{false};
  std::thread phase_change([&cancel] {
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); cancel.store(true);
  });
  std::string error;
  const auto begin=std::chrono::steady_clock::now();
  const bool ready=aubo_sorting_core::waitForObservationTransforms(transformer,
      {{"base_link","upperArm_Link"}},ros::Time(10,28000000),.15,[&cancel]{return cancel.load();},error);
  phase_change.join();
  EXPECT_FALSE(ready);
  EXPECT_TRUE(error.empty());
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count(),.12);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
