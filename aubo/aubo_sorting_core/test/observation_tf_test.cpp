#include <aubo_sorting_core/observation_tf.hpp>
#include <gtest/gtest.h>

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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
