#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

class WorkspaceCloudFilter
{
public:
  WorkspaceCloudFilter()
    : private_nh_("~"), tf_listener_(tf_buffer_)
  {
    private_nh_.param<std::string>("input", input_topic_,
                                   "/workspace_camera/depth/color/points");
    private_nh_.param<std::string>("output", output_topic_,
                                   "/workspace_camera/points_for_moveit");
    private_nh_.param<std::string>("target_frame", target_frame_, "base_link");
    private_nh_.param("minimum_range", minimum_range_, 0.15);
    private_nh_.param("maximum_range", maximum_range_, 2.5);
    private_nh_.param("transform_timeout", transform_timeout_, 0.10);

    std::vector<double> ignored_min;
    std::vector<double> ignored_max;
    private_nh_.param("ignored_box_min", ignored_min,
                      std::vector<double>{ 0.30, -0.70, -0.10 });
    private_nh_.param("ignored_box_max", ignored_max,
                      std::vector<double>{ 1.20, 0.70, 0.21 });
    if (ignored_min.size() != 3 || ignored_max.size() != 3)
      throw std::runtime_error("ignored_box_min/max must contain three values");
    for (std::size_t index = 0; index < 3; ++index)
    {
      ignored_min_[index] = ignored_min[index];
      ignored_max_[index] = ignored_max[index];
      if (ignored_min_[index] > ignored_max_[index])
        throw std::runtime_error("ignored_box_min must not exceed ignored_box_max");
    }

    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
    subscriber_ = nh_.subscribe(input_topic_, 1, &WorkspaceCloudFilter::callback, this);
    ROS_INFO_STREAM("Workspace cloud filter: " << input_topic_ << " -> " << output_topic_
                    << " in " << target_frame_);
  }

private:
  bool ignored(double x, double y, double z) const
  {
    return x >= ignored_min_[0] && x <= ignored_max_[0] &&
           y >= ignored_min_[1] && y <= ignored_max_[1] &&
           z >= ignored_min_[2] && z <= ignored_max_[2];
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    sensor_msgs::PointCloud2 transformed;
    try
    {
      const auto transform = tf_buffer_.lookupTransform(
          target_frame_, input->header.frame_id, input->header.stamp,
          ros::Duration(transform_timeout_));
      tf2::doTransform(*input, transformed, transform);
    }
    catch (const tf2::TransformException& error)
    {
      ROS_WARN_THROTTLE(1.0, "Workspace point cloud TF failed: %s", error.what());
      return;
    }

    std::vector<std::array<float, 3>> points;
    points.reserve(static_cast<std::size_t>(transformed.width) * transformed.height);
    sensor_msgs::PointCloud2ConstIterator<float> x_it(transformed, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_it(transformed, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_it(transformed, "z");
    for (; x_it != x_it.end(); ++x_it, ++y_it, ++z_it)
    {
      const float x = *x_it;
      const float y = *y_it;
      const float z = *z_it;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        continue;
      const double range = std::sqrt(x * x + y * y + z * z);
      if (range < minimum_range_ || range > maximum_range_ || ignored(x, y, z))
        continue;
      points.push_back({ x, y, z });
    }

    sensor_msgs::PointCloud2 output;
    output.header = transformed.header;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");
    for (const auto& point : points)
    {
      *out_x = point[0];
      *out_y = point[1];
      *out_z = point[2];
      ++out_x;
      ++out_y;
      ++out_z;
    }
    output.is_dense = true;
    publisher_.publish(output);
  }

  ros::NodeHandle nh_, private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  std::string input_topic_, output_topic_, target_frame_;
  std::array<double, 3> ignored_min_{}, ignored_max_{};
  double minimum_range_{ 0.15 }, maximum_range_{ 2.5 }, transform_timeout_{ 0.10 };
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "workspace_cloud_filter");
  try
  {
    WorkspaceCloudFilter filter;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("Workspace cloud filter initialization failed: %s", error.what());
    return 1;
  }
  return 0;
}
