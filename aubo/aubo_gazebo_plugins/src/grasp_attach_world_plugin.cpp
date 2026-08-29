#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <boost/bind.hpp>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace gazebo
{
class SortingGraspAttachPlugin : public WorldPlugin
{
public:
  SortingGraspAttachPlugin() : running_(false), pending_action_(Action::NONE)
  {
  }

  ~SortingGraspAttachPlugin() override
  {
    running_ = false;
    ros_queue_.disable();
    if (ros_node_)
    {
      ros_node_->shutdown();
    }
    if (ros_queue_thread_.joinable())
    {
      ros_queue_thread_.join();
    }
    update_connection_.reset();
    grasp_joint_.reset();
  }

  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;
    robot_model_name_ = readSdf(sdf, "robot_model", "aubo_i5");
    palm_link_name_ = readSdf(sdf, "palm_link", "wrist3_Link");
    object_link_name_ = readSdf(sdf, "object_link", "block_link");
    max_attach_distance_ = sdf->HasElement("max_attach_distance") ?
        sdf->Get<double>("max_attach_distance") : 0.18;
    const std::string attach_topic =
        readSdf(sdf, "attach_topic", "/sorting/grasp/attach");
    const std::string detach_topic =
        readSdf(sdf, "detach_topic", "/sorting/grasp/detach");
    const std::string status_topic =
        readSdf(sdf, "status_topic", "/sorting/grasp/status");

    if (!ros::isInitialized())
    {
      gzerr << "[aubo_grasp_attach] ROS is not initialized. "
            << "Launch Gazebo through gazebo_ros.\n";
      return;
    }

    ros_node_.reset(new ros::NodeHandle(""));
    ros::SubscribeOptions attach_options =
        ros::SubscribeOptions::create<std_msgs::String>(
            attach_topic, 1,
            boost::bind(&SortingGraspAttachPlugin::attachCallback, this, _1),
            ros::VoidPtr(), &ros_queue_);
    ros::SubscribeOptions detach_options =
        ros::SubscribeOptions::create<std_msgs::String>(
            detach_topic, 1,
            boost::bind(&SortingGraspAttachPlugin::detachCallback, this, _1),
            ros::VoidPtr(), &ros_queue_);
    attach_subscriber_ = ros_node_->subscribe(attach_options);
    detach_subscriber_ = ros_node_->subscribe(detach_options);
    status_publisher_ = ros_node_->advertise<std_msgs::String>(status_topic, 1, true);

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SortingGraspAttachPlugin::onUpdate, this));
    running_ = true;
    ros_queue_thread_ = std::thread(&SortingGraspAttachPlugin::queueThread, this);
    publishStatus("ready");
    ROS_INFO_STREAM("Sorting grasp plugin ready: " << robot_model_name_
                    << "::" << palm_link_name_);
  }

private:
  enum class Action
  {
    NONE,
    ATTACH,
    DETACH
  };

  static std::string readSdf(const sdf::ElementPtr& sdf,
                             const std::string& key,
                             const std::string& fallback)
  {
    return sdf->HasElement(key) ? sdf->Get<std::string>(key) : fallback;
  }

  void attachCallback(const std_msgs::StringConstPtr& message)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    pending_model_name_ = message->data;
    pending_action_ = Action::ATTACH;
  }

  void detachCallback(const std_msgs::StringConstPtr& message)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    pending_model_name_ = message->data;
    pending_action_ = Action::DETACH;
  }

  void queueThread()
  {
    while (running_ && ros_node_ && ros_node_->ok())
    {
      ros_queue_.callAvailable(ros::WallDuration(0.01));
    }
  }

  void onUpdate()
  {
    Action action = Action::NONE;
    std::string model_name;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      action = pending_action_;
      model_name.swap(pending_model_name_);
      pending_action_ = Action::NONE;
    }
    if (action == Action::ATTACH)
    {
      attachInPhysicsThread(model_name);
    }
    else if (action == Action::DETACH)
    {
      detachInPhysicsThread();
    }
  }

  void attachInPhysicsThread(const std::string& object_model_name)
  {
    if (object_model_name.empty())
    {
      publishStatus("error:empty_object_model");
      return;
    }
    if (attached_model_name_ == object_model_name && grasp_joint_)
    {
      publishStatus("attached:" + object_model_name);
      return;
    }

    detachInPhysicsThread();
    physics::ModelPtr robot_model = world_->ModelByName(robot_model_name_);
    physics::ModelPtr object_model = world_->ModelByName(object_model_name);
    if (!robot_model || !object_model)
    {
      publishStatus("error:model_not_found:" + object_model_name);
      return;
    }

    physics::LinkPtr palm_link = robot_model->GetLink(palm_link_name_);
    physics::LinkPtr object_link = object_model->GetLink(object_link_name_);
    if (!palm_link || !object_link)
    {
      publishStatus("error:link_not_found:" + object_model_name);
      return;
    }

    const double attach_distance =
        palm_link->WorldPose().Pos().Distance(object_link->WorldPose().Pos());
    if (attach_distance > max_attach_distance_)
    {
      publishStatus("error:object_too_far:" + object_model_name);
      ROS_ERROR_STREAM("Refusing to attach " << object_model_name
                       << ": palm distance " << attach_distance
                       << " m exceeds " << max_attach_distance_ << " m");
      return;
    }

    grasp_joint_ = world_->Physics()->CreateJoint("fixed", robot_model);
    if (!grasp_joint_)
    {
      publishStatus("error:create_joint_failed:" + object_model_name);
      return;
    }
    const ignition::math::Pose3d relative_pose =
        object_link->WorldPose() - palm_link->WorldPose();
    grasp_joint_->Attach(palm_link, object_link);
    grasp_joint_->Load(palm_link, object_link, relative_pose);
    grasp_joint_->Init();
    attached_model_name_ = object_model_name;
    publishStatus("attached:" + object_model_name);
    ROS_INFO_STREAM("Attached sorting object " << object_model_name);
  }

  void detachInPhysicsThread()
  {
    const std::string previous_model = attached_model_name_;
    if (grasp_joint_)
    {
      grasp_joint_->Detach();
      grasp_joint_->Fini();
      grasp_joint_.reset();
    }
    attached_model_name_.clear();
    if (!previous_model.empty())
    {
      publishStatus("detached:" + previous_model);
      ROS_INFO_STREAM("Detached sorting object " << previous_model);
    }
  }

  void publishStatus(const std::string& status)
  {
    if (!status_publisher_)
    {
      return;
    }
    std_msgs::String message;
    message.data = status;
    status_publisher_.publish(message);
  }

  physics::WorldPtr world_;
  physics::JointPtr grasp_joint_;
  event::ConnectionPtr update_connection_;
  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::CallbackQueue ros_queue_;
  ros::Subscriber attach_subscriber_;
  ros::Subscriber detach_subscriber_;
  ros::Publisher status_publisher_;
  std::thread ros_queue_thread_;
  std::atomic<bool> running_;
  std::mutex command_mutex_;
  Action pending_action_;
  std::string pending_model_name_;
  std::string attached_model_name_;
  std::string robot_model_name_;
  std::string palm_link_name_;
  std::string object_link_name_;
  double max_attach_distance_;
};

GZ_REGISTER_WORLD_PLUGIN(SortingGraspAttachPlugin)
}  // namespace gazebo
