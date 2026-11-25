#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

namespace gazebo {
class QuadrupedProxy : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    if (!ros::isInitialized()) {
      gzerr << "QuadrupedProxy requires gazebo_ros\n";
      return;
    }
    model_ = model;
    const auto ns = sdf->Get<std::string>("robotNamespace");
    odom_frame_ = ns + "/odom";
    base_frame_ = ns + "/base_footprint";
    node_.reset(new ros::NodeHandle("/" + ns));
    publish_odom_ = sdf->Get<bool>("publishOdometry", true).first;
    if (publish_odom_) {
      odom_ = node_->advertise<nav_msgs::Odometry>("odom", 5);
      tf_.reset(new tf2_ros::TransformBroadcaster);
    }
    command_ = node_->subscribe("cmd_vel", 1, &QuadrupedProxy::OnCommand, this,
                                ros::TransportHints().tcpNoDelay());
    update_ = event::Events::ConnectWorldUpdateBegin(
        [this](const common::UpdateInfo& info) { OnUpdate(info); });
  }

 private:
  using Clock = std::chrono::steady_clock;
  void OnCommand(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    x_ = std::isfinite(msg->linear.x) ? msg->linear.x : 0.0;
    yaw_ = std::isfinite(msg->angular.z) ? msg->angular.z : 0.0;
    received_ = Clock::now();
  }

  void OnUpdate(const common::UpdateInfo& info) {
    const double sim = info.simTime.Double();
    const auto now = Clock::now();
    double x = 0.0, yaw = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // A reset or a long gap between physics ticks requires a new command.
      if (sim < last_sim_ ||
          (last_update_ != Clock::time_point{} && now - last_update_ > std::chrono::milliseconds(350))) {
        received_ = {};
        last_odom_ = sim - 1.0;
      }
      if (received_ != Clock::time_point{} && now - received_ < std::chrono::milliseconds(250)) {
        x = std::max(0.0, std::min(0.35, x_));
        yaw = std::max(-0.7, std::min(0.7, yaw_));
      }
    }
    last_update_ = now;
    last_sim_ = sim;

    const auto pose = model_->WorldPose();
    const auto velocity = model_->WorldLinearVel();
    const auto angular = model_->WorldAngularVel();
    const double heading = pose.Rot().Yaw();
    const double c = std::cos(heading), s = std::sin(heading);
    model_->SetLinearVel({c * x, s * x, velocity.Z()});
    model_->SetAngularVel({0.0, 0.0, yaw});
    if (!publish_odom_ || sim - last_odom_ < 0.033) return;
    last_odom_ = sim;

    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::isSimTime() ? ros::Time(info.simTime.sec, info.simTime.nsec)
                                             : ros::Time::now();
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = base_frame_;
    msg.pose.pose.position.x = pose.Pos().X();
    msg.pose.pose.position.y = pose.Pos().Y();
    msg.pose.pose.orientation.z = std::sin(heading * 0.5);
    msg.pose.pose.orientation.w = std::cos(heading * 0.5);
    msg.twist.twist.linear.x = c * velocity.X() + s * velocity.Y();
    msg.twist.twist.linear.y = -s * velocity.X() + c * velocity.Y();
    msg.twist.twist.angular.z = angular.Z();
    geometry_msgs::TransformStamped tf;
    tf.header = msg.header;
    tf.child_frame_id = msg.child_frame_id;
    tf.transform.translation.x = msg.pose.pose.position.x;
    tf.transform.translation.y = msg.pose.pose.position.y;
    tf.transform.rotation = msg.pose.pose.orientation;
    tf_->sendTransform(tf);
    odom_.publish(msg);
  }

  physics::ModelPtr model_;
  std::unique_ptr<ros::NodeHandle> node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  ros::Publisher odom_;
  std::string odom_frame_, base_frame_;
  bool publish_odom_ = true;
  std::mutex mutex_;
  Clock::time_point received_{}, last_update_{};
  double x_ = 0.0, yaw_ = 0.0, last_sim_ = 0.0, last_odom_ = -1.0;
  ros::Subscriber command_;
  event::ConnectionPtr update_;
};
GZ_REGISTER_MODEL_PLUGIN(QuadrupedProxy)
}  // namespace gazebo
