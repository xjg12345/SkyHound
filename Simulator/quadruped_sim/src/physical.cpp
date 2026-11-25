#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_broadcaster.h>

namespace gazebo {
class PhysicalDog final : public ModelPlugin {
  using Clock = std::chrono::steady_clock;
  static constexpr double kLength = 0.2;
  static constexpr double kHeight = 0.344;
  struct Leg {
    std::string name;
    double x, y, offset;
    std::array<physics::JointPtr, 3> joints;
  };

 public:
  ~PhysicalDog() override {
    update_.reset();
    command_.shutdown();
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr options) override {
    if (!ros::isInitialized()) return;
    model_ = model;
    ns_ = options->Get<std::string>("robotNamespace");
    for (auto& leg : legs_) {
      const std::array<std::string, 3> parts{{"hip", "thigh", "calf"}};
      for (size_t i = 0; i < parts.size(); ++i) {
        leg.joints[i] = model_->GetJoint(leg.name + "_" + parts[i] + "_joint");
        if (!leg.joints[i]) {
          gzerr << "PhysicalDog: missing joint for " << leg.name << '\n';
          return;
        }
      }
    }
    node_.reset(new ros::NodeHandle("/" + ns_));
    tf_.reset(new tf2_ros::TransformBroadcaster);
    odom_ = node_->advertise<nav_msgs::Odometry>("odom", 5);
    joints_ = node_->advertise<sensor_msgs::JointState>("joint_states", 5);
    ready_ = node_->advertise<std_msgs::Bool>("physical_ready", 1, true);
    fault_ = node_->advertise<std_msgs::Bool>("physical_fault", 1, true);
    command_ = node_->subscribe("cmd_vel", 1, &PhysicalDog::Receive, this,
                                ros::TransportHints().tcpNoDelay());
    update_ = event::Events::ConnectWorldUpdateBegin(
        [this](const common::UpdateInfo& info) { Update(info); });
  }

 private:
  static double clamp(double x, double bound) { return std::clamp(x, -bound, bound); }
  static std::array<double, 3> IK(double x, double y, double down, double bend_sign) {
    const double d = std::hypot(y, down);
    const double knee = bend_sign * std::acos(std::clamp((x*x + d*d) / (2*kLength*kLength) - 1, -0.95, 0.98));
    return {{std::atan2(y, down), std::atan2(-x, d) - knee / 2, knee}};
  }

  void Receive(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    vx_ = std::isfinite(msg->linear.x) ? std::clamp(msg->linear.x, 0.0, 0.30) : 0.0;
    wz_ = std::isfinite(msg->angular.z) ? clamp(msg->angular.z, .70) : 0.0;
    received_ = Clock::now();
  }

  void Update(const common::UpdateInfo& info) {
    const double sim = info.simTime.Double();
    const double dt = std::clamp(sim - last_sim_, 0.0, .02);
    const auto wall = Clock::now();
    if (start_sim_ < 0 || sim < last_sim_) {
      start_sim_ = sim;
      last_pub_ = sim - 1;
      phase_ = 0;
      stand_time_ = 0;
      stood_ = false;
    }
    double vx = 0, wz = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (sim < last_sim_ || (last_wall_ != Clock::time_point{} &&
          wall - last_wall_ > std::chrono::milliseconds(350))) received_ = {};
      if (received_ != Clock::time_point{} && wall - received_ < std::chrono::milliseconds(250)) {
        vx = vx_;
        wz = wz_;
      }
    }
    last_sim_ = sim;
    last_wall_ = wall;
    const auto pose = model_->WorldPose();
    const auto rpy = pose.Rot().Euler();
    const auto angular = pose.Rot().RotateVectorReverse(model_->WorldAngularVel());
    const auto velocity = pose.Rot().RotateVectorReverse(model_->WorldLinearVel());
    if (sim - start_sim_ > .3 &&
        (std::abs(rpy.X()) > .65 || std::abs(rpy.Y()) > .65 || pose.Pos().Z() < .19))
      fallen_ = true;  // Latched until model respawn; never resume a retained goal.
    if (!stood_) {
      const bool steady = pose.Pos().Z() > .29 && std::abs(rpy.X()) < .12
                          && std::abs(rpy.Y()) < .12 && velocity.Length() < .07
                          && angular.Length() < .20;
      stand_time_ = steady ? stand_time_ + dt : 0;
      stood_ = stand_time_ >= .60;
    }
    const bool ready = !fallen_ && stood_;
    if (!ready) vx = wz = 0;
    forward_ += clamp(vx - forward_, .9 * dt);
    turn_ += clamp(wz - turn_, 2.5 * dt);
    const double moving = std::clamp(std::max(std::abs(forward_) / .075, std::abs(turn_) / .18), 0.0, 1.0);
    if (moving > .001) phase_ = std::fmod(phase_ + dt / .46, 1.0);
    // Damp joint oscillation in the neutral stance without retuning the trot.
    const double damping = moving <= .001 ? .85 : .12;

    for (auto& leg : legs_) {
      constexpr double duty = .68;
      const double p = std::fmod(phase_ + leg.offset, 1.0);
      const bool swing = p >= duty && moving > .001;
      const double u = swing ? (p - duty) / (1-duty) : p / duty;
      const double scale = swing ? -.5 + u*u*(3-2*u) : .5-u;
      const double lift = swing ? .055 * moving * std::sin(std::acos(-1.) * u) : 0;
      const double x = (forward_ - turn_*leg.y)*.46*duty*scale;
      const double y = turn_*leg.x*.46*duty*scale;
      const double bend_sign = std::copysign(1., leg.x);
      const auto target = IK(x, y, kHeight - lift, bend_sign);
      // Gazebo joint zero is the bent-knee reference pose defined in the SDF.
      const auto neutral = IK(0, 0, kHeight, bend_sign);
      for (size_t i = 0; i < target.size(); ++i) {
        auto& joint = leg.joints[i];
        const double effort = (i == 0 ? 26. : 40.) * (target[i] - neutral[i] - joint->Position(0))
                              - damping * joint->GetVelocity(0);
        joint->SetForce(0, fallen_ ? clamp(-.20*joint->GetVelocity(0), 3) : clamp(effort, 24));
      }
    }
    if (sim - last_pub_ >= .033) {
      last_pub_ = sim;
      Publish(info, pose, rpy, velocity, angular, ready);
    }
  }

  void Publish(const common::UpdateInfo& info, const ignition::math::Pose3d& pose,
               const ignition::math::Vector3d& rpy, const ignition::math::Vector3d& velocity,
               const ignition::math::Vector3d& angular, bool ready) {
    const auto stamp = ros::Time::isSimTime() ? ros::Time(info.simTime.sec, info.simTime.nsec) : ros::Time::now();
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = ns_ + "/odom";
    odom.child_frame_id = ns_ + "/base_footprint";
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();
    odom.pose.pose.orientation.z = std::sin(rpy.Z()/2);
    odom.pose.pose.orientation.w = std::cos(rpy.Z()/2);
    odom.twist.twist.linear.x = velocity.X();
    odom.twist.twist.linear.y = velocity.Y();
    odom.twist.twist.angular.z = angular.Z();
    odom_.publish(odom);

    geometry_msgs::TransformStamped frame;
    frame.header = odom.header;
    frame.child_frame_id = odom.child_frame_id;
    frame.transform.translation.x = pose.Pos().X();
    frame.transform.translation.y = pose.Pos().Y();
    frame.transform.rotation = odom.pose.pose.orientation;
    tf_->sendTransform(frame);
    frame.header.frame_id = odom.child_frame_id;
    frame.child_frame_id = ns_ + "/base_link";
    frame.transform.translation.x = frame.transform.translation.y = 0;
    frame.transform.translation.z = pose.Pos().Z();
    const ignition::math::Quaterniond tilt(rpy.X(), rpy.Y(), 0);
    frame.transform.rotation.x = tilt.X();
    frame.transform.rotation.y = tilt.Y();
    frame.transform.rotation.z = tilt.Z();
    frame.transform.rotation.w = tilt.W();
    tf_->sendTransform(frame);

    sensor_msgs::JointState joints;
    joints.header.stamp = stamp;
    for (const auto& leg : legs_) {
      for (const auto& joint : leg.joints) {
        joints.name.push_back(joint->GetName());
        joints.position.push_back(joint->Position(0));
        joints.velocity.push_back(joint->GetVelocity(0));
      }
    }
    joints_.publish(joints);
    std_msgs::Bool state;
    state.data = ready;
    ready_.publish(state);
    state.data = fallen_;
    fault_.publish(state);
  }

  std::array<Leg, 4> legs_{{{"fl", .22, .125, 0., {}}, {"fr", .22, -.125, .5, {}},
                            {"rl", -.22, .125, .5, {}}, {"rr", -.22, -.125, 0., {}}}};
  physics::ModelPtr model_;
  std::string ns_;
  std::unique_ptr<ros::NodeHandle> node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  ros::Publisher odom_, joints_, ready_, fault_;
  std::mutex mutex_;
  Clock::time_point received_{}, last_wall_{};
  double vx_ = 0, wz_ = 0, forward_ = 0, turn_ = 0, phase_ = 0;
  double start_sim_ = -1, last_sim_ = 0, last_pub_ = -1;
  double stand_time_ = 0;
  bool fallen_ = false, stood_ = false;
  ros::Subscriber command_;
  event::ConnectionPtr update_;
};
GZ_REGISTER_MODEL_PLUGIN(PhysicalDog)
}  // namespace gazebo
