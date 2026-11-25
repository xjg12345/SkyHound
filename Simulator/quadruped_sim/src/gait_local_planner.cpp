#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

namespace prometheus_quadruped_sim {
class GaitLocalPlanner final : public nav_core::BaseLocalPlanner {
 public:
  void initialize(std::string, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap) override {
    tf_ = tf;
    costmap_ = costmap;
    ros::NodeHandle nh;
    odom_sub_ = nh.subscribe("odom", 1, &GaitLocalPlanner::receiveOdom, this);
  }

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override {
    plan_ = plan;
    reached_ = false;
    settling_since_ = {};
    return !plan_.empty();
  }

  bool isGoalReached() override { return reached_; }

  bool computeVelocityCommands(geometry_msgs::Twist& cmd) override {
    cmd = {};
    geometry_msgs::PoseStamped robot;
    if (plan_.empty() || !costmap_->isCurrent() || !costmap_->getRobotPose(robot)) {
      settling_since_ = {};
      return false;
    }

    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(robot.header.frame_id, plan_.back().header.frame_id,
                                       ros::Time(0));
    } catch (const tf2::TransformException&) {
      settling_since_ = {};
      return false;
    }
    const auto inLocal = [&](size_t index) {
      geometry_msgs::PoseStamped pose;
      tf2::doTransform(plan_[index], pose, transform);
      return pose.pose;
    };
    const auto& start = robot.pose.position;
    const double yaw = tf2::getYaw(robot.pose.orientation);
    const auto goal = inLocal(plan_.size() - 1);
    const double distance = std::hypot(goal.position.x - start.x, goal.position.y - start.y);
    const bool at_position = distance <= .19;
    double error = wrap(tf2::getYaw(goal.orientation) - yaw);
    if (at_position && std::abs(error) <= .24) {
      const auto now = ros::Time::now();
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (odom_stamp_.isZero() || now < odom_stamp_ || (now - odom_stamp_).toSec() > .25 ||
          !(odom_speed_ < .025)) {
        settling_since_ = {};
      } else if (settling_since_.isZero() || now < settling_since_) {
        settling_since_ = now;
      }
      reached_ = !settling_since_.isZero() && (now - settling_since_).toSec() >= .8;
      return true;
    }
    settling_since_ = {};
    if (!at_position) {
      size_t nearest = 0;
      double best = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < plan_.size(); ++i) {
        const auto p = inLocal(i).position;
        const double d = std::hypot(p.x - start.x, p.y - start.y);
        if (d < best) {
          best = d;
          nearest = i;
        }
      }
      auto aim = inLocal(nearest).position;
      double arc = 0;
      for (size_t i = nearest + 1; i < plan_.size() && arc < .42; ++i) {
        const auto next = inLocal(i).position;
        arc += std::hypot(next.x - aim.x, next.y - aim.y);
        aim = next;
      }
      error = wrap(std::atan2(aim.y - start.y, aim.x - start.x) - yaw);
    }
    const double w = std::clamp(1.6 * error, -.55, .55);
    const double v = at_position || std::abs(error) > .58 ? 0.0 :
        std::clamp(.8 * distance, .10, .28) * std::cos(error);
    auto& grid = *costmap_->getCostmap();
    const auto footprint = costmap_->getRobotFootprint();
    std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*grid.getMutex());
    for (const double scale : {1.0, .5, 0.0}) {
      if (scale == 0 && std::abs(w) < .12) continue;
      if (clear(grid, footprint, start.x, start.y, yaw, v * scale, w)) {
        cmd.linear.x = v * scale;
        cmd.angular.z = w;
        return true;
      }
    }
    return false;  // move_base publishes zero and applies its configured failure policy.
  }

 private:
  void receiveOdom(const nav_msgs::Odometry::ConstPtr& odom) {
    const auto& twist = odom->twist.twist;
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_stamp_ = odom->header.stamp;
    odom_speed_ = std::max(std::hypot(twist.linear.x, twist.linear.y), std::abs(twist.angular.z));
  }

  static double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

  static bool clear(costmap_2d::Costmap2D& grid,
                    const std::vector<geometry_msgs::Point>& footprint,
                    double x, double y, double yaw, double v, double w) {
    // Sample a short swept footprint including the current pose. Include interior cells.
    for (int step = 0; step <= 14; ++step) {
      std::vector<costmap_2d::MapLocation> polygon, cells;
      for (const auto& p : footprint) {
        costmap_2d::MapLocation cell;
        if (!grid.worldToMap(x + p.x * std::cos(yaw) - p.y * std::sin(yaw),
                             y + p.x * std::sin(yaw) + p.y * std::cos(yaw), cell.x, cell.y))
          return false;
        polygon.push_back(cell);
      }
      grid.convexFillCells(polygon, cells);
      for (const auto& cell : cells) {
        if (grid.getCost(cell.x, cell.y) >= costmap_2d::LETHAL_OBSTACLE) return false;
      }
      x += .05 * v * std::cos(yaw + .025 * w);
      y += .05 * v * std::sin(yaw + .025 * w);
      yaw += .05 * w;
    }
    return true;
  }

  tf2_ros::Buffer* tf_ = nullptr;
  costmap_2d::Costmap2DROS* costmap_ = nullptr;
  std::vector<geometry_msgs::PoseStamped> plan_;
  ros::Time settling_since_;
  bool reached_ = false;
  std::mutex odom_mutex_;
  ros::Time odom_stamp_;
  double odom_speed_ = 0;
  ros::Subscriber odom_sub_;
};
}  // namespace prometheus_quadruped_sim

PLUGINLIB_EXPORT_CLASS(prometheus_quadruped_sim::GaitLocalPlanner, nav_core::BaseLocalPlanner)
