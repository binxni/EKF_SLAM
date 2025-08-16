#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"
#include "tf2/utils.h"

class GlobalToPolarNode : public rclcpp::Node
{
public:
  GlobalToPolarNode()
  : Node("global_to_polar_node")
  {
    path_sub_ = create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
      "/planned_path_with_velocity", 10,
      std::bind(&GlobalToPolarNode::pathCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/pf/pose/odom", 10,
      std::bind(&GlobalToPolarNode::odomCallback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&GlobalToPolarNode::scanCallback, this, std::placeholders::_1));
    grid_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "/polar_grid", 10);
  }

private:
  void pathCallback(const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg)
  {
    path_ = *msg;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_ = *msg;
    odom_received_ = true;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    if (!odom_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No odom received yet");
      return;
    }
    if (scan->ranges.size() != 1080) {
      RCLCPP_WARN(get_logger(), "Scan size %zu not 1080", scan->ranges.size());
      return;
    }

    std::vector<float> grid(1080, std::numeric_limits<float>::infinity());

    double robot_x = odom_.pose.pose.position.x;
    double robot_y = odom_.pose.pose.position.y;
    double yaw = tf2::getYaw(odom_.pose.pose.orientation);

    for (const auto & pt : path_.points) {
      if (pt.transforms.empty()) {
        continue;
      }
      const auto & tr = pt.transforms[0].translation;
      double dx = tr.x - robot_x;
      double dy = tr.y - robot_y;
      double rel_angle = std::atan2(dy, dx) - yaw;
      // normalize angle
      while (rel_angle > M_PI) rel_angle -= 2.0 * M_PI;
      while (rel_angle < -M_PI) rel_angle += 2.0 * M_PI;
      int idx = static_cast<int>(std::round((rel_angle - scan->angle_min) / scan->angle_increment));
      if (idx >= 0 && idx < 1080) {
        float dist = std::sqrt(dx * dx + dy * dy);
        if (!std::isfinite(grid[idx]) || dist < grid[idx]) {
          grid[idx] = dist;
        }
      }
    }

    for (auto & v : grid) {
      if (!std::isfinite(v)) {
        v = 0.0f;
      }
    }

    auto msg = sensor_msgs::msg::LaserScan();
    msg.header.stamp = scan->header.stamp;
    msg.header.frame_id = "base_link";
    msg.angle_min = scan->angle_min;
    msg.angle_max = scan->angle_max;
    msg.angle_increment = scan->angle_increment;
    msg.range_min = 0.0;
    msg.range_max = 100.0;
    msg.ranges = grid;
    grid_pub_->publish(msg);
  }

  trajectory_msgs::msg::MultiDOFJointTrajectory path_;
  nav_msgs::msg::Odometry odom_;
  bool odom_received_ {false};

  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr grid_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalToPolarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

