#ifndef EKF_SLAM_NODE_HPP_
#define EKF_SLAM_NODE_HPP_

#include <memory>
#include <string>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "visualization/trajectory_visualizer.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "preprocessing/laser_processor.hpp"
#include "core/slam_system.hpp"
#include "mapping/occupancy_mapper.hpp"

namespace ekf_slam
{

class SlamNode : public rclcpp::Node
{
public:
  SlamNode();
  ~SlamNode();

  // Perform initialization that requires shared_from_this()
  void initialize();

private:

  double noise_x_, noise_y_, noise_theta_;
  double meas_range_noise_, meas_bearing_noise_;
  double assoc_thresh_;
  int scan_downsample_;
  int map_width_, map_height_;
  double resolution_;
  
  // LaserScan 수신 콜백
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // odom 수신 콜백 (predict용)
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Occupancy map publish 함수
  void publishMap();

  // EKF SLAM 알고리즘 객체
  std::shared_ptr<ekf_slam::EkfSlamSystem> ekf_;

  // LaserScan → Observation 변환기
  std::shared_ptr<laser::LaserProcessor> laser_processor_;

  // Occupancy Mapper 추가
  std::shared_ptr<OccupancyMapper> occupancy_mapper_;

  // ROS2 Subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // ROS2 Publisher for map
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

  // Trajectory visualizer
  std::shared_ptr<TrajectoryVisualizer> trajectory_visualizer_;

  // 주기적 맵 출력을 위한 타이머
  rclcpp::TimerBase::SharedPtr map_timer_;

  // 이전 시간 저장 (dt 계산용)
  rclcpp::Time last_cmd_time_;

  // TF 처리
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace ekf_slam

#endif  // EKF_SLAM_NODE_HPP_
