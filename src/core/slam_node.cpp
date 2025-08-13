#include "core/slam_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include <cmath>
#include <memory>

namespace ekf_slam {

SlamNode::SlamNode() : Node("ekf_slam_node") {
  // TF2 초기화
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // EKF SLAM 시스템 초기화
  // 파라미터 선언
  this->declare_parameter("control_noise.x", 0.01);
  this->declare_parameter("control_noise.y", 0.01);
  this->declare_parameter("control_noise.theta", 0.01);
  this->declare_parameter("measurement_noise.range", 0.05);
  this->declare_parameter("measurement_noise.bearing", 0.05);
  this->declare_parameter("data_association.threshold", 5.99);
  this->declare_parameter("data_association.ratio", 0.8);
  this->declare_parameter("scan_downsample.step", 10);
  this->declare_parameter("map.width", 1500);
  this->declare_parameter("map.height", 1500);
  this->declare_parameter("map.resolution", 0.05);
  this->declare_parameter("map.save_prefix", "map");

  // 파라미터 불러오기 → 바로 멤버 변수에 저장
  this->get_parameter("control_noise.x", noise_x_);
  this->get_parameter("control_noise.y", noise_y_);
  this->get_parameter("control_noise.theta", noise_theta_);
  this->get_parameter("measurement_noise.range", meas_range_noise_);
  this->get_parameter("measurement_noise.bearing", meas_bearing_noise_);
  this->get_parameter("data_association.threshold", assoc_thresh_);
  this->get_parameter("data_association.ratio", assoc_ratio_);
  this->get_parameter("scan_downsample.step", scan_downsample_);
  this->get_parameter("map.width", map_width_);
  this->get_parameter("map.height", map_height_);
  this->get_parameter("map.resolution", resolution_);
  this->get_parameter("map.save_prefix", map_save_prefix_);

  // EKF SLAM 시스템 생성시 멤버 변수 사용
  ekf_ = std::make_shared<EkfSlamSystem>(noise_x_, noise_y_, noise_theta_,
                                         meas_range_noise_, meas_bearing_noise_,
                                         assoc_thresh_, assoc_ratio_);

  RCLCPP_INFO(this->get_logger(), "EKF SLAM Node Initialized.");
}

void SlamNode::initialize() {
  // LaserScan 전처리기 초기화
  laser_processor_ = std::make_shared<laser::LaserProcessor>(
      shared_from_this(), tf_buffer_.get(), "base_link",
      static_cast<std::size_t>(scan_downsample_));

  RCLCPP_INFO(this->get_logger(),
              "Laser Processor Initialized with downsample step: %d",
              scan_downsample_);

  // Occupancy Mapper 초기화 추가
  occupancy_mapper_ =
      std::make_shared<OccupancyMapper>(shared_from_this(), // 현재 노드 전달
                                        map_width_, map_height_, resolution_);
  RCLCPP_INFO(this->get_logger(),
              "Occupancy Mapper Initialized with size: %dx%d, resolution: %.2f",
              map_width_, map_height_, resolution_);

  // Start occupancy mapping
  occupancy_mapper_->startMapping();
  RCLCPP_INFO(this->get_logger(), "Occupancy Mapping started.");

  // Initialize previous command time using the node's clock type so that
  // future time calculations are performed with a consistent time source.
  // This prevents errors such as "can't subtract times with different
  // time sources" when subtracting ROS time from system time.
  // Disambiguate constructor by explicitly specifying seconds and
  // nanoseconds to ensure the clock type is preserved.
  last_cmd_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

  // odom subscription (EKF predict input)
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&SlamNode::odomCallback, this, std::placeholders::_1));

  // LaserScan subscription (EKF update input)
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&SlamNode::scanCallback, this, std::placeholders::_1));

  // Map publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

  // Trajectory visualizer
  trajectory_visualizer_ =
      std::make_shared<TrajectoryVisualizer>(shared_from_this());

  // Periodic map publish timer (1 second interval)
  map_timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                       std::bind(&SlamNode::publishMap, this));

  // Service to save map
  save_map_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "save_map",
      std::bind(&SlamNode::saveMapServiceCallback, this, std::placeholders::_1,
                std::placeholders::_2));
}

SlamNode::~SlamNode() {
  if (occupancy_mapper_) {
    occupancy_mapper_->stopMapping();
  }
}

void SlamNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Odometry provides orientation from which yaw is extracted.
  double v = msg->twist.twist.linear.x; // 선속도
  double yaw = tf2::getYaw(msg->pose.pose.orientation); // 요각(rad)

  // Use the same clock type for both the incoming message timestamp and the
  // stored previous timestamp to avoid mixing ROS and system time sources.
  rclcpp::Time current_time(msg->header.stamp,
                            this->get_clock()->get_clock_type());
  double dt = (current_time - last_cmd_time_).seconds();
  last_cmd_time_ = current_time;

  if (dt <= 0.0 || dt > 1.0)
    return; // 너무 큰 간격은 무시

  ekf_->predict(v, yaw, dt); // EKF 예측 수행
}

void SlamNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // LaserScan → 관측값 변환
  auto observations = laser_processor_->process(*msg);

  // EKF SLAM 업데이트
  ekf_->update(observations);

  // 현재 추정된 로봇 pose 출력
  auto pose = ekf_->getCurrentPose();
  RCLCPP_INFO(this->get_logger(), "Pose: x=%.2f, y=%.2f, θ=%.2f", pose(0),
              pose(1), pose(2));

  // occupancy mapping을 위한 데이터 전송
  occupancy_mapper_->addScanData(pose, *msg);

  // Update trajectory visualization
  if (trajectory_visualizer_) {
    trajectory_visualizer_->addPose(pose(0), pose(1), this->now());
  }
}

void SlamNode::publishMap() {
  if (occupancy_mapper_ && occupancy_mapper_->isInitialized()) {
    auto occupancy_grid = occupancy_mapper_->getOccupancyGrid();
    map_pub_->publish(occupancy_grid);

    // 디버그 정보
    static int map_count = 0;
    if (++map_count % 10 == 0) {
      RCLCPP_INFO(this->get_logger(), "Published occupancy map #%d", map_count);
    }
  }
}

void SlamNode::saveMapServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  if (occupancy_mapper_ && occupancy_mapper_->isInitialized()) {
    bool ok = occupancy_mapper_->saveMap(map_save_prefix_);
    res->success = ok;
    res->message = ok ? "Map saved" : "Failed to save map";
  } else {
    res->success = false;
    res->message = "Mapper not initialized";
  }
}

} // namespace ekf_slam

// main
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ekf_slam::SlamNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
