#include "core/slam_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <cmath>
#include <iomanip>
#include <memory>
#include <tf2/utils.h>

#include "utils/geometry_utils.hpp"

namespace ekf_slam {

SlamNode::SlamNode() : Node("ekf_slam_node") {
  // Log 파일 초기화
  log_file_.open("log/ekf_log.csv", std::ios::out | std::ios::app);
  if (log_file_.tellp() == 0) {
    log_file_ << "timestamp,stage,pose_x,pose_y,pose_theta,";
    log_file_
        << "cov_00,cov_01,cov_02,cov_10,cov_11,cov_12,cov_20,cov_21,cov_22\n";
  }
  log_file_ << std::fixed << std::setprecision(6);

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
  this->declare_parameter("scan_downsample.step", 10);
  this->declare_parameter("map.width", 1500);
  this->declare_parameter("map.height", 1500);
  this->declare_parameter("map.resolution", 0.05);

  // 파라미터 불러오기 → 바로 멤버 변수에 저장
  this->get_parameter("control_noise.x", noise_x_);
  this->get_parameter("control_noise.y", noise_y_);
  this->get_parameter("control_noise.theta", noise_theta_);
  this->get_parameter("measurement_noise.range", meas_range_noise_);
  this->get_parameter("measurement_noise.bearing", meas_bearing_noise_);
  this->get_parameter("data_association.threshold", assoc_thresh_);
  this->get_parameter("scan_downsample.step", scan_downsample_);
  this->get_parameter("map.width", map_width_);
  this->get_parameter("map.height", map_height_);
  this->get_parameter("map.resolution", resolution_);

  // EKF SLAM 시스템 생성시 멤버 변수 사용
  ekf_ = std::make_shared<EkfSlamSystem>(noise_x_, noise_y_, noise_theta_,
                                         meas_range_noise_, meas_bearing_noise_,
                                         assoc_thresh_);

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

  // Initialize previous command time (use message timestamps)
  last_cmd_time_ = rclcpp::Time(0);

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
}

SlamNode::~SlamNode() {
  if (occupancy_mapper_) {
    occupancy_mapper_->stopMapping();
  }
  if (log_file_.is_open()) {
    log_file_.close();
  }
}

void SlamNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double v = msg->twist.twist.linear.x; // 선속도
  double yaw = tf2::getYaw(msg->pose.pose.orientation);

  rclcpp::Time current_time = msg->header.stamp;

  if (!initial_pose_received_) {
    ekf_->setPose(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
    initial_pose_received_ = true;
    last_cmd_time_ = current_time;
    last_yaw_ = yaw;
    return;
  }

  double dt = (current_time - last_cmd_time_).seconds();
  last_cmd_time_ = current_time;

  if (dt <= 0.0 || dt > 1.0)
    return; // 너무 큰 간격은 무시

  // 우선적으로 odom에서 제공하는 각속도 사용
  double w = msg->twist.twist.angular.z;
  if (std::isnan(w) || std::fabs(w) < 1e-6) {
    w = ekf_slam::utils::normalizeAngle(yaw - last_yaw_) / dt;
  }
  last_yaw_ = yaw;

  // 저속 구간에서는 노이즈를 줄이기 위해 각속도 제한
  if (std::fabs(v) < 0.01 && std::fabs(w) < 0.01) {
    w = 0.0;
  }

  // Predict 이전 상태 기록
  auto pre_pose = ekf_->getCurrentPose();
  auto pre_cov = ekf_->getCovariance().block(0, 0, 3, 3);
  log_file_ << current_time.seconds() << ",predict_before," << pre_pose(0)
            << "," << pre_pose(1) << "," << pre_pose(2) << "," << pre_cov(0, 0)
            << "," << pre_cov(0, 1) << "," << pre_cov(0, 2) << ","
            << pre_cov(1, 0) << "," << pre_cov(1, 1) << "," << pre_cov(1, 2)
            << "," << pre_cov(2, 0) << "," << pre_cov(2, 1) << ","
            << pre_cov(2, 2) << "\n";

  ekf_->predict(v, w, dt); // EKF 예측 수행

  // Predict 이후 상태 기록
  auto post_pose = ekf_->getCurrentPose();
  auto post_cov = ekf_->getCovariance().block(0, 0, 3, 3);
  log_file_ << current_time.seconds() << ",predict_after," << post_pose(0)
            << "," << post_pose(1) << "," << post_pose(2) << ","
            << post_cov(0, 0) << "," << post_cov(0, 1) << "," << post_cov(0, 2)
            << "," << post_cov(1, 0) << "," << post_cov(1, 1) << ","
            << post_cov(1, 2) << "," << post_cov(2, 0) << "," << post_cov(2, 1)
            << "," << post_cov(2, 2) << "\n";
  log_file_.flush();
}

void SlamNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // LaserScan → 관측값 변환
  auto observations = laser_processor_->process(*msg);

  double time_sec = msg->header.stamp.seconds();

  // Update 이전 상태 기록
  auto pre_pose = ekf_->getCurrentPose();
  auto pre_cov = ekf_->getCovariance().block(0, 0, 3, 3);
  log_file_ << time_sec << ",update_before," << pre_pose(0) << ","
            << pre_pose(1) << "," << pre_pose(2) << "," << pre_cov(0, 0) << ","
            << pre_cov(0, 1) << "," << pre_cov(0, 2) << "," << pre_cov(1, 0)
            << "," << pre_cov(1, 1) << "," << pre_cov(1, 2) << ","
            << pre_cov(2, 0) << "," << pre_cov(2, 1) << "," << pre_cov(2, 2)
            << "\n";

  // EKF SLAM 업데이트
  double timestamp = msg->header.stamp.sec +
                     msg->header.stamp.nanosec * 1e-9;
  ekf_->update(observations, timestamp);

  // Update 이후 상태 기록 및 현재 추정 출력
  auto pose = ekf_->getCurrentPose();
  auto post_cov = ekf_->getCovariance().block(0, 0, 3, 3);
  log_file_ << time_sec << ",update_after," << pose(0) << "," << pose(1) << ","
            << pose(2) << "," << post_cov(0, 0) << "," << post_cov(0, 1) << ","
            << post_cov(0, 2) << "," << post_cov(1, 0) << "," << post_cov(1, 1)
            << "," << post_cov(1, 2) << "," << post_cov(2, 0) << ","
            << post_cov(2, 1) << "," << post_cov(2, 2) << "\n";
  log_file_.flush();
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
