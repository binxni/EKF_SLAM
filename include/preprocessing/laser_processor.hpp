#ifndef EKF_SLAM_LASER_PROCESSOR_HPP_
#define EKF_SLAM_LASER_PROCESSOR_HPP_

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"

namespace ekf_slam::laser
{

struct Observation
{
  double range;
  double bearing;
  int landmark_id;  // -1이면 새 landmark
};

// Laser 전처리 유틸 클래스
class LaserProcessor
{
public:
  LaserProcessor( //생성자, 노드와 버퍼 그리고 기준 좌표 프레임을 받아 저장한다.
    rclcpp::Node::SharedPtr node,
    tf2_ros::Buffer * tf_buffer,
    const std::string & base_frame);

  std::vector<Observation> process( //scan 데이터를 obervation(range, bearing) list로 변환한다.
    const sensor_msgs::msg::LaserScan & scan);

private:
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer * tf_buffer_;
  std::string base_frame_;

  bool isInverted(const std::string & laser_frame, const rclcpp::Time & stamp);
  void invertScan(sensor_msgs::msg::LaserScan & scan);

};

}  // namespace ekf_slam::laser

#endif  // EKF_SLAM_LASER_PROCESSOR_HPP_
