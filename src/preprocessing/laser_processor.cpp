#include "preprocessing/laser_processor.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include <vector>
#include <algorithm>

namespace ekf_slam::laser
{

LaserProcessor::LaserProcessor(
  rclcpp::Node::SharedPtr node,
  tf2_ros::Buffer * tf_buffer,
  const std::string & base_frame,
  std::size_t downsample_step)
: node_(node), tf_buffer_(tf_buffer), base_frame_(base_frame),
  downsample_step_(downsample_step ? downsample_step : 1)
{
}

std::vector<Observation> LaserProcessor::process(
  const sensor_msgs::msg::LaserScan & scan)
{
  std::vector<Observation> observations;
  sensor_msgs::msg::LaserScan scan_copy = scan;

  if (isInverted(scan.header.frame_id, scan.header.stamp)) {
    invertScan(scan_copy);
  }

  try {
    auto tf = tf_buffer_->lookupTransform(base_frame_, scan.header.frame_id, scan.header.stamp, rclcpp::Duration::from_seconds(0.1));
    double theta = tf2::getYaw(tf.transform.rotation);

    double angle = scan_copy.angle_min;
    std::size_t step = std::max<std::size_t>(1, downsample_step_);
    for (std::size_t i = 0; i < scan_copy.ranges.size(); i += step) {
      double r = scan_copy.ranges[i];
      if (r >= scan_copy.range_min && r <= scan_copy.range_max) {
        double bearing = normalizeAngle(angle + theta);
        observations.push_back({r, bearing, -1});
      }
      angle += scan_copy.angle_increment * step;
    }

  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", e.what());
  }

  return observations;
}

bool LaserProcessor::isInverted(const std::string & laser_frame, const rclcpp::Time & stamp)
{
  try {
    geometry_msgs::msg::Vector3Stamped z_vector;
    z_vector.vector.x = 0.0;
    z_vector.vector.y = 0.0;
    z_vector.vector.z = 1.0;
    z_vector.header.stamp = stamp;
    z_vector.header.frame_id = base_frame_;

    auto z_in_laser = tf_buffer_->transform(z_vector, laser_frame);
    return z_in_laser.vector.z <= 0.0;
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(node_->get_logger(), "TF transform for inversion check failed: %s", e.what());
    return false;
  }
}

void LaserProcessor::invertScan(sensor_msgs::msg::LaserScan & scan)
{
  std::reverse(scan.ranges.begin(), scan.ranges.end());
  std::reverse(scan.intensities.begin(), scan.intensities.end());
  scan.angle_min = -scan.angle_max;
  scan.angle_max = -scan.angle_min;
  scan.angle_increment = -scan.angle_increment;
}

double LaserProcessor::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

}  // namespace ekf_slam::laser
