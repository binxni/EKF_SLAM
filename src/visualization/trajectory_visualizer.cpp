#include "visualization/trajectory_visualizer.hpp"

#include "geometry_msgs/msg/point.hpp"

namespace ekf_slam {

TrajectoryVisualizer::TrajectoryVisualizer(rclcpp::Node::SharedPtr node)
{
  marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("trajectory", 10);

  path_points_marker_.header.frame_id = "map";
  path_points_marker_.ns = "trajectory";
  path_points_marker_.id = 0;
  path_points_marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  path_points_marker_.action = visualization_msgs::msg::Marker::ADD;
  path_points_marker_.pose.orientation.w = 1.0;
  path_points_marker_.scale.x = 0.1;
  path_points_marker_.scale.y = 0.1;
  path_points_marker_.scale.z = 0.1;
  path_points_marker_.color.r = 0.0;
  path_points_marker_.color.g = 1.0;
  path_points_marker_.color.b = 0.0;
  path_points_marker_.color.a = 1.0;

  path_line_marker_.header.frame_id = "map";
  path_line_marker_.ns = "trajectory";
  path_line_marker_.id = 1;
  path_line_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_line_marker_.action = visualization_msgs::msg::Marker::ADD;
  path_line_marker_.pose.orientation.w = 1.0;
  path_line_marker_.scale.x = 0.05;
  path_line_marker_.color.r = 1.0;
  path_line_marker_.color.g = 0.0;
  path_line_marker_.color.b = 0.0;
  path_line_marker_.color.a = 1.0;
}

void TrajectoryVisualizer::addPose(double x, double y, const rclcpp::Time& stamp)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = 0.0;
  path_points_marker_.points.push_back(p);
  path_line_marker_.points.push_back(p);

  path_points_marker_.header.stamp = stamp;
  path_line_marker_.header.stamp = stamp;

  marker_pub_->publish(path_points_marker_);
  marker_pub_->publish(path_line_marker_);
}

}  // namespace ekf_slam
