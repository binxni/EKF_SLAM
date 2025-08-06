#ifndef TRAJECTORY_VISUALIZER_HPP_
#define TRAJECTORY_VISUALIZER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace ekf_slam {

class TrajectoryVisualizer
{
public:
  explicit TrajectoryVisualizer(rclcpp::Node::SharedPtr node);

  void addPose(double x, double y, const rclcpp::Time& stamp);

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  visualization_msgs::msg::Marker path_points_marker_;
  visualization_msgs::msg::Marker path_line_marker_;
};

}  // namespace ekf_slam

#endif  // TRAJECTORY_VISUALIZER_HPP_
