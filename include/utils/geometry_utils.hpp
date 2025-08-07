#ifndef EKF_SLAM_UTILS_GEOMETRY_UTILS_HPP_
#define EKF_SLAM_UTILS_GEOMETRY_UTILS_HPP_

#include <Eigen/Dense>
#include <cmath>

namespace ekf_slam {
namespace utils {

// Polar → Cartesian 변환
inline Eigen::Vector2d polarToCartesian(double range, double bearing) {
  return Eigen::Vector2d(range * std::cos(bearing), range * std::sin(bearing));
}

// Cartesian → Polar 변환 [range, bearing]
inline Eigen::Vector2d cartesianToPolar(const Eigen::Vector2d &point) {
  double range = point.norm();
  double bearing = std::atan2(point.y(), point.x());
  return Eigen::Vector2d(range, bearing);
}

// World 좌표 → Robot 좌표계 변환
inline Eigen::Vector2d worldToRobot(const Eigen::Vector2d &point,
                                    const Eigen::Vector3d &robot_pose) {
  double dx = point.x() - robot_pose.x();
  double dy = point.y() - robot_pose.y();
  double theta = robot_pose.z();

  double x_r = std::cos(theta) * dx + std::sin(theta) * dy;
  double y_r = -std::sin(theta) * dx + std::cos(theta) * dy;

  return Eigen::Vector2d(x_r, y_r);
}

// Robot 좌표계 → World 좌표 변환
inline Eigen::Vector2d robotToWorld(const Eigen::Vector2d &local_point,
                                    const Eigen::Vector3d &robot_pose) {
  double x = robot_pose.x() + std::cos(robot_pose.z()) * local_point.x() -
             std::sin(robot_pose.z()) * local_point.y();
  double y = robot_pose.y() + std::sin(robot_pose.z()) * local_point.x() +
             std::cos(robot_pose.z()) * local_point.y();
  return Eigen::Vector2d(x, y);
}

// 각도 정규화 함수 [-pi, pi]
inline double normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

} // namespace utils
} // namespace ekf_slam

#endif // EKF_SLAM_UTILS_GEOMETRY_UTILS_HPP_
