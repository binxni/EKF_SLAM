#ifndef EKF_SLAM_UTILS_MATH_UTILS_HPP_
#define EKF_SLAM_UTILS_MATH_UTILS_HPP_

#include <algorithm>
#include <cmath>

namespace ekf_slam {
namespace utils {

// 각도를 -π ~ π 범위로 정규화
inline double wrapToPi(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// 특정 범위로 값 제한
inline double clamp(double value, double min_val, double max_val) {
  return std::max(min_val, std::min(value, max_val));
}

// 두 점 사이의 유클리디안 거리 계산
inline double distance(double x1, double y1, double x2, double y2) {
  return std::hypot(x2 - x1, y2 - y1);
}

// 제곱 함수
inline double square(double x) {
  return x * x;
}

}  // namespace utils
}  // namespace ekf_slam

#endif  // EKF_SLAM_UTILS_MATH_UTILS_HPP_
