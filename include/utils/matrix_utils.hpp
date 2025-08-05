#ifndef EKF_SLAM_UTILS_MATRIX_UTILS_HPP_
#define EKF_SLAM_UTILS_MATRIX_UTILS_HPP_

#include <Eigen/Dense>
#include <cmath>

namespace ekf_slam {
namespace utils {

// 2x2 회전 행렬 생성
inline Eigen::Matrix2d make2x2RotationMatrix(double theta) {
  Eigen::Matrix2d R;
  R << std::cos(theta), -std::sin(theta),
       std::sin(theta),  std::cos(theta);
  return R;
}

// 3x3 상태 예측 자코비안 (예: [x, y, θ]) ← robot motion model용
inline Eigen::Matrix3d make3x3JacobianMotion(double theta, double v, double w, double dt) {
  Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
  G(0, 2) = -v * dt * std::sin(theta);
  G(1, 2) =  v * dt * std::cos(theta);
  return G;
}

// 2x3 관측 자코비안 (로봇 상태 [x, y, θ]에 대한 ∂h/∂x) - 측정은 (range, bearing)
inline Eigen::Matrix<double, 2, 3> make2x3JacobianObservation(
  const Eigen::Vector2d& delta, double q)
{
  double dx = delta.x();
  double dy = delta.y();

  Eigen::Matrix<double, 2, 3> H;
  H << -dx / std::sqrt(q), -dy / std::sqrt(q), 0,
        dy / q,           -dx / q,           -1;
  return H;
}

// Skew 대칭 행렬 (2D 회전에 사용될 수 있음)
inline Eigen::Matrix2d makeSkewSymmetric(double scalar) {
  Eigen::Matrix2d S;
  S << 0, -scalar,
       scalar, 0;
  return S;
}

}  // namespace utils
}  // namespace ekf_slam

#endif  // EKF_SLAM_UTILS_MATRIX_UTILS_HPP_
