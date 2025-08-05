#ifndef EKF_SLAM_UTILS_JACOBIAN_UTILS_HPP_
#define EKF_SLAM_UTILS_JACOBIAN_UTILS_HPP_

#include <Eigen/Dense>
#include <cmath>
namespace ekf_slam::utils
{

Eigen::Matrix3d computeMotionJacobian(double v, double theta, double dt)
{
  Eigen::Matrix3d Gx = Eigen::Matrix3d::Identity();
  Gx(0, 2) = -v * std::sin(theta) * dt;
  Gx(1, 2) =  v * std::cos(theta) * dt;
  return Gx;
}

Eigen::MatrixXd computeObservationJacobian(const Eigen::VectorXd& mu, int idx)
{
  double dx = mu(idx)     - mu(0);
  double dy = mu(idx + 1) - mu(1);
  double q = dx * dx + dy * dy;
  double sqrt_q = std::sqrt(q);

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, mu.size());

  H(0,0) = -dx / sqrt_q;
  H(0,1) = -dy / sqrt_q;
  H(1,0) =  dy / q;
  H(1,1) = -dx / q;
  H(1,2) = -1.0;

  H(0,idx)     =  dx / sqrt_q;
  H(0,idx + 1) =  dy / sqrt_q;
  H(1,idx)     = -dy / q;
  H(1,idx + 1) =  dx / q;

  return H;
}


}  // namespace ekf_slam::utils

#endif
