#ifndef EKF_SLAM_UTILS_JACOBIAN_UTILS_HPP_
#define EKF_SLAM_UTILS_JACOBIAN_UTILS_HPP_

#include <Eigen/Dense>
#include <cmath>
namespace ekf_slam::utils {

/**
 * @brief Motion model Jacobian with respect to the robot pose.
 *
 * Uses a bicycle/unicycle model where angular velocity `w` is applied over
 * the time step `dt`. When `w` is close to zero, the Jacobian reduces to the
 * standard straight-line form.
 */
inline Eigen::Matrix3d computeMotionJacobian(double v, double theta, double w,
                                             double dt) {
  Eigen::Matrix3d Gx = Eigen::Matrix3d::Identity();
  if (std::fabs(w) < 1e-6) {
    Gx(0, 2) = -v * std::sin(theta) * dt;
    Gx(1, 2) = v * std::cos(theta) * dt;
  } else {
    Gx(0, 2) = v / w * (std::cos(theta + w * dt) - std::cos(theta));
    Gx(1, 2) = v / w * (std::sin(theta + w * dt) - std::sin(theta));
  }
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
