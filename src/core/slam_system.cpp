#include "core/slam_system.hpp"
#include "association/data_association.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/geometry_utils.hpp"
#include "utils/jacobian_utils.hpp"
#include <Eigen/SparseCholesky>
#include <cmath>
#include <sstream>

namespace ekf_slam {
EkfSlamSystem::EkfSlamSystem(double noise_x, double noise_y, double noise_theta,
                             double meas_range_noise, double meas_bearing_noise,
                             double assoc_thresh)
    : noise_x_(noise_x), noise_y_(noise_y), noise_theta_(noise_theta),
      meas_range_noise_(meas_range_noise),
      meas_bearing_noise_(meas_bearing_noise), data_associator_(assoc_thresh),
      next_landmark_id_(0) {
  mu_ = Eigen::VectorXd::Zero(3);
  sigma_ = Eigen::MatrixXd::Identity(3, 3) * 1e-3;
  info_matrix_ = sigma_.inverse().sparseView();
  info_vector_ = info_matrix_ * mu_;
}

void EkfSlamSystem::setPose(double x, double y, double theta) {
  mu_(0) = x;
  mu_(1) = y;
  mu_(2) = utils::normalizeAngle(theta);
  info_vector_ = info_matrix_ * mu_;
}

// -----------------------------
// 1. Predict
// -----------------------------
void EkfSlamSystem::predict(double v, double w, double dt) {
  double theta = mu_(2);

  // Integrate the bicycle model exactly for better turning behaviour
  if (std::fabs(w) > 1e-6) {
    mu_(0) += (v / w) * (std::sin(theta + w * dt) - std::sin(theta));
    mu_(1) += (v / w) * (-std::cos(theta + w * dt) + std::cos(theta));
  } else {
    mu_(0) += v * std::cos(theta) * dt;
    mu_(1) += v * std::sin(theta) * dt;
  }
  mu_(2) = utils::normalizeAngle(theta + w * dt);

  // 자코비안 계산 (Gx)
  Eigen::Matrix3d Gx = ekf_slam::utils::computeMotionJacobian(v, theta, w, dt);

  // 제어 노이즈
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  R(0, 0) = noise_x_;     // x
  R(1, 1) = noise_y_;     // y
  R(2, 2) = noise_theta_; // theta

  // 공분산 예측
  Eigen::MatrixXd Fx = Eigen::MatrixXd::Zero(3, mu_.size());
  Fx.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();

  Eigen::MatrixXd G_bar = Eigen::MatrixXd::Identity(mu_.size(), mu_.size());
  G_bar.block<3, 3>(0, 0) = Gx;

  sigma_ = G_bar * sigma_ * G_bar.transpose() + Fx.transpose() * R * Fx;

  info_matrix_ = sigma_.inverse().sparseView();
  info_vector_ = info_matrix_ * mu_;
}

// -----------------------------
// 2. Update
// -----------------------------
void EkfSlamSystem::update(
    const std::vector<ekf_slam::laser::Observation> &observations) {
  auto logger = rclcpp::get_logger("EkfSlamSystem");
  for (const auto &obs : observations) {
    Eigen::Matrix2d Q = getMeasurementNoiseMatrix();
    int id =
        data_associator_.associate(obs, mu_, sigma_, landmark_index_map_, Q);
    if (id == -1) {
      RCLCPP_DEBUG(logger, "No association found. Adding new landmark id %d",
                   next_landmark_id_);
      id = next_landmark_id_++;
      addLandmark(obs, id);
      continue;
    }

    int idx = landmark_index_map_[id];
    double lx = mu_(idx);
    double ly = mu_(idx + 1);
    double dx = lx - mu_(0);
    double dy = ly - mu_(1);
    double q = dx * dx + dy * dy;

    double z_hat_range = std::sqrt(q);
    double z_hat_bearing = std::atan2(dy, dx) - mu_(2);
    double z_hat_bearing_norm = utils::normalizeAngle(z_hat_bearing);

    double measured_bearing = utils::normalizeAngle(obs.bearing);
    Eigen::Vector2d z_hat(z_hat_range, z_hat_bearing_norm);
    Eigen::Vector2d z(obs.range, measured_bearing);
    Eigen::Vector2d innovation = z - z_hat;
    innovation(1) = utils::normalizeAngle(innovation(1));

    Eigen::MatrixXd H = ekf_slam::utils::computeObservationJacobian(mu_, idx);
    Eigen::Matrix2d S = H * sigma_ * H.transpose() + Q;
    double mahalanobis = innovation.transpose() * S.inverse() * innovation;
    RCLCPP_DEBUG(logger,
                 "Association result -> id: %d, Mahalanobis distance: %.3f", id,
                 mahalanobis);

    double range_thresh = 3.0 * std::sqrt(Q(0, 0));
    double bearing_thresh = 3.0 * std::sqrt(Q(1, 1));
    if (std::abs(innovation(0)) > range_thresh ||
        std::abs(innovation(1)) > bearing_thresh) {
      RCLCPP_WARN(logger,
                  "Innovation exceeds threshold: range %.3f, bearing %.3f",
                  innovation(0), innovation(1));
    }
    RCLCPP_DEBUG(logger, "Innovation: [%.3f, %.3f]", innovation(0),
                 innovation(1));

    Eigen::Matrix2d Q_inv = Q.inverse();
    Eigen::MatrixXd Ht_Qinv = H.transpose() * Q_inv;

    info_matrix_ += (Ht_Qinv * H).sparseView();
    info_vector_ += Ht_Qinv * (innovation + H * mu_);

    Eigen::MatrixXd info_dense(info_matrix_);
    std::stringstream ss_info;
    ss_info << info_dense;
    RCLCPP_DEBUG(logger, "info_matrix_:\n%s", ss_info.str().c_str());

    std::stringstream ss_mu;
    ss_mu << mu_.transpose();
    RCLCPP_DEBUG(logger, "mu_: %s", ss_mu.str().c_str());
  }

  sparsifyInformationMatrix(1e-6);
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(info_matrix_);
  mu_ = solver.solve(info_vector_);
  mu_(2) = utils::normalizeAngle(mu_(2));
  sigma_ = solver.solve(
      Eigen::MatrixXd::Identity(info_matrix_.rows(), info_matrix_.cols()));
  info_vector_ = info_matrix_ * mu_;
}

// -----------------------------
// 3. Landmark 추가
// -----------------------------
void EkfSlamSystem::addLandmark(const laser::Observation &obs,
                                int landmark_id) {
  extendState(landmark_id, obs);
}

// -----------------------------
// 4. 상태 확장 (landmark 추가 시)
// -----------------------------
void EkfSlamSystem::extendState(int landmark_id,
                                const laser::Observation &obs) {
  double x = mu_(0);
  double y = mu_(1);
  double theta = mu_(2);

  double lx = x + obs.range * std::cos(theta + obs.bearing);
  double ly = y + obs.range * std::sin(theta + obs.bearing);

  int old_size = mu_.size();

  // 하이퍼파라미터 값 사용
  double range_noise_var = meas_range_noise_ * meas_range_noise_;
  double bearing_noise_var = meas_bearing_noise_ * meas_bearing_noise_;

  // 공분산을 먼저 확장
  expandCovarianceWithLandmark(old_size, obs.range, obs.bearing, theta,
                               range_noise_var, bearing_noise_var);

  // 상태 벡터 크기를 확장
  mu_.conservativeResize(old_size + 2);
  mu_(old_size) = lx;
  mu_(old_size + 1) = ly;

  landmark_index_map_[landmark_id] = old_size;

  info_matrix_ = sigma_.inverse().sparseView();
  info_vector_ = info_matrix_ * mu_;
}
// -----------------------------
// 5. 기타 유틸
// -----------------------------
bool EkfSlamSystem::hasLandmark(int landmark_id) const {
  return landmark_index_map_.count(landmark_id) > 0;
}

Eigen::Vector3d EkfSlamSystem::getCurrentPose() const {
  Eigen::Vector3d pose = mu_.head<3>();
  pose(2) = utils::normalizeAngle(pose(2));
  return pose;
}

void EkfSlamSystem::expandCovarianceWithLandmark(int old_size, double range,
                                                 double bearing, double theta,
                                                 double range_noise_var,
                                                 double bearing_noise_var) {
  int new_size = old_size + 2;

  // 기존 로봇 상태 공분산 추출 (x, y, theta assumed at 0~2)
  Eigen::Matrix3d Sigma_xx = sigma_.block(0, 0, 3, 3);

  double theta_phi = theta + bearing;
  double sin_tp = std::sin(theta_phi);
  double cos_tp = std::cos(theta_phi);

  // Gx: 로봇 상태에서 랜드마크 위치로 가는 Jacobian
  Eigen::Matrix<double, 2, 3> Gx;
  Gx << 1, 0, -range * sin_tp, 0, 1, range * cos_tp;

  // Gz: 관측에서 랜드마크 위치로 가는 Jacobian
  Eigen::Matrix2d Gz;
  Gz << cos_tp, -range * sin_tp, sin_tp, range * cos_tp;

  // 관측 노이즈 공분산 R
  Eigen::Matrix2d R;
  R << range_noise_var, 0, 0, bearing_noise_var;

  // 계산
  Eigen::Matrix2d Sigma_r = Gz * R * Gz.transpose();
  Eigen::Matrix<double, 2, 3> Sigma_xr = Gx * Sigma_xx;

  // 공분산 행렬 확장
  Eigen::MatrixXd new_sigma = Eigen::MatrixXd::Zero(new_size, new_size);
  new_sigma.block(0, 0, old_size, old_size) = sigma_;
  new_sigma.block(old_size, 0, 2, 3) = Sigma_xr;
  new_sigma.block(0, old_size, 3, 2) = Sigma_xr.transpose();
  new_sigma.block(old_size, old_size, 2, 2) = Sigma_r;

  // 나머지 0 채움 (혹시 모르니 안전하게)
  if (old_size > 3) {
    new_sigma.block(old_size, 3, 2, old_size - 3).setZero();
    new_sigma.block(3, old_size, old_size - 3, 2).setZero();
  }

  sigma_ = new_sigma;
}

Eigen::Matrix2d EkfSlamSystem::getMeasurementNoiseMatrix() const {
  Eigen::Matrix2d Q = Eigen::Matrix2d::Zero();
  Q(0, 0) = meas_range_noise_ * meas_range_noise_;
  Q(1, 1) = meas_bearing_noise_ * meas_bearing_noise_;
  return Q;
}

void EkfSlamSystem::sparsifyInformationMatrix(double threshold) {
  info_matrix_.prune(threshold);
}

} // namespace ekf_slam
