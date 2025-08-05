#include "core/slam_system.hpp"
#include "utils/jacobian_utils.hpp"
#include "data_association.hpp"
#include <cmath>

namespace ekf_slam 
{
EkfSlamSystem::EkfSlamSystem(double wheel_base, double noise_x, double noise_y,
  double noise_theta, double meas_range_noise, double meas_bearing_noise, double assoc_thresh)
  : wheel_base_(wheel_base), noise_x_(noise_x), noise_y_(noise_y),
    noise_theta_(noise_theta),
    meas_range_noise_(meas_range_noise * meas_range_noise),
    meas_bearing_noise_(meas_bearing_noise * meas_bearing_noise),
    data_associator_(assoc_thresh)
{
  mu_ = Eigen::VectorXd::Zero(3);
  sigma_ = Eigen::MatrixXd::Identity(3, 3) * 1e-3;
}

// -----------------------------
// 1. Predict
// -----------------------------
void EkfSlamSystem::predict(double v, double delta, double dt)
{
  double theta = mu_(2);  // 현재 heading

  // 상태 변화량 계산
  double dx = v * std::cos(theta) * dt;
  double dy = v * std::sin(theta) * dt;
  double dtheta = (v / wheel_base_) * std::tan(delta) * dt;

  mu_(0) += dx;
  mu_(1) += dy;
  mu_(2) += dtheta;

  // 자코비안 계산 (Gx)
  Eigen::Matrix3d Gx = ekf_slam::utils::computeMotionJacobian(v, theta, dt);

  // 제어 노이즈 
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  R(0,0) = noise_x_; //x
  R(1,1) = noise_y_; //y
  R(2,2) = noise_theta_; //theta

  // 공분산 예측
  Eigen::MatrixXd Fx = Eigen::MatrixXd::Zero(3, mu_.size());
  Fx.block(0,0,3,3) = Eigen::Matrix3d::Identity();

  sigma_ = sigma_ + Fx.transpose() * Gx.transpose() * R * Gx * Fx;
}

// -----------------------------
// 2. Update
// -----------------------------
void EkfSlamSystem::update(std::vector<laser::Observation>& observations)
{
  Eigen::Matrix2d Q = Eigen::Matrix2d::Zero();
  Q(0,0) = meas_range_noise_;
  Q(1,1) = meas_bearing_noise_;

  for (auto& obs : observations)
  {
    // 데이터 연관
    int id = data_associator_.associate(obs, mu_, sigma_, landmark_index_map_, Q);
    if (id == -1) {
      id = next_landmark_id_++;
      obs.landmark_id = id;
      addLandmark(obs, id);
      continue;
    } else {
      obs.landmark_id = id;
    }

    int idx = landmark_index_map_[id];
    double lx = mu_(idx);
    double ly = mu_(idx + 1);

    double dx = lx - mu_(0);
    double dy = ly - mu_(1);
    double q = dx*dx + dy*dy;

    double z_hat_range = std::sqrt(q);
    double z_hat_bearing = std::atan2(dy, dx) - mu_(2);
    double z_hat_bearing_norm = std::atan2(std::sin(z_hat_bearing), std::cos(z_hat_bearing));

    // 측정 예측 벡터
    Eigen::Vector2d z_hat(z_hat_range, z_hat_bearing_norm);
    Eigen::Vector2d z(obs.range, obs.bearing);
    Eigen::Vector2d innovation = z - z_hat;
    innovation(1) = std::atan2(std::sin(innovation(1)), std::cos(innovation(1)));  // normalize

    // 자코비안 H
    Eigen::MatrixXd H = ekf_slam::utils::computeObservationJacobian(mu_, idx);

    // 칼만 게인
    Eigen::MatrixXd S = H * sigma_ * H.transpose() + Q;
    Eigen::MatrixXd K = sigma_ * H.transpose() * S.inverse();

    // 상태, 공분산 업데이트
    mu_ = mu_ + K * innovation;
    mu_(2) = std::atan2(std::sin(mu_(2)), std::cos(mu_(2)));
    sigma_ = (Eigen::MatrixXd::Identity(mu_.size(), mu_.size()) - K * H) * sigma_;
  }
}

// -----------------------------
// 3. Landmark 추가
// -----------------------------
void EkfSlamSystem::addLandmark(const laser::Observation& obs, int landmark_id)
{
  extendState(landmark_id, obs);
}

// -----------------------------
// 4. 상태 확장 (landmark 추가 시)
// -----------------------------
void EkfSlamSystem::extendState(int landmark_id, const laser::Observation& obs)
{
  double x = mu_(0);
  double y = mu_(1);
  double theta = mu_(2);

  double lx = x + obs.range * std::cos(theta + obs.bearing);
  double ly = y + obs.range * std::sin(theta + obs.bearing);

  int old_size = mu_.size();
  mu_.conservativeResize(old_size + 2);
  mu_(old_size) = lx;
  mu_(old_size + 1) = ly;

  // 하이퍼파라미터 값 사용
  double range_noise_var = meas_range_noise_;
  double bearing_noise_var = meas_bearing_noise_;
  
  expandCovarianceWithLandmark(
      obs.range, obs.bearing, theta,
      range_noise_var, bearing_noise_var);

  landmark_index_map_[landmark_id] = old_size;
}
// -----------------------------
// 5. 기타 유틸
// -----------------------------
bool EkfSlamSystem::hasLandmark(int landmark_id) const
{
  return landmark_index_map_.count(landmark_id) > 0;
}

Eigen::Vector3d EkfSlamSystem::getCurrentPose() const
{
  return mu_.head<3>();
}

void EkfSlamSystem::expandCovarianceWithLandmark(
    double range, double bearing, double theta,
    double range_noise_var, double bearing_noise_var)
{
  int old_size = mu_.size();
  int new_size = old_size + 2;

  // 기존 로봇 상태 공분산 추출 (x, y, theta assumed at 0~2)
  Eigen::Matrix3d Sigma_xx = sigma_.block(0, 0, 3, 3);

  double theta_phi = theta + bearing;
  double sin_tp = std::sin(theta_phi);
  double cos_tp = std::cos(theta_phi);

  // Gx: 로봇 상태에서 랜드마크 위치로 가는 Jacobian
  Eigen::Matrix<double, 2, 3> Gx;
  Gx << 1, 0, -range * sin_tp,
        0, 1,  range * cos_tp;

  // Gz: 관측에서 랜드마크 위치로 가는 Jacobian
  Eigen::Matrix2d Gz;
  Gz << cos_tp, -range * sin_tp,
        sin_tp,  range * cos_tp;

  // 관측 노이즈 공분산 R
  Eigen::Matrix2d R;
  R << range_noise_var, 0,
       0, bearing_noise_var;

  // 계산
  Eigen::Matrix2d Sigma_r = Gz * R * Gz.transpose();
  Eigen::Matrix<double, 2, 3> Sigma_xr = Gx * Sigma_xx;

  // 공분산 행렬 확장
  sigma_.conservativeResize(new_size, new_size);
  sigma_.block(old_size, 0, 2, 3) = Sigma_xr;
  sigma_.block(0, old_size, 3, 2) = Sigma_xr.transpose();
  sigma_.block(old_size, old_size, 2, 2) = Sigma_r;

  // 나머지 0 채움 (혹시 모르니 안전하게)
  if (old_size > 3) {
    sigma_.block(old_size, 3, 2, old_size - 3).setZero();
    sigma_.block(3, old_size, old_size - 3, 2).setZero();
  }
}

}  // namespace ekf_slam
