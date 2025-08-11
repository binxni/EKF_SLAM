#ifndef EKF_SLAM_SYSTEM_HPP_
#define EKF_SLAM_SYSTEM_HPP_

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unordered_map>
#include <vector>

#include "association/data_association.hpp"
#include "preprocessing/laser_processor.hpp"

namespace ekf_slam {

class EkfSlamSystem {
public:
  EkfSlamSystem(double noise_x, double noise_y, double noise_theta,
                double meas_range_noise, double meas_bearing_noise,
                double data_association_thresh, double data_association_ratio);

  // 예측: 제어입력 (선속도, 요속도), 시간 간격
  void predict(double v, double yaw_rate, double dt);

  // 업데이트: 관측값 (range-bearing)
  void update(const std::vector<laser::Observation> &observations);

  // 랜드마크 추가
  void addLandmark(const laser::Observation &obs, int landmark_id);

  // 현재 로봇 위치 (x, y, theta)
  Eigen::Vector3d getCurrentPose() const;

  // 랜드마크가 이미 있는지 확인
  bool hasLandmark(int landmark_id) const;

private:
  // 상태 벡터: [x, y, theta, l1_x, l1_y, l2_x, l2_y, ...]
  Eigen::VectorXd mu_;

  // 공분산 행렬
  Eigen::MatrixXd sigma_;

  // Sparse Extended Information Filter representation
  Eigen::SparseMatrix<double> info_matrix_;
  Eigen::VectorXd info_vector_;

  double noise_x_, noise_y_, noise_theta_; // control noise
  double meas_range_noise_, meas_bearing_noise_;

  // 랜드마크 ID → mu_ 인덱스 매핑
  std::unordered_map<int, int> landmark_index_map_;

  ekf_slam::DataAssociation data_associator_;
  int next_landmark_id_;

  // 상태 확장 함수
  void extendState(int landmark_id, const laser::Observation &obs);

  // 공분산 확장 함수
  void expandCovarianceWithLandmark(int old_size, double range, double bearing,
                                    double theta, double range_noise_var,
                                    double bearing_noise_var);

  Eigen::Matrix2d getMeasurementNoiseMatrix() const;

  void sparsifyInformationMatrix(double threshold);
};

} // namespace ekf_slam

#endif // EKF_SLAM_SYSTEM_HPP_
