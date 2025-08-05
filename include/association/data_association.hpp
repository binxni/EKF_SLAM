#ifndef EKF_SLAM_DATA_ASSOCIATION_HPP
#define EKF_SLAM_DATA_ASSOCIATION_HPP

#include <Eigen/Dense>
#include <unordered_map>

namespace ekf_slam {

// Forward declaration for laser observation type
namespace laser {
struct Observation;
}  // namespace laser

class DataAssociation {
public:
    DataAssociation(double mahalanobis_thresh);

    // 관측값에 대해 연관된 landmark_id 반환 (-1이면 신규)
    int associate(
        const laser::Observation& obs,
        const Eigen::VectorXd& mu,
        const Eigen::MatrixXd& sigma,
        const std::unordered_map<int, int>& landmark_index_map,
        const Eigen::Matrix2d& Q); // 측정 노이즈 공분산

private:
    double threshold_;  // 마할라노비스 거리 임계값
};

}  // namespace ekf_slam

#endif  // EKF_SLAM_DATA_ASSOCIATION_HPP
