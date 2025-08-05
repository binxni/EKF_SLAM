#include "association/data_association.hpp"
#include "preprocessing/laser_processor.hpp"
#include <cmath>
#include <limits>

namespace ekf_slam {

DataAssociation::DataAssociation(double mahalanobis_thresh)
: threshold_(mahalanobis_thresh) {}

int DataAssociation::associate(
    const laser::Observation& obs,
    const Eigen::VectorXd& mu,
    const Eigen::MatrixXd& sigma,
    const std::unordered_map<int, int>& landmark_index_map,
    const Eigen::Matrix2d& Q)
{
    double min_dist = threshold_;
    int matched_id = -1;

    for (const auto& [landmark_id, idx] : landmark_index_map) {
        double lx = mu(idx);
        double ly = mu(idx + 1);
        double rx = mu(0);
        double ry = mu(1);
        double theta = mu(2);

        double dx = lx - rx;
        double dy = ly - ry;
        double q = dx * dx + dy * dy;
        double sqrt_q = std::sqrt(q);

        Eigen::Vector2d z_pred(sqrt_q, std::atan2(dy, dx) - theta);
        Eigen::Vector2d z_obs(obs.range, obs.bearing);
        Eigen::Vector2d innovation = z_obs - z_pred;

        Eigen::Matrix<double, 2, 5> H;
        H << -sqrt_q * dx / q, -sqrt_q * dy / q, 0,  sqrt_q * dx / q,  sqrt_q * dy / q,
              dy / q,         -dx / q,         -1, -dy / q,           dx / q;

        Eigen::MatrixXd Sigma_x = Eigen::MatrixXd::Zero(5, 5);
        Sigma_x.block<3,3>(0,0) = sigma.block<3,3>(0,0);
        Sigma_x.block<3,2>(0,3) = sigma.block(0, idx, 3, 2);
        Sigma_x.block<2,3>(3,0) = sigma.block(idx, 0, 2, 3);
        Sigma_x.block<2,2>(3,3) = sigma.block(idx, idx, 2, 2);

        Eigen::Matrix2d S = H * Sigma_x * H.transpose() + Q;

        double dist = innovation.transpose() * S.inverse() * innovation;
        if (dist < min_dist) {
            min_dist = dist;
            matched_id = landmark_id;
        }
    }

    return matched_id;
}

}  // namespace ekf_slam
