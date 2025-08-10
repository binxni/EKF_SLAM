#include "association/data_association.hpp"
#include "preprocessing/laser_processor.hpp"
#include "utils/geometry_utils.hpp"
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>

namespace ekf_slam {

DataAssociation::DataAssociation(double mahalanobis_thresh)
    : threshold_(mahalanobis_thresh) {}

int DataAssociation::associate(
    const laser::Observation &obs, const Eigen::VectorXd &mu,
    const Eigen::MatrixXd &sigma,
    const std::unordered_map<int, int> &landmark_index_map,
    const Eigen::Matrix2d &Q, Eigen::Vector2d &innovation_out,
    double &mahalanobis_out) {
  double min_dist = std::numeric_limits<double>::infinity();

  int matched_id = -1;
  Eigen::Vector2d best_innovation = Eigen::Vector2d::Zero();
  double best_dist = std::numeric_limits<double>::infinity();

  for (const auto &[landmark_id, idx] : landmark_index_map) {
    double lx = mu(idx);
    double ly = mu(idx + 1);
    double rx = mu(0);
    double ry = mu(1);
    double theta = mu(2);

    double dx = lx - rx;
    double dy = ly - ry;
    double q = dx * dx + dy * dy;
    double sqrt_q = std::sqrt(q);

    double pred_bearing = utils::normalizeAngle(std::atan2(dy, dx) - theta);
    double meas_bearing = utils::normalizeAngle(obs.bearing);

    Eigen::Vector2d z_pred(sqrt_q, pred_bearing);
    Eigen::Vector2d z_obs(obs.range, meas_bearing);
    Eigen::Vector2d innovation = z_obs - z_pred;
    innovation(1) = utils::normalizeAngle(innovation(1));

    Eigen::Matrix<double, 2, 5> H;
    H << -sqrt_q * dx / q, -sqrt_q * dy / q, 0, sqrt_q * dx / q,
        sqrt_q * dy / q, dy / q, -dx / q, -1, -dy / q, dx / q;

    Eigen::MatrixXd Sigma_x = Eigen::MatrixXd::Zero(5, 5);
    Sigma_x.block<3, 3>(0, 0) = sigma.block<3, 3>(0, 0);
    Sigma_x.block<3, 2>(0, 3) = sigma.block(0, idx, 3, 2);
    Sigma_x.block<2, 3>(3, 0) = sigma.block(idx, 0, 2, 3);
    Sigma_x.block<2, 2>(3, 3) = sigma.block(idx, idx, 2, 2);

    Eigen::Matrix2d S = H * Sigma_x * H.transpose() + Q;

    double dist = innovation.transpose() * S.inverse() * innovation;
    RCLCPP_INFO(rclcpp::get_logger("DataAssociation"),
                "Landmark %d: dist=%.4f, innovation=[%.4f, %.4f]",
                landmark_id, dist, innovation(0), innovation(1));
    if (dist < min_dist) {
      min_dist = dist;
      matched_id = landmark_id;
      best_innovation = innovation;
      best_dist = dist;
    }
  }


  innovation_out = best_innovation;
  mahalanobis_out = best_dist;

  if (min_dist < threshold_) {
    RCLCPP_INFO(rclcpp::get_logger("DataAssociation"),
                "Matched landmark %d with distance %.4f",
                matched_id, min_dist);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("DataAssociation"),
                "No landmark within gate. Min distance %.4f exceeds threshold %.4f",
                min_dist, threshold_);
    matched_id = -1;
  }


  return matched_id;
}

} // namespace ekf_slam
