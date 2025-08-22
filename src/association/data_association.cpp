#include "association/data_association.hpp"
#include "association/icp.hpp"
#include "preprocessing/laser_processor.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <limits>

namespace ekf_slam {

DataAssociation::DataAssociation(double mahalanobis_thresh, double ratio_thresh)
    : threshold_(mahalanobis_thresh), ratio_thresh_(ratio_thresh) {}

int DataAssociation::associate(
    const laser::Observation &obs, const Eigen::VectorXd &mu,
    const Eigen::MatrixXd & /*sigma*/,
    const std::unordered_map<int, int> &landmark_index_map,
    const Eigen::Matrix2d & /*Q*/) {
  // Transform observation to world coordinates
  double rx = mu(0);
  double ry = mu(1);
  double theta = mu(2);
  Eigen::Vector2d obs_local(obs.range * std::cos(obs.bearing),
                            obs.range * std::sin(obs.bearing));
  Eigen::Rotation2Dd rot(theta);
  Eigen::Vector2d obs_world = rot * obs_local + Eigen::Vector2d(rx, ry);

  // Prepare target landmarks
  std::vector<Eigen::Vector2d> landmarks;
  std::vector<int> ids;
  for (const auto &[landmark_id, idx] : landmark_index_map) {
    landmarks.emplace_back(mu(idx), mu(idx + 1));
    ids.push_back(landmark_id);
  }

  // Run ICP between observation point and map landmarks
  std::vector<Eigen::Vector2d> src{obs_world};
  ICPResult icp_res = runICP(src, landmarks, 10, 1e-6);

  if (icp_res.correspondences.empty() || icp_res.correspondences[0] < 0) {
    return -1;
  }

  double min_dist = icp_res.errors[0];
  int matched_idx = icp_res.correspondences[0];

  double second_min = std::numeric_limits<double>::infinity();
  for (std::size_t j = 0; j < landmarks.size(); ++j) {
    if (static_cast<int>(j) == matched_idx)
      continue;
    double d = (obs_world - landmarks[j]).squaredNorm();
    if (d < second_min) {
      second_min = d;
    }
  }

  double ratio = min_dist / second_min;
  if (second_min == std::numeric_limits<double>::infinity()) {
    ratio = 0.0; // only one landmark candidate
  }

  if (min_dist < threshold_ && ratio < ratio_thresh_) {
    return ids[matched_idx];
  }
  return -1;
}

} // namespace ekf_slam
