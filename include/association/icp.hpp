#ifndef EKF_SLAM_ICP_HPP
#define EKF_SLAM_ICP_HPP

#include <Eigen/Dense>
#include <vector>

namespace ekf_slam {

struct ICPResult {
  Eigen::Matrix2d R; // rotation
  Eigen::Vector2d t; // translation
  std::vector<int>
      correspondences;        // index of matched target for each source point
  std::vector<double> errors; // squared distances for each source point
};

// Simple 2D point-to-point ICP implementation.
// Aligns source points to target points and returns the final transform and
// correspondences.
ICPResult runICP(const std::vector<Eigen::Vector2d> &src,
                 const std::vector<Eigen::Vector2d> &dst,
                 int max_iterations = 20, double tolerance = 1e-6);

} // namespace ekf_slam

#endif // EKF_SLAM_ICP_HPP
