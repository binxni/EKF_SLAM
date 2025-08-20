#include "association/icp.hpp"
#include <limits>

namespace ekf_slam {

ICPResult runICP(const std::vector<Eigen::Vector2d> &src,
                 const std::vector<Eigen::Vector2d> &dst, int max_iterations,
                 double tolerance) {
  ICPResult result;
  result.R.setIdentity();
  result.t.setZero();
  result.correspondences.assign(src.size(), -1);
  result.errors.assign(src.size(), std::numeric_limits<double>::infinity());

  if (src.empty() || dst.empty()) {
    return result;
  }

  Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
  Eigen::Vector2d t = Eigen::Vector2d::Zero();
  double prev_error = std::numeric_limits<double>::max();

  for (int iter = 0; iter < max_iterations; ++iter) {
    // Find correspondences
    double total_error = 0.0;
    std::vector<Eigen::Vector2d> src_matched(src.size());
    std::vector<Eigen::Vector2d> dst_matched(src.size());

    for (std::size_t i = 0; i < src.size(); ++i) {
      Eigen::Vector2d p = R * src[i] + t;
      double best = std::numeric_limits<double>::infinity();
      int best_idx = -1;
      for (std::size_t j = 0; j < dst.size(); ++j) {
        double dist = (p - dst[j]).squaredNorm();
        if (dist < best) {
          best = dist;
          best_idx = static_cast<int>(j);
        }
      }
      src_matched[i] = src[i];
      dst_matched[i] = dst[best_idx];
      result.correspondences[i] = best_idx;
      result.errors[i] = best;
      total_error += best;
    }

    double mean_error = total_error / src.size();
    if (std::abs(prev_error - mean_error) < tolerance) {
      break;
    }
    prev_error = mean_error;

    // Compute centroids
    Eigen::Vector2d centroid_src = Eigen::Vector2d::Zero();
    Eigen::Vector2d centroid_dst = Eigen::Vector2d::Zero();
    for (std::size_t i = 0; i < src_matched.size(); ++i) {
      centroid_src += src_matched[i];
      centroid_dst += dst_matched[i];
    }
    centroid_src /= static_cast<double>(src_matched.size());
    centroid_dst /= static_cast<double>(dst_matched.size());

    // Compute covariance matrix
    Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
    for (std::size_t i = 0; i < src_matched.size(); ++i) {
      W += (src_matched[i] - centroid_src) *
           (dst_matched[i] - centroid_dst).transpose();
    }

    // SVD for rotation
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU |
                                                 Eigen::ComputeFullV);
    Eigen::Matrix2d R_iter = svd.matrixV() * svd.matrixU().transpose();
    Eigen::Vector2d t_iter = centroid_dst - R_iter * centroid_src;

    R = R_iter * R;
    t = R_iter * t + t_iter;
  }

  result.R = R;
  result.t = t;
  return result;
}

} // namespace ekf_slam
