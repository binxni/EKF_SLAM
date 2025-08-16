#include <fstream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

class DataLoggerNode : public rclcpp::Node
{
public:
  DataLoggerNode()
  : Node("data_logger_node"), counter_(0)
  {
    horizon_sec_ = declare_parameter<double>("horizon_sec", 1.6);
    dt_ = declare_parameter<double>("dt", 0.1);
    forward_only_ = declare_parameter<bool>("forward_only", false);
    normalize_ = declare_parameter<bool>("normalize", false);
    downsample_ = declare_parameter<int>("downsample", 1);
    std::string file = declare_parameter<std::string>("output_csv_file", "data_log.csv");

    file_.open(file, std::ios::out | std::ios::app);
    if (!file_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s", file.c_str());
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    scan_sub_.subscribe(this, "/scan");
    grid_sub_.subscribe(this, "/polar_grid");
    path_sub_.subscribe(this, "/planned_path_with_velocity");

    sync_ = std::make_shared<Synchronizer>(Synchronizer(SyncPolicy(10), scan_sub_, grid_sub_, path_sub_));
    sync_->registerCallback(&DataLoggerNode::dataCallback, this);
  }

private:
  using PathMsg = trajectory_msgs::msg::MultiDOFJointTrajectory;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::LaserScan,
    sensor_msgs::msg::LaserScan,
    PathMsg>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void dataCallback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr grid,
    const PathMsg::ConstSharedPtr path)
  {
    if (counter_++ % std::max(1, downsample_) != 0) {
      return;
    }

    if (scan->ranges.size() != 1080 || grid->ranges.size() != 1080) {
      RCLCPP_WARN(get_logger(), "Scan or grid size mismatch");
      return;
    }

    std::vector<double> scan_vals(1080);
    for (size_t i = 0; i < 1080; ++i) {
      double r = scan->ranges[i];
      if (!std::isfinite(r) || r > scan->range_max || r < scan->range_min) {
        r = scan->range_max;
      }
      if (normalize_) {
        r /= scan->range_max;
      }
      scan_vals[i] = r;
    }

    std::vector<double> grid_vals(1080);
    for (size_t i = 0; i < 1080; ++i) {
      double g = grid->ranges[i];
      if (!std::isfinite(g)) {
        g = 0.0;
      }
      grid_vals[i] = g;
    }

    std::vector<double> times;
    std::vector<double> xs;
    std::vector<double> ys;

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("base_link", "map", scan->header.stamp, rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "TF lookup failed: %s", e.what());
      return;
    }
    tf2::Transform tf_map_base;
    tf2::fromMsg(tf.transform, tf_map_base);

    for (const auto & pt : path->points) {
      if (pt.transforms.empty()) {
        continue;
      }
      const auto & tr = pt.transforms[0].translation;
      tf2::Vector3 p_map(tr.x, tr.y, 0.0);
      tf2::Vector3 p_base = tf_map_base * p_map;
      double t = rclcpp::Time(pt.time_from_start).seconds();
      times.push_back(t);
      xs.push_back(p_base.x());
      ys.push_back(p_base.y());
    }

    if (times.empty()) {
      RCLCPP_WARN(get_logger(), "Empty path points");
      return;
    }

    int label_count = static_cast<int>(horizon_sec_ / dt_);
    std::vector<double> label_x(label_count, 0.0);
    std::vector<double> label_y(label_count, 0.0);
    double last_x = 0.0, last_y = 0.0;

    for (int i = 0; i < label_count; ++i) {
      double target_t = i * dt_;
      double x = xs.back();
      double y = ys.back();

      if (target_t <= times.front()) {
        x = xs.front();
        y = ys.front();
      } else if (target_t >= times.back()) {
        x = xs.back();
        y = ys.back();
      } else {
        auto upper = std::upper_bound(times.begin(), times.end(), target_t);
        size_t idx = upper - times.begin();
        double t1 = times[idx - 1];
        double t2 = times[idx];
        double ratio = (target_t - t1) / (t2 - t1);
        x = xs[idx - 1] + ratio * (xs[idx] - xs[idx - 1]);
        y = ys[idx - 1] + ratio * (ys[idx] - ys[idx - 1]);
      }

      if (forward_only_ && x <= 0.0) {
        x = last_x;
        y = last_y;
      } else {
        last_x = x;
        last_y = y;
      }

      if (normalize_) {
        x /= scan->range_max;
        y /= scan->range_max;
      }
      label_x[i] = x;
      label_y[i] = y;
    }

    if (file_.is_open()) {
      file_ << std::fixed << std::setprecision(6)
            << rclcpp::Time(scan->header.stamp).seconds();
      for (double v : scan_vals) {
        file_ << "," << v;
      }
      for (double v : grid_vals) {
        file_ << "," << v;
      }
      for (double v : label_x) {
        file_ << "," << v;
      }
      for (double v : label_y) {
        file_ << "," << v;
      }
      file_ << "\n";
    }
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> grid_sub_;
  message_filters::Subscriber<PathMsg> path_sub_;
  std::shared_ptr<Synchronizer> sync_;

  std::ofstream file_;
  double horizon_sec_;
  double dt_;
  bool forward_only_;
  bool normalize_;
  int downsample_;
  size_t counter_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DataLoggerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

