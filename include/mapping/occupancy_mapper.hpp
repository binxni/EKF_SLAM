#ifndef OCCUPANCY_MAPPER_HPP_
#define OCCUPANCY_MAPPER_HPP_

#include <vector>
#include <mutex>
#include <thread>
#include <queue>
#include <atomic>
#include <condition_variable>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <Eigen/Dense>

namespace ekf_slam {

struct OccupancyScanData { //thread간 데이터 전달을 위한 패키징
    Eigen::Vector3d robot_pose;  // [x, y, theta]
    sensor_msgs::msg::LaserScan scan;
    rclcpp::Time timestamp;
};

class OccupancyMapper { //파라미터 처리
public:
    OccupancyMapper(rclcpp::Node::SharedPtr node,
                   int map_width = 2000, 
                   int map_height = 2000, 
                   double resolution = 0.05);
    ~OccupancyMapper();

    // 메인 인터페이스
    void startMapping();
    void stopMapping();
    void addScanData(const Eigen::Vector3d& robot_pose, 
                     const sensor_msgs::msg::LaserScan& scan);
    
    // 맵 데이터 접근
    nav_msgs::msg::OccupancyGrid getOccupancyGrid() const;
    bool saveMap(const std::string& file_prefix) const;
    bool isInitialized() const { return initialized_; }
    
    // 설정
    void setLogOddsParameters(double occ_prob = 0.7, double free_prob = 0.3,
                              double min_weight = 0.1, double max_weight = 1.0);
    void setMaxRange(double max_range) { max_range_ = max_range; }

private:
    rclcpp::Node::SharedPtr node_;
    
    // 맵 파라미터
    int map_width_, map_height_;
    double resolution_;
    double origin_x_, origin_y_;  // 맵 중심점 (미터)
    double max_range_;
    
    // Log-odds 파라미터
    double log_odds_occ_, log_odds_free_;
    double log_odds_max_, log_odds_min_;
    double min_weight_, max_weight_;
    
    // 맵 데이터
    std::vector<std::vector<double>> log_odds_map_;
    mutable std::mutex map_mutex_;
    
    // 스레딩
    std::thread mapping_thread_;
    std::atomic<bool> running_;
    std::queue<OccupancyScanData> data_queue_;
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    
    // 상태
    std::atomic<bool> initialized_;
    
    // 내부 함수들
    void mappingThreadFunction(); //mapping thread 실행되는 main loop
    void processScanData(const OccupancyScanData& data); // data 처리 
    void updateRay(double start_x, double start_y, double end_x, double end_y, bool is_hit); // lay casting
    std::pair<int, int> worldToGrid(double x, double y) const;
    bool isValidCell(int x, int y) const;
    std::vector<std::pair<int, int>> bresenhamLine(int x0, int y0, int x1, int y1) const;
    void updateLogOdds(int x, int y, double update_value);
    int8_t logOddsToOccupancy(double log_odds) const;
};

} // namespace ekf_slam

#endif // OCCUPANCY_MAPPER_HPP_
