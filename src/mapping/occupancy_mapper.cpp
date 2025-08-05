// src/occupancy_mapper.cpp
#include "mapping/occupancy_mapper.hpp"
#include <cmath>
#include <algorithm>

namespace ekf_slam {

OccupancyMapper::OccupancyMapper(rclcpp::Node::SharedPtr node,
                                int map_width, int map_height, double resolution)
    : node_(node),
      map_width_(map_width),
      map_height_(map_height),
      resolution_(resolution),
      max_range_(10.0),
      running_(false),
      initialized_(false) {
    
    // 맵 원점 설정 (맵 중앙)
    origin_x_ = map_width_ * resolution_ / 2.0;
    origin_y_ = map_height_ * resolution_ / 2.0;
    
    // Log-odds 파라미터 초기화
    setLogOddsParameters();
    
    // 맵 초기화
    log_odds_map_.resize(map_height_, std::vector<double>(map_width_, 0.0));
    
    initialized_ = true;
    
    RCLCPP_INFO(node_->get_logger(), 
                "OccupancyMapper initialized: %dx%d, resolution=%.2f", 
                map_width_, map_height_, resolution_);
}

OccupancyMapper::~OccupancyMapper() {
    stopMapping();
}

void OccupancyMapper::setLogOddsParameters(double occ_prob, double free_prob) {
    log_odds_occ_ = std::log(occ_prob / (1.0 - occ_prob));
    log_odds_free_ = std::log(free_prob / (1.0 - free_prob));
    log_odds_max_ = std::log(0.95 / 0.05);
    log_odds_min_ = std::log(0.05 / 0.95);
}

void OccupancyMapper::startMapping() {
    if (!running_.load()) {
        running_ = true;
        mapping_thread_ = std::thread(&OccupancyMapper::mappingThreadFunction, this);
        RCLCPP_INFO(node_->get_logger(), "Occupancy mapping thread started");
    }
}

void OccupancyMapper::stopMapping() {
    if (running_.load()) {
        running_ = false;
        queue_cv_.notify_all();
        if (mapping_thread_.joinable()) {
            mapping_thread_.join();
        }
        RCLCPP_INFO(node_->get_logger(), "Occupancy mapping thread stopped");
    }
}

void OccupancyMapper::addScanData(const Eigen::Vector3d& robot_pose,
                                  const sensor_msgs::msg::LaserScan& scan) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    
    // 큐 크기 제한 (메모리 보호)
    while (data_queue_.size() > 50) {
        data_queue_.pop();
    }
    
    OccupancyScanData data;
    data.robot_pose = robot_pose;
    data.scan = scan;
    data.timestamp = rclcpp::Clock().now();
    
    data_queue_.push(data);
    queue_cv_.notify_one();
}

void OccupancyMapper::mappingThreadFunction() {
    while (running_.load()) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        
        // 데이터가 있을 때까지 대기 (100ms 타임아웃)
        if (queue_cv_.wait_for(lock, std::chrono::milliseconds(100), 
                              [this] { return !data_queue_.empty() || !running_.load(); })) {
            
            if (!running_.load()) break;
            
            if (!data_queue_.empty()) {
                OccupancyScanData data = data_queue_.front();
                data_queue_.pop();
                lock.unlock();
                
                // 스캔 데이터 처리
                processScanData(data);
            }
        }
    }
}

void OccupancyMapper::processScanData(const OccupancyScanData& data) {
    const auto& robot_pose = data.robot_pose;
    const auto& scan = data.scan;
    
    double robot_x = robot_pose(0);
    double robot_y = robot_pose(1);
    double robot_theta = robot_pose(2);
    
    // 각 레이저 포인트 처리
    double angle = scan.angle_min;
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        double range = scan.ranges[i];
        
        // 유효한 거리인지 확인
        if (range < scan.range_min || range > scan.range_max || 
            range > max_range_ || std::isnan(range) || std::isinf(range)) {
            angle += scan.angle_increment;
            continue;
        }
        
        // 절대 각도 계산 (로봇 좌표계 기준)
        double absolute_angle = robot_theta + angle;
        
        // 끝점 계산
        double end_x = robot_x + range * std::cos(absolute_angle);
        double end_y = robot_y + range * std::sin(absolute_angle);
        
        // 레이 업데이트
        updateRay(robot_x, robot_y, end_x, end_y, true);
        
        angle += scan.angle_increment;
    }
}

void OccupancyMapper::updateRay(double start_x, double start_y, 
                                double end_x, double end_y, bool is_hit) {
    auto start_grid = worldToGrid(start_x, start_y);
    auto end_grid = worldToGrid(end_x, end_y);
    
    // Bresenham 알고리즘으로 직선상의 모든 점 가져오기
    auto ray_points = bresenhamLine(start_grid.first, start_grid.second,
                                   end_grid.first, end_grid.second);
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    // 경로상의 모든 점을 free로 설정 (끝점 제외)
    for (size_t i = 0; i < ray_points.size() - 1; ++i) {
        updateLogOdds(ray_points[i].first, ray_points[i].second, log_odds_free_);
    }
    
    // 끝점을 occupied로 설정 (hit인 경우)
    if (is_hit && !ray_points.empty()) {
        auto end_point = ray_points.back();
        updateLogOdds(end_point.first, end_point.second, log_odds_occ_);
    }
}

std::pair<int, int> OccupancyMapper::worldToGrid(double x, double y) const {
    int grid_x = static_cast<int>((x + origin_x_) / resolution_);
    int grid_y = static_cast<int>((y + origin_y_) / resolution_);
    return {grid_x, grid_y};
}

bool OccupancyMapper::isValidCell(int x, int y) const {
    return x >= 0 && x < map_width_ && y >= 0 && y < map_height_;
}

std::vector<std::pair<int, int>> OccupancyMapper::bresenhamLine(int x0, int y0, int x1, int y1) const {
    std::vector<std::pair<int, int>> points;
    
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    int x = x0, y = y0;
    
    while (true) {
        if (isValidCell(x, y)) {
            points.emplace_back(x, y);
        }
        
        if (x == x1 && y == y1) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
    
    return points;
}

void OccupancyMapper::updateLogOdds(int x, int y, double update_value) {
    if (isValidCell(x, y)) {
        log_odds_map_[y][x] += update_value;
        log_odds_map_[y][x] = std::clamp(log_odds_map_[y][x], log_odds_min_, log_odds_max_);
    }
}

int8_t OccupancyMapper::logOddsToOccupancy(double log_odds) const {
    if (std::abs(log_odds) < 1e-6) {
        return -1; // Unknown
    }
    
    double prob = 1.0 - 1.0 / (1.0 + std::exp(log_odds));
    
    if (prob > 0.65) {
        return 100; // Occupied
    } else if (prob < 0.35) {
        return 0;   // Free
    } else {
        return -1;  // Unknown
    }
}

nav_msgs::msg::OccupancyGrid OccupancyMapper::getOccupancyGrid() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    nav_msgs::msg::OccupancyGrid grid;
    
    // 헤더 설정
    grid.header.stamp = rclcpp::Clock().now();
    grid.header.frame_id = "map";
    
    // 맵 정보 설정
    grid.info.resolution = resolution_;
    grid.info.width = map_width_;
    grid.info.height = map_height_;
    grid.info.origin.position.x = -origin_x_;
    grid.info.origin.position.y = -origin_y_;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    
    // 데이터 변환
    grid.data.resize(map_width_ * map_height_);
    for (int y = 0; y < map_height_; ++y) {
        for (int x = 0; x < map_width_; ++x) {
            int index = y * map_width_ + x;
            grid.data[index] = logOddsToOccupancy(log_odds_map_[y][x]);
        }
    }
    
    return grid;
}

} // namespace ekf_slam
