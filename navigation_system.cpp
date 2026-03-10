// navigation_system.cpp 
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <memory>
#include <optional>
#include <limits>
#include <mutex>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <fstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::chrono_literals;


struct PerformanceMetrics {
    // Planning Performance
    int total_planning_calls = 0;
    int successful_paths = 0;
    int failed_paths = 0;
    double total_planning_time_ms = 0.0;
    double min_planning_time_ms = std::numeric_limits<double>::max();
    double max_planning_time_ms = 0.0;
    
    // Path Quality
    double total_path_length_m = 0.0;
    double total_path_smoothness = 0.0;
    int total_waypoints = 0;
    
    // Obstacle Detection
    int total_obstacles_detected = 0;
    int total_replanning_events = 0;
    int obstacles_reported_to_osrm = 0;
    
    // Stop Sign Performance
    int stop_signs_detected = 0;
    int stop_signs_obeyed = 0;
    double total_stop_duration_s = 0.0;
    
    // Communication (Jetson ↔ Raspberry Pi)
    int destinations_received_from_ui = 0;
    int paths_sent_to_ui = 0;
    int status_updates_sent = 0;
    int osrm_updates_sent = 0;
    
    // System Performance
    std::chrono::system_clock::time_point start_time;
    double total_runtime_s = 0.0;
    
    // Computational Efficiency
    int a_star_iterations_total = 0;
    int dstar_iterations_total = 0;
    
    void saveToCSV(const std::string& filename) {
        std::ofstream file(filename);
        file << "Metric,Value,Unit\n";
        file << "Total Planning Calls," << total_planning_calls << ",count\n";
        file << "Successful Paths," << successful_paths << ",count\n";
        file << "Failed Paths," << failed_paths << ",count\n";
        file << "Success Rate," << (total_planning_calls > 0 ? (100.0 * successful_paths / total_planning_calls) : 0) << ",%\n";
        file << "Avg Planning Time," << (total_planning_calls > 0 ? total_planning_time_ms / total_planning_calls : 0) << ",ms\n";
        file << "Min Planning Time," << (min_planning_time_ms == std::numeric_limits<double>::max() ? 0 : min_planning_time_ms) << ",ms\n";
        file << "Max Planning Time," << max_planning_time_ms << ",ms\n";
        file << "Avg Path Length," << (successful_paths > 0 ? total_path_length_m / successful_paths : 0) << ",m\n";
        file << "Avg Waypoints," << (successful_paths > 0 ? (double)total_waypoints / successful_paths : 0) << ",count\n";
        file << "Total Obstacles," << total_obstacles_detected << ",count\n";
        file << "Replanning Events," << total_replanning_events << ",count\n";
        file << "OSRM Updates," << osrm_updates_sent << ",count\n";
        file << "Stop Signs Detected," << stop_signs_detected << ",count\n";
        file << "Stop Signs Obeyed," << stop_signs_obeyed << ",count\n";
        file << "Avg Stop Duration," << (stop_signs_obeyed > 0 ? total_stop_duration_s / stop_signs_obeyed : 0) << ",s\n";
        file << "Destinations from UI," << destinations_received_from_ui << ",count\n";
        file << "Paths to UI," << paths_sent_to_ui << ",count\n";
        file << "Status Updates," << status_updates_sent << ",count\n";
        file << "Total Runtime," << total_runtime_s << ",s\n";
        file << "Planning Hz," << (total_runtime_s > 0 ? total_planning_calls / total_runtime_s : 0) << ",Hz\n";
        file.close();
    }
};

// ==============================================================================
// STRUCTURES
// ==============================================================================

struct GridCell {
    int x, y;
    bool operator==(const GridCell& o) const { return x == o.x && y == o.y; }
    bool operator!=(const GridCell& o) const { return !(*this == o); }
    GridCell operator+(const GridCell& o) const { return {x + o.x, y + o.y}; }
};

struct GridCellHash {
    std::size_t operator()(const GridCell& c) const {
        return std::hash<int>()(c.x) ^ (std::hash<int>()(c.y) << 1);
    }
};

struct Obstacle {
    double x, y;
    double radius;
    std::chrono::system_clock::time_point timestamp;
    bool reported_to_osrm;
};

struct DStarNode {
    GridCell cell;
    double g, rhs;
    double key[2];
    bool operator<(const DStarNode& o) const {
        if (key[0] != o.key[0]) return key[0] > o.key[0];
        return key[1] > o.key[1];
    }
};

struct AStarNode {
    GridCell cell;
    double f_score;
    bool operator<(const AStarNode& o) const { return f_score > o.f_score; }
};

// ==============================================================================
// PATH PLANNING MODULE (PRODUCTION)
// ==============================================================================

class PathPlanningModule : public rclcpp::Node {
public:
    PathPlanningModule() : Node("path_planning_module"), km_(0.0) {
        
        // Start metrics timer
        metrics_.start_time = std::chrono::system_clock::now();
        
        // ========================================
        // PARAMETERS
        // ========================================
        this->declare_parameter("origin_latitude", 45.3876);
        this->declare_parameter("origin_longitude", -75.6960);
        this->declare_parameter("use_dstar_lite", true);
        this->declare_parameter("replan_threshold", 5);
        this->declare_parameter("obstacle_detection_range", 5.0);
        this->declare_parameter("obstacle_threshold_size", 0.3);
        this->declare_parameter("path_clearance_check", 2.0);
        this->declare_parameter("osrm_update_cooldown", 5.0);
        this->declare_parameter("inflation_radius", 2);
        this->declare_parameter("map_width", 150);
        this->declare_parameter("map_height", 150);
        this->declare_parameter("map_resolution", 0.5);
        this->declare_parameter("stop_sign_min_confidence", 0.7);
        this->declare_parameter("stop_sign_stop_duration", 3.0);
        this->declare_parameter("metrics_output_file", "/home/nvidia/metrics.csv");
        
        origin_lat_ = this->get_parameter("origin_latitude").as_double();
        origin_lon_ = this->get_parameter("origin_longitude").as_double();
        use_dstar_lite_ = this->get_parameter("use_dstar_lite").as_bool();
        replan_threshold_ = this->get_parameter("replan_threshold").as_int();
        obstacle_detection_range_ = this->get_parameter("obstacle_detection_range").as_double();
        obstacle_threshold_size_ = this->get_parameter("obstacle_threshold_size").as_double();
        path_clearance_check_ = this->get_parameter("path_clearance_check").as_double();
        osrm_update_cooldown_ = this->get_parameter("osrm_update_cooldown").as_double();
        inflation_radius_ = this->get_parameter("inflation_radius").as_int();
        stop_sign_min_confidence_ = this->get_parameter("stop_sign_min_confidence").as_double();
        stop_sign_stop_duration_ = this->get_parameter("stop_sign_stop_duration").as_double();
        metrics_output_file_ = this->get_parameter("metrics_output_file").as_string();
        
        meters_per_degree_lat_ = 111320.0;
        meters_per_degree_lon_ = 111320.0 * std::cos(origin_lat_ * M_PI / 180.0);
        
        // ========================================
        // QoS PROFILES
        // ========================================
        auto qos_reliable = rclcpp::QoS(10).reliable();
        auto qos_best_effort = rclcpp::QoS(10).best_effort();
        auto qos_transient = rclcpp::QoS(10).transient_local().reliable();
        
        // ========================================
        // SUBSCRIBERS
        // ========================================
        
        // From Eric's OSRM/Local Route Mapping
        osrm_route_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/osrm/route", qos_reliable,
            std::bind(&PathPlanningModule::osrmRouteCallback, this, std::placeholders::_1));
        
        // From UI Raspberry Pi - GPS Destination
        destination_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/destination_coordinate", qos_reliable,
            std::bind(&PathPlanningModule::destinationCallback, this, std::placeholders::_1));
        
        // From LiDAR (Perception Team)
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/filtered_velodyne_points", qos_best_effort,
            std::bind(&PathPlanningModule::scanCallback, this, std::placeholders::_1));
        
        // From Localization
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos_best_effort,
            std::bind(&PathPlanningModule::odomCallback, this, std::placeholders::_1));
        
        
        // From Camera (Stop Sign Detection)
        stop_sign_detected_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/aav/stop_sign_detected", qos_reliable,
            std::bind(&PathPlanningModule::stopSignDetectedCallback, this, std::placeholders::_1));
        
        stop_sign_confidence_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/aav/stop_sign_confidence", qos_reliable,
            std::bind(&PathPlanningModule::stopSignConfidenceCallback, this, std::placeholders::_1));
        
        // ========================================
        // PUBLISHERS
        // ========================================
        
        // To UI Raspberry Pi + Pure Pursuit
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_plan", qos_reliable);
        
        // To Velocity Planner
        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_costmap", qos_reliable);
        
        // To UI Raspberry Pi (OSRM updates)
        osrm_update_pub_ = this->create_publisher<std_msgs::msg::String>("/osrm/obstacle_update", qos_reliable);
        
        // To UI Raspberry Pi (Status)
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/nav/status", qos_reliable);
        
        // Stop sign status for downstream controllers
        stop_sign_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/stop_sign_status", qos_reliable);
        
        // ========================================
        // TIMERS
        // ========================================
        obstacle_detection_timer_ = this->create_wall_timer(
            200ms, std::bind(&PathPlanningModule::detectObstacles, this));
        
        path_validation_timer_ = this->create_wall_timer(
            500ms, std::bind(&PathPlanningModule::validatePathClearance, this));
        
        replan_timer_ = this->create_wall_timer(
            500ms, std::bind(&PathPlanningModule::checkForReplanning, this));
        
        stop_sign_timer_ = this->create_wall_timer(
            100ms, std::bind(&PathPlanningModule::checkStopSignStatus, this));
        
        metrics_timer_ = this->create_wall_timer(
            10s, std::bind(&PathPlanningModule::updateMetrics, this));
        
        initializeMap();
        last_osrm_update_time_ = this->now();
        
        printStartup();
    }
    
    ~PathPlanningModule() {
        // Save metrics on shutdown
        metrics_.total_runtime_s = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::system_clock::now() - metrics_.start_time).count();
        
        metrics_.saveToCSV(metrics_output_file_);
        RCLCPP_INFO(this->get_logger(), "📊 Metrics saved to: %s", metrics_output_file_.c_str());
    }

private:
    void printStartup() {
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "PATH PLANNING & OBSTACLE AVOIDANCE");
        RCLCPP_INFO(this->get_logger(), "     PRODUCTION VERSION 1.0");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), " Platform: NVIDIA Jetson Orin Nano");
        RCLCPP_INFO(this->get_logger(), "Network: ROS2 DDS (Jetson ↔ Raspberry Pi)");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Team: Navigation & Logic Group");
        RCLCPP_INFO(this->get_logger(), "   • Bhavaan - Path Planning");
        RCLCPP_INFO(this->get_logger(), "   • Emeka - Obstacle Integration");
        RCLCPP_INFO(this->get_logger(), "   • Eric - Testing & Validation");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "COMMUNICATION:");
        RCLCPP_INFO(this->get_logger(), "   IN  ← /destination_coordinate (Raspberry Pi)");
        RCLCPP_INFO(this->get_logger(), "   IN  ← /scan (LiDAR)");
        RCLCPP_INFO(this->get_logger(), "   IN  ← /aav/stop_sign_detected (Camera)");
        RCLCPP_INFO(this->get_logger(), "   OUT → /global_plan (to UI & Pure Pursuit)");
        RCLCPP_INFO(this->get_logger(), "   OUT → /osrm/obstacle_update (to UI/OSRM)");
        RCLCPP_INFO(this->get_logger(), "   OUT → /nav/status (to UI)");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "GPS Origin: (%.6f°, %.6f°)", origin_lat_, origin_lon_);
        RCLCPP_INFO(this->get_logger(), "Algorithm: %s", use_dstar_lite_ ? "D* Lite + A*" : "A* only");
        RCLCPP_INFO(this->get_logger(), "Stop Sign: %.0f%% confidence, %.1fs stop", 
                    stop_sign_min_confidence_ * 100, stop_sign_stop_duration_);
        RCLCPP_INFO(this->get_logger(), "Metrics: %s", metrics_output_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "READY - Waiting for destination from UI...");
    }
    
    void initializeMap() {
        width_ = this->get_parameter("map_width").as_int();
        height_ = this->get_parameter("map_height").as_int();
        resolution_ = this->get_parameter("map_resolution").as_double();
        
        map_.header.frame_id = "map";
        map_.info.width = width_;
        map_.info.height = height_;
        map_.info.resolution = resolution_;
        map_.info.origin.position.x = -width_ * resolution_ / 2.0;
        map_.info.origin.position.y = -height_ * resolution_ / 2.0;
        map_.info.origin.orientation.w = 1.0;
        
        map_.data.resize(width_ * height_, 0);
        previous_map_.resize(width_ * height_, 0);
        map_initialized_ = true;
    }
    
    // ========================================
    // GPS CONVERSION
    // ========================================
    std::pair<double, double> gpsToMap(double lat, double lon) {
        return {(lon - origin_lon_) * meters_per_degree_lon_,
                (lat - origin_lat_) * meters_per_degree_lat_};
    }
    
    std::pair<double, double> mapToGps(double x, double y) {
        return {y / meters_per_degree_lat_ + origin_lat_,
                x / meters_per_degree_lon_ + origin_lon_};
    }
    
    // ========================================
    // CALLBACKS
    // ========================================
    
    void osrmRouteCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty OSRM route");
            return;
        }
        
        osrm_route_ = *msg;
        route_received_ = true;
        
        RCLCPP_INFO(this->get_logger(), "OSRM Route: %zu waypoints (from Eric/UI)", 
                    msg->poses.size());
        
        detected_obstacles_.clear();
        
        if (odom_received_) {
            planLocalPath();
        }
    }
    
    void destinationCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        
        double lat = msg->x;  // UI sends latitude in Point.x
        double lon = msg->y;  // UI sends longitude in Point.y
        
        // Validate GPS
        if (std::abs(lat) > 90.0 || std::abs(lon) > 180.0) {
            RCLCPP_ERROR(this->get_logger(), 
                        " Invalid GPS: lat=%.6f, lon=%.6f", lat, lon);
            return;
        }
        
        // Convert GPS → Map
        auto [x, y] = gpsToMap(lat, lon);
        goal_pose_.position.x = x;
        goal_pose_.position.y = y;
        goal_received_ = true;
        
        // METRICS
        metrics_.destinations_received_from_ui++;
        
        RCLCPP_INFO(this->get_logger(), 
                   "🎯 Destination from UI: GPS(%.6f°, %.6f°) → Map(%.2f, %.2f)m", 
                   lat, lon, x, y);
        
        if (odom_received_) {
            planLocalPath();
        } else {
            RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
        }
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        scan_ = *msg;
        scan_received_ = true;
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_pose_ = msg->pose.pose;
        odom_received_ = true;
    }
    
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_ = *msg;
        width_ = map_.info.width;
        height_ = map_.info.height;
        resolution_ = map_.info.resolution;
        map_initialized_ = true;
    }
    
    void stopSignDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        stop_sign_detected_ = msg->data;
        
        if (msg->data && !stop_sign_was_detected_) {
            metrics_.stop_signs_detected++;
            stop_sign_was_detected_ = true;
            RCLCPP_INFO(this->get_logger(), "STOP SIGN DETECTED!");
        }
        
        if (!msg->data) {
            stop_sign_was_detected_ = false;
        }
    }
    
    void stopSignConfidenceCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        stop_sign_confidence_ = msg->data;
    }
    
    // ========================================
    // STOP SIGN LOGIC - CRITICAL FIX
    // ========================================
    void checkStopSignStatus() {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        
        if (!odom_received_) return;
        
        bool should_stop = false;
        
        if (stop_sign_detected_ && stop_sign_confidence_ >= stop_sign_min_confidence_) {
            
            if (!stop_sign_stopping_) {
                stop_sign_stop_start_time_ = this->now();
                stop_sign_stopping_ = true;
                metrics_.stop_signs_obeyed++;
                
                RCLCPP_WARN(this->get_logger(), 
                           "STOPPING for stop sign (conf: %.0f%%)", 
                           stop_sign_confidence_ * 100);
            }
            
            double elapsed = (this->now() - stop_sign_stop_start_time_).seconds();
            
            if (elapsed < stop_sign_stop_duration_) {
                should_stop = true;
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "Stopped: %.1f/%.1fs", 
                                    elapsed, stop_sign_stop_duration_);
            } else {
                // Done stopping
                metrics_.total_stop_duration_s += elapsed;
                RCLCPP_INFO(this->get_logger(), "Stop sign cleared - resuming");
                stop_sign_stopping_ = false;
            }
            
        } else {
            if (stop_sign_stopping_) {
                RCLCPP_INFO(this->get_logger(), "Stop sign no longer detected");
            }
            stop_sign_stopping_ = false;
        }
        
        // Publish status
        std_msgs::msg::Bool status_msg;
        status_msg.data = should_stop;
        stop_sign_status_pub_->publish(status_msg);
    }
    
    // ========================================
    // OBSTACLE DETECTION
    // ========================================
    void detectObstacles() {
        std::lock_guard<std::mutex> lock_scan(scan_mutex_);
        std::lock_guard<std::mutex> lock_odom(odom_mutex_);
        std::lock_guard<std::mutex> lock_map(map_mutex_);
        
        if (!scan_received_ || !odom_received_) return;
        
        auto now = std::chrono::system_clock::now();
        std::vector<std::pair<double, double>> points;
        
        for (size_t i = 0; i < scan_.ranges.size(); ++i) {
            float r = scan_.ranges[i];
            if (r < scan_.range_min || r > scan_.range_max || 
                std::isnan(r) || r > obstacle_detection_range_) continue;
            
            float a = scan_.angle_min + i * scan_.angle_increment;
            double yaw = 2.0 * std::atan2(current_pose_.orientation.z, current_pose_.orientation.w);
            double ga = yaw + a;
            double ox = current_pose_.position.x + r * std::cos(ga);
            double oy = current_pose_.position.y + r * std::sin(ga);
            
            points.push_back({ox, oy});
            
            GridCell cell = worldToGrid(ox, oy);
            if (cell.x >= 0 && cell.x < width_ && cell.y >= 0 && cell.y < height_) {
                int idx = cell.y * width_ + cell.x;
                if (map_.data[idx] < 50) {
                    map_.data[idx] = 100;
                    changed_cells_count_++;
                }
            }
        }
        
        clusterObstacles(points, now);
        publishLocalCostmap();
    }
    
    void clusterObstacles(const std::vector<std::pair<double, double>>& points,
                         std::chrono::system_clock::time_point now) {
        if (points.empty()) return;
        
        const double cluster_dist = obstacle_threshold_size_;
        std::vector<bool> assigned(points.size(), false);
        
        for (size_t i = 0; i < points.size(); ++i) {
            if (assigned[i]) continue;
            
            std::vector<std::pair<double, double>> cluster = {points[i]};
            assigned[i] = true;
            
            for (size_t j = i + 1; j < points.size(); ++j) {
                if (assigned[j]) continue;
                double d = std::hypot(points[i].first - points[j].first,
                                     points[i].second - points[j].second);
                if (d < cluster_dist) {
                    cluster.push_back(points[j]);
                    assigned[j] = true;
                }
            }
            
            if (cluster.size() >= 3) {
                double cx = 0, cy = 0;
                for (auto& [x, y] : cluster) { cx += x; cy += y; }
                cx /= cluster.size();
                cy /= cluster.size();
                
                double max_r = 0;
                for (auto& [x, y] : cluster) {
                    max_r = std::max(max_r, std::hypot(x - cx, y - cy));
                }
                
                bool is_new = true;
                for (auto& obs : detected_obstacles_) {
                    if (std::hypot(obs.x - cx, obs.y - cy) < 1.0) {
                        obs.timestamp = now;
                        is_new = false;
                        break;
                    }
                }
                
                if (is_new) {
                    detected_obstacles_.push_back({cx, cy, max_r + 0.3, now, false});
                    metrics_.total_obstacles_detected++;
                }
            }
        }
        
        auto timeout = std::chrono::seconds(5);
        detected_obstacles_.erase(
            std::remove_if(detected_obstacles_.begin(), detected_obstacles_.end(),
                [now, timeout](const Obstacle& o) {
                    return (now - o.timestamp) > timeout;
                }),
            detected_obstacles_.end()
        );
    }
    
    void publishLocalCostmap() {
        nav_msgs::msg::OccupancyGrid costmap;
        costmap.header.stamp = this->now();
        costmap.header.frame_id = "base_link";
        costmap.info.resolution = 0.1;
        costmap.info.width = costmap.info.height = 100;
        costmap.info.origin.position.x = costmap.info.origin.position.y = -5.0;
        costmap.info.origin.orientation.w = 1.0;
        costmap.data.resize(10000, 0);
        
        for (const auto& obs : detected_obstacles_) {
            double dx = obs.x - current_pose_.position.x;
            double dy = obs.y - current_pose_.position.y;
            double yaw = 2.0 * std::atan2(current_pose_.orientation.z, current_pose_.orientation.w);
            
            double lx = dx * std::cos(-yaw) - dy * std::sin(-yaw);
            double ly = dx * std::sin(-yaw) + dy * std::cos(-yaw);
            
            int gx = (int)((lx + 5.0) / 0.1);
            int gy = (int)((ly + 5.0) / 0.1);
            
            if (gx >= 0 && gx < 100 && gy >= 0 && gy < 100) {
                int r_cells = (int)(obs.radius / 0.1);
                for (int dy = -r_cells; dy <= r_cells; ++dy) {
                    for (int dx = -r_cells; dx <= r_cells; ++dx) {
                        int nx = gx + dx, ny = gy + dy;
                        if (nx >= 0 && nx < 100 && ny >= 0 && ny < 100) {
                            if (std::hypot(dx * 0.1, dy * 0.1) <= obs.radius) {
                                costmap.data[ny * 100 + nx] = 100;
                            }
                        }
                    }
                }
            }
        }
        
        costmap_pub_->publish(costmap);
    }
    
    // ========================================
    // PATH PLANNING - CRITICAL FIX
    // ========================================
    void planLocalPath() {
        std::lock_guard<std::mutex> lock_map(map_mutex_);
        std::lock_guard<std::mutex> lock_goal(goal_mutex_);
        std::lock_guard<std::mutex> lock_odom(odom_mutex_);
        
        // CRITICAL FIX: Don't plan while stopped at stop sign
        if (stop_sign_stopping_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "Planning paused - stopped at stop sign");
            publishStatus("stopped_at_stop_sign");
            return;
        }
        
        if (!map_initialized_ || !odom_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "Waiting for map and odometry...");
            return;
        }
        
        GridCell start = worldToGrid(current_pose_.position.x, current_pose_.position.y);
        GridCell goal;
        
        if (route_received_ && !osrm_route_.poses.empty()) {
            auto& target = osrm_route_.poses.back().pose.position;
            goal = worldToGrid(target.x, target.y);
        } else if (goal_received_) {
            goal = worldToGrid(goal_pose_.position.x, goal_pose_.position.y);
        } else {
            return;
        }
        
        // METRICS: Start timing
        auto t0 = std::chrono::steady_clock::now();
        metrics_.total_planning_calls++;
        
        std::vector<GridCell> path;
        int iterations = 0;
        
        if (use_dstar_lite_ && dstar_initialized_) {
            path = dStarLite();
            iterations = dstar_iterations_total;
        } else {
            path = aStar(start, goal, &iterations);
            metrics_.a_star_iterations_total += iterations;
            if (use_dstar_lite_ && !path.empty()) {
                initializeDStarLite(goal);
            }
        }
        
        // METRICS: End timing
        auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();
        
        metrics_.total_planning_time_ms += dt_ms;
        metrics_.min_planning_time_ms = std::min(metrics_.min_planning_time_ms, (double)dt_ms);
        metrics_.max_planning_time_ms = std::max(metrics_.max_planning_time_ms, (double)dt_ms);
        
        if (path.empty()) {
            RCLCPP_ERROR(this->get_logger(), " No path found!");
            publishStatus("planning_failed");
            metrics_.failed_paths++;
            return;
        }
        
        metrics_.successful_paths++;
        metrics_.total_waypoints += path.size();
        
        // Calculate path length
        double path_length = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            auto [x1, y1] = gridToWorld(path[i-1].x, path[i-1].y);
            auto [x2, y2] = gridToWorld(path[i].x, path[i].y);
            path_length += std::hypot(x2 - x1, y2 - y1);
        }
        metrics_.total_path_length_m += path_length;
        
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";
        
        for (auto& c : path) {
            geometry_msgs::msg::PoseStamped p;
            p.header = path_msg.header;
            auto [x, y] = gridToWorld(c.x, c.y);
            p.pose.position.x = x;
            p.pose.position.y = y;
            p.pose.orientation.w = 1.0;
            path_msg.poses.push_back(p);
        }
        
        {
            std::lock_guard<std::mutex> lock_path(path_mutex_);
            current_path_cells_ = path;
        }
        
        path_pub_->publish(path_msg);
        metrics_.paths_sent_to_ui++;
        
        previous_map_ = map_.data;
        changed_cells_count_ = 0;
        
        RCLCPP_INFO(this->get_logger(), 
                   "✅ Path: %zu waypoints, %.1fm, %ld ms → UI", 
                   path.size(), path_length, dt_ms);
        
        publishStatus("path_ready");
    }
    
    // ========================================
    // A* ALGORITHM
    // ========================================
    std::vector<GridCell> aStar(GridCell start, GridCell goal, int* iterations_out) {
        std::priority_queue<AStarNode> open;
        std::unordered_set<GridCell, GridCellHash> closed;
        std::unordered_map<GridCell, GridCell, GridCellHash> from;
        std::unordered_map<GridCell, double, GridCellHash> g_score;
        
        g_score[start] = 0.0;
        open.push({start, heuristic(start, goal)});
        
        const std::vector<GridCell> dirs = {
            {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}
        };
        
        int iter = 0;
        while (!open.empty() && iter++ < width_ * height_) {
            auto curr = open.top(); open.pop();
            if (curr.cell == goal) {
                std::vector<GridCell> path;
                GridCell c = goal;
                while (c != start) { path.push_back(c); c = from[c]; }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                if (iterations_out) *iterations_out = iter;
                return path;
            }
            if (closed.count(curr.cell)) continue;
            closed.insert(curr.cell);
            
            for (auto& d : dirs) {
                GridCell n = curr.cell + d;
                if (isOccupied(n) || closed.count(n)) continue;
                
                double cost = (d.x && d.y) ? 1.414 : 1.0;
                double tg = g_score[curr.cell] + cost;
                
                if (!g_score.count(n) || tg < g_score[n]) {
                    from[n] = curr.cell;
                    g_score[n] = tg;
                    open.push({n, tg + heuristic(n, goal)});
                }
            }
        }
        if (iterations_out) *iterations_out = iter;
        return {};
    }
    
    // ========================================
    // D* LITE (abbreviated for brevity)
    // ========================================
    void initializeDStarLite(const GridCell& goal) {
        dstar_g_.clear();
        dstar_rhs_.clear();
        dstar_open_.clear();
        
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                GridCell c = {x, y};
                dstar_g_[c] = dstar_rhs_[c] = std::numeric_limits<double>::infinity();
            }
        }
        
        goal_cell_ = goal;
        dstar_rhs_[goal] = 0.0;
        
        DStarNode n;
        n.cell = goal;
        n.g = std::numeric_limits<double>::infinity();
        n.rhs = 0.0;
        calculateKey(n, start_cell_);
        dstar_open_.insert(n);
        
        km_ = 0.0;
        last_start_ = start_cell_;
        dstar_initialized_ = true;
    }
    
    void calculateKey(DStarNode& n, const GridCell& start) {
        double m = std::min(n.g, n.rhs);
        n.key[0] = m + heuristic(start, n.cell) + km_;
        n.key[1] = m;
    }
    
    std::vector<GridCell> dStarLite() {
        if (!dstar_initialized_) return {};
        
        GridCell start = worldToGrid(current_pose_.position.x, current_pose_.position.y);
        
        if (start != last_start_) {
            km_ += heuristic(last_start_, start);
            last_start_ = start;
        }
        
        computeShortestPath(start);
        
        std::vector<GridCell> path;
        GridCell curr = start;
        
        while (curr != goal_cell_) {
            path.push_back(curr);
            GridCell best = curr;
            double best_cost = std::numeric_limits<double>::infinity();
            
            for (auto& n : getNeighbors(curr)) {
                if (isOccupied(n)) continue;
                double c = dstar_g_[n] + moveCost(curr, n);
                if (c < best_cost) {
                    best_cost = c;
                    best = n;
                }
            }
            
            if (best == curr || path.size() > (size_t)(width_ * height_)) return {};
            curr = best;
        }
        
        path.push_back(goal_cell_);
        return path;
    }
    
    void computeShortestPath(const GridCell& start) {
        int iter = 0;
        while (!dstar_open_.empty() && iter++ < width_ * height_) {
            auto it = dstar_open_.begin();
            DStarNode u = *it;
            dstar_open_.erase(it);
            
            DStarNode sn;
            sn.cell = start;
            sn.g = dstar_g_[start];
            sn.rhs = dstar_rhs_[start];
            calculateKey(sn, start);
            
            if (u.key[0] >= sn.key[0] && dstar_rhs_[start] == dstar_g_[start]) break;
            
            if (dstar_g_[u.cell] > dstar_rhs_[u.cell]) {
                dstar_g_[u.cell] = dstar_rhs_[u.cell];
                for (auto& pred : getNeighbors(u.cell)) {
                    if (pred != goal_cell_) updateVertex(pred, start);
                }
            } else {
                dstar_g_[u.cell] = std::numeric_limits<double>::infinity();
                updateVertex(u.cell, start);
                for (auto& pred : getNeighbors(u.cell)) {
                    if (pred != goal_cell_) updateVertex(pred, start);
                }
            }
        }
        dstar_iterations_total += iter;
    }
    
    void updateVertex(const GridCell& c, const GridCell& start) {
        if (c != goal_cell_) {
            double min_rhs = std::numeric_limits<double>::infinity();
            for (auto& s : getNeighbors(c)) {
                if (!isOccupied(s)) {
                    min_rhs = std::min(min_rhs, dstar_g_[s] + moveCost(c, s));
                }
            }
            dstar_rhs_[c] = min_rhs;
        }
        
        auto it = dstar_open_.begin();
        while (it != dstar_open_.end()) {
            if (it->cell == c) it = dstar_open_.erase(it);
            else ++it;
        }
        
        if (dstar_g_[c] != dstar_rhs_[c]) {
            DStarNode n;
            n.cell = c;
            n.g = dstar_g_[c];
            n.rhs = dstar_rhs_[c];
            calculateKey(n, start);
            dstar_open_.insert(n);
        }
    }
    
    // ========================================
    // PATH VALIDATION
    // ========================================
    void validatePathClearance() {
        std::lock_guard<std::mutex> lock_path(path_mutex_);
        std::lock_guard<std::mutex> lock_odom(odom_mutex_);
        
        if (current_path_cells_.empty() || !odom_received_) return;
        
        std::vector<Obstacle> blocking;
        for (const auto& obs : detected_obstacles_) {
            for (size_t i = 0; i < std::min((size_t)10, current_path_cells_.size()); ++i) {
                auto [x, y] = gridToWorld(current_path_cells_[i].x, current_path_cells_[i].y);
                double d = std::hypot(obs.x - x, obs.y - y);
                if (d < obs.radius + 0.5) {
                    if (!obs.reported_to_osrm) blocking.push_back(obs);
                    break;
                }
            }
        }
        
        if (!blocking.empty()) {
            auto now = this->now();
            if ((now - last_osrm_update_time_).seconds() >= osrm_update_cooldown_) {
                sendObstacleUpdateToOSRM(blocking);
                last_osrm_update_time_ = now;
            }
        }
    }
    
    void sendObstacleUpdateToOSRM(std::vector<Obstacle>& obs) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6);
        oss << "{\"type\":\"obstacle_update\",\"obstacles\":[";
        
        for (size_t i = 0; i < obs.size(); ++i) {
            auto [lat, lon] = mapToGps(obs[i].x, obs[i].y);
            oss << "{\"lat\":" << lat << ",\"lon\":" << lon 
                << ",\"radius\":" << obs[i].radius << "}";
            if (i < obs.size() - 1) oss << ",";
            
            for (auto& o : detected_obstacles_) {
                if (std::abs(o.x - obs[i].x) < 0.1 && std::abs(o.y - obs[i].y) < 0.1) {
                    o.reported_to_osrm = true;
                    metrics_.obstacles_reported_to_osrm++;
                }
            }
        }
        
        oss << "]}";
        std_msgs::msg::String msg;
        msg.data = oss.str();
        osrm_update_pub_->publish(msg);
        metrics_.osrm_updates_sent++;
        
        RCLCPP_WARN(this->get_logger(), "🚨 Obstacle update → UI/OSRM (%zu obstacles)", obs.size());
    }
    
    void checkForReplanning() {
        std::lock_guard<std::mutex> lock_map(map_mutex_);
        std::lock_guard<std::mutex> lock_path(path_mutex_);
        
        if (current_path_cells_.empty() || !dstar_initialized_) return;
        
        if (changed_cells_count_ >= replan_threshold_) {
            RCLCPP_INFO(this->get_logger(), "🔄 Replanning: %d cells changed", changed_cells_count_);
            metrics_.total_replanning_events++;
            
            for (size_t i = 0; i < map_.data.size(); ++i) {
                if (map_.data[i] != previous_map_[i]) {
                    GridCell c = {(int)(i % width_), (int)(i / width_)};
                    GridCell start = worldToGrid(current_pose_.position.x, current_pose_.position.y);
                    updateVertex(c, start);
                    for (auto& n : getNeighbors(c)) updateVertex(n, start);
                }
            }
            
            planLocalPath();
        }
    }
    
    void publishStatus(const std::string& s) {
        std_msgs::msg::String m;
        m.data = s;
        status_pub_->publish(m);
        metrics_.status_updates_sent++;
    }
    
    void updateMetrics() {
        metrics_.total_runtime_s = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::system_clock::now() - metrics_.start_time).count();
        
        RCLCPP_INFO(this->get_logger(), 
                   "📊 Metrics: %d plans, %.1f%% success, %.1f ms avg, %d obstacles",
                   metrics_.total_planning_calls,
                   metrics_.total_planning_calls > 0 ? 100.0 * metrics_.successful_paths / metrics_.total_planning_calls : 0,
                   metrics_.total_planning_calls > 0 ? metrics_.total_planning_time_ms / metrics_.total_planning_calls : 0,
                   metrics_.total_obstacles_detected);
    }
    
    // ========================================
    // UTILITIES
    // ========================================
    GridCell worldToGrid(double x, double y) {
        return {(int)((x - map_.info.origin.position.x) / resolution_),
                (int)((y - map_.info.origin.position.y) / resolution_)};
    }
    
    std::pair<double, double> gridToWorld(int x, int y) {
        return {(x + 0.5) * resolution_ + map_.info.origin.position.x,
                (y + 0.5) * resolution_ + map_.info.origin.position.y};
    }
    
    bool isOccupied(const GridCell& c) {
        if (c.x < 0 || c.x >= width_ || c.y < 0 || c.y >= height_) return true;
        int i = c.y * width_ + c.x;
        return i < 0 || i >= (int)map_.data.size() || map_.data[i] > 50;
    }
    
    std::vector<GridCell> getNeighbors(const GridCell& c) {
        return {{c.x+1,c.y},{c.x-1,c.y},{c.x,c.y+1},{c.x,c.y-1},
                {c.x+1,c.y+1},{c.x+1,c.y-1},{c.x-1,c.y+1},{c.x-1,c.y-1}};
    }
    
    double heuristic(const GridCell& a, const GridCell& b) {
        return std::hypot(a.x - b.x, a.y - b.y);
    }
    
    double moveCost(const GridCell& a, const GridCell& b) {
        return (std::abs(a.x - b.x) + std::abs(a.y - b.y) == 2) ? 1.414 : 1.0;
    }
    
    // ========================================
    // MEMBERS
    // ========================================
    
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr osrm_route_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr destination_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sign_detected_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr stop_sign_confidence_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr osrm_update_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_sign_status_pub_;
    
    rclcpp::TimerBase::SharedPtr obstacle_detection_timer_;
    rclcpp::TimerBase::SharedPtr path_validation_timer_;
    rclcpp::TimerBase::SharedPtr replan_timer_;
    rclcpp::TimerBase::SharedPtr stop_sign_timer_;
    rclcpp::TimerBase::SharedPtr metrics_timer_;
    
    std::mutex path_mutex_, scan_mutex_, odom_mutex_, goal_mutex_, map_mutex_;
    
    nav_msgs::msg::OccupancyGrid map_;
    nav_msgs::msg::Path osrm_route_;
    std::vector<GridCell> current_path_cells_;
    std::vector<int8_t> previous_map_;
    sensor_msgs::msg::LaserScan scan_;
    geometry_msgs::msg::Pose current_pose_, goal_pose_;
    std::vector<Obstacle> detected_obstacles_;
    rclcpp::Time last_osrm_update_time_;
    
    bool map_initialized_ = false, scan_received_ = false;
    bool odom_received_ = false, goal_received_ = false, route_received_ = false;
    
    double origin_lat_, origin_lon_, meters_per_degree_lat_, meters_per_degree_lon_;
    double obstacle_detection_range_, obstacle_threshold_size_;
    double path_clearance_check_, osrm_update_cooldown_;
    bool use_dstar_lite_;
    int replan_threshold_, inflation_radius_;
    int width_, height_, changed_cells_count_ = 0;
    double resolution_;
    
    bool stop_sign_detected_ = false;
    bool stop_sign_was_detected_ = false;
    float stop_sign_confidence_ = 0.0;
    bool stop_sign_stopping_ = false;
    rclcpp::Time stop_sign_stop_start_time_;
    double stop_sign_min_confidence_;
    double stop_sign_stop_duration_;
    
    std::unordered_map<GridCell, double, GridCellHash> dstar_g_, dstar_rhs_;
    std::set<DStarNode> dstar_open_;
    double km_;
    GridCell start_cell_, goal_cell_, last_start_;
    bool dstar_initialized_ = false;
    int dstar_iterations_total = 0;
    
    PerformanceMetrics metrics_;
    std::string metrics_output_file_;
};

// ==============================================================================
// MAIN
// ==============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlanningModule>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    RCLCPP_INFO(node->get_logger(), "READY - Press Ctrl+C to save metrics and exit");
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
