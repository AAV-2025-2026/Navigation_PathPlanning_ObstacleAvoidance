

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <memory>
#include <limits>
#include <mutex>
#include <chrono>
#include <sstream>
#include <iomanip>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::chrono_literals;

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
    double x, y, radius;
    std::chrono::steady_clock::time_point timestamp;
    std::string source;
};

struct DStarNode {
    GridCell cell;
    double g, rhs;
    double key[2];
    bool operator<(const DStarNode& o) const {
        if (std::abs(key[0] - o.key[0]) > 1e-6) return key[0] > o.key[0];
        return key[1] > o.key[1];
    }
};

struct AStarNode {
    GridCell cell;
    double f_score;
    bool operator<(const AStarNode& o) const { return f_score > o.f_score; }
};

//Path PLanning 


class PathPlanningModule : public rclcpp::Node {
public:
    PathPlanningModule() : Node("path_planning_module") {
        
        // Parameters
        this->declare_parameter("origin_latitude", 45.3847);
        this->declare_parameter("origin_longitude", -75.6962);
        this->declare_parameter("map_width", 150);
        this->declare_parameter("map_height", 150);
        this->declare_parameter("map_resolution", 0.5);
        this->declare_parameter("use_dstar_lite", true);
        this->declare_parameter("replan_threshold", 10);
        this->declare_parameter("obstacle_detection_range", 5.0);
        this->declare_parameter("lidar_downsample_leaf_size", 0.2);
        this->declare_parameter("max_planning_iterations", 5000);
        this->declare_parameter("stop_sign_stop_distance", 3.0);
        
        origin_lat_ = this->get_parameter("origin_latitude").as_double();
        origin_lon_ = this->get_parameter("origin_longitude").as_double();
        width_ = this->get_parameter("map_width").as_int();
        height_ = this->get_parameter("map_height").as_int();
        resolution_ = this->get_parameter("map_resolution").as_double();
        use_dstar_lite_ = this->get_parameter("use_dstar_lite").as_bool();
        replan_threshold_ = this->get_parameter("replan_threshold").as_int();
        obstacle_detection_range_ = this->get_parameter("obstacle_detection_range").as_double();
        lidar_downsample_leaf_ = this->get_parameter("lidar_downsample_leaf_size").as_double();
        max_planning_iterations_ = this->get_parameter("max_planning_iterations").as_int();
        stop_sign_stop_distance_ = this->get_parameter("stop_sign_stop_distance").as_double();
        
        // GPS conversion
        meters_per_degree_lat_ = 111320.0;
        meters_per_degree_lon_ = 111320.0 * std::cos(origin_lat_ * M_PI / 180.0);
        
        // QoS
        auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        auto qos_sensor = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        
        // Subscribers from the other nodes
        
        // UI TEAM - GPS destination
        ui_goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/destination_coordinate", qos_reliable,
            std::bind(&PathPlanningModule::uiGoalCallback, this, std::placeholders::_1));
        
        // LIDAR TEAM - RAW point cloud (you handle filtering)
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", qos_sensor,
            std::bind(&PathPlanningModule::lidarCallback, this, std::placeholders::_1));
        
        // CAMERA TEAM - Stop sign detection
        stop_sign_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/aav/stop_sign_detected", qos_reliable,
            std::bind(&PathPlanningModule::stopSignCallback, this, std::placeholders::_1));
        
        stop_sign_confidence_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/aav/stop_sign_confidence", qos_reliable,
            std::bind(&PathPlanningModule::stopSignConfidenceCallback, this, std::placeholders::_1));
        
        // ODOMETRY - Fallback positioning
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos_sensor,
            std::bind(&PathPlanningModule::odomCallback, this, std::placeholders::_1));
        
        // ======================================================================
        // PUBLISHERS - TO UI TEAM & VISUALIZATION
        // ======================================================================
        
        // UI TEAM - Optimal path in GPS coordinates
        optimal_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/optimal_path", qos_reliable);
        
        // UI TEAM - Distance in meters
        distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/path_distance", qos_reliable);
        
        // UI TEAM - Number of obstacles detected
        obstacles_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/obstacles_detected", qos_reliable);
        
        // UI TEAM - Obstacle locations as a JSON File
        obstacle_locations_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/obstacle_locations", qos_reliable);
        
        // UI TEAM - Navigation status
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/nav_status", qos_reliable);
        
        // UI TEAM - Request new route from OSRM
        route_request_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/request_new_route", qos_reliable);
        
        // VISUALIZATION - Costmap for debugging
        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/local_costmap", qos_reliable);
        
        // Timers (optimized for CPU)
        obstacle_timer_ = this->create_wall_timer(
            400ms, std::bind(&PathPlanningModule::obstacleTimerCallback, this));
        
        replan_timer_ = this->create_wall_timer(
            1000ms, std::bind(&PathPlanningModule::checkForReplanning, this));
        
        // Initialize
        initializeMap();
        printStartup();
    }

private:
    // ROS2
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ui_goal_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sign_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr stop_sign_confidence_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimal_path_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_locations_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr route_request_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    
    rclcpp::TimerBase::SharedPtr obstacle_timer_;
    rclcpp::TimerBase::SharedPtr replan_timer_;
    
    // State
    nav_msgs::msg::OccupancyGrid map_;
    std::vector<GridCell> current_path_cells_;
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Pose goal_pose_;
    std::vector<Obstacle> detected_obstacles_;
    
    bool map_initialized_ = false;
    bool odom_received_ = false;
    bool goal_received_ = false;
    float stop_sign_confidence_ = 0.0f;
    
    std::chrono::steady_clock::time_point last_lidar_process_;
    
    // D* Lite
    bool dstar_initialized_ = false;
    GridCell dstar_start_, dstar_goal_;
    double km_ = 0.0;
    int changed_cells_count_ = 0;
    std::unordered_map<GridCell, DStarNode, GridCellHash> dstar_nodes_;
    std::priority_queue<DStarNode> dstar_open_list_;
    std::unordered_set<GridCell, GridCellHash> changed_cells_;
    
    // Parameters
    double origin_lat_, origin_lon_;
    int width_, height_;
    double resolution_;
    bool use_dstar_lite_;
    int replan_threshold_;
    double obstacle_detection_range_;
    double lidar_downsample_leaf_;
    int max_planning_iterations_;
    double stop_sign_stop_distance_;
    
    double meters_per_degree_lat_;
    double meters_per_degree_lon_;
    
    std::mutex map_mutex_;
    std::mutex goal_mutex_;
    std::mutex odom_mutex_;
    std::mutex obstacles_mutex_;
    
    // 8-directional movement
    const std::vector<GridCell> directions_ = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1},
        {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
    };
    
    // ==========================================================================
    // INITIALIZATION
    // ==========================================================================
    
    void initializeMap() {
        map_.header.frame_id = "map";
        map_.info.resolution = resolution_;
        map_.info.width = width_;
        map_.info.height = height_;
        map_.info.origin.position.x = -width_ * resolution_ / 2.0;
        map_.info.origin.position.y = -height_ * resolution_ / 2.0;
        map_.info.origin.orientation.w = 1.0;
        map_.data.resize(width_ * height_, 0);
        map_initialized_ = true;
    }
    
    void printStartup() {
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "🧭 PATH PLANNING & OBSTACLE AVOIDANCE");
        RCLCPP_INFO(this->get_logger(), "   Jetson Orin Nano - CPU Target: 25-32%%");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "📥 SUBSCRIPTIONS:");
        RCLCPP_INFO(this->get_logger(), "   ← /destination_coordinate (UI Team)");
        RCLCPP_INFO(this->get_logger(), "   ← /velodyne_points (LiDAR - RAW)");
        RCLCPP_INFO(this->get_logger(), "   ← /aav/stop_sign_detected (Camera)");
        RCLCPP_INFO(this->get_logger(), "   ← /aav/stop_sign_confidence (Camera)");
        RCLCPP_INFO(this->get_logger(), "   ← /odom (Odometry)");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "📤 PUBLICATIONS (To UI Team):"); 
        RCLCPP_INFO(this->get_logger(), "   → /optimal_path (GPS waypoints)");
        RCLCPP_INFO(this->get_logger(), "   → /path_distance (meters)");
        RCLCPP_INFO(this->get_logger(), "   → /obstacles_detected (count)");
        RCLCPP_INFO(this->get_logger(), "   → /obstacle_locations (GPS JSON)");
        RCLCPP_INFO(this->get_logger(), "   → /nav_status (status string)");
        RCLCPP_INFO(this->get_logger(), "   → /request_new_route (reroute trigger)");
        RCLCPP_INFO(this->get_logger(), "   → /local_costmap (visualization)");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "🌍 Origin: (%.6f°, %.6f°)", origin_lat_, origin_lon_);
        RCLCPP_INFO(this->get_logger(), "🗺️  Map: %dx%d cells (%.1fm resolution)", 
                    width_, height_, resolution_);
        RCLCPP_INFO(this->get_logger(), "========================================");
    }
    
    // ==========================================================================
    // CALLBACKS - RECEIVE FROM UI & SENSORS
    // ==========================================================================
    
    void uiGoalCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        
        // UI sends: Point.x = latitude, Point.y = longitude
        double goal_lat = msg->x;
        double goal_lon = msg->y;
        
        // Convert GPS → Map
        double x = (goal_lon - origin_lon_) * meters_per_degree_lon_;
        double y = (goal_lat - origin_lat_) * meters_per_degree_lat_;
        
        goal_pose_.position.x = x;
        goal_pose_.position.y = y;
        goal_pose_.position.z = 0.0;
        goal_pose_.orientation.w = 1.0;
        
        goal_received_ = true;
        
        RCLCPP_INFO(this->get_logger(), 
            "🎯 Goal from UI: (%.6f°, %.6f°) → Map: (%.2fm, %.2fm)",
            goal_lat, goal_lon, x, y);
        
        publishStatus("planning");
        
        if (odom_received_) {
            planOptimalPath();
        }
    }
    
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Throttle to reduce CPU
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_lidar_process_).count() < 250) {
            return;
        }
        last_lidar_process_ = now;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        if (cloud->points.empty()) return;
        
        // Downsample (YOU handle this, not LiDAR team)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(lidar_downsample_leaf_, lidar_downsample_leaf_, lidar_downsample_leaf_);
        vg.filter(*cloud_filtered);
        
        std::lock_guard<std::mutex> lock_map(map_mutex_);
        std::lock_guard<std::mutex> lock_odom(odom_mutex_);
        
        // Update map (every 15th point for CPU efficiency)
        for (size_t i = 0; i < cloud_filtered->points.size(); i += 15) {
            const auto& pt = cloud_filtered->points[i];
            double dist = std::hypot(pt.x, pt.y);
            
            if (dist < 0.3 || dist > obstacle_detection_range_) continue;
            
            double wx = current_pose_.position.x + pt.x;
            double wy = current_pose_.position.y + pt.y;
            
            GridCell cell = worldToGrid(wx, wy);
            if (!isValidCell(cell)) continue;
            
            int idx = cell.y * width_ + cell.x;
            if (idx >= 0 && idx < static_cast<int>(map_.data.size())) {
                if (map_.data[idx] < 100) {
                    map_.data[idx] = 100;
                    changed_cells_.insert(cell);
                    changed_cells_count_++;
                }
            }
        }
    }
    
    void stopSignCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) return;
        
        RCLCPP_WARN(this->get_logger(), "🛑 STOP SIGN DETECTED!");
        
        std::lock_guard<std::mutex> lock_odom(odom_mutex_);
        std::lock_guard<std::mutex> lock_obs(obstacles_mutex_);
        
        // Add stop sign as obstacle
        double obs_x = current_pose_.position.x + stop_sign_stop_distance_ * 
                      std::cos(getYawFromPose(current_pose_));
        double obs_y = current_pose_.position.y + stop_sign_stop_distance_ * 
                      std::sin(getYawFromPose(current_pose_));
        
        Obstacle obs;
        obs.x = obs_x;
        obs.y = obs_y;
        obs.radius = 1.5;
        obs.timestamp = std::chrono::steady_clock::now();
        obs.source = "camera";
        
        detected_obstacles_.push_back(obs);
    }
    
    void stopSignConfidenceCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        stop_sign_confidence_ = msg->data;
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_pose_ = msg->pose.pose;
        odom_received_ = true;
    }
    
    // ==========================================================================
    // PATH PLANNING - A* ALGORITHM
    // ==========================================================================
    
    std::vector<GridCell> aStar(const GridCell& start, const GridCell& goal) {
        auto t0 = std::chrono::steady_clock::now();
        
        std::priority_queue<AStarNode> open;
        std::unordered_set<GridCell, GridCellHash> closed;
        std::unordered_map<GridCell, GridCell, GridCellHash> came_from;
        std::unordered_map<GridCell, double, GridCellHash> g_score;
        
        g_score[start] = 0.0;
        open.push({start, heuristic(start, goal)});
        
        int iterations = 0;
        
        while (!open.empty() && iterations < max_planning_iterations_) {
            iterations++;
            
            AStarNode current = open.top();
            open.pop();
            
            if (current.cell == goal) {
                auto path = reconstructPath(came_from, current.cell);
                auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - t0);
                
                RCLCPP_INFO(this->get_logger(), "✅ A*: %zu waypoints, %ld ms",
                            path.size(), dt.count());
                return path;
            }
            
            if (closed.count(current.cell)) continue;
            closed.insert(current.cell);
            
            for (const auto& dir : directions_) {
                GridCell neighbor = current.cell + dir;
                
                if (!isValidCell(neighbor) || isObstacle(neighbor) || closed.count(neighbor)) {
                    continue;
                }
                
                double move_cost = (std::abs(dir.x) + std::abs(dir.y) == 2) ? 1.414 : 1.0;
                double tentative_g = g_score[current.cell] + move_cost;
                
                if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
                    came_from[neighbor] = current.cell;
                    g_score[neighbor] = tentative_g;
                    double f = tentative_g + heuristic(neighbor, goal);
                    open.push({neighbor, f});
                }
            }
        }
        
        RCLCPP_WARN(this->get_logger(), "❌ A* failed");
        return {};
    }
    
    // ==========================================================================
    // D* LITE ALGORITHM (Unchanged - already optimized)
    // ==========================================================================
    
    void initializeDStarLite(const GridCell& start, const GridCell& goal) {
        dstar_start_ = start;
        dstar_goal_ = goal;
        km_ = 0.0;
        
        dstar_nodes_.clear();
        while (!dstar_open_list_.empty()) dstar_open_list_.pop();
        changed_cells_.clear();
        changed_cells_count_ = 0;
        
        DStarNode& goal_node = dstar_nodes_[goal];
        goal_node.cell = goal;
        goal_node.g = std::numeric_limits<double>::infinity();
        goal_node.rhs = 0.0;
        
        calculateKey(goal_node);
        dstar_open_list_.push(goal_node);
        
        dstar_initialized_ = true;
    }
    
    std::vector<GridCell> dStarLitePlan() {
        if (!dstar_initialized_) return {};
        
        auto t0 = std::chrono::steady_clock::now();
        
        computeShortestPath();
        auto path = extractDStarPath();
        
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0);
        
        RCLCPP_INFO(this->get_logger(), "✅ D*Lite: %zu waypoints, %ld ms",
                    path.size(), dt.count());
        
        return path;
    }
    
    void computeShortestPath() {
        int iterations = 0;
        
        while (!dstar_open_list_.empty() && iterations < max_planning_iterations_) {
            iterations++;
            
            DStarNode u = dstar_open_list_.top();
            dstar_open_list_.pop();
            
            auto& start_node = dstar_nodes_[dstar_start_];
            if (compareKeys(u.key, calculateKeyForCell(dstar_start_)) >= 0 &&
                start_node.rhs == start_node.g) {
                break;
            }
            
            auto& u_node = dstar_nodes_[u.cell];
            calculateKey(u_node);
            
            if (u.g > u_node.rhs) {
                u_node.g = u_node.rhs;
                for (const auto& dir : directions_) {
                    GridCell s = u.cell + dir;
                    if (isValidCell(s) && !isObstacle(s)) {
                        updateVertex(s);
                    }
                }
            } else {
                u_node.g = std::numeric_limits<double>::infinity();
                updateVertex(u.cell);
                for (const auto& dir : directions_) {
                    GridCell s = u.cell + dir;
                    if (isValidCell(s) && !isObstacle(s)) {
                        updateVertex(s);
                    }
                }
            }
        }
    }
    
    void updateVertex(const GridCell& s) {
        if (s != dstar_goal_) {
            double min_rhs = std::numeric_limits<double>::infinity();
            
            for (const auto& dir : directions_) {
                GridCell s_prime = s + dir;
                if (!isValidCell(s_prime) || isObstacle(s_prime)) continue;
                
                double move_cost = (std::abs(dir.x) + std::abs(dir.y) == 2) ? 1.414 : 1.0;
                double cost = dstar_nodes_[s_prime].g + move_cost;
                if (cost < min_rhs) min_rhs = cost;
            }
            dstar_nodes_[s].rhs = min_rhs;
        }
        
        auto& s_node = dstar_nodes_[s];
        if (s_node.g != s_node.rhs) {
            calculateKey(s_node);
            dstar_open_list_.push(s_node);
        }
    }
    
    void calculateKey(DStarNode& node) {
        auto start_node = dstar_nodes_[dstar_start_];
        double min_g_rhs = std::min(node.g, node.rhs);
        node.key[0] = min_g_rhs + heuristic(dstar_start_, node.cell) + km_;
        node.key[1] = min_g_rhs;
    }
    
    double* calculateKeyForCell(const GridCell& cell) {
        static double key[2];
        auto& node = dstar_nodes_[cell];
        double min_val = std::min(node.g, node.rhs);
        key[0] = min_val + heuristic(dstar_start_, cell) + km_;
        key[1] = min_val;
        return key;
    }
    
    int compareKeys(const double k1[2], const double k2[2]) {
        if (std::abs(k1[0] - k2[0]) > 1e-6) return (k1[0] < k2[0]) ? -1 : 1;
        if (std::abs(k1[1] - k2[1]) > 1e-6) return (k1[1] < k2[1]) ? -1 : 1;
        return 0;
    }
    
    std::vector<GridCell> extractDStarPath() {
        std::vector<GridCell> path;
        GridCell current = dstar_start_;
        int max_steps = width_ * height_;
        int steps = 0;
        
        while (current != dstar_goal_ && steps < max_steps) {
            steps++;
            path.push_back(current);
            
            GridCell best_next = current;
            double best_cost = std::numeric_limits<double>::infinity();
            
            for (const auto& dir : directions_) {
                GridCell neighbor = current + dir;
                if (!isValidCell(neighbor) || isObstacle(neighbor)) continue;
                
                double move_cost = (std::abs(dir.x) + std::abs(dir.y) == 2) ? 1.414 : 1.0;
                double cost = dstar_nodes_[neighbor].g + move_cost;
                
                if (cost < best_cost) {
                    best_cost = cost;
                    best_next = neighbor;
                }
            }
            
            if (best_next == current) break;
            current = best_next;
        }
        
        if (current == dstar_goal_) {
            path.push_back(dstar_goal_);
        }
        return path;
    }
    
    // ==========================================================================
    // OPTIMAL PATH COMPUTATION & SEND TO UI
    // ==========================================================================
    
    void planOptimalPath() {
        std::lock_guard<std::mutex> lock_map(map_mutex_);
        std::lock_guard<std::mutex> lock_goal(goal_mutex_);
        std::lock_guard<std::mutex> lock_odom(odom_mutex_);
        
        if (!map_initialized_ || !odom_received_ || !goal_received_) return;
        
        GridCell start = worldToGrid(current_pose_.position.x, current_pose_.position.y);
        GridCell goal = worldToGrid(goal_pose_.position.x, goal_pose_.position.y);
        
        std::vector<GridCell> path;
        
        if (use_dstar_lite_ && dstar_initialized_) {
            dstar_start_ = start;
            km_ += heuristic(dstar_start_, start);
            
            for (const auto& cell : changed_cells_) {
                updateVertex(cell);
            }
            changed_cells_.clear();
            changed_cells_count_ = 0;
            
            path = dStarLitePlan();
        } else {
            path = aStar(start, goal);
            if (!path.empty() && use_dstar_lite_) {
                initializeDStarLite(start, goal);
            }
        }
        
        if (!path.empty()) {
            current_path_cells_ = path;
            
            // Convert to GPS and send to UI
            nav_msgs::msg::Path gps_path = cellsToGPSPath(path);
            optimal_path_pub_->publish(gps_path);
            
            // Calculate and send distance to UI
            double distance = calculatePathDistance(path);
            std_msgs::msg::Float32 dist_msg;
            dist_msg.data = distance;
            distance_pub_->publish(dist_msg);
            
            // Send obstacle count to UI
            std_msgs::msg::Int32 obs_msg;
            obs_msg.data = detected_obstacles_.size();
            obstacles_pub_->publish(obs_msg);
            
            publishStatus("ready");
            
            RCLCPP_INFO(this->get_logger(), 
                "📊 Sent to UI: %.1fm distance, %zu obstacles",
                distance, detected_obstacles_.size());
        } else {
            publishStatus("blocked");
        }
    }
    
    nav_msgs::msg::Path cellsToGPSPath(const std::vector<GridCell>& cells) {
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "gps";
        
        for (const auto& cell : cells) {
            double x_map, y_map;
            gridToWorld(cell, x_map, y_map);
            
            // Convert Map → GPS
            double lat = origin_lat_ + (y_map / meters_per_degree_lat_);
            double lon = origin_lon_ + (x_map / meters_per_degree_lon_);
            
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = lat;  // Latitude in x
            pose.pose.position.y = lon;  // Longitude in y
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            
            path.poses.push_back(pose);
        }
        
        return path;
    }
    
    double calculatePathDistance(const std::vector<GridCell>& path) {
        double distance = 0.0;
        for (size_t i = 0; i < path.size() - 1; ++i) {
            int dx = path[i+1].x - path[i].x;
            int dy = path[i+1].y - path[i].y;
            distance += std::hypot(dx, dy) * resolution_;
        }
        return distance;
    }
    
    // ==========================================================================
    // REPLANNING
    // ==========================================================================
    
    void checkForReplanning() {
        std::lock_guard<std::mutex> lock(map_mutex_);
        
        if (!dstar_initialized_ || !use_dstar_lite_) return;
        
        if (changed_cells_count_ >= replan_threshold_) {
            RCLCPP_INFO(this->get_logger(), "🔄 Replan: %d cells changed", 
                        changed_cells_count_);
            planOptimalPath();
        }
    }
    
    void obstacleTimerCallback() {
        std::lock_guard<std::mutex> lock_obs(obstacles_mutex_);
        std::lock_guard<std::mutex> lock_map(map_mutex_);
        
        // Remove old obstacles
        auto now = std::chrono::steady_clock::now();
        for (auto it = detected_obstacles_.begin(); it != detected_obstacles_.end();) {
            auto age = std::chrono::duration_cast<std::chrono::seconds>(
                now - it->timestamp);
            if (age.count() > 10) {
                it = detected_obstacles_.erase(it);
            } else {
                ++it;
            }
        }
        
        // Update map with current obstacles
        for (const auto& obs : detected_obstacles_) {
            GridCell center = worldToGrid(obs.x, obs.y);
            int cells_radius = static_cast<int>(obs.radius / resolution_) + 1;
            
            for (int dy = -cells_radius; dy <= cells_radius; ++dy) {
                for (int dx = -cells_radius; dx <= cells_radius; ++dx) {
                    if (dx*dx + dy*dy > cells_radius*cells_radius) continue;
                    
                    GridCell cell = {center.x + dx, center.y + dy};
                    if (!isValidCell(cell)) continue;
                    
                    int idx = cell.y * width_ + cell.x;
                    if (idx >= 0 && idx < static_cast<int>(map_.data.size())) {
                        if (map_.data[idx] < 100) {
                            map_.data[idx] = 100;
                            changed_cells_.insert(cell);
                            changed_cells_count_++;
                        }
                    }
                }
            }
        }
        
        // Publish costmap
        map_.header.stamp = this->now();
        costmap_pub_->publish(map_);
        
        // Send obstacle count to UI
        std_msgs::msg::Int32 obs_msg;
        obs_msg.data = detected_obstacles_.size();
        obstacles_pub_->publish(obs_msg);
        
        // NEW: Send obstacle GPS locations to UI
        sendObstacleLocationsToUI();
        
        // NEW: Check if we need to request new route from OSRM
        checkIfNeedNewRoute();
    }
    
    void publishStatus(const std::string& s) {
        std_msgs::msg::String m;
        m.data = s;
        status_pub_->publish(m);
    }
    
    // ==========================================================================
    // UTILITIES
    // ==========================================================================
    
    GridCell worldToGrid(double x, double y) {
        int gx = static_cast<int>((x - map_.info.origin.position.x) / resolution_);
        int gy = static_cast<int>((y - map_.info.origin.position.y) / resolution_);
        return {gx, gy};
    }
    
    void gridToWorld(const GridCell& cell, double& x, double& y) {
        x = map_.info.origin.position.x + (cell.x + 0.5) * resolution_;
        y = map_.info.origin.position.y + (cell.y + 0.5) * resolution_;
    }
    
    bool isValidCell(const GridCell& cell) {
        return cell.x >= 0 && cell.x < width_ && cell.y >= 0 && cell.y < height_;
    }
    
    bool isObstacle(const GridCell& cell) {
        int idx = cell.y * width_ + cell.x;
        return idx >= 0 && idx < static_cast<int>(map_.data.size()) && 
               map_.data[idx] > 50;
    }
    
    double heuristic(const GridCell& a, const GridCell& b) {
        return std::hypot(a.x - b.x, a.y - b.y);
    }
    
    double getYawFromPose(const geometry_msgs::msg::Pose& pose) {
        double siny_cosp = 2.0 * (pose.orientation.w * pose.orientation.z + 
                                   pose.orientation.x * pose.orientation.y);
        double cosy_cosp = 1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + 
                                         pose.orientation.z * pose.orientation.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }
    
    std::vector<GridCell> reconstructPath(
        const std::unordered_map<GridCell, GridCell, GridCellHash>& came_from,
        GridCell current) {
        
        std::vector<GridCell> path;
        path.push_back(current);
        
        while (came_from.count(current)) {
            current = came_from.at(current);
            path.push_back(current);
        }
        
        std::reverse(path.begin(), path.end());
        return path;
    }
    
    void sendObstacleLocationsToUI() {
        std::lock_guard<std::mutex> lock(obstacles_mutex_);
        
        if (detected_obstacles_.empty()) {
            // Send empty JSON if no obstacles
            std_msgs::msg::String msg;
            msg.data = "{\"obstacles\":[]}";
            obstacle_locations_pub_->publish(msg);
            return;
        }
        
        // Convert obstacles to GPS coordinates and create JSON
        std::ostringstream json;
        json << "{\"obstacles\":[";
        
        bool first = true;
        for (const auto& obs : detected_obstacles_) {
            // Convert map coordinates → GPS
            double lat = origin_lat_ + (obs.y / meters_per_degree_lat_);
            double lon = origin_lon_ + (obs.x / meters_per_degree_lon_);
            
            if (!first) json << ",";
            json << "{\"lat\":" << std::fixed << std::setprecision(6) << lat
                 << ",\"lon\":" << lon
                 << ",\"radius\":" << std::setprecision(2) << obs.radius
                 << ",\"source\":\"" << obs.source << "\"}";
            first = false;
        }
        
        json << "]}";
        
        std_msgs::msg::String msg;
        msg.data = json.str();
        obstacle_locations_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), 
            "📍 Sent %zu obstacle locations to UI", detected_obstacles_.size());
    }
    
    void checkIfNeedNewRoute() {
        std::lock_guard<std::mutex> lock(obstacles_mutex_);
        
        if (!goal_received_ || current_path_cells_.empty()) return;
        
        // Check if obstacles are blocking the current path
        int blocked_waypoints = 0;
        for (const auto& cell : current_path_cells_) {
            double wx, wy;
            gridToWorld(cell, wx, wy);
            
            for (const auto& obs : detected_obstacles_) {
                double dist = std::hypot(obs.x - wx, obs.y - wy);
                if (dist < obs.radius + 1.0) {  // 1m safety margin
                    blocked_waypoints++;
                    break;
                }
            }
        }
        
        // If >20% of path is blocked, request new route from OSRM
        double blocked_ratio = static_cast<double>(blocked_waypoints) / 
                              current_path_cells_.size();
        
        if (blocked_ratio > 0.2) {
            RCLCPP_WARN(this->get_logger(), 
                "⚠️  Path significantly blocked (%.0f%%), requesting new route from OSRM",
                blocked_ratio * 100.0);
            
            // Tell UI Team to request new route from OSRM
            std_msgs::msg::String msg;
            msg.data = "path_blocked_need_reroute";
            route_request_pub_->publish(msg);
            
            publishStatus("requesting_reroute");
        }
    }
};

// ==============================================================================
// MAIN
// ==============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlanningModule>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

