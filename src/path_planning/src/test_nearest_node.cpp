#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "path_planning/waypoint.hpp"

// Forward declare the function so we can link it
namespace path_planning {
    int find_nearest_waypoint(const std::vector<Waypoint>& waypoints, double car_x, double car_y);
}

class WaypointTrackerNode : public rclcpp::Node {
public:
    WaypointTrackerNode() : Node("waypoint_tracker_node") {
        // Declare and get the car_index parameter
        this->declare_parameter<int>("car_index", 0);
        int car_index = this->get_parameter("car_index").as_int();
        
        RCLCPP_INFO(this->get_logger(), "Initializing Waypoint Tracker for Car Index: %d", car_index);

        // Load waypoints from planned_route.csv or full_road_map.csv
        std::string home_dir = getenv("HOME");
        std::string wp_file = home_dir + "/autoCar_ws/waypoints/planned_route.csv";
        load_waypoints(wp_file);
        
        // Construct the topic name dynamically using the car_index
        // For example: /prius_0/odom or /car_1/odom
        // Adjust the string formatting according to your generic tf/namespace rules!
        std::string odom_topic = (car_index == 0) ? "/prius/odom" : "/prius_" + std::to_string(car_index) + "/odom";

        // Subscribe to odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&WaypointTrackerNode::odom_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Listening to %s. Waiting for car movement...", odom_topic.c_str());
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double car_x = msg->pose.pose.position.x;
        double car_y = msg->pose.pose.position.y;
        
        if (waypoints_.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waypoints list is empty!");
            return;
        }

        int nearest_idx = path_planning::find_nearest_waypoint(waypoints_, car_x, car_y);
        if (nearest_idx >= 0 && nearest_idx < static_cast<int>(waypoints_.size())) {
            double wp_x = waypoints_[nearest_idx].x;
            double wp_y = waypoints_[nearest_idx].y;
            double dist = std::sqrt(std::pow(wp_x - car_x, 2) + std::pow(wp_y - car_y, 2));

            // Print throttle to avoid spamming the console
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Car (%.2f, %.2f) | Nearest WP: index %d (%.2f, %.2f) | Dist: %.2f meters",
                car_x, car_y, nearest_idx, wp_x, wp_y, dist);
        }
    }

    void load_waypoints(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open waypoints file: %s", filepath.c_str());
            return;
        }
        
        std::string line;
        bool is_first = true;
        while (std::getline(file, line)) {
            if (is_first) { is_first = false; continue; } // skip header
            std::stringstream ss(line);
            std::string val;
            path_planning::Waypoint wp;
            
            // Expected CSV format: step,x,y,z,yaw,grid_x,grid_y
            // (From planned_route.csv or full_road_map.csv, depending on columns)
            // Lets just extract elements split by comma
            std::vector<std::string> tokens;
            while (std::getline(ss, val, ',')) {
                tokens.push_back(val);
            }
            if (tokens.size() >= 3) {
                // Handle either full_road_map (x,y,z,yaw) or planned_route (step,x,y,z...)
                try {
                    if (tokens.size() == 4) { // likely full_road_map (x,y,z,yaw)
                        wp.x = std::stod(tokens[0]);
                        wp.y = std::stod(tokens[1]);
                    } else if (tokens.size() >= 5) { // likely planned_route (step,x,y,z,yaw)
                        wp.x = std::stod(tokens[1]);
                        wp.y = std::stod(tokens[2]);
                    }
                    waypoints_.push_back(wp);
                } catch (...) { /* ignoring bad lines */ }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s", waypoints_.size(), filepath.c_str());
    }

    std::vector<path_planning::Waypoint> waypoints_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
