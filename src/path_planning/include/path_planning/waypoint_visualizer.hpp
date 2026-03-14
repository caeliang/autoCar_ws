#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "path_planning/waypoint.hpp"

namespace path_planning {

/// Publish waypoints as RViz markers
class WaypointVisualizer {
public:
    WaypointVisualizer(rclcpp::Node* node,
                       const std::string& topic = "/waypoints/markers",
                       const std::string& frame_id = "map");

    /// Publish all waypoints (spheres + numbers + connecting lines)
    void publish(const WaypointList& waypoints, int active_idx = -1);

private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
    std::string frame_id_;
    int last_count_ = 0;  // for clearing old markers
};

}  // namespace path_planning
