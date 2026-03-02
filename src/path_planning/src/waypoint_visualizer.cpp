#include "path_planning/waypoint_visualizer.hpp"
#include <cmath>

namespace path_planning {

WaypointVisualizer::WaypointVisualizer(
    rclcpp::Node* node,
    const std::string& topic,
    const std::string& frame_id)
    : frame_id_(frame_id)
{
    pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        topic, rclcpp::QoS(1).transient_local());
}

void WaypointVisualizer::publish(const WaypointList& waypoints, int active_idx) {
    visualization_msgs::msg::MarkerArray ma;
    auto stamp = rclcpp::Clock().now();

    // ── Delete old markers ───────────────────────────────────────────
    if (last_count_ > 0) {
        visualization_msgs::msg::Marker del;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        del.header.frame_id = frame_id_;
        del.header.stamp = stamp;
        ma.markers.push_back(del);
    }

    if (waypoints.empty()) {
        pub_->publish(ma);
        last_count_ = 0;
        return;
    }

    int id = 0;

    // ── Sphere for each waypoint ─────────────────────────────────────
    for (size_t i = 0; i < waypoints.size(); ++i) {
        const auto& wp = waypoints[i];

        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = frame_id_;
        sphere.header.stamp = stamp;
        sphere.ns = "waypoints";
        sphere.id = id++;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position.x = wp.x;
        sphere.pose.position.y = wp.y;
        sphere.pose.position.z = 0.3;  // biraz yukarıda göster
        sphere.pose.orientation.w = 1.0;

        // Active waypoint daha büyük ve yeşil
        bool is_active = (static_cast<int>(i) == active_idx);
        double size = is_active ? 0.8 : 0.5;
        sphere.scale.x = size;
        sphere.scale.y = size;
        sphere.scale.z = size;

        sphere.color.a = 0.9f;
        if (is_active) {
            sphere.color.r = 0.0f; sphere.color.g = 1.0f; sphere.color.b = 0.0f;
        } else if (static_cast<int>(i) < active_idx) {
            // Geçilmiş waypoint: gri
            sphere.color.r = 0.5f; sphere.color.g = 0.5f; sphere.color.b = 0.5f;
            sphere.color.a = 0.4f;
        } else {
            // Gelecek waypoint: mavi
            sphere.color.r = 0.2f; sphere.color.g = 0.4f; sphere.color.b = 1.0f;
        }

        sphere.lifetime = rclcpp::Duration(0, 0);  // permanent
        ma.markers.push_back(sphere);

        // ── Text label (index number) ────────────────────────────────
        visualization_msgs::msg::Marker text;
        text.header = sphere.header;
        text.ns = "wp_labels";
        text.id = id++;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.x = wp.x;
        text.pose.position.y = wp.y;
        text.pose.position.z = 1.0;
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.5;
        text.color.r = 1.0f; text.color.g = 1.0f; text.color.b = 1.0f; text.color.a = 1.0f;
        text.text = std::to_string(i);
        text.lifetime = rclcpp::Duration(0, 0);
        ma.markers.push_back(text);

        // ── Arrow showing orientation ────────────────────────────────
        visualization_msgs::msg::Marker arrow;
        arrow.header = sphere.header;
        arrow.ns = "wp_arrows";
        arrow.id = id++;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.pose.position.x = wp.x;
        arrow.pose.position.y = wp.y;
        arrow.pose.position.z = 0.3;
        arrow.pose.orientation.x = 0.0;
        arrow.pose.orientation.y = 0.0;
        arrow.pose.orientation.z = std::sin(wp.yaw * 0.5);
        arrow.pose.orientation.w = std::cos(wp.yaw * 0.5);
        arrow.scale.x = 1.0;  // length
        arrow.scale.y = 0.15; // width
        arrow.scale.z = 0.15; // height
        arrow.color.r = 1.0f; arrow.color.g = 1.0f; arrow.color.b = 0.0f; arrow.color.a = 0.8f;
        arrow.lifetime = rclcpp::Duration(0, 0);
        ma.markers.push_back(arrow);
    }

    // ── Line strip connecting waypoints ──────────────────────────────
    if (waypoints.size() > 1) {
        visualization_msgs::msg::Marker line;
        line.header.frame_id = frame_id_;
        line.header.stamp = stamp;
        line.ns = "wp_path";
        line.id = id++;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.1;  // line width
        line.color.r = 0.0f; line.color.g = 0.8f; line.color.b = 1.0f; line.color.a = 0.6f;
        line.lifetime = rclcpp::Duration(0, 0);

        for (const auto& wp : waypoints) {
            geometry_msgs::msg::Point p;
            p.x = wp.x;
            p.y = wp.y;
            p.z = 0.2;
            line.points.push_back(p);
        }
        ma.markers.push_back(line);
    }

    last_count_ = id;
    pub_->publish(ma);
}

}  // namespace path_planning
