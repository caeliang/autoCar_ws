#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace path_planning {

/// Basit waypoint: sadece x, y, yaw
struct Waypoint {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;  // radyan — aracın gittiği yön
    std::string lane_id;
    std::string direction_group;

    /// 2D mesafe
    double distance2D(const Waypoint& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    /// PoseStamped'e çevir (goal publish için)
    geometry_msgs::msg::PoseStamped toPoseStamped(
        const std::string& frame_id,
        const rclcpp::Time& stamp) const
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = frame_id;
        ps.header.stamp = stamp;
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = 0.0;
        // yaw → quaternion (sadece z ekseni dönüşü)
        ps.pose.orientation.x = 0.0;
        ps.pose.orientation.y = 0.0;
        ps.pose.orientation.z = std::sin(yaw / 2.0);
        ps.pose.orientation.w = std::cos(yaw / 2.0);
        return ps;
    }

    /// PoseStamped'den oluştur
    static Waypoint fromPoseStamped(const geometry_msgs::msg::PoseStamped& ps) {
        Waypoint wp;
        wp.x = ps.pose.position.x;
        wp.y = ps.pose.position.y;
        // quaternion → yaw
        const auto& q = ps.pose.orientation;
        wp.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        return wp;
    }
};

using WaypointList = std::vector<Waypoint>;

}  // namespace path_planning
