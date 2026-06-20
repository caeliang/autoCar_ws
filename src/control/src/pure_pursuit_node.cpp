/// @file pure_pursuit_node.cpp
/// @brief Pure Pursuit v6 — body-frame corrected, path-based
///
/// ★ KRİTİK: Prius modelinin burnu -Y yönünde (gövde çerçevesinde).
///   - İleri hareket: cmd.linear.y = -hız  (linear.x DEĞİL!)
///   - Odom yaw → harita yaw: car_heading = odom_yaw − π/2
///   - angular.z: standart ROS (pozitif = sola dönüş) — doğru
///
/// Heading:  /prius/odom'dan (düzeltme uygulanır: −π/2)
/// Pozisyon: /localization/pose'dan (ICP — harita çerçevesinde)
/// Yol:      /waypoints/path (nav_msgs/Path — manager'dan)

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace control {

static double normalize_angle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode()
        : Node("pure_pursuit")
    {
        // ── Parametreler ────────────────────────────────────────────
        declare_parameter<double>("max_speed", 2.0);
        declare_parameter<double>("min_speed", 0.3);
        declare_parameter<double>("straight_speed", 2.0);
        declare_parameter<double>("turn_speed", 0.7);
        declare_parameter<double>("min_lookahead", 3.0);
        declare_parameter<double>("max_lookahead", 7.0);
        declare_parameter<double>("lookahead_ratio", 2.0);
        declare_parameter<double>("max_omega", 0.7);
        declare_parameter<double>("speed_filter_alpha", 0.18);
        declare_parameter<double>("curvature_slowdown_gain", 4.0);
        declare_parameter<bool>("debug_enable", true);

        // Yeni global parametreler
        declare_parameter<bool>("global_brake", false);
        declare_parameter<double>("global_speed", get_parameter("straight_speed").as_double());

        max_speed_ = get_parameter("max_speed").as_double();
        min_speed_ = get_parameter("min_speed").as_double();
        straight_speed_ = get_parameter("straight_speed").as_double();
        turn_speed_ = get_parameter("turn_speed").as_double();
        min_la_    = get_parameter("min_lookahead").as_double();
        max_la_    = get_parameter("max_lookahead").as_double();
        la_ratio_  = get_parameter("lookahead_ratio").as_double();
        max_omega_ = get_parameter("max_omega").as_double();
        speed_filter_alpha_ = get_parameter("speed_filter_alpha").as_double();
        curvature_slowdown_gain_ = get_parameter("curvature_slowdown_gain").as_double();
        debug_enable_ = get_parameter("debug_enable").as_bool();

        // ── Subscribers ─────────────────────────────────────────────
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/waypoints/path", rclcpp::QoS(10),
            [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
                path_ = *msg;
                path_ok_ = !path_.poses.empty();
                path_received_ = true;
                waypoint_count_ = static_cast<int>(path_.poses.size());
                if (path_ok_) {
                    const auto& first = path_.poses.front().pose.position;
                    const auto& last = path_.poses.back().pose.position;
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[PP PATH] poses=%d frame='%s' first=(%.3f, %.3f) last=(%.3f, %.3f)",
                        waypoint_count_,
                        path_.header.frame_id.c_str(),
                        first.x, first.y,
                        last.x, last.y);
                } else {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[PP PATH] EMPTY_WAYPOINT_LIST frame='%s'",
                        path_.header.frame_id.c_str());
                }
            });

        // Pozisyon — localizer (harita çerçevesinde)
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/localization/pose", rclcpp::SensorDataQoS(),
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                car_x_ = msg->pose.position.x;
                car_y_ = msg->pose.position.y;
                pose_ok_ = true;
                pose_received_ = true;
                pose_frame_id_ = msg->header.frame_id;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[PP POSE] frame='%s' position=(%.3f, %.3f, %.3f)",
                    pose_frame_id_.c_str(),
                    msg->pose.position.x,
                    msg->pose.position.y,
                    msg->pose.position.z);
            });

        // Heading + hız — odom (yaw'a −π/2 düzeltmesi uygulanır)
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/prius/odom", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                auto& q = msg->pose.pose.orientation;
                double raw_yaw = std::atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                // ★ Prius modeli: burun = −Y body.
                //    model_yaw=0 iken araba güneye bakıyor.
                //    Harita yaw'ı = model_yaw − π/2
                car_yaw_ = normalize_angle(raw_yaw - M_PI_2);

                car_speed_ = std::sqrt(
                    msg->twist.twist.linear.x * msg->twist.twist.linear.x +
                    msg->twist.twist.linear.y * msg->twist.twist.linear.y);
                odom_ok_ = true;
                odom_received_ = true;
                odom_frame_id_ = msg->header.frame_id;
                odom_x_ = msg->pose.pose.position.x;
                odom_y_ = msg->pose.pose.position.y;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[PP ODOM] frame='%s' odom_pos=(%.3f, %.3f) speed=%.3f raw_yaw=%.1fdeg corrected_yaw=%.1fdeg",
                    odom_frame_id_.c_str(),
                    odom_x_, odom_y_, car_speed_,
                    raw_yaw * 180.0 / M_PI,
                    car_yaw_ * 180.0 / M_PI);
            });

        status_sub_ = create_subscription<std_msgs::msg::String>(
            "/waypoints/status", rclcpp::QoS(10).transient_local(),
            [this](std_msgs::msg::String::ConstSharedPtr msg) {
                status_received_ = true;
                last_status_ = msg->data;
                finished_ = (msg->data == "FINISHED" ||
                             msg->data == "NO_WAYPOINTS");
                RCLCPP_WARN(get_logger(),
                    "[PP STATUS] last_status='%s' finished_flag=%s",
                    last_status_.c_str(), finished_ ? "true" : "false");
                if (finished_) {
                    RCLCPP_ERROR(get_logger(),
                        "[PP STATUS] FINISHED_ROUTE set by /waypoints/status='%s'. Vehicle will not move until waypoint manager republishes an active path/status.",
                        last_status_.c_str());
                }
            });

        // ── Publishers ──────────────────────────────────────────────
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/prius/cmd_vel", rclcpp::QoS(10));
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            "/pure_pursuit/lookahead", rclcpp::QoS(1));

        // ── Timer 20 Hz ─────────────────────────────────────────────
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() { control_loop(); });

        RCLCPP_INFO(get_logger(),
            "PurePursuit v6 — body-frame corrected, "
            "straight_speed=%.1f, turn_speed=%.1f, lookahead=[%.1f, %.1f], max_omega=%.2f",
            straight_speed_, turn_speed_, min_la_, max_la_, max_omega_);
    }

private:
    // ═══════════════════════════════════════════════════════════════
    //  ANA KONTROL DÖNGÜSÜ  (Basit Nokta Takibi - A* / Keyboard Mantığı)
    // ═══════════════════════════════════════════════════════════════
    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;  // Varsayılan: Hepsi 0 (DUR)
        tick_++;

        if (!path_received_) {
            publish_stop(cmd, "NO_PATH");
            return;
        }
        if (path_.poses.empty()) {
            publish_stop(cmd, "EMPTY_WAYPOINT_LIST");
            return;
        }
        if (!path_ok_) {
            publish_stop(cmd, "NO_PATH");
            return;
        }
        if (!pose_ok_) {
            publish_stop(cmd, "NO_POSE");
            return;
        }
        if (!odom_ok_) {
            publish_stop(cmd, "NO_ODOM");
            return;
        }
        if (finished_) {
            publish_stop(cmd, "FINISHED_ROUTE");
            return;
        }

        waypoint_count_ = static_cast<int>(path_.poses.size());
        current_waypoint_index_ = find_nearest_on_path();
        if (current_waypoint_index_ < 0) {
            publish_stop(cmd, "EMPTY_WAYPOINT_LIST");
            return;
        }
        nearest_distance_ = std::sqrt(
            std::pow(path_.poses[current_waypoint_index_].pose.position.x - car_x_, 2) +
            std::pow(path_.poses[current_waypoint_index_].pose.position.y - car_y_, 2));

        // 1) Hedef noktayı seç: hızla büyüyen, path uzunluğu üzerinden ölçülen lookahead.
        double target_x = car_x_;
        double target_y = car_y_;
        lookahead_index_ = -1;
        lookahead_distance_ = 0.0;

        double desired_lookahead = std::clamp(
            std::max(min_la_, car_speed_ * la_ratio_),
            min_la_,
            max_la_);

        if (!find_lookahead_point(current_waypoint_index_, desired_lookahead,
                                  target_x, target_y, lookahead_index_,
                                  lookahead_distance_)) {
            target_x = path_.poses.back().pose.position.x;
            target_y = path_.poses.back().pose.position.y;
            lookahead_index_ = static_cast<int>(path_.poses.size()) - 1;
            lookahead_distance_ = std::sqrt(std::pow(target_x - car_x_, 2) + std::pow(target_y - car_y_, 2));
            if (lookahead_distance_ < 0.25) {
                publish_stop(cmd, "GOAL_REACHED");
                return;
            }
        }

        if (lookahead_index_ < 0) {
            publish_stop(cmd, "LOOKAHEAD_NOT_FOUND");
            return;
        }
        
        // 2) Hedef açıyı hesapla (Robot → Target)
        double dx = target_x - car_x_;
        double dy = target_y - car_y_;
        double target_yaw = std::atan2(dy, dx);
        
        double nearest_path_yaw = get_path_yaw(current_waypoint_index_);
        double lookahead_path_yaw = get_path_yaw(lookahead_index_);
        double yaw_error = normalize_angle(nearest_path_yaw - car_yaw_);
        double pure_pursuit_alpha = normalize_angle(target_yaw - car_yaw_);
        double alpha = normalize_angle(0.75 * pure_pursuit_alpha + 0.25 * yaw_error);
        double lateral_error = calc_lateral_error(current_waypoint_index_);
        
        // 4) Komutları üret: eğrilik ve heading hatasına göre hız azalt.
        double k_angular = 1.0;
        
        // Global parametreleri anlık olarak oku
        bool is_braking = get_parameter("global_brake").as_bool();
        double current_global_speed = get_parameter("global_speed").as_double();
        
        // Araç fren yapıyorsa ya da rota bitmişse hızı 0 yap
        if (is_braking || finished_) {
            current_global_speed = 0.0;
        }

        double path_curvature = estimate_path_curvature(current_waypoint_index_);
        double curvature_blend = std::clamp(path_curvature * curvature_slowdown_gain_, 0.0, 1.0);
        double curvature_speed = straight_speed_
            - curvature_blend * (straight_speed_ - turn_speed_);
        double heading_blend = std::clamp(std::max(std::abs(alpha), std::abs(yaw_error)) / 0.9, 0.0, 1.0);
        double heading_speed = straight_speed_
            - heading_blend * (straight_speed_ - turn_speed_);

        double speed = std::min({current_global_speed, max_speed_, curvature_speed, heading_speed});
        if (speed > 0.0) {
            speed = std::max(speed, min_speed_);
        }

        if (!filtered_speed_initialized_) {
            filtered_speed_ = speed;
            filtered_speed_initialized_ = true;
        } else {
            double alpha_filter = std::clamp(speed_filter_alpha_, 0.01, 1.0);
            filtered_speed_ = filtered_speed_ + alpha_filter * (speed - filtered_speed_);
        }
        speed = filtered_speed_;

        // Açıyı orantısal katsayı ile z ye bas
        double angular_z = k_angular * alpha;
        
        // Dönüşü limitle
        angular_z = std::clamp(angular_z, -max_omega_, max_omega_);

        if (is_braking) {
            filtered_speed_ = 0.0;
            publish_stop(cmd, "VEHICLE_STOPPED_BY_CONDITION");
            return;
        }
        
        // Prius aracı modeli için KRİTİK: İleri yön Y ekseninde negatiftir (cmd.linear.y = -hız)
        cmd.linear.x = 0.0;
        cmd.linear.y = -speed;
        cmd.angular.z = angular_z;
        
        last_target_x_ = target_x;
        last_target_y_ = target_y;
        last_alpha_ = alpha;
        last_target_speed_ = speed;
        last_output_omega_ = angular_z;
        last_cross_track_error_ = nearest_distance_;
        last_heading_error_ = alpha;
        last_path_curvature_ = path_curvature;
        last_nearest_path_yaw_ = nearest_path_yaw;
        last_lookahead_path_yaw_ = lookahead_path_yaw;
        last_yaw_error_ = yaw_error;
        last_lateral_error_ = lateral_error;
        parse_pose_metadata(current_waypoint_index_, last_nearest_lane_id_, last_nearest_direction_group_);
        parse_pose_metadata(lookahead_index_, last_lookahead_lane_id_, last_lookahead_direction_group_);
        log_command_debug("RUNNING", target_x, target_y, speed, angular_z, alpha);
        cmd_pub_->publish(cmd);

        // Hedeflenen noktayı RVIZ için çizdir
        publish_marker(target_x, target_y);

        // ── Debug ───────────────────────────────────────────────────
        if (tick_ % 20 == 0) {
            RCLCPP_INFO(get_logger(),
                "[Basit Takip] Pos(%.1f, %.1f) | Hedef(%.1f, %.1f) | car_yaw: %.0f° path_yaw: %.0f° target_yaw: %.0f° | alpha: %.0f° lat_err: %.2f | v: %.1f, w: %.2f",
                car_x_, car_y_, target_x, target_y,
                car_yaw_ * 180.0 / M_PI,
                nearest_path_yaw * 180.0 / M_PI,
                target_yaw * 180.0 / M_PI,
                alpha * 180.0 / M_PI,
                lateral_error,
                speed, angular_z);
        }
    }

    void publish_stop(const geometry_msgs::msg::Twist& cmd, const std::string& reason)
    {
        last_stop_reason_ = reason;
        log_health_report(reason);
        cmd_pub_->publish(cmd);
    }

    void log_health_report(const std::string& reason)
    {
        if (!debug_enable_) return;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "[PP STOP] reason=%s | PATH_OK=%s POSE_OK=%s ODOM_OK=%s STATUS_OK=%s LOOKAHEAD_OK=%s PURE_PURSUIT_RUNNING=false | "
            "path_received=%s pose_received=%s odom_received=%s status_received=%s finished_flag=%s waypoint_count=%d "
            "current_waypoint_index=%d nearest_distance=%.3f lookahead_index=%d lookahead_distance=%.3f "
            "nearest_lane='%s' nearest_group='%s' lookahead_lane='%s' lookahead_group='%s' last_status='%s'",
            reason.c_str(),
            path_ok_ ? "true" : "false",
            pose_ok_ ? "true" : "false",
            odom_ok_ ? "true" : "false",
            (!finished_) ? "true" : "false",
            (lookahead_index_ >= 0) ? "true" : "false",
            path_received_ ? "true" : "false",
            pose_received_ ? "true" : "false",
            odom_received_ ? "true" : "false",
            status_received_ ? "true" : "false",
            finished_ ? "true" : "false",
            waypoint_count_,
            current_waypoint_index_,
            nearest_distance_,
            lookahead_index_,
            lookahead_distance_,
            last_nearest_lane_id_.c_str(),
            last_nearest_direction_group_.c_str(),
            last_lookahead_lane_id_.c_str(),
            last_lookahead_direction_group_.c_str(),
            last_status_.c_str());
    }

    void log_command_debug(const std::string& state,
                           double target_x,
                           double target_y,
                           double linear_speed,
                           double angular_speed,
                           double alpha)
    {
        if (!debug_enable_) return;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[PP CMD] state=%s | PATH_OK=true POSE_OK=true ODOM_OK=true STATUS_OK=%s LOOKAHEAD_OK=true PURE_PURSUIT_RUNNING=true | "
            "current_pose=(%.3f, %.3f, %.1fdeg) target_pose=(%.3f, %.3f) "
            "cross_track_error=%.3f lateral_error=%.3f heading_error=%.1fdeg yaw_error=%.1fdeg "
            "nearest_wp_yaw=%.1fdeg vehicle_yaw=%.1fdeg lookahead_path_yaw=%.1fdeg lookahead_distance=%.3f "
            "nearest_index=%d lookahead_index=%d waypoint_count=%d target_speed=%.3f output_omega=%.3f alpha=%.1fdeg "
            "nearest_lane='%s' nearest_direction_group='%s' lookahead_lane='%s' lookahead_direction_group='%s' "
            "cmd.linear.y=%.3f cmd.angular.z=%.3f last_status='%s'",
            state.c_str(),
            (!finished_) ? "true" : "false",
            car_x_, car_y_, car_yaw_ * 180.0 / M_PI,
            target_x, target_y,
            nearest_distance_,
            last_lateral_error_,
            alpha * 180.0 / M_PI,
            last_yaw_error_ * 180.0 / M_PI,
            last_nearest_path_yaw_ * 180.0 / M_PI,
            car_yaw_ * 180.0 / M_PI,
            last_lookahead_path_yaw_ * 180.0 / M_PI,
            lookahead_distance_,
            current_waypoint_index_,
            lookahead_index_,
            waypoint_count_,
            linear_speed,
            angular_speed,
            alpha * 180.0 / M_PI,
            last_nearest_lane_id_.c_str(),
            last_nearest_direction_group_.c_str(),
            last_lookahead_lane_id_.c_str(),
            last_lookahead_direction_group_.c_str(),
            -linear_speed,
            angular_speed,
            last_status_.c_str());
    }

    // ── Path üzerinde en yakın noktayı bul ──────────────────────────
    int find_nearest_on_path()
    {
        int best = -1;
        double best_d2 = 1e18;
        int n = static_cast<int>(path_.poses.size());

        for (int i = 0; i < n; ++i) {
            double dx = path_.poses[i].pose.position.x - car_x_;
            double dy = path_.poses[i].pose.position.y - car_y_;
            double d2 = dx * dx + dy * dy;
            if (d2 < best_d2) {
                best_d2 = d2;
                best = i;
            }
        }
        return best;
    }

    // ── Path'in nearest noktasındaki teğet yönü ─────────────────────
    double get_path_yaw(int idx)
    {
        if (idx < 0 || idx >= static_cast<int>(path_.poses.size())) return 0.0;
        // Path pose'undaki orientation'dan yaw çıkar
        auto& q = path_.poses[idx].pose.orientation;
        return std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    double calc_lateral_error(int idx)
    {
        if (idx < 0 || idx >= static_cast<int>(path_.poses.size())) return 0.0;
        double path_yaw = get_path_yaw(idx);
        double dx = car_x_ - path_.poses[idx].pose.position.x;
        double dy = car_y_ - path_.poses[idx].pose.position.y;
        return -std::sin(path_yaw) * dx + std::cos(path_yaw) * dy;
    }

    void parse_pose_metadata(int idx, std::string& lane_id, std::string& direction_group)
    {
        lane_id = "<none>";
        direction_group = "<none>";
        if (idx < 0 || idx >= static_cast<int>(path_.poses.size())) return;
        const std::string meta = path_.poses[idx].header.frame_id;
        size_t first = meta.find('|');
        if (first == std::string::npos) return;
        size_t second = meta.find('|', first + 1);
        if (second == std::string::npos) return;
        lane_id = meta.substr(first + 1, second - first - 1);
        direction_group = meta.substr(second + 1);
        if (lane_id.empty()) lane_id = "<none>";
        if (direction_group.empty()) direction_group = "<none>";
    }

    // ── Path boyunca yürüyerek lookahead noktası bul ────────────────
    bool find_lookahead_point(int from_idx, double la_dist,
                              double& out_x, double& out_y,
                              int& out_idx, double& out_dist)
    {
        double accumulated = 0.0;
        int n = static_cast<int>(path_.poses.size());

        if (from_idx >= 0 && from_idx < n) {
            double dx0 = path_.poses[from_idx].pose.position.x - car_x_;
            double dy0 = path_.poses[from_idx].pose.position.y - car_y_;
            accumulated = std::sqrt(dx0 * dx0 + dy0 * dy0);
            if (accumulated >= la_dist) {
                out_x = path_.poses[from_idx].pose.position.x;
                out_y = path_.poses[from_idx].pose.position.y;
                out_idx = from_idx;
                out_dist = accumulated;
                return true;
            }
        }

        for (int i = from_idx; i < n - 1; ++i) {
            double dx = path_.poses[i + 1].pose.position.x
                      - path_.poses[i].pose.position.x;
            double dy = path_.poses[i + 1].pose.position.y
                      - path_.poses[i].pose.position.y;
            double seg_len = std::sqrt(dx * dx + dy * dy);

            if (seg_len < 1e-6) continue;

            if (accumulated + seg_len >= la_dist) {
                double t = (la_dist - accumulated) / seg_len;
                out_x = path_.poses[i].pose.position.x + t * dx;
                out_y = path_.poses[i].pose.position.y + t * dy;
                out_idx = i + 1;
                out_dist = la_dist;
                return true;
            }
            accumulated += seg_len;
        }
        return false;
    }

    double estimate_path_curvature(int idx)
    {
        int n = static_cast<int>(path_.poses.size());
        if (n < 5 || idx < 0) return 0.0;

        int i0 = std::clamp(idx, 0, n - 1);
        int i1 = std::clamp(idx + 3, 0, n - 1);
        int i2 = std::clamp(idx + 6, 0, n - 1);
        if (i0 == i1 || i1 == i2) return 0.0;

        double x0 = path_.poses[i0].pose.position.x;
        double y0 = path_.poses[i0].pose.position.y;
        double x1 = path_.poses[i1].pose.position.x;
        double y1 = path_.poses[i1].pose.position.y;
        double x2 = path_.poses[i2].pose.position.x;
        double y2 = path_.poses[i2].pose.position.y;

        double a = std::hypot(x1 - x0, y1 - y0);
        double b = std::hypot(x2 - x1, y2 - y1);
        double c = std::hypot(x2 - x0, y2 - y0);
        if (a < 1e-6 || b < 1e-6 || c < 1e-6) return 0.0;

        double twice_area = std::abs((x1 - x0) * (y2 - y0)
                                   - (y1 - y0) * (x2 - x0));
        return 2.0 * twice_area / (a * b * c);
    }

    // ── Kalan path uzunluğu ─────────────────────────────────────────
    double calc_remaining_length(int from_idx)
    {
        double total = 0.0;
        int n = static_cast<int>(path_.poses.size());
        for (int i = from_idx; i < n - 1; ++i) {
            double dx = path_.poses[i + 1].pose.position.x
                      - path_.poses[i].pose.position.x;
            double dy = path_.poses[i + 1].pose.position.y
                      - path_.poses[i].pose.position.y;
            total += std::sqrt(dx * dx + dy * dy);
        }
        return total;
    }

    // ── RViz: lookahead noktası ─────────────────────────────────────
    void publish_marker(double lx, double ly)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = now();
        m.ns = "lookahead";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = lx;
        m.pose.position.y = ly;
        m.pose.position.z = 0.5;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.5; m.scale.y = 0.5; m.scale.z = 0.5;
        m.color.r = 1.0f; m.color.g = 0.3f;
        m.color.b = 0.0f; m.color.a = 0.9f;
        m.lifetime = rclcpp::Duration::from_seconds(0.2);
        marker_pub_->publish(m);
    }

    // ── Parametreler ────────────────────────────────────────────────
    double max_speed_, min_speed_;
    double straight_speed_, turn_speed_;
    double min_la_, max_la_, la_ratio_;
    double max_omega_;
    double speed_filter_alpha_;
    double curvature_slowdown_gain_;
    bool debug_enable_ = true;

    // ── Durum ───────────────────────────────────────────────────────
    double car_x_ = 0, car_y_ = 0, car_yaw_ = 0, car_speed_ = 0;
    double odom_x_ = 0, odom_y_ = 0;
    bool pose_ok_ = false, odom_ok_ = false, path_ok_ = false;
    bool path_received_ = false, pose_received_ = false, odom_received_ = false;
    bool status_received_ = false;
    bool finished_ = false;
    int tick_ = 0;
    int waypoint_count_ = 0;
    int current_waypoint_index_ = -1;
    int lookahead_index_ = -1;
    double nearest_distance_ = std::numeric_limits<double>::quiet_NaN();
    double lookahead_distance_ = 0.0;
    double last_target_x_ = 0.0, last_target_y_ = 0.0;
    double last_alpha_ = 0.0;
    double last_target_speed_ = 0.0;
    double last_output_omega_ = 0.0;
    double last_cross_track_error_ = 0.0;
    double last_heading_error_ = 0.0;
    double last_path_curvature_ = 0.0;
    double last_nearest_path_yaw_ = 0.0;
    double last_lookahead_path_yaw_ = 0.0;
    double last_yaw_error_ = 0.0;
    double last_lateral_error_ = 0.0;
    double filtered_speed_ = 0.0;
    bool filtered_speed_initialized_ = false;
    std::string pose_frame_id_;
    std::string odom_frame_id_;
    std::string last_nearest_lane_id_ = "<none>";
    std::string last_nearest_direction_group_ = "<none>";
    std::string last_lookahead_lane_id_ = "<none>";
    std::string last_lookahead_direction_group_ = "<none>";
    std::string last_status_ = "<none>";
    std::string last_stop_reason_ = "<none>";
    nav_msgs::msg::Path path_;

    // ── ROS ─────────────────────────────────────────────────────────
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<control::PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
