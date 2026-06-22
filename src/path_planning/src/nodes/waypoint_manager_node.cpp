/// @file waypoint_manager_node.cpp
/// @brief Waypoint Manager v6 — basit sıralı takip
///
/// CSV'deki waypoint'leri sırayla (index 0, 1, 2, ... N) takip eder.
/// Segment tespiti YOK — CSV zaten sürekli bir rota içerir
/// (kavşak ark waypoint'leri dahil).
///
/// Loop modunda: son waypoint'ten sonra ilk waypoint'e döner.
///
/// Yayınlar:
///   /waypoints/path     (nav_msgs/Path)     — ilerideki yol (pure pursuit için)
///   /waypoints/markers  (MarkerArray)       — RViz görselleştirme
///   /waypoints/status   (String)            — durum bilgisi
///
/// Dinler:
///   /localization/pose  (PoseStamped)       — araç konumu (ICP)
///   /prius/odom         (Odometry)          — araç yaw (sim)

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cmath>
#include <algorithm>

#include "path_planning/waypoint.hpp"
#include "path_planning/waypoint_io.hpp"
#include "path_planning/waypoint_visualizer.hpp"

namespace path_planning {

static double normalize_angle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

class WaypointManagerNode : public rclcpp::Node {
public:
    WaypointManagerNode()
        : Node("waypoint_manager")
    {
        declare_parameter<std::string>("waypoint_file", "");
        declare_parameter<std::string>("frame_id", "map");
        declare_parameter<double>("goal_tolerance", 1.5);
        declare_parameter<bool>("loop", true);
        declare_parameter<double>("publish_rate", 10.0);
        declare_parameter<int>("path_horizon", 50);
        declare_parameter<bool>("start_from_first", true);
        declare_parameter<int>("resync_window", 120);
        declare_parameter<double>("resync_distance", 3.0);

        frame_id_     = get_parameter("frame_id").as_string();
        goal_tol_     = get_parameter("goal_tolerance").as_double();
        loop_         = get_parameter("loop").as_bool();
        path_horizon_ = get_parameter("path_horizon").as_int();
        start_from_first_ = get_parameter("start_from_first").as_bool();
        resync_window_ = get_parameter("resync_window").as_int();
        resync_distance_ = get_parameter("resync_distance").as_double();

        load_waypoints();
        create_interfaces();

        RCLCPP_INFO(get_logger(),
            "WaypointManager v6 — %zu waypoint, loop=%s, tol=%.1fm",
            waypoints_.size(), loop_ ? "true" : "false", goal_tol_);
    }

private:
    // ═══════════════════════════════════════════════════════════════
    //  YÜKLEME
    // ═══════════════════════════════════════════════════════════════
    void load_waypoints()
    {
        std::string file = get_parameter("waypoint_file").as_string();
        if (file.empty()) {
            RCLCPP_WARN(get_logger(), "waypoint_file boş!");
            return;
        }
        try {
            bool is_csv = (file.size() >= 4 &&
                file.substr(file.size() - 4) == ".csv");
            waypoints_ = is_csv ? loadWaypointsCSV(file)
                                : loadWaypoints(file);
            RCLCPP_INFO(get_logger(), "✓ %zu waypoint yüklendi: %s",
                        waypoints_.size(), file.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Dosya okunamadı: %s", e.what());
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  ROS ARAYÜZLER
    // ═══════════════════════════════════════════════════════════════
    void create_interfaces()
    {
        viz_ = std::make_unique<WaypointVisualizer>(
            this, "/waypoints/markers", frame_id_);

        // Pozisyon — localizer
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/localization/pose", rclcpp::SensorDataQoS(),
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                car_x_ = msg->pose.position.x;
                car_y_ = msg->pose.position.y;
                pose_ok_ = true;
            });

        // Heading — odom (★ Prius: burun = −Y body, düzeltme −π/2)
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/prius/odom", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                auto& q = msg->pose.pose.orientation;
                double raw_yaw = std::atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                car_yaw_ = normalize_angle(raw_yaw - M_PI_2);
                odom_ok_ = true;
            });

        // Publishers
        path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "/waypoints/path", rclcpp::QoS(10));
        status_pub_ = create_publisher<std_msgs::msg::String>(
            "/waypoints/status", rclcpp::QoS(10).transient_local());

        // Reset servisi
        reset_srv_ = create_service<std_srvs::srv::Trigger>(
            "/waypoints/reset",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                current_wp_ = 0;
                finished_   = false;
                initialized_ = false;
                laps_ = 0;
                res->success = true;
                res->message = "Sıfırlandı";
                RCLCPP_INFO(get_logger(), "⟲ Sıfırlandı");
            });

        // Timer
        double rate = get_parameter("publish_rate").as_double();
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            [this]() { update(); });
    }

    // ═══════════════════════════════════════════════════════════════
    //  ANA GÜNCELLEME (10 Hz)
    // ═══════════════════════════════════════════════════════════════
    void update()
    {
        publish_status();

        if (waypoints_.empty() || finished_) return;
        if (!pose_ok_ || !odom_ok_) return;

        // İlk başlatma: en yakın waypoint'i bul
        if (!initialized_) {
            current_wp_ = start_from_first_ ? 0 : find_nearest_ahead();
            initialized_ = true;
            RCLCPP_INFO(get_logger(),
                "▶ WP %d'den başlanıyor (%.1f,%.1f), start_from_first=%s",
                current_wp_, waypoints_[current_wp_].x,
                waypoints_[current_wp_].y,
                start_from_first_ ? "true" : "false");
        }

        // Waypoint ilerleme takibi
        track_progress();

        // Path yayınla
        publish_path();
        publish_viz();
    }

    // ═══════════════════════════════════════════════════════════════
    //  WAYPOINT İLERLEME — sıralı, çoklu-advance
    //
    //  Döngüde ilerler: yakın veya geçilmiş WP'leri TEK ÇEVRİMDE
    //  geçer. Bu sayede 0.3m aralıklı ark WP'lerinde ve başlangıçta
    //  (NW ark yakınında) manager araçtan öne kaçmaz.
    // ═══════════════════════════════════════════════════════════════
    void track_progress()
    {
        resync_to_nearest_forward_waypoint();
        int max_advances = 50;  // sonsuz döngü önleme

        for (int attempt = 0; attempt < max_advances; ++attempt) {
            double dx = waypoints_[current_wp_].x - car_x_;
            double dy = waypoints_[current_wp_].y - car_y_;
            double dist = std::sqrt(dx * dx + dy * dy);

            // Ulaşıldı → ilerle
            if (dist < goal_tol_) {
                advance();
                continue;
            }

            // Geçilmiş mi? (arkada + yakın)
            double local_x = dx * std::cos(car_yaw_)
                            + dy * std::sin(car_yaw_);
            if (local_x < 0 && dist < goal_tol_ * 2.0) {
                advance();
                continue;
            }

            break;  // WP önde ve yeterince uzak → bekle
        }
    }

    void resync_to_nearest_forward_waypoint()
    {
        int n = static_cast<int>(waypoints_.size());
        if (n <= 0) return;

        int best_idx = current_wp_;
        double best_dist = 1e18;
        int window = std::max(1, resync_window_);
        int max_idx = loop_ ? current_wp_ + window : std::min(n - 1, current_wp_ + window);

        for (int raw = current_wp_; raw <= max_idx; ++raw) {
            int idx = loop_ ? (raw % n) : raw;
            double dx = waypoints_[idx].x - car_x_;
            double dy = waypoints_[idx].y - car_y_;
            double d = std::sqrt(dx * dx + dy * dy);
            if (d < best_dist) {
                best_dist = d;
                best_idx = idx;
            }
        }

        if (best_idx != current_wp_ && best_dist < resync_distance_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "↻ Route resync: WP %d -> %d (dist=%.2fm)",
                current_wp_, best_idx, best_dist);
            current_wp_ = best_idx;
        }
    }

    void advance()
    {
        int n = static_cast<int>(waypoints_.size());
        current_wp_++;

        if (current_wp_ >= n) {
            if (loop_) {
                current_wp_ = 0;
                laps_++;
                RCLCPP_INFO(get_logger(), "⟲ Tur %d tamamlandı, devam!", laps_);
            } else {
                current_wp_ = n - 1;
                finished_ = true;
                RCLCPP_INFO(get_logger(), "🏁 Rota tamamlandı!");
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  İLK BAŞLATMA: Araç yönüne uygun en yakın waypoint
    //
    //  ★ Yaw ağırlığı yüksek (5.0): başlangıçta NW ark WP'lerinin
    //    (yaw≈-124°) yerine güney düz WP 0'ın (yaw=-90°) seçilmesini
    //    sağlar. Yaw filtresi: max 25° fark.
    // ═══════════════════════════════════════════════════════════════
    int find_nearest_ahead()
    {
        int best = 0;
        double best_score = 1e9;
        int n = static_cast<int>(waypoints_.size());

        for (int i = 0; i < n; ++i) {
            double dx = waypoints_[i].x - car_x_;
            double dy = waypoints_[i].y - car_y_;
            double dist = std::sqrt(dx * dx + dy * dy);

            // Yaw uyumu kontrolü — sıkı filtre
            double yaw_diff = std::abs(normalize_angle(
                waypoints_[i].yaw - car_yaw_));
            if (yaw_diff > 0.44) continue;  // >25° fark → atla

            // Önde mi?
            double local_x = dx * std::cos(car_yaw_)
                            + dy * std::sin(car_yaw_);
            if (local_x < -1.0) continue;  // gerideki atla

            // Yaw ağırlığı yüksek → tam yaw eşleşmesi tercih edilir
            double score = dist + yaw_diff * 5.0;
            if (score < best_score) {
                best_score = score;
                best = i;
            }
        }
        return best;
    }

    // ═══════════════════════════════════════════════════════════════
    //  PATH YAYINLAMA
    //
    //  current_wp'den başlayarak ilerideki path_horizon WP yayınlanır.
    //  Arama tabanlı track_progress sayesinde current_wp hep
    //  araçla senkron kalır, lookback gerekmiyor.
    // ═══════════════════════════════════════════════════════════════
    void publish_path()
    {
        if (finished_) return;

        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = frame_id_;
        path_msg.header.stamp = now();

        int n = static_cast<int>(waypoints_.size());
        int count = std::min(path_horizon_, n);

        for (int i = 0; i < count; ++i) {
            int idx = (current_wp_ + i) % n;
            auto pose = waypoints_[idx].toPoseStamped(frame_id_, now());
            if (!waypoints_[idx].lane_id.empty() || !waypoints_[idx].direction_group.empty()) {
                pose.header.frame_id = frame_id_ + "|" +
                    waypoints_[idx].lane_id + "|" +
                    waypoints_[idx].direction_group;
            }
            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
    }

    void publish_viz()
    {
        viz_->publish(waypoints_, current_wp_);
    }

    void publish_status()
    {
        std_msgs::msg::String msg;

        if (waypoints_.empty()) {
            msg.data = "NO_WAYPOINTS";
        } else if (finished_) {
            msg.data = "FINISHED";
        } else {
            msg.data = "WP:" + std::to_string(current_wp_) +
                       "/" + std::to_string(waypoints_.size()) +
                       " LAP:" + std::to_string(laps_);
        }
        status_pub_->publish(msg);
    }

    // ── State ───────────────────────────────────────────────────────
    WaypointList waypoints_;
    std::string frame_id_;
    double goal_tol_     = 1.5;
    bool loop_           = true;
    int path_horizon_    = 50;
    bool start_from_first_ = true;
    int resync_window_ = 120;
    double resync_distance_ = 3.0;

    int current_wp_      = 0;
    bool finished_       = false;
    bool initialized_    = false;
    bool pose_ok_ = false, odom_ok_ = false;
    double car_x_ = 0, car_y_ = 0, car_yaw_ = 0;
    int laps_ = 0;

    // ── ROS ─────────────────────────────────────────────────────────
    std::unique_ptr<WaypointVisualizer> viz_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace path_planning

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<path_planning::WaypointManagerNode>());
    rclcpp::shutdown();
    return 0;
}
