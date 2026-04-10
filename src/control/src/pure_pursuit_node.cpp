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
        declare_parameter<double>("min_lookahead", 1.5);
        declare_parameter<double>("max_lookahead", 5.0);
        declare_parameter<double>("lookahead_ratio", 1.5);
        declare_parameter<double>("max_omega", 1.0);

        // Yeni global parametreler
        declare_parameter<bool>("global_brake", false);
        declare_parameter<double>("global_speed", get_parameter("max_speed").as_double());

        max_speed_ = get_parameter("max_speed").as_double();
        min_speed_ = get_parameter("min_speed").as_double();
        min_la_    = get_parameter("min_lookahead").as_double();
        max_la_    = get_parameter("max_lookahead").as_double();
        la_ratio_  = get_parameter("lookahead_ratio").as_double();
        max_omega_ = get_parameter("max_omega").as_double();

        // ── Subscribers ─────────────────────────────────────────────
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/waypoints/path", rclcpp::QoS(10),
            [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
                path_ = *msg;
                path_ok_ = !path_.poses.empty();
            });

        // Pozisyon — localizer (harita çerçevesinde)
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/localization/pose", rclcpp::SensorDataQoS(),
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                car_x_ = msg->pose.position.x;
                car_y_ = msg->pose.position.y;
                pose_ok_ = true;
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
            });

        status_sub_ = create_subscription<std_msgs::msg::String>(
            "/waypoints/status", rclcpp::QoS(10).transient_local(),
            [this](std_msgs::msg::String::ConstSharedPtr msg) {
                finished_ = (msg->data == "FINISHED" ||
                             msg->data == "NO_WAYPOINTS");
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
            "max_speed=%.1f, max_omega=%.2f",
            max_speed_, max_omega_);
    }

private:
    // ═══════════════════════════════════════════════════════════════
    //  ANA KONTROL DÖNGÜSÜ  (Basit Nokta Takibi - A* / Keyboard Mantığı)
    // ═══════════════════════════════════════════════════════════════
    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;  // Varsayılan: Hepsi 0 (DUR)

        if (!pose_ok_ || !odom_ok_ || !path_ok_ || finished_) {
            cmd_pub_->publish(cmd);
            return;
        }

        // 1) Hedef noktayı seç (manager bize local yolu veriyor, ilk elemanlarımız sıradaki waypointlerimiz)
        // Araca en az 1.5 - 2.0 metre uzaklıktaki ilk waypointi hedef alıyoruz ki yalpalama yapmasın
        double target_x = car_x_;
        double target_y = car_y_;
        bool target_found = false;
        
        for (const auto& pose_stamped : path_.poses) {
            double tx = pose_stamped.pose.position.x;
            double ty = pose_stamped.pose.position.y;
            double dist = std::sqrt(std::pow(tx - car_x_, 2) + std::pow(ty - car_y_, 2));
            
            // Eğer nokta 1.0 metreden uzaksa onu hedef olarak al
            if (dist > min_la_) {
                target_x = tx;
                target_y = ty;
                target_found = true;
                break;
            }
        }
        
        // Eğer hepsi 1.0 metreden yakınsa, sıradaki son noktayı hedefleriz
        if (!target_found && !path_.poses.empty()) {
            target_x = path_.poses.back().pose.position.x;
            target_y = path_.poses.back().pose.position.y;
        }
        
        // 2) Hedef açıyı hesapla (Robot → Target)
        double dx = target_x - car_x_;
        double dy = target_y - car_y_;
        double target_yaw = std::atan2(dy, dx);
        
        // 3) Mevcut açı ile aradaki açı farkı = dönüş komutu (alpha)
        double alpha = normalize_angle(target_yaw - car_yaw_);
        
        // 4) Komutları Üret (Keyboard mantığının otomatik hali)
        double k_angular = 1.5;   // Dönüş katsayısı (ne kadar sert dönsün)
        
        // Global parametreleri anlık olarak oku
        bool is_braking = get_parameter("global_brake").as_bool();
        double current_global_speed = get_parameter("global_speed").as_double();
        
        // Araç fren yapıyorsa ya da rota bitmişse hızı 0 yap
        if (is_braking || finished_) {
            current_global_speed = 0.0;
        }

        double speed = current_global_speed; // Varsayılan: global hız seviyesi
        
        // Eğer açı farkı çok yüksekse (örn: 45 dereceden fazla), hızı düşür ve sert dön,
        // ancak sadece eğer bir hız varsa yap (fren varsa zaten 0)
        if (speed > 0.0) {
            if (std::abs(alpha) > 0.8) {
                speed = std::min(speed, min_speed_); // Köşelerde yavaşla
            } else if (std::abs(alpha) > 0.4) {
                speed = std::min(speed, current_global_speed * 0.6); // Sert dönüşte yavaşla
            }
        }

        // Açıyı orantısal katsayı ile z ye bas
        double angular_z = k_angular * alpha;
        
        // Dönüşü limitle
        angular_z = std::clamp(angular_z, -max_omega_, max_omega_);
        
        // Prius aracı modeli için KRİTİK: İleri yön Y ekseninde negatiftir (cmd.linear.y = -hız)
        cmd.linear.x = 0.0;
        cmd.linear.y = -speed;
        cmd.angular.z = angular_z;
        
        cmd_pub_->publish(cmd);

        // Hedeflenen noktayı RVIZ için çizdir
        publish_marker(target_x, target_y);

        // ── Debug ───────────────────────────────────────────────────
        tick_++;
        if (tick_ % 20 == 0) {
            RCLCPP_INFO(get_logger(),
                "[Basit Takip] Pos(%.1f, %.1f) | Hedef(%.1f, %.1f) | car_yaw: %.0f° target_yaw: %.0f° | alpha: %.0f° | v: %.1f, w: %.2f",
                car_x_, car_y_, target_x, target_y,
                car_yaw_ * 180.0 / M_PI,
                target_yaw * 180.0 / M_PI,
                alpha * 180.0 / M_PI,
                speed, angular_z);
        }
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
        // Path pose'undaki orientation'dan yaw çıkar
        auto& q = path_.poses[idx].pose.orientation;
        return std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    // ── Path boyunca yürüyerek lookahead noktası bul ────────────────
    bool find_lookahead_point(int from_idx, double la_dist,
                              double& out_x, double& out_y)
    {
        double accumulated = 0.0;
        int n = static_cast<int>(path_.poses.size());

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
                return true;
            }
            accumulated += seg_len;
        }
        return false;
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
    double min_la_, max_la_, la_ratio_;
    double max_omega_;

    // ── Durum ───────────────────────────────────────────────────────
    double car_x_ = 0, car_y_ = 0, car_yaw_ = 0, car_speed_ = 0;
    bool pose_ok_ = false, odom_ok_ = false, path_ok_ = false;
    bool finished_ = false;
    int tick_ = 0;
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
