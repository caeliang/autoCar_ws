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

namespace path_planning {

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
    //  ANA KONTROL DÖNGÜSÜ  (20 Hz)
    // ═══════════════════════════════════════════════════════════════
    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;  // hepsi 0

        if (!pose_ok_ || !odom_ok_ || !path_ok_ || finished_) {
            cmd_pub_->publish(cmd);
            return;
        }

        // 1) Path üzerinde en yakın noktayı bul
        int nearest = find_nearest_on_path();
        if (nearest < 0) {
            cmd_pub_->publish(cmd);
            return;
        }

        // 2) Path yönü (nearest noktasındaki teğet)
        double path_yaw = get_path_yaw(nearest);
        double heading_err = normalize_angle(car_yaw_ - path_yaw);

        // 3) Dinamik lookahead (hıza orantılı)
        double la = std::clamp(car_speed_ * la_ratio_, min_la_, max_la_);

        // 4) Path boyunca lookahead noktası bul
        double la_x, la_y;
        if (!find_lookahead_point(nearest, la, la_x, la_y)) {
            la_x = path_.poses.back().pose.position.x;
            la_y = path_.poses.back().pose.position.y;
        }

        // 5) Araç koordinat sistemine dönüştür
        double dx = la_x - car_x_;
        double dy = la_y - car_y_;
        double local_y = -dx * std::sin(car_yaw_) + dy * std::cos(car_yaw_);
        double alpha = std::atan2(local_y,
                        dx * std::cos(car_yaw_) + dy * std::sin(car_yaw_));

        // ═══════════════════════════════════════════════════════════
        //  KONTROL STRATEJİSİ
        //
        //  ★ KURAL: Heading error küçükse → ASLA DURMA.
        //    Araç yoldan yana kaymışsa alpha>90° olabilir ama
        //    heading doğruysa ileri gidip yola dönmesi lazım.
        //    Dur-dön sadece heading gerçekten yanlışsa yapılır.
        // ═══════════════════════════════════════════════════════════

        double speed = 0.0;
        double omega = 0.0;

        double abs_heading = std::abs(heading_err);

        if (abs_heading > 1.05) {
            // ★ Heading çok yanlış (>60°) → DUR ve DÖN
            speed = 0.0;
            omega = (heading_err > 0 ? -1.0 : 1.0) * max_omega_;
        }
        else if (abs_heading > 0.52) {
            // Heading kısmen yanlış (30°-60°) → YAVAŞ + AGRESIF DÖNÜŞ
            speed = min_speed_;
            omega = -1.5 * heading_err;
            omega = std::clamp(omega, -max_omega_, max_omega_);
        }
        else {
            // ── Heading iyi (<30°) → HER ZAMAN İLERİ GİT ───────────
            //    Alpha ne olursa olsun dur, pure pursuit + heading
            //    düzeltme ile yola geri dön.

            // Lookahead hedefi: path yönünde, araçtan ileride bir nokta
            // (lateral offset varsa nearest yerine ileri projeksiyonu al)
            double target_x, target_y;

            if (std::abs(alpha) > M_PI / 2.0) {
                // Lookahead arkada → path yönünde la metre ileriye hedefle
                target_x = car_x_ + la * std::cos(path_yaw);
                target_y = car_y_ + la * std::sin(path_yaw);
            } else {
                target_x = la_x;
                target_y = la_y;
            }

            double tdx = target_x - car_x_;
            double tdy = target_y - car_y_;
            double t_local_y = -tdx * std::sin(car_yaw_)
                              + tdy * std::cos(car_yaw_);
            double t_dist = std::sqrt(tdx * tdx + tdy * tdy);

            double curvature = (t_dist > 0.01)
                ? 2.0 * t_local_y / (t_dist * t_dist)
                : 0.0;

            speed = max_speed_;

            // Lateral offset büyükse biraz yavaşla
            double nearest_dx = path_.poses[nearest].pose.position.x - car_x_;
            double nearest_dy = path_.poses[nearest].pose.position.y - car_y_;
            double lateral_dist = std::sqrt(nearest_dx * nearest_dx +
                                            nearest_dy * nearest_dy);
            if (lateral_dist > 3.0) {
                speed *= 0.5;
            } else if (lateral_dist > 1.5) {
                speed *= 0.7;
            }

            // Path sonuna yakınsa yavaşla
            double remaining = calc_remaining_length(nearest);
            if (remaining < 4.0) {
                speed *= std::max(0.3, remaining / 4.0);
            }

            speed = std::max(speed, min_speed_);

            // Pure pursuit curvature + heading error düzeltme
            omega = speed * curvature - 0.8 * heading_err;
            omega = std::clamp(omega, -max_omega_, max_omega_);
        }

        // ── cmd_vel gönder ──────────────────────────────────────────
        // ★ Prius: ileri = linear.y negatif (burun = −Y body)
        cmd.linear.x = 0.0;
        cmd.linear.y = -speed;  // negatif: -Y yönünde = ileri
        cmd.angular.z = omega;
        cmd_pub_->publish(cmd);

        publish_marker(la_x, la_y);

        // ── Debug ───────────────────────────────────────────────────
        tick_++;
        if (tick_ % 60 == 0) {
            RCLCPP_INFO(get_logger(),
                "Pos(%.1f,%.1f) yaw=%.0f° path_yaw=%.0f° "
                "h_err=%.0f° alpha=%.0f° v=%.2f w=%.2f",
                car_x_, car_y_,
                car_yaw_ * 180.0 / M_PI,
                path_yaw * 180.0 / M_PI,
                heading_err * 180.0 / M_PI,
                alpha * 180.0 / M_PI,
                speed, omega);
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

}  // namespace path_planning

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<path_planning::PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
