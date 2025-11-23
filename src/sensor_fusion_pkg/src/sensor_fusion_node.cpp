#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_fusion_pkg/kalman_filter.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <chrono>
#include <filesystem>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class SimpleFusionNode : public rclcpp::Node
{
public:
    SimpleFusionNode()
        : Node("simple_fusion_node")
    {
        fused_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/fused_odom", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&SimpleFusionNode::odom_callback, this, std::placeholders::_1));

        // Kalman Filter initialization (use the KalmanFilter helper class)
        kf_ = std::make_unique<KalmanFilter>(4, 2);
        VectorXd x0 = VectorXd::Zero(4); // [x, y, vx, vy]
        MatrixXd P0 = MatrixXd::Identity(4,4);
        kf_->init(x0, P0);

        dt_ = 0.05; // assume 20 Hz

        // Constant velocity model
        F_ = MatrixXd::Identity(4,4);
        F_(0,2) = dt_;
        F_(1,3) = dt_;

        Q_ = 0.5 * MatrixXd::Identity(4,4);   // process noise
        H_ = MatrixXd::Zero(2,4);
        H_(0,0) = 1; H_(1,1) = 1;

        // Make measurement noise configurable so you can tune how much the
        // filter trusts incoming odometry. Larger R => less trust in measurements
        // and smoother fused output.
        double measurement_noise = this->declare_parameter("measurement_noise", 0.5);
        R_ = measurement_noise * MatrixXd::Identity(2,2);   // measurement noise

        // Pre-allocate reusable Eigen objects to avoid allocations per callback
        z_.resize(2);

        // Prepare CSV logging (append mode). Default path is workspace root under HOME/autoCar_ws
        try {
            log_path_ = std::filesystem::path(std::getenv("HOME")) / "autoCar_ws" / "fusion_log.csv";
        } catch (...) {
            log_path_ = std::filesystem::path("fusion_log.csv");
        }
        // Overwrite existing log on each run: open in truncate mode so we start
        // with a fresh file containing only the header. This avoids appending
        // and prevents mixing runs.
        log_file_.open(log_path_, std::ios::out | std::ios::trunc);
        if (log_file_.is_open()) {
            log_file_ << "time,odom_x,odom_y,fused_x,fused_y,lidar_avg\n";
            log_file_.flush();
        }
        write_count_ = 0;

        RCLCPP_INFO(this->get_logger(), "SimpleFusionNode started, logging to: %s", log_path_.c_str());
    }

    ~SimpleFusionNode() override
    {
        if (log_file_.is_open()) {
            log_file_.flush();
            log_file_.close();
        }
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Measurement (reuse preallocated vector)
        z_(0) = msg->pose.pose.position.x;
        z_(1) = msg->pose.pose.position.y;

        // --- Predict ---
        kf_->predict(F_, Q_);

        // --- Update ---
        kf_->update(z_, H_, R_);

        VectorXd x = kf_->state();

        // --- Publish fused odom ---
        nav_msgs::msg::Odometry fused;
        fused.header.stamp = this->now();
        fused.header.frame_id = "map";
        fused.pose.pose.position.x = x(0);
        fused.pose.pose.position.y = x(1);
        fused.pose.pose.position.z = 0.0;

        fused.twist.twist.linear.x = x(2);
        fused.twist.twist.linear.y = x(3);

        fused_pub_->publish(fused);

        // Log to CSV: timestamp (ms since epoch), odom_x, odom_y, fused_x, fused_y, lidar_avg (empty)
        if (log_file_.is_open()) {
            auto now = std::chrono::system_clock::now();
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            log_file_ << ms << "," << z_(0) << "," << z_(1) << "," << x(0) << "," << x(1) << "," << "" << "\n";
            // Don't flush every line; flush periodically to reduce IO overhead
            if (++write_count_ >= 50) {
                log_file_.flush();
                write_count_ = 0;
            }
        }
    }

    std::unique_ptr<KalmanFilter> kf_;
    MatrixXd F_, Q_, H_, R_;
    double dt_;

    VectorXd z_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::ofstream log_file_;
    std::filesystem::path log_path_;
    int write_count_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleFusionNode>());
    rclcpp::shutdown();
    return 0;
}
