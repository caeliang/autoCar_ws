#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_fusion/kalman_filter.hpp"
#include <Eigen/Dense>

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

        // Lidar-based pose (e.g., from scan-matching or localization) subscription
        lidar_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/lidar_odom", 10, std::bind(&SimpleFusionNode::lidar_callback, this, std::placeholders::_1));

    // Kalman Filter initialization (use the KalmanFilter helper class)
    // State vector: [x, y, vx, vy, bx, by] where bx,by are lidar bias/offsets
    kf_ = std::make_unique<KalmanFilter>(6, 4);
        VectorXd x0 = VectorXd::Zero(6); // [x, y, vx, vy, bx, by]
        MatrixXd P0 = MatrixXd::Identity(6,6) * 1.0;
        kf_->init(x0, P0);

        dt_ = 0.05; // assume 20 Hz

        // Constant velocity model with bias states (bx,by) modeled as random walk
        F_ = MatrixXd::Identity(6,6);
        F_(0,2) = dt_;
        F_(1,3) = dt_;
        // bx,by remain (1 on diagonal)

        Q_ = MatrixXd::Zero(6,6);
        double process_pos = this->declare_parameter("process_noise_pos", 0.1);
        double process_vel = this->declare_parameter("process_noise_vel", 0.1);
        double process_bias = this->declare_parameter("process_noise_bias", 1e-3);
        Q_(0,0) = process_pos; Q_(1,1) = process_pos;
        Q_(2,2) = process_vel; Q_(3,3) = process_vel;
        Q_(4,4) = process_bias; Q_(5,5) = process_bias;

    // Measurement model for odom: we measure x,y,vx,vy (no bias terms)
    H_odom_ = MatrixXd::Zero(4,6);
    H_odom_(0,0) = 1; H_odom_(1,1) = 1; H_odom_(2,2) = 1; H_odom_(3,3) = 1;

    // Measurement model for lidar: lidar measures x+bx, y+by
    H_lidar_ = MatrixXd::Zero(2,6);
    H_lidar_(0,0) = 1; H_lidar_(0,4) = 1;
    H_lidar_(1,1) = 1; H_lidar_(1,5) = 1;

    // Measurement noises
    double meas_pos_noise = this->declare_parameter("measurement_noise_pos", 0.5);
    double meas_vel_noise = this->declare_parameter("measurement_noise_vel", 0.5);
    double meas_lidar_pos = this->declare_parameter("measurement_noise_lidar", 0.2);
    R_odom_ = MatrixXd::Zero(4,4);
    R_odom_(0,0) = meas_pos_noise; R_odom_(1,1) = meas_pos_noise;
    R_odom_(2,2) = meas_vel_noise; R_odom_(3,3) = meas_vel_noise;

    R_lidar_ = MatrixXd::Zero(2,2);
    R_lidar_(0,0) = meas_lidar_pos; R_lidar_(1,1) = meas_lidar_pos;

    // Pre-allocate reusable Eigen objects to avoid allocations per callback
    z_odom_.resize(4);
    z_lidar_.resize(2);

        RCLCPP_INFO(this->get_logger(), "SimpleFusionNode started - pure sensor fusion (no logging)");
    }

    ~SimpleFusionNode() override
    {
        RCLCPP_INFO(this->get_logger(), "SimpleFusionNode shutting down");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Measurement (reuse preallocated vector): include linear velocity from odom twist
        z_odom_(0) = msg->pose.pose.position.x;
        z_odom_(1) = msg->pose.pose.position.y;
        z_odom_(2) = msg->twist.twist.linear.x;
        z_odom_(3) = msg->twist.twist.linear.y;

        // --- Predict ---
        kf_->predict(F_, Q_);

        // --- Update with odom measurement ---
        kf_->update(z_odom_, H_odom_, R_odom_);

        VectorXd x = kf_->state();

        // Publish fused odom
        publish_fused(x);
    }

    void lidar_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Build lidar measurement z = [x_lidar, y_lidar]
        z_lidar_(0) = msg->pose.pose.position.x;
        z_lidar_(1) = msg->pose.pose.position.y;

        // Predict then update with lidar measurement
        kf_->predict(F_, Q_);
        kf_->update(z_lidar_, H_lidar_, R_lidar_);

        VectorXd x = kf_->state();

        // publish fused after lidar update
        publish_fused(x);
    }

    std::unique_ptr<KalmanFilter> kf_;
    MatrixXd F_, Q_;
    MatrixXd H_odom_, H_lidar_;
    MatrixXd R_odom_, R_lidar_;
    double dt_;

    VectorXd z_odom_, z_lidar_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_sub_;

    // helper to publish fused odom from state vector (reuse message to avoid allocations)
    nav_msgs::msg::Odometry fused_msg_;
    void publish_fused(const VectorXd &x) {
        // update header and fields in-place
        fused_msg_.header.stamp = this->now();
        fused_msg_.header.frame_id = "map";
        fused_msg_.pose.pose.position.x = x(0);
        fused_msg_.pose.pose.position.y = x(1);
        fused_msg_.pose.pose.position.z = 0.0;
        fused_msg_.twist.twist.linear.x = x(2);
        fused_msg_.twist.twist.linear.y = x(3);
        fused_pub_->publish(fused_msg_);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleFusionNode>());
    rclcpp::shutdown();
    return 0;
}
