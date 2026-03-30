#include <chrono>
#include <cmath>
#include <algorithm>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>

namespace control
{

double pid_update(
  const double error,
  const double dt,
  const double kp,
  const double ki,
  const double kd,
  const double i_min,
  const double i_max,
  double & integral,
  double & prev_error);

class LaneControllerNode : public rclcpp::Node
{
public:
  LaneControllerNode()
  : Node("lane_controller")
  {
    cmd_topic_ = this->declare_parameter<std::string>("cmd_topic", "/prius/cmd_vel");
    kp_ = this->declare_parameter<double>("kp", 0.4);
    ki_ = this->declare_parameter<double>("ki", 0.005);
    kd_ = this->declare_parameter<double>("kd", 0.02);
    target_speed_ = this->declare_parameter<double>("target_speed", 0.6);
    speed_sign_ = this->declare_parameter<double>("speed_sign", 1.0);
    max_angular_ = this->declare_parameter<double>("max_angular", 0.8);
    heading_gain_ = this->declare_parameter<double>("heading_gain", 0.2);
    // ROS convention: +angular.z is CCW (turn left). Our lane errors are defined such that
    // +lateral_error means lane center is to the right of the image center (need to turn right).
    // Therefore steer_sign should match the actual steering direction needed.
    steer_sign_ = this->declare_parameter<double>("steer_sign", 1.0);
    error_alpha_ = this->declare_parameter<double>("error_alpha", 0.15);
    min_speed_factor_ = this->declare_parameter<double>("min_speed_factor", 0.05);
    omega_speed_gain_ = this->declare_parameter<double>("omega_speed_gain", 0.8);
    log_control_ = this->declare_parameter<bool>("log_control", false);

    lane_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/lane/error", 10,
      std::bind(&LaneControllerNode::on_lane_error, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&LaneControllerNode::on_timer, this));

    last_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "lane_controller started, cmd_topic=%s", cmd_topic_.c_str());
  }

private:
  void on_lane_error(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    lane_valid_ = (msg->z > 0.5);

    // Minimal low-pass filter for smoother control (reduced from 0.15 to 0.05)
    const double a = std::clamp(error_alpha_, 0.0, 1.0);
    filt_lateral_error_ = (1.0 - a) * filt_lateral_error_ + a * msg->x;
    filt_heading_error_ = (1.0 - a) * filt_heading_error_ + a * msg->y;

    last_lateral_error_ = std::clamp(filt_lateral_error_, -1.0, 1.0);
    last_heading_error_ = std::clamp(filt_heading_error_, -1.0, 1.0);
    have_lane_ = true;

    if (!lane_valid_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Lane not detected. Using fallback error (z=%.2f)", msg->z);
    }
  }

  void on_timer()
  {
    geometry_msgs::msg::Twist cmd;

    if (!have_lane_) {
      cmd_pub_->publish(cmd);
      return;
    }

    const rclcpp::Time now = this->now();
    const double dt = (now - last_time_).seconds();
    last_time_ = now;

    // If lane is invalid, gradually reduce speed and zero steering
    if (!lane_valid_) {
      // Reset PID state when lane is lost (prevents stale integral from causing a hard turn on re-acquire).
      integral_ = 0.0;
      prev_error_ = 0.0;

      // Exponential falloff: speed decays to 0
      cmd.linear.x = cmd_speed_fallback_ * 0.8;  // 80% decay per cycle (50ms)
      cmd_speed_fallback_ = cmd.linear.x;
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);
      return;
    }

    // Lane is valid - reset fallback and control normally
    cmd_speed_fallback_ = target_speed_;

    const double lateral_cmd = pid_update(
      last_lateral_error_, dt, kp_, ki_, kd_, -2.0, 2.0, integral_, prev_error_);

    // Steering: combine lateral error correction with heading error for curves
    // Negate heading_gain to flip sign (heading error sign convention fix)
    double omega = lateral_cmd - heading_gain_ * last_heading_error_;
    omega *= steer_sign_;  // Apply sign correction (1.0 or -1.0)

    if (omega > max_angular_) {
      omega = max_angular_;
    } else if (omega < -max_angular_) {
      omega = -max_angular_;
    }

    // Speed reduction based on lateral error and steering magnitude (prevents spin/U-turn behavior).
    const double min_sf = std::clamp(min_speed_factor_, 0.0, 1.0);
    const double lat_term = std::abs(last_lateral_error_);
    const double omg_term = (max_angular_ > 1e-6) ? (std::abs(omega) / max_angular_) : 1.0;
    const double slow = std::clamp(1.0 - std::max(lat_term, omega_speed_gain_ * omg_term), min_sf, 1.0);

    // Prius Gazebo model uses linear.y for forward/backward (not linear.x)
    // Negative because the vehicle orientation in Gazebo is rotated
    cmd.linear.y = -speed_sign_ * target_speed_ * slow;
    // Never allow accidental reverse unless explicitly requested via speed_sign.
    if (speed_sign_ >= 0.0) {
      cmd.linear.y = std::min(0.0, cmd.linear.y);  // Negative for forward
    }
    cmd.angular.z = omega;

    if (log_control_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "err(lat=%.3f head=%.3f valid=%d) cmd(v=%.3f w=%.3f)",
        last_lateral_error_, last_heading_error_, lane_valid_ ? 1 : 0,
        cmd.linear.y, cmd.angular.z);
    }

    cmd_pub_->publish(cmd);
  }

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr lane_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_time_;
  double last_lateral_error_{0.0};
  double last_heading_error_{0.0};
  double integral_{0.0};
  double prev_error_{0.0};
  double cmd_speed_fallback_{0.0};  // For smooth deceleration when lane is lost

  double kp_{0.4};
  double ki_{0.005};
  double kd_{0.02};
  double target_speed_{0.6};
  double speed_sign_{1.0};
  double max_angular_{0.8};
  double heading_gain_{0.2};
  double steer_sign_{1.0};
  double error_alpha_{0.15};
  double min_speed_factor_{0.05};
  double omega_speed_gain_{0.8};
  bool log_control_{false};
  std::string cmd_topic_{"/prius/cmd_vel"};

  bool have_lane_{false};
  bool lane_valid_{false};
  double filt_lateral_error_{0.0};
  double filt_heading_error_{0.0};
};

}  // namespace control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control::LaneControllerNode>());
  rclcpp::shutdown();
  return 0;
}
