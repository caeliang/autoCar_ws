#include <algorithm>
#include <cmath>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace path_planning
{

class PurePursuitNode : public rclcpp::Node
{
public:
  PurePursuitNode()
  : Node("pure_pursuit")
  {
    cmd_topic_ = this->declare_parameter<std::string>("cmd_topic", "/prius/cmd_vel");
    target_speed_ = this->declare_parameter<double>("target_speed", 1.8);
    lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 2.2);
    max_omega_ = this->declare_parameter<double>("max_omega", 1.0);

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/waypoints/path", 10,
      std::bind(&PurePursuitNode::on_path, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&PurePursuitNode::on_timer, this));
  }

private:
  void on_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    path_ = *msg;
    have_path_ = !path_.poses.empty();
  }

  void on_timer()
  {
    geometry_msgs::msg::Twist cmd;
    if (!have_path_) {
      cmd_pub_->publish(cmd);
      return;
    }

    const auto & target_pose = path_.poses[std::min<size_t>(4, path_.poses.size() - 1)].pose;
    const double tx = target_pose.position.x;
    const double ty = target_pose.position.y;

    const double dist = std::hypot(tx, ty);
    if (dist < 1e-3) {
      cmd_pub_->publish(cmd);
      return;
    }

    const double curvature = 2.0 * ty / (lookahead_distance_ * lookahead_distance_);
    cmd.linear.x = target_speed_;
    cmd.angular.z = std::clamp(target_speed_ * curvature, -max_omega_, max_omega_);

    cmd_pub_->publish(cmd);
  }

  double target_speed_{1.8};
  double lookahead_distance_{2.2};
  double max_omega_{1.0};
  std::string cmd_topic_{"/prius/cmd_vel"};

  bool have_path_{false};
  nav_msgs::msg::Path path_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace path_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_planning::PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}
