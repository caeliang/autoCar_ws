#include <algorithm>

#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace path_planning
{

class LanePathFollower : public rclcpp::Node
{
public:
  LanePathFollower()
  : Node("lane_path_follower")
  {
    lane_weight_ = this->declare_parameter<double>("lane_weight", 0.8);
    path_weight_ = this->declare_parameter<double>("path_weight", 0.2);

    lane_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/lane/error", 10,
      std::bind(&LanePathFollower::on_lane, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/waypoints/path", 10,
      std::bind(&LanePathFollower::on_path, this, std::placeholders::_1));

    fused_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/lane/fused_error", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&LanePathFollower::on_timer, this));
  }

private:
  void on_lane(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    lane_error_ = *msg;
    have_lane_ = true;
  }

  void on_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    path_ = *msg;
    have_path_ = !path_.poses.empty();
  }

  void on_timer()
  {
    if (!have_lane_ && !have_path_) {
      return;
    }

    geometry_msgs::msg::Vector3 fused;
    double path_lateral = 0.0;
    double path_heading = 0.0;

    if (have_path_ && !path_.poses.empty()) {
      const auto & p = path_.poses.front().pose.position;
      path_lateral = p.y;
      path_heading = 0.0;
    }

    fused.x = lane_weight_ * lane_error_.x + path_weight_ * path_lateral;
    fused.y = lane_weight_ * lane_error_.y + path_weight_ * path_heading;
    fused.z = 0.0;

    fused_pub_->publish(fused);
  }

  double lane_weight_{0.8};
  double path_weight_{0.2};

  bool have_lane_{false};
  bool have_path_{false};

  geometry_msgs::msg::Vector3 lane_error_;
  nav_msgs::msg::Path path_;

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr lane_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr fused_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace path_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_planning::LanePathFollower>());
  rclcpp::shutdown();
  return 0;
}
