#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace perception
{

bool detect_lane(
  const cv::Mat & bgr,
  double & lateral_error,
  double & heading_error,
  cv::Mat * debug_out = nullptr);

class LaneDetectorNode : public rclcpp::Node
{
public:
  LaneDetectorNode()
  : Node("lane_detector")
  {
    image_topic_ = this->declare_parameter<std::string>("image_topic", "/prius/front_camera/image_raw");
    image_qos_depth_ = this->declare_parameter<int>("image_qos_depth", 10);
    image_qos_reliable_ = this->declare_parameter<bool>("image_qos_reliable", true);

    publish_zero_on_failure_ = this->declare_parameter<bool>("publish_zero_on_failure", true);
    show_debug_ = this->declare_parameter<bool>("show_debug", false);
    show_debug_window_ = this->declare_parameter<bool>("show_debug_window", show_debug_);
    publish_debug_image_ = this->declare_parameter<bool>("publish_debug_image", show_debug_);
    debug_image_topic_ = this->declare_parameter<std::string>("debug_image_topic", "/lane/debug_image");

    rclcpp::QoS image_qos(rclcpp::KeepLast(std::max(1, image_qos_depth_)));
    if (image_qos_reliable_) {
      image_qos.reliable();
    } else {
      image_qos.best_effort();
    }
    image_qos.durability_volatile();

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, image_qos,
      std::bind(&LaneDetectorNode::on_image, this, std::placeholders::_1));

    lane_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/lane/error", 10);

    if (publish_debug_image_) {
      // Best-effort is usually fine for debug visualization.
      debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, rclcpp::SensorDataQoS());
    }

    if (show_debug_window_) {
      init_debug_window();
    }

    RCLCPP_INFO(this->get_logger(), "lane_detector started, image_topic=%s", image_topic_.c_str());
    RCLCPP_INFO(
      this->get_logger(), "image_qos: depth=%d reliability=%s",
      image_qos_depth_, image_qos_reliable_ ? "reliable" : "best_effort");
    if (show_debug_) {
      RCLCPP_INFO(
        this->get_logger(), "debug: window=%s topic=%s publish_debug_image=%s",
        show_debug_window_ ? "on" : "off", debug_image_topic_.c_str(), publish_debug_image_ ? "true" : "false");
    }
  }

  ~LaneDetectorNode() override
  {
    if (debug_window_initialized_) {
      try {
        cv::destroyWindow(kDebugWindowName);
      } catch (...) {
        // ignore UI backend shutdown errors
      }
    }
  }

private:
  void init_debug_window()
  {
    if (debug_window_initialized_) {
      return;
    }
    try {
      cv::namedWindow(kDebugWindowName, cv::WINDOW_NORMAL);
      cv::resizeWindow(kDebugWindowName, 960, 540);
      debug_window_initialized_ = true;
    } catch (const cv::Exception & e) {
      RCLCPP_WARN(this->get_logger(), "OpenCV window init failed: %s", e.what());
      debug_window_initialized_ = false;
    }
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv::Mat bgr;
    try {
      if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
        bgr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
        cv::Mat rgb = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      } else if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
        cv::Mat gray = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
        cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
      } else {
        // Fallback: ask cv_bridge for BGR8 conversion.
        bgr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN(this->get_logger(), "cv_bridge error: %s", e.what());
      return;
    } catch (const cv::Exception & e) {
      RCLCPP_WARN(this->get_logger(), "OpenCV convert error: %s", e.what());
      return;
    }



    double lateral = 0.0;
    double heading = 0.0;
    cv::Mat dbg;
    cv::Mat * dbg_ptr = show_debug_ ? &dbg : nullptr;

    bool ok = false;
    try {
      ok = detect_lane(bgr, lateral, heading, dbg_ptr);
    } catch (const cv::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "OpenCV detect error: %s", e.what());
      return;
    }

    if (!ok && !publish_zero_on_failure_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Lane not detected. No /lane/error published (publish_zero_on_failure=false).");
      return;
    }

    geometry_msgs::msg::Vector3 out;
    out.x = ok ? lateral : 0.0;
    out.y = ok ? heading : 0.0;
    out.z = ok ? 1.0 : 0.0;  // 1: lane found, 0: fallback
    lane_pub_->publish(out);

    if (!ok) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Lane not detected. Publishing zero error fallback.");
    }

    // Debug topic publish (preferred: view with rqt_image_view)
    if (publish_debug_image_ && show_debug_ && debug_pub_ != nullptr && !dbg.empty()) {
      try {
        auto debug_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, dbg).toImageMsg();
        debug_pub_->publish(*debug_msg);
      } catch (...) {
        // Ignore debug publish errors
      }
    }

    // Debug visualization window (optional; may be inconvenient or unavailable in headless setups)
    if (show_debug_window_ && show_debug_ && !dbg.empty()) {
      try {
        if (!debug_window_initialized_) {
          init_debug_window();
        }

        if (debug_window_initialized_) {
          // Check if window still exists
          const double visible = cv::getWindowProperty(kDebugWindowName, cv::WND_PROP_VISIBLE);
          if (visible < 1.0) {
            debug_window_initialized_ = false;
            init_debug_window();
          }

          if (debug_window_initialized_) {
            cv::imshow(kDebugWindowName, dbg);
            cv::waitKey(1);
          }
        }
      } catch (const std::exception & e) {
        // Silently ignore if X11 not available
      } catch (...) {
        // Ignore any exception
      }
    }
  }

  std::string image_topic_{"/prius/front_camera/image_raw"};
  int image_qos_depth_{10};
  bool image_qos_reliable_{true};
  bool publish_zero_on_failure_{true};
  bool show_debug_{false};
  bool show_debug_window_{false};
  bool publish_debug_image_{false};
  std::string debug_image_topic_{"/lane/debug_image"};
  bool debug_window_initialized_{false};

  static constexpr const char * kDebugWindowName = "lane_detector";

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr lane_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};

}  // namespace perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception::LaneDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
