#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <algorithm>

/**
 * Bu node "waypoint_tracker" node'undan yayinlanan en yakin waypoint
 * koordinatini alarak onu gercek matris indeksine (grid_x, grid_y) donusturur.
 */
class CarIndexFinderNode : public rclcpp::Node {
public:
    CarIndexFinderNode() : Node("car_index_finder_node") {
        // Parametre ile car index okuma
        this->declare_parameter<int>("car_index", 0);
        int car_index = this->get_parameter("car_index").as_int();

        RCLCPP_INFO(this->get_logger(), "Initializing Car Index Finder for Car Index: %d", car_index);

        // Tracker node'undan gelen konumu dinle
        std::string sub_topic = (car_index == 0) ? "/prius/nearest_waypoint" : "/prius_" + std::to_string(car_index) + "/nearest_waypoint";
        
        wp_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            sub_topic, 10,
            std::bind(&CarIndexFinderNode::waypoint_callback, this, std::placeholders::_1)
        );

        std::string pub_topic = (car_index == 0) ? "/prius/car_matrix_index" : "/prius_" + std::to_string(car_index) + "/car_matrix_index";
        grid_pub_ = this->create_publisher<geometry_msgs::msg::Point>(pub_topic, 10);
        
        RCLCPP_INFO(this->get_logger(), "Listening to %s to find matrix index...", sub_topic.c_str());
    }

private:
    void waypoint_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        double wp_x = msg->x;
        double wp_y = msg->y;
        
        int grid_x = 0;
        int grid_y = 0;
        
        // Cikarilan waypoint koordinatini Grid (Matris) duzlemine cevir
        world_to_grid(wp_x, wp_y, grid_x, grid_y);
        
        // C++ Finder Node'u bunu bir baska test/python node'u tarafindan okunsun diye mesh ediyor:
        geometry_msgs::msg::Point grid_msg;
        grid_msg.x = grid_x;
        grid_msg.y = grid_y;
        grid_msg.z = 0.0;
        grid_pub_->publish(grid_msg);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Received WP: (%.2f, %.2f) => Matrix Index [Y (Row): %d, X (Col): %d]",
            wp_x, wp_y, grid_y, grid_x);
    }

    void world_to_grid(double world_x, double world_y, int& grid_x, int& grid_y) {
        // Python generate_route.py icerisindeki matris yapisi (1.5x olcekli)
        const int grid_width = 59;
        const int grid_height = 68;
        const double min_x = -28.8 * 1.5;
        const double max_x =  28.8 * 1.5;
        const double min_y = -32.5 * 1.5;
        const double max_y =  33.8 * 1.5;

        double x_span = max_x - min_x;
        double y_span = max_y - min_y;

        grid_x = std::round((world_x - min_x) / x_span * (grid_width - 1));
        grid_y = std::round((max_y - world_y) / y_span * (grid_height - 1));

        // Clamping (Limitleri asmamasi icin margin kontrolu)
        grid_x = std::max(0, std::min(grid_x, grid_width - 1));
        grid_y = std::max(0, std::min(grid_y, grid_height - 1));
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr wp_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr grid_pub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CarIndexFinderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}