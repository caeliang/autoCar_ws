#include "rclcpp/rclcpp.hpp"
#include "path_planning/AStar.hpp" 
#include <iostream>
#include <vector>
#include <algorithm>

// ROS 2 Entegrasyonu için A* Mantığını bir Sınıfa Alın
class PathPlannerNode : public rclcpp::Node
{
public:
    // Sınıfın kurucu (constructor) metodu
    PathPlannerNode() : Node("path_planner_node")
    {
        RCLCPP_INFO(this->get_logger(), "Path Planner Node has started and will now calculate a path.");
        
        // A* algoritmasını çalıştıran metodu çağır
        calculate_and_print_path();
    }

private:
    // A* Hesaplama ve Konsola Yazdırma Metodu
    void calculate_and_print_path()
    {
        AStar::Generator generator;

        // 25x25 grid boyutu
        generator.setWorldSize({25, 25});
        generator.setHeuristic(AStar::Heuristic::euclidean);
        generator.setDiagonalMovement(false);

        // 🔹 Engelleri ekle (örnek)
        for (int i = 5; i <= 15; ++i) {
            generator.addCollision({i, 10}); // (x,y)
        }

        // 🔹 Başlangıç ve hedef
        AStar::Vec2i start = {0, 0};
        AStar::Vec2i end   = {20, 20};

        RCLCPP_INFO(this->get_logger(), "Generating path from [%d, %d] to [%d, %d]...", 
                    start.x, start.y, end.x, end.y);

        auto path = generator.findPath(start, end);
        std::reverse(path.begin(), path.end());

        if (path.empty()) {
            RCLCPP_WARN(this->get_logger(), "No path found!");
            return;
        }

        std::string path_str = "Path found: [";
        for (size_t i = 0; i < path.size(); ++i) {
            path_str += "[" + std::to_string(path[i].x) + " " + std::to_string(path[i].y) + "]";
            if (i < path.size() - 1) path_str += ", ";
        }
        path_str += "]";
        RCLCPP_INFO(this->get_logger(), "%s", path_str.c_str());

    }

};
// Ana Program Giriş Noktası (ROS 2 Standardı)
int main(int argc, char **argv)
{
    // ROS 2 Sistemini Başlat
    rclcpp::init(argc, argv);
    
    // Node'u Oluştur ve Çalıştır
    // make_shared ile PathPlannerNode sınıfından bir nesne oluşturulur.
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    
    // ROS 2 Sistemini Kapat
    rclcpp::shutdown();
    
    return 0;
}