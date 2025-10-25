#include "rclcpp/rclcpp.hpp"
#include "path_planning/AStar.hpp" 
#include <iostream>
#include <vector>
#include <algorithm>
#include "path_planning/visualize.hpp"
#include "path_planning/global.hpp"

// ROS 2 Entegrasyonu için A* Mantığını bir Sınıfa Alın
class PathPlannerNode : public rclcpp::Node
{
public:
    // Sınıfın kurucu (constructor) metodu
    PathPlannerNode() : Node("path_planner_node")
    {
        RCLCPP_INFO(this->get_logger(), "Path Planner Node has started and will now calculate a path.");

        std::vector<AStar::Vec2i> path = calculate_and_print_path();
        // Görselleştirme: global map'i kullan
        visualizeMapAndPath(g_map, path);
    }

private:
    std::vector<AStar::Vec2i> calculate_and_print_path()
    {
        // Use global map
        int rows = g_map.size();
        int cols = g_map[0].size();
        
        AStar::Generator generator;
        for (int y = 0; y < rows; ++y)
        {
            for (int x = 0; x < cols; ++x)
            {
                if (g_map[x][y] == 1)
                    generator.addCollision({x, y});
            }
        }
        generator.setWorldSize({cols, rows});
        generator.setDiagonalMovement(false);
        generator.setHeuristic(&AStar::Heuristic::manhattan);




        AStar::Vec2i start = {0, 0};
        AStar::Vec2i end   = {4, 4};

        RCLCPP_INFO(this->get_logger(), "Generating path from [%d, %d] to [%d, %d]...", 
                    start.x, start.y, end.x, end.y);

        auto path = generator.findPath(start, end);
        std::reverse(path.begin(), path.end());

        if (path.empty()) {
            RCLCPP_WARN(this->get_logger(), "No path found!");
            return {};
        }

        std::string path_str = "Path found: [";
        for (size_t i = 0; i < path.size(); ++i) {
            path_str += "[" + std::to_string(path[i].x) + " " + std::to_string(path[i].y) + "]";
            if (i < path.size() - 1) path_str += ", ";
        }
        path_str += "]";
        RCLCPP_INFO(this->get_logger(), "%s", path_str.c_str());
        return path;
    }

};
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