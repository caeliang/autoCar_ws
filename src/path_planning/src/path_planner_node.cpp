#include "rclcpp/rclcpp.hpp"
#include "path_planning/AStar.hpp" 
#include <iostream>
#include <vector>
#include <algorithm>
#include "path_planning/visualize.hpp"
#include "path_planning/global.hpp"
#include <chrono>

class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode() : Node("path_planner_node")
    {
        RCLCPP_INFO(this->get_logger(), "Path Planner Node has started and will now calculate a path.");
        auto tvis0 = std::chrono::high_resolution_clock::now();

    std::vector<AStar::Vec2i> path = calculate_and_print_path();
    // Görselleştirme: global map'i kullan
    visualizeMapAndPath(g_map, path);
        auto tvis1 = std::chrono::high_resolution_clock::now();
    auto vis_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(tvis1 - tvis0).count();
    RCLCPP_INFO(this->get_logger(), "Visualization took %lld ns", (long long)vis_ns);
    }

private:
    std::vector<AStar::Vec2i> calculate_and_print_path()
    {
        // Use global map
        int rows = g_map.size();
        int cols = g_map[0].size();
        
        AStar::Generator generator;
        // The grid (g_map) is row-major with origin at top-left (row 0 = top).
        // The A* implementation treats increasing y as moving "up" from a
        // bottom-left origin. To make both systems consistent (use top-left),
        // flip Y when passing coordinates into A* and flip back on return.
        auto gridToAstar = [rows](const AStar::Vec2i &g) -> AStar::Vec2i {
            return AStar::Vec2i{ g.x, (rows - 1) - g.y };
        };
        auto astarToGrid = [rows](const AStar::Vec2i &a) -> AStar::Vec2i {
            return AStar::Vec2i{ a.x, (rows - 1) - a.y };
        };

        for (int y = 0; y < rows; ++y)
        {
            for (int x = 0; x < cols; ++x)
            {
                // g_map is row-major: g_map[row][col]
                if (g_map[y][x] == 1)
                    generator.addCollision(gridToAstar({x, y}));
            }
        }
        generator.setWorldSize({cols, rows});
        generator.setDiagonalMovement(false);
        generator.setHeuristic(&AStar::Heuristic::manhattan);





    // Define start/end in grid coords (top-left origin) then convert to A* coords
    AStar::Vec2i start_grid = {0, 0};
    AStar::Vec2i end_grid   = {4, 4};
    AStar::Vec2i start = gridToAstar(start_grid);
    AStar::Vec2i end   = gridToAstar(end_grid);

    RCLCPP_INFO(this->get_logger(), "Generating path from [%d, %d] to [%d, %d] (grid coords)...", 
        start_grid.x, start_grid.y, end_grid.x, end_grid.y);

    auto t0 = std::chrono::high_resolution_clock::now();
    auto path = generator.findPath(start, end);
    auto t1 = std::chrono::high_resolution_clock::now();
    auto gen_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
    std::reverse(path.begin(), path.end());
    RCLCPP_INFO(this->get_logger(), "Path generation took %lld ns", (long long)gen_ns);

        if (path.empty()) {
            RCLCPP_WARN(this->get_logger(), "No path found!");
            return {};
        }

        // Convert returned path (A* coords) back to grid coords (top-left origin)
        std::vector<AStar::Vec2i> path_grid;
        path_grid.reserve(path.size());
        for (const auto &p : path) {
            path_grid.push_back(astarToGrid(p));
        }

        std::string path_str = "Path found: ";
        for (size_t i = 0; i < path_grid.size(); ++i) {
            path_str += "[" + std::to_string(path_grid[i].x) + " " + std::to_string(path_grid[i].y) + "]";
            if (i < path_grid.size() - 1) path_str += ", ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", path_str.c_str());
        return path_grid;
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