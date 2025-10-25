#include "path_planning/visualize.hpp"
#include "path_planning/matplotlibcpp.h"
#include <map>
#include <string>

namespace plt = matplotlibcpp;

void path_planning::visualizeMapAndPath(const std::vector<std::vector<int>>& map,
                                        const std::vector<AStar::Vec2i>& path)
{
    int rows = map.size();
    int cols = map.empty() ? 0 : map[0].size();

    plt::figure_size(700, 700);

    // Collect obstacle coordinates first so we can plot them in a single call (helps legend)
    std::vector<double> obs_x, obs_y;
    for (int y = 0; y < rows; ++y)
    {
        for (int x = 0; x < cols; ++x)
        {
            if (map[y][x] == 1)
            {
                obs_x.push_back((double)x + 0.5);
                obs_y.push_back((double)rows - y - 0.5);
            }
        }
    }

    if (!obs_x.empty())
    {
        std::map<std::string, std::string> obs_kw{{"color", "#444444"}, {"marker", "s"}, {"label", "Obstacle"}};
        plt::scatter(obs_x, obs_y, 300.0, obs_kw);
    }

    // Path (swapped)
    std::vector<double> path_x, path_y;
    for (auto &p : path)
    {
        path_x.push_back((double)p.y + 0.5); // x <- original y
        path_y.push_back((double)rows - p.x - 0.5); // y <- original x (flipped vertically)
    }

    if (!path_x.empty())
    {
        std::map<std::string, std::string> path_kw{{"color", "#d62728"}, {"linewidth", "2"}, {"label", "Path"}};
        plt::plot(path_x, path_y, path_kw);

        // Mark start and goal points with distinct markers and labels
        auto start = path.front();
        auto goal = path.back();
        std::vector<double> sx = { (double)start.y + 0.5 };
        std::vector<double> sy = { (double)rows - start.x - 0.5 };
        std::vector<double> gx = { (double)goal.y + 0.5 };
        std::vector<double> gy = { (double)rows - goal.x - 0.5 };

        std::map<std::string, std::string> start_kw{{"color", "#2ca02c"}, {"marker", "o"}, {"label", "Start"}};
        std::map<std::string, std::string> goal_kw{{"color", "#1f77b4"}, {"marker", "X"}, {"label", "Goal"}};
        plt::scatter(sx, sy, 200.0, start_kw);
        plt::scatter(gx, gy, 200.0, goal_kw);

        // Annotate start/goal with their grid coordinates (original indices)
        std::string start_txt = "S(" + std::to_string(start.x) + "," + std::to_string(start.y) + ")";
        std::string goal_txt = "G(" + std::to_string(goal.x) + "," + std::to_string(goal.y) + ")";
        plt::text(sx[0] + 0.05, sy[0] + 0.05, start_txt);
        plt::text(gx[0] + 0.05, gy[0] + 0.05, goal_txt);
    }

    // Axis ticks aligned to grid cell centers with integer labels
    std::vector<double> xticks_pos; std::vector<std::string> xticks_labels;
    for (int i = 0; i < rows; ++i) { xticks_pos.push_back(i + 0.5); xticks_labels.push_back(std::to_string(i)); }
    std::vector<double> yticks_pos; std::vector<std::string> yticks_labels;
    for (int i = 0; i < cols; ++i) { yticks_pos.push_back((double)rows - i - 0.5); yticks_labels.push_back(std::to_string(i)); }
    plt::xticks(xticks_pos, xticks_labels);
    plt::yticks(yticks_pos, yticks_labels);

    plt::grid(true);
    plt::xlabel("X (swapped) — original column index");
    plt::ylabel("Y (swapped) — original row index");
    plt::title("Path visualization (X and Y swapped)");
    plt::axis("equal");
    plt::legend();
    plt::tight_layout();

    plt::show();
}
