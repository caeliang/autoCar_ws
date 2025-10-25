// Helper visualization function declaration
#pragma once

#include <vector>
#include "path_planning/AStar.hpp"

namespace path_planning {
    // Forward declaration - implemented in cpp file.
    void visualizeMapAndPath(const std::vector<std::vector<int>>& map,
                             const std::vector<AStar::Vec2i>& path);
}

// Also provide C-style visible symbol for the node file to call without namespace.
inline void visualizeMapAndPath(const std::vector<std::vector<int>>& map,
                                const std::vector<AStar::Vec2i>& path)
{
    path_planning::visualizeMapAndPath(map, path);
}
