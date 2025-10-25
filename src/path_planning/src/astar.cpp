#include "path_planning/astar.hpp"

AStar::AStar(const std::vector<std::vector<int>> &grid)
    : grid_(grid), rows_(grid.size()), cols_(grid[0].size()) {}

double AStar::heuristic(const Node &a, const Node &b) const
{
    // Manhattan distance
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

bool AStar::isValid(int x, int y) const
{
    return x >= 0 && y >= 0 && x < rows_ && y < cols_ && grid_[x][y] != 0;
}

std::pair<double, std::vector<Node>> AStar::findPath(const Node &start, const Node &goal)
{
    std::priority_queue<NodePriority, std::vector<NodePriority>, std::greater<NodePriority>> openSet;
    std::unordered_map<Node, Node, NodeHash> cameFrom;
    std::unordered_map<Node, double, NodeHash> costSoFar;

    openSet.push({0.0, start});
    cameFrom[start] = start;
    costSoFar[start] = 0.0;

    std::vector<std::pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

    while (!openSet.empty())
    {
        Node current = openSet.top().node;
        openSet.pop();

        if (current == goal)
            break;

        for (auto [dx, dy] : directions)
        {
            int nx = current.x + dx;
            int ny = current.y + dy;
            if (!isValid(nx, ny))
                continue;

            Node neighbor{nx, ny};
            double newCost = costSoFar[current] + 1.0;

            if (costSoFar.find(neighbor) == costSoFar.end() || newCost < costSoFar[neighbor])
            {
                costSoFar[neighbor] = newCost;
                double priority = newCost + heuristic(neighbor, goal);
                openSet.push({priority, neighbor});
                cameFrom[neighbor] = current;
            }
        }
    }

    // Path reconstruction
    std::vector<Node> path;
    Node current = goal;
    if (cameFrom.find(goal) == cameFrom.end())
        return {std::numeric_limits<double>::infinity(), path};

    while (current != start)
    {
        path.push_back(current);
        current = cameFrom[current];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return {costSoFar[goal], path};
}
