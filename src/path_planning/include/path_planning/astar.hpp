#pragma once
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <utility>
#include <functional>
#include <limits>

struct Node
{
    int x, y;
    bool operator==(const Node &other) const { return x == other.x && y == other.y; }
    bool operator!=(const Node &other) const { return !(*this == other); }
};

struct NodeHash
{
    std::size_t operator()(const Node &n) const noexcept
    {
        return std::hash<int>()(n.x * 10000 + n.y);
    }
};

struct NodePriority
{
    double cost;
    Node node;
    bool operator>(const NodePriority &other) const
    {
        return cost > other.cost;
    }
};

class AStar
{
public:
    AStar(const std::vector<std::vector<int>> &grid);

    std::pair<double, std::vector<Node>> findPath(const Node &start, const Node &goal);

private:
    double heuristic(const Node &a, const Node &b) const;
    bool isValid(int x, int y) const;
    std::vector<std::vector<int>> grid_;
    int rows_, cols_;
};
