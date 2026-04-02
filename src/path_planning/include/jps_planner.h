#ifndef JPS_PLANNER_H
#define JPS_PLANNER_H

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <set>
#include <fstream>
#include <iostream>
#include "direction_calculator.h"

using namespace std;

struct JPSNode {
    Point pos;
    double g, h;
    JPSNode* parent;

    JPSNode(Point _pos, double _g, double _h, JPSNode* _parent = nullptr) 
        : pos(_pos), g(_g), h(_h), parent(_parent) {}

    double f() const { return g + h; }
};

struct CompareJPSNode {
    bool operator()(const JPSNode* a, const JPSNode* b) const {
        return a->f() > b->f();
    }
};

class JPSPlanner {
private:
    vector<vector<int>> grid;
    int width, height;

    double heuristic(Point a, Point b) {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }

    bool isValid(Point p) {
        return p.x >= 0 && p.x < width && p.y >= 0 && p.y < height;
    }

    bool isObstacle(Point p) {
        return grid[p.y][p.x] == 0;
    }

    bool isWalkable(Point p) {
        return isValid(p) && !isObstacle(p);
    }

    vector<Point> getForcedNeighbors(Point current, Point parent) {
        vector<Point> forced;
        
        if (parent.x == current.x) {
            int dy = (current.y > parent.y) ? 1 : -1;
            
            Point left = {current.x - 1, current.y};
            Point right = {current.x + 1, current.y};
            Point leftDiag = {current.x - 1, current.y + dy};
            Point rightDiag = {current.x + 1, current.y + dy};
            
            if (isValid(left) && isObstacle(left) && isWalkable(leftDiag)) {
                forced.push_back(leftDiag);
            }
            if (isValid(right) && isObstacle(right) && isWalkable(rightDiag)) {
                forced.push_back(rightDiag);
            }
        } else if (parent.y == current.y) {
            int dx = (current.x > parent.x) ? 1 : -1;
            
            Point top = {current.x, current.y - 1};
            Point bottom = {current.x, current.y + 1};
            Point topDiag = {current.x + dx, current.y - 1};
            Point bottomDiag = {current.x + dx, current.y + 1};
            
            if (isValid(top) && isObstacle(top) && isWalkable(topDiag)) {
                forced.push_back(topDiag);
            }
            if (isValid(bottom) && isObstacle(bottom) && isWalkable(bottomDiag)) {
                forced.push_back(bottomDiag);
            }
        }
        
        return forced;
    }

    Point jump(Point current, int dx, int dy, Point goal) {
        Point next = {current.x + dx, current.y + dy};

        if (!isWalkable(next)) {
            return {-1, -1};
        }

        if (next == goal) {
            return next;
        }

        if (dx != 0 && dy != 0) {
            if (jump({current.x + dx, current.y}, dx, 0, goal).x != -1 ||
                jump({current.x, current.y + dy}, 0, dy, goal).y != -1) {
                return next;
            }
        }

        auto forced = getForcedNeighbors(next, current);
        if (!forced.empty()) {
            return next;
        }

        return jump(next, dx, dy, goal);
    }

public:
    JPSPlanner() : width(0), height(0) {}

    bool loadMap(const string& filename) {
        ifstream file(filename);
        if (!file.is_open()) {
            cerr << "Hata: Harita dosyasi acilamadi -> " << filename << endl;
            return false;
        }

        string line;
        while (getline(file, line)) {
            vector<int> row;
            for (char c : line) {
                if (c == '0' || c == '1') {
                    row.push_back(c - '0');
                }
            }
            if (!row.empty()) {
                grid.push_back(row);
                width = row.size();
            }
        }
        height = grid.size();
        return true;
    }

    vector<Point> findPath(Point start, Point goal) {
        if (!isValid(start) || !isValid(goal)) {
            cerr << "Hata: Baslangic veya Hedef harita sinirlarinin disinda!" << endl;
            return {};
        }
        if (isObstacle(start) || isObstacle(goal)) {
            cerr << "Hata: Baslangic veya Hedef noktasi engel ustunde!" << endl;
            return {};
        }

        priority_queue<JPSNode*, vector<JPSNode*>, CompareJPSNode> openSet;
        set<Point> closedSet;
        set<Point> openSetPoints;
        vector<JPSNode*> allNodes;

        JPSNode* startNode = new JPSNode(start, 0.0, heuristic(start, goal));
        openSet.push(startNode);
        openSetPoints.insert(start);
        allNodes.push_back(startNode);

        int dx_dirs[] = {0, 1, 0, -1, 1, 1, -1, -1};
        int dy_dirs[] = {1, 0, -1, 0, 1, -1, 1, -1};

        while (!openSet.empty()) {
            JPSNode* current = openSet.top();
            openSet.pop();
            openSetPoints.erase(current->pos);

            if (current->pos == goal) {
                vector<Point> path;
                JPSNode* temp = current;
                while (temp != nullptr) {
                    path.push_back(temp->pos);
                    temp = temp->parent;
                }
                reverse(path.begin(), path.end());
                
                for (JPSNode* n : allNodes) delete n;
                return path;
            }

            closedSet.insert(current->pos);

            for (int dir = 0; dir < 8; ++dir) {
                int dx = dx_dirs[dir];
                int dy = dy_dirs[dir];

                Point jumpNode = jump(current->pos, dx, dy, goal);

                if (jumpNode.x == -1) continue;
                if (closedSet.count(jumpNode)) continue;
                if (openSetPoints.count(jumpNode)) continue;

                double moveCost = sqrt(pow(jumpNode.x - current->pos.x, 2) + 
                                       pow(jumpNode.y - current->pos.y, 2));
                double newG = current->g + moveCost;

                JPSNode* jumpNodeObj = new JPSNode(jumpNode, newG, heuristic(jumpNode, goal), current);
                openSet.push(jumpNodeObj);
                openSetPoints.insert(jumpNode);
                allNodes.push_back(jumpNodeObj);
            }
        }

        for (JPSNode* n : allNodes) delete n;
        return {};
    }
};

#endif // JPS_PLANNER_H