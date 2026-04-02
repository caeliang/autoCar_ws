#include <iostream>
#include <vector>
#include <queue>
#include <fstream>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <chrono>

using namespace std;

struct Point {
    int x, y;
    bool operator==(const Point& o) const { return x == o.x && y == o.y; }
};

// Hash — unordered_map için şart
struct PointHash {
    size_t operator()(const Point& p) const {
        return hash<int>()(p.x) ^ (hash<int>()(p.y) << 16);
    }
};

struct Node {
    Point pos;
    double g, f;
    Point parent;
    bool hasParent;
    bool operator>(const Node& o) const { return f > o.f; }
};

class AStarPlanner {
private:
    vector<vector<int>> grid;
    int width, height;

    // ✅ Octile distance — sqrt'siz, çok hızlı
    inline double heuristic(Point a, Point b) {
        int dx = abs(a.x - b.x);
        int dy = abs(a.y - b.y);
        return (dx + dy) + (1.4142135 - 2.0) * min(dx, dy);
    }

    inline bool isValid(int x, int y) {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    inline bool isWalkable(int x, int y) {
        return isValid(x, y) && grid[y][x] == 1;
    }

public:
    bool loadMap(const string& filename) {
        ifstream file(filename);
        if (!file.is_open()) return false;

        string line;
        while (getline(file, line)) {
            vector<int> row;
            for (char c : line)
                if (c == '0' || c == '1')
                    row.push_back(c - '0');
            if (!row.empty()) {
                grid.push_back(row);
                width = row.size();
            }
        }
        height = grid.size();
        cout << "Map: " << width << "x" << height << endl;
        return true;
    }

    vector<Point> findPath(Point start, Point goal) {
        if (!isWalkable(start.x, start.y) || !isWalkable(goal.x, goal.y)) {
            cerr << "Start/goal gecersiz!" << endl;
            return {};
        }

        priority_queue<Node, vector<Node>, greater<Node>> openSet;
        
        // ✅ g_score map — duplicate ve güncelleme kontrolü
        unordered_map<Point, double, PointHash> gScore;
        unordered_map<Point, Point, PointHash> cameFrom;
        unordered_map<Point, bool, PointHash> inClosed;

        gScore[start] = 0.0;
        openSet.push({start, 0.0, heuristic(start, goal), {-1,-1}, false});

        const int dx[] = {0,1,0,-1,1,1,-1,-1};
        const int dy[] = {1,0,-1,0,1,-1,1,-1};
        const double cost[] = {1,1,1,1,1.4142,1.4142,1.4142,1.4142};

        while (!openSet.empty()) {
            Node cur = openSet.top(); openSet.pop();
            Point pos = cur.pos;

            // ✅ Closed set kontrolü
            if (inClosed[pos]) continue;
            inClosed[pos] = true;

            if (pos == goal) {
                // Yolu geri izle
                vector<Point> path;
                Point p = goal;
                while (!(p == start)) {
                    path.push_back(p);
                    p = cameFrom[p];
                }
                path.push_back(start);
                reverse(path.begin(), path.end());
                return path;
            }

            for (int i = 0; i < 8; i++) {
                int nx = pos.x + dx[i];
                int ny = pos.y + dy[i];
                Point np = {nx, ny};

                if (!isWalkable(nx, ny) || inClosed[np]) continue;

                double newG = gScore[pos] + cost[i];

                // ✅ Sadece daha iyi yol varsa ekle
                if (gScore.find(np) == gScore.end() || newG < gScore[np]) {
                    gScore[np] = newG;
                    cameFrom[np] = pos;
                    double f = newG + heuristic(np, goal);
                    openSet.push({np, newG, f, pos, true});
                }
            }
        }
        return {};
    }
};

int main(int argc, char** argv) {
    if (argc < 4) {
        cout << "Kullanim: ./planner <map.txt> <hedef_x> <hedef_y>" << endl;
        return 1;
    }

    AStarPlanner planner;
    if (!planner.loadMap(argv[1])) return -1;

    Point start = {2, 2};
    Point goal  = {stoi(argv[2]), stoi(argv[3])};

    auto t1 = chrono::high_resolution_clock::now();
    auto path = planner.findPath(start, goal);
    auto t2 = chrono::high_resolution_clock::now();

    cout << "SURE: " 
         << chrono::duration_cast<chrono::milliseconds>(t2-t1).count() 
         << " ms" << endl;

    if (!path.empty())
        cout << "Rota bulundu! " << path.size() << " nokta." << endl;
    else
        cout << "Yol bulunamadi!" << endl;

    return 0;
}