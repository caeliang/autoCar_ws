#ifndef DIRECTION_CALCULATOR_H
#define DIRECTION_CALCULATOR_H

#include <string>
#include <vector>

struct Point {
    int x, y;
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(const Point& other) const {
        return !(*this == other);
    }
    bool operator<(const Point& other) const {
        return x < other.x || (x == other.x && y < other.y);
    }
};

enum Direction {
    DOGU,          // East (1, 0)
    BATI,          // West (-1, 0)
    KUZEY,         // North (0, -1)
    GUNEY,         // South (0, 1)
    KUZEY_DOGU,    // NorthEast (1, -1)
    KUZEY_BATI,    // NorthWest (-1, -1)
    GUNEY_DOGU,    // SouthEast (1, 1)
    GUNEY_BATI,    // SouthWest (-1, 1)
    BILINMEYEN     // Unknown
};

class DirectionCalculator {
public:
    static Direction calculateDirection(const Point& from, const Point& to) {
        int dx = to.x - from.x;
        int dy = to.y - from.y;

        if (dx == 1 && dy == 0) return DOGU;
        else if (dx == -1 && dy == 0) return BATI;
        else if (dx == 0 && dy == 1) return GUNEY;
        else if (dx == 0 && dy == -1) return KUZEY;
        else if (dx == 1 && dy == 1) return GUNEY_DOGU;
        else if (dx == 1 && dy == -1) return KUZEY_DOGU;
        else if (dx == -1 && dy == 1) return GUNEY_BATI;
        else if (dx == -1 && dy == -1) return KUZEY_BATI;
        else return BILINMEYEN;
    }

    static std::string directionToString(Direction dir) {
        switch (dir) {
            case DOGU: return "Dogu";
            case BATI: return "Bati";
            case KUZEY: return "Kuzey";
            case GUNEY: return "Guney";
            case KUZEY_DOGU: return "KuzeyDogu";
            case KUZEY_BATI: return "KuzeyBati";
            case GUNEY_DOGU: return "GuneyDogu";
            case GUNEY_BATI: return "GuneyBati";
            default: return "Bilinmeyen";
        }
    }

    static std::vector<Direction> getDirectionsFromPath(const std::vector<Point>& path) {
        std::vector<Direction> directions;
        for (size_t i = 1; i < path.size(); ++i) {
            directions.push_back(calculateDirection(path[i-1], path[i]));
        }
        return directions;
    }

    static std::vector<std::string> directionsToStrings(const std::vector<Direction>& directions) {
        std::vector<std::string> result;
        for (const auto& dir : directions) {
            result.push_back(directionToString(dir));
        }
        return result;
    }
};

#endif // DIRECTION_CALCULATOR_H
