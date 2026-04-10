#include "path_planning/waypoint_io.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iomanip>
#include <algorithm>
#include <cmath>

namespace path_planning {

// ──────────────────────────────────────────────────────────────────────
//  Minimal YAML parser — sadece x, y, yaw (eski format da desteklenir)
// ──────────────────────────────────────────────────────────────────────

static std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end   = s.find_last_not_of(" \t\r\n");
    return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
}

WaypointList loadWaypoints(const std::string& filepath) {
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) {
        throw std::runtime_error("Cannot open waypoint file: " + filepath);
    }

    WaypointList waypoints;
    double cur_x = 0, cur_y = 0, cur_yaw = 0;
    double cur_qz = 0, cur_qw = 1;  // eski format geriye uyumluluk
    bool in_waypoint = false;
    bool has_yaw = false;   // yeni format: doğrudan yaw
    bool has_qz = false;    // eski format: quaternion'dan yaw
    int field_count = 0;

    auto flush_waypoint = [&]() {
        if (field_count >= 2) {
            Waypoint wp;
            wp.x = cur_x;
            wp.y = cur_y;
            if (has_yaw) {
                wp.yaw = cur_yaw;
            } else if (has_qz) {
                // quaternion → yaw (eski format)
                wp.yaw = std::atan2(2.0 * cur_qw * cur_qz, 1.0 - 2.0 * cur_qz * cur_qz);
            } else {
                wp.yaw = 0.0;
            }
            waypoints.push_back(wp);
        }
    };

    std::string line;
    while (std::getline(ifs, line)) {
        std::string trimmed = trim(line);

        if (trimmed.empty() || trimmed[0] == '#') continue;
        if (trimmed == "waypoints:") continue;

        // Yeni waypoint: "- x: value"
        if (trimmed.size() > 2 && trimmed[0] == '-' && trimmed[1] == ' ') {
            flush_waypoint();
            cur_x = 0; cur_y = 0; cur_yaw = 0;
            cur_qz = 0; cur_qw = 1;
            in_waypoint = true;
            has_yaw = false;
            has_qz = false;
            field_count = 0;
            trimmed = trim(trimmed.substr(2));
        }

        if (!in_waypoint) continue;

        auto colon = trimmed.find(':');
        if (colon == std::string::npos) continue;

        std::string key = trim(trimmed.substr(0, colon));
        std::string val = trim(trimmed.substr(colon + 1));
        if (val.empty()) continue;

        double v = std::stod(val);

        if      (key == "x")   { cur_x = v; field_count++; }
        else if (key == "y")   { cur_y = v; field_count++; }
        else if (key == "yaw") { cur_yaw = v; has_yaw = true; }
        else if (key == "qz")  { cur_qz = v; has_qz = true; }
        else if (key == "qw")  { cur_qw = v; }
        // z, qx, qy, speed → yoksay (geriye uyumluluk)
    }

    flush_waypoint();  // son waypoint
    return waypoints;
}

bool saveWaypoints(const std::string& filepath, const WaypointList& waypoints) {
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) return false;

    ofs << std::fixed << std::setprecision(4);
    ofs << "# Waypoint file — sadece x, y, yaw\n";
    ofs << "# Frame: map\n";
    ofs << "# Total: " << waypoints.size() << " waypoints\n\n";
    ofs << "waypoints:\n";

    for (size_t i = 0; i < waypoints.size(); ++i) {
        const auto& wp = waypoints[i];
        ofs << "  - x: "   << wp.x   << "\n"
            << "    y: "   << wp.y   << "\n"
            << "    yaw: " << wp.yaw << "\n";
        if (i + 1 < waypoints.size()) ofs << "\n";
    }

    return true;
}

// ──────────────────────────────────────────────────────────────────────
//  CSV loader — index,x,y,yaw
// ──────────────────────────────────────────────────────────────────────

WaypointList loadWaypointsCSV(const std::string& filepath) {
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) {
        throw std::runtime_error("Cannot open CSV file: " + filepath);
    }

    WaypointList waypoints;
    std::string line;
    
    int x_idx = -1, y_idx = -1, yaw_idx = -1;
    bool header_parsed = false;

    while (std::getline(ifs, line)) {
        std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#') continue;

        std::vector<std::string> tokens;
        std::istringstream ss(trimmed);
        std::string token;
        while (std::getline(ss, token, ',')) {
            tokens.push_back(trim(token));
        }

        if (!header_parsed) {
            for (size_t i = 0; i < tokens.size(); ++i) {
                if (tokens[i] == "x") x_idx = i;
                else if (tokens[i] == "y") y_idx = i;
                else if (tokens[i] == "yaw") yaw_idx = i;
            }
            if (x_idx == -1 || y_idx == -1) {
                throw std::runtime_error("CSV file must contain at least 'x' and 'y' columns in the header.");
            }
            header_parsed = true;
            continue;
        }

        if (tokens.size() <= static_cast<size_t>(std::max(x_idx, y_idx))) continue;

        Waypoint wp;
        try {
            wp.x = std::stod(tokens[x_idx]);
            wp.y = std::stod(tokens[y_idx]);
            if (yaw_idx != -1 && tokens.size() > static_cast<size_t>(yaw_idx)) {
                // The CSV files have yaw in degrees, so we convert it to radians.
                wp.yaw = std::stod(tokens[yaw_idx]) * M_PI / 180.0;
            } else {
                wp.yaw = 0.0;
            }
            waypoints.push_back(wp);
        } catch (...) {
            // Ignore malformed rows
        }
    }

    // Eğer yaw sütunu yoksa veya hepsi 0 ise, ardışık noktalardan yaw hesapla
    if (yaw_idx == -1) {
        for (size_t i = 0; i < waypoints.size() - 1; ++i) {
            double dx = waypoints[i+1].x - waypoints[i].x;
            double dy = waypoints[i+1].y - waypoints[i].y;
            waypoints[i].yaw = std::atan2(dy, dx);
        }
        if (!waypoints.empty() && waypoints.size() > 1) {
            waypoints.back().yaw = waypoints[waypoints.size() - 2].yaw;
        }
    }

    return waypoints;
}

}  // namespace path_planning
