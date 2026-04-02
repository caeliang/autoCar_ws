#ifndef WAYPOINT_FINDER_H
#define WAYPOINT_FINDER_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>
#include <iostream>

struct Waypoint {
    double x, y, yaw;
    int index;
};

struct NearestResult {
    Waypoint waypoint;
    double distance;
    int grid_x, grid_y;  // En yakin matris indeksleri
};

class WaypointFinder {
private:
    std::vector<Waypoint> waypoints;
    int grid_width, grid_height;
    double grid_resolution;  // Grid uzerindeki 1 pixel = grid_resolution birim

public:
    WaypointFinder(int width = 59, int height = 68, double resolution = 1.0) 
        : grid_width(width), grid_height(height), grid_resolution(resolution) {}

    /**
     * CSV dosyasindan waypoint'leri yukler
     * @param filename CSV dosya yolu (x,y,yaw formatinda)
     * @return Yuklu olup olmadigi
     */
    bool loadWaypoints(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Hata: Waypoint dosyasi acilamadi -> " << filename << std::endl;
            return false;
        }

        waypoints.clear();
        std::string line;
        int index = 0;

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;  // Bos satir ve yorum atla

            std::stringstream ss(line);
            double x, y, yaw;
            char comma;

            if (ss >> x >> comma >> y >> comma >> yaw) {
                waypoints.push_back({x, y, yaw, index});
                index++;
            }
        }

        file.close();
        std::cout << "Toplamda " << waypoints.size() << " waypoint yuklendi." << std::endl;
        return true;
    }

    /**
     * Aracin mevcut konumundan en yakin waypoint'i bulur
     * @param vehicle_x Aracin X koordinati
     * @param vehicle_y Aracin Y koordinati
     * @return En yakin waypoint ve grid indeksleri
     */
    NearestResult findNearestWaypoint(double vehicle_x, double vehicle_y) {
        if (waypoints.empty()) {
            std::cerr << "Hata: Waypoint listesi bos!" << std::endl;
            return {{0, 0, 0, -1}, std::numeric_limits<double>::max(), -1, -1};
        }

        double min_distance = std::numeric_limits<double>::max();
        Waypoint nearest_waypoint = waypoints[0];

        // Tum waypoint'lerle arasi mesafeyi hesapla
        for (const auto& wp : waypoints) {
            double dx = wp.x - vehicle_x;
            double dy = wp.y - vehicle_y;
            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance < min_distance) {
                min_distance = distance;
                nearest_waypoint = wp;
            }
        }

        // En yakin waypoint'in matris indeksini hesapla
        // Varsayim: matris (0,0)'dan baslar ve grid_resolution ile scale'lenir
        int grid_x = static_cast<int>(nearest_waypoint.x / grid_resolution);
        int grid_y = static_cast<int>(nearest_waypoint.y / grid_resolution);

        // Sinirlar icinde kontrol et
        grid_x = std::max(0, std::min(grid_x, grid_width - 1));
        grid_y = std::max(0, std::min(grid_y, grid_height - 1));

        return {nearest_waypoint, min_distance, grid_x, grid_y};
    }

    /**
     * Aracin konumundan en yakin grid indeksini direkt hesaplar
     * @param vehicle_x Aracin X koordinati
     * @param vehicle_y Aracin Y koordinati
     * @return Grid X ve Y indeksleri
     */
    std::pair<int, int> findNearestGridIndex(double vehicle_x, double vehicle_y) {
        int grid_x = static_cast<int>(vehicle_x / grid_resolution);
        int grid_y = static_cast<int>(vehicle_y / grid_resolution);

        grid_x = std::max(0, std::min(grid_x, grid_width - 1));
        grid_y = std::max(0, std::min(grid_y, grid_height - 1));

        return {grid_x, grid_y};
    }

    /**
     * Waypointlerin sayisini dondurur
     */
    int getWaypointCount() const {
        return waypoints.size();
    }

    /**
     * Belirli indexteki waypoint'i dondurur
     */
    Waypoint getWaypoint(int index) const {
        if (index >= 0 && index < static_cast<int>(waypoints.size())) {
            return waypoints[index];
        }
        return {0, 0, 0, -1};
    }
};

#endif // WAYPOINT_FINDER_H
