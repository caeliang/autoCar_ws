#include <vector>
#include <cmath>
#include "path_planning/waypoint.hpp"

namespace path_planning {

/**
 * @brief Aracın koordinatlarına en yakın waypoint'in indeksini bulur.
 * 
 * @param waypoints Mevcut waypoint listesi
 * @param car_x Aracın X koordinatı
 * @param car_y Aracın Y koordinatı
 * @return int En yakın waypoint'in indeksi
 */
int find_nearest_waypoint(const std::vector<Waypoint>& waypoints, double car_x, double car_y) {
    if (waypoints.empty()) return -1;

    int nearest_idx = 0;
    double min_dist_sq = 1e9;

    for (size_t i = 0; i < waypoints.size(); ++i) {
        double dx = waypoints[i].x - car_x;
        double dy = waypoints[i].y - car_y;
        double dist_sq = dx * dx + dy * dy;

        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            nearest_idx = static_cast<int>(i);
        }
    }

    return nearest_idx;
}

} // namespace path_planning
