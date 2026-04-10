#include <vector>
#include <cmath>
#include "path_planning/waypoint.hpp"

namespace path_planning {

/**
 * @brief Aracın koordinatlarına en yakın waypoint'in indeksini bulur.
 * Sadece çok yüksek yaw hatalarına (ters şerit vs.) makul bir ceza uygulayarak 
 * uzak noktalara atlamasını engeller.
 */
int find_nearest_waypoint(const std::vector<Waypoint>& waypoints, double car_x, double car_y, double car_yaw) {
    if (waypoints.empty()) return -1;

    int nearest_idx = 0;
    double min_dist = 1e9;
    double min_cost = 1e9;

    for (size_t i = 0; i < waypoints.size(); ++i) {
        double dx = waypoints[i].x - car_x;
        double dy = waypoints[i].y - car_y;
        double dist = std::hypot(dx, dy);

        // Yaw farkını (-pi ile pi arasına çek)
        double yaw_diff = std::abs(waypoints[i].yaw - car_yaw);
        while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
        yaw_diff = std::abs(yaw_diff);

        // Cost, basitçe mesafe + yön sapması odaklıdır
        double cost = dist;
        
        // Ters yöndeki waypoints'lere ceza (örneğin M_PI/2 = 90 dereceden fazla)
        // Cezayı absürd bir rakam yerine +10.0 metre veriyoruz ki
        // algoritma yan şeride (+2m uzaklık, +10m ceza = 12m cost) atlamak yerine 
        // kendi şeridinde ileri/gerideki (+1m uzaklık, +0m ceza = 1m cost) noktaları seçsin.
        if (yaw_diff > M_PI / 2.0) {
            cost += 15.0; // Ters yön veya çok ters açı cezası (15 metre)
        }

        if (cost < min_cost) {
            min_cost = cost;
            nearest_idx = static_cast<int>(i);
        }
    }

    return nearest_idx;
}

// Eski imzayı geri uyumluluk için koruyoruz
int find_nearest_waypoint(const std::vector<Waypoint>& waypoints, double car_x, double car_y) {
    return find_nearest_waypoint(waypoints, car_x, car_y, 0.0);
}

} // namespace path_planning
