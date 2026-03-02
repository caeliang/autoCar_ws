#pragma once

#include <string>
#include "path_planning/waypoint.hpp"

namespace path_planning {

/// Load waypoints from YAML file.
/// Format (basit):
///   waypoints:
///     - x: 1.0
///       y: 2.0
///       yaw: 0.0
///
/// Eski format (qx/qy/qz/qw/z/speed) da desteklenir — geriye uyumluluk.
WaypointList loadWaypoints(const std::string& filepath);

/// Load waypoints from CSV file.
/// CSV format: index,x,y,yaw
WaypointList loadWaypointsCSV(const std::string& filepath);

/// Save waypoints to YAML file (sadece x, y, yaw)
bool saveWaypoints(const std::string& filepath, const WaypointList& waypoints);

}  // namespace path_planning
