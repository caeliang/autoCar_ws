# Path Planning Package Structure

Organized modular structure for path planning, waypoint tracking, and autonomous vehicle control.

## Directory Layout

```
path_planning/
├── src/
│   ├── core/                          # Utility libraries and base components
│   │   ├── waypoint_utils.cpp         # Waypoint distance/angle calculations
│   │   ├── waypoint_io.cpp            # CSV/file I/O operations
│   │   └── waypoint_visualizer.cpp    # RViz visualization helpers
│   │
│   ├── nodes/                         # ROS2 Node implementations
│   │   ├── waypoint_tracker.cpp       # Tracks nearest waypoint to vehicle
│   │   ├── car_index_finder.cpp       # Converts world coords → grid matrix index
│   │   ├── waypoint_manager_node.cpp  # Manages waypoint sequences
│   │   ├── pure_pursuit_node.cpp      # Pure pursuit control algorithm
│   │   └── lane_path_follower.cpp     # Lane following controller
│   │
│   ├── algorithms/                    # Pathfinding and planning algorithms
│   │   ├── path_planning_main.cpp     # A* + ROS integration (main entry point)
│   │   └── path_planning_export.cpp   # Export paths to CSV
│   │
│   └── test_nearest_node.cpp          # (Legacy) single vehicle tester
│
├── scripts/
│   ├── path_planning/                 # Pathfinding and route generation
│   │   ├── generate_route.py          # A* algorithm with yaw-aware navigation
│   │   ├── generate_full_map.py       # Build complete road map from network
│   │   └── road_network.py            # Road network definition
│   │
│   ├── visualization/                 # Visualization & recording tools
│   │   ├── waypoint_visualizer.py     # Plot waypoints on grid
│   │   └── waypoint_recorder.py       # Record vehicle trajectories
│   │
│   └── test_nearest_waypoint.py       # Test node for tracking + indexing
│
├── include/path_planning/
│   ├── core/                          # Headers for core utilities
│   ├── nodes/                         # Headers for node implementations
│   ├── algorithms/                    # Headers for algorithms
│   ├── waypoint.hpp                   # Waypoint data structure
│   ├── waypoint_io.hpp                # I/O function declarations
│   └── waypoint_visualizer.hpp        # Visualization headers
│
├── config/                            # YAML/parameter files
│   └── waypoint_params.yaml
│
├── launch/                            # ROS2 launch files
│   ├── waypoint_follow.launch.py      # Launch waypoint follower
│   ├── waypoint_record.launch.py      # Launch waypoint recorder
│   └── waypoint_viz.launch.py         # Launch visualization
│
├── CMakeLists.txt                     # Build configuration
├── package.xml                        # ROS2 package metadata
└── STRUCTURE.md                       # This file

```

## Module Descriptions

### Core (`src/core/`)
**Purpose**: Reusable utility functions and base components

| File | Purpose |
|------|---------|
| `waypoint_utils.cpp` | Find nearest waypoint, calculate distances, yaw penalties |
| `waypoint_io.cpp` | Load/save CSVs, coordinate transformations |
| `waypoint_visualizer.cpp` | RViz marker creation, visualization helpers |

### Nodes (`src/nodes/`)
**Purpose**: ROS2 Node implementations for real-time vehicle control

| Node | Functionality |
|------|---------------|
| `waypoint_tracker` | Subscribes to `/prius/odom`, publishes nearest waypoint to `/prius/nearest_waypoint` |
| `car_index_finder` | Subscribes to waypoint, converts to grid index, publishes to `/prius/car_matrix_index` |
| `waypoint_manager` | Manages waypoint sequences, tracks progress |
| `pure_pursuit_node` | Pure pursuit steering control |
| `lane_path_follower` | Lane-based path following |

### Algorithms (`src/algorithms/`)
**Purpose**: Pathfinding and planning algorithms

| Program | Purpose |
|---------|---------|
| `path_planning_main` | Main ROS2 application: A* planning + waypoint tracking + grid indexing |
| `path_planning_export` | Export computed paths to CSV format |

### Scripts (`scripts/`)
**Purpose**: Python utilities for offline processing and testing

#### Path Planning Scripts (`scripts/path_planning/`)
- **generate_route.py**: A* pathfinding with yaw-aware navigation
- **generate_full_map.py**: Build complete road map from road network
- **road_network.py**: Road network graph definition and utilities

#### Visualization Scripts (`scripts/visualization/`)
- **waypoint_visualizer.py**: Plot and visualize waypoints on grid
- **waypoint_recorder.py**: Record vehicle trajectories

#### Testing
- **test_nearest_waypoint.py**: Integrated test for tracker + finder

## Build & Usage

### Build
```bash
cd /home/ranim/autoCar_ws
colcon build --packages-select path_planning
```

### Run Main Path Planning
```bash
source install/setup.bash
ros2 run path_planning path_planning_main 45 55
```
*Generates route from current vehicle position to grid (45, 55)*

### Run Individual Nodes
```bash
ros2 run path_planning waypoint_tracker
ros2 run path_planning car_index_finder
ros2 run path_planning waypoint_manager
```

### Run Python Scripts
```bash
python3 scripts/path_planning/generate_route.py 3 3 45 55      # Start (3,3), Goal (45,55)
python3 scripts/visualization/waypoint_visualizer.py
python3 scripts/test_nearest_waypoint.py
```

## Key Features

✅ **Modular Design**: Each component has a single responsibility
✅ **ROS2 Integration**: Pub/sub communication between nodes  
✅ **A* Pathfinding**: Yaw-aware navigation with lane penalties
✅ **Grid Mapping**: World → grid coordinate transformation (1.5x scaled)
✅ **Visualization**: RViz markers and Python plotting tools
✅ **Testing**: Integrated test scripts for development

## Configuration

- **Grid Size**: 68×59 (height × width)
- **Coordinate Scale**: 1.5x (world coordinates normalized)
- **World Bounds**: X ∈ [-43.2, 43.2], Y ∈ [-48.75, 50.7]
- **Default Waypoint File**: `waypoints/full_road_map.csv`
- **Default Grid File**: `matrices/road_grid_4wide.txt`

## Topics (ROS2)

| Topic | Type | Purpose |
|-------|------|---------|
| `/prius/odom` | nav_msgs/Odometry | Vehicle odometry (input) |
| `/prius/nearest_waypoint` | geometry_msgs/Point | Nearest waypoint output |
| `/prius/car_matrix_index` | geometry_msgs/Point | Grid matrix index output |

---

**Last Updated**: April 10, 2026  
**Maintainer**: Path Planning Team
