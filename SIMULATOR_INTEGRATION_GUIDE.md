# 🚀 Simulator Integration - A* + Pure Pursuit

## System Architecture

```
┌─ Path Planning Pipeline ────────────────────┐
│                                             │
│  A* Planner                                 │
│  (generate_route.py)                        │
│         ↓                                   │
│  Planned Path (/waypoints/path)             │
│         ↓                                   │
│  Waypoint Manager                           │
│  Waypoint Tracker                           │
│  Car Index Finder                           │
│                                             │
└────────────────────┬────────────────────────┘
                     │
                     ↓ /waypoints/path
            ┌────────────────────┐
            │   Pure Pursuit     │
            │   Controller       │
            │                    │
            │ Inputs:            │
            │ - /prius/odom      │
            │ - /waypoints/path  │
            │ - /localization/pose
            │                    │
            │ Output:            │
            │ - /prius/cmd_vel   │
            └────────────────────┘
                     │
                     ↓
              ┌──────────────┐
              │ Gazebo Sim   │
              │              │
              │ • Prius      │
              │ • Motion     │
              │ • Sensors    │
              └──────────────┘
```

## 📋 Terminal Setup (5 terminals gerekli)

### Terminal 1: Gazebo Simulator
```bash
cd ~/autoCar_ws
source install/setup.bash
# Gazebo başlat (mevcut launch file'ını kullan)
ros2 launch gazebo_ros gazebo.launch.py
```

### Terminal 2: Localization (ICP)
```bash
cd ~/autoCar_ws
source install/setup.bash
# ICP localization
ros2 run perception icp_localization  # veya mevcut localization node
```

### Terminal 3: Full Autonomous System
```bash
cd ~/autoCar_ws
source install/setup.bash
# A* Path Planning + Pure Pursuit Control
ros2 launch path_planning full_autonomous_system.launch.py \
  waypoint_file:=waypoints/full_road_map.csv
```

### Terminal 4: Waypoint Visualization (optional)
```bash
cd ~/autoCar_ws
source install/setup.bash
ros2 run path_planning waypoint_visualizer
```

### Terminal 5: ROS2 Topic Monitor
```bash
cd ~/autoCar_ws
source install/setup.bash
# Monitor topics
ros2 topic echo /waypoints/path
# Başka terminalde:
ros2 topic echo /prius/cmd_vel
```

---

## 🔥 Sistem Akışı (Step by Step)

### Adım 1: Path Generation (Offline)
```bash
python3 scripts/path_planning/generate_route.py \
  --start 3 3 \
  --goal 45 55 \
  --output planned_route.csv
```
**Output:** `planned_route.csv` (A* path)

### Adım 2: Waypoint Manager Başlar
```
"Tamam, path'im var. /waypoints/status yayınlıyorum"
```
**Yayınlar:**
- `/waypoints/path` (nav_msgs/Path)
- `/waypoints/status` (std_msgs/String: "ACTIVE" veya "READY")

### Adım 3: Waypoint Tracker Başlar
```
"Robot konumunu dinliyorum (/prius/odom)"
"Ardından hayalet en yakın waypoint'i buluyorum"
```
**Yayınlar:**
- `/prius/nearest_waypoint` (geometry_msgs/Point)

### Adım 4: Car Index Finder Başlar
```
"Waypoint'i grid koordinatına çeviriyorum"
```
**Yayınlar:**
- `/prius/car_matrix_index` (geometry_msgs/Point - grid indeksleri)

### Adım 5: Pure Pursuit Başlar
```
"Path'i alıyorum, robot pozisyonunu biliyorum"
"Heading'i hesaplıyorum:"
  - target yaw = atan2(dy, dx)
  - current yaw = odom'dan
  - error = target - current
  
"Steering açısını hesaplıyorum (Pure Pursuit formula):"
  - alpha = 2*L*y / (x² + y²)
  - steering += 0.8 * heading_error
  
"cmd_vel yayınlıyorum → Gazebo"
```
**Yayınlar:**
- `/prius/cmd_vel` (geometry_msgs/Twist)
  - linear.y = -speed  (Prius burnu -Y'de)
  - angular.z = omega  (steering)

### Adım 6: Gazebo Hareket Eder
```
Robot önceki konumundan yeni konuma gider → /prius/odom güncellenir
```

### Adım 7: Loop (20 Hz)
Pure Pursuit 50ms'de bir (20 Hz):
1. Odom oku
2. Localization pose oku
3. Path oku
4. Steering hesapla
5. cmd_vel gönder
6. Tekrar

---

## 🎮 Kontrol Parametreleri (real-time tuning)

Pure Pursuit düzgün çalışması için:

```yaml
# Düşük hız (stabil, yavaş)
max_speed: 1.0        # m/s
lookahead_ratio: 1.0  # ≈ 1m lookahead

# Orta hız (balansız)
max_speed: 2.0
lookahead_ratio: 1.5

# Yüksek hız (çok duyarlı)
max_speed: 3.0
lookahead_ratio: 2.0
```

ROS param ile runtime değiştir:
```bash
ros2 param set /pure_pursuit max_speed 1.5
ros2 param get /pure_pursuit max_speed
```

---

## 📊 Debug Topics

### Monitoring
```bash
# Path'i hızlıca kontrol et
ros2 topic echo /waypoints/path --once

# Robot heading ve position
ros2 topic echo /prius/odom

# Pure Pursuit çıkış (steering)
ros2 topic echo /prius/cmd_vel

# Localization
ros2 topic echo /localization/pose
```

### Visualization (RViz)
```bash
ros2 run rviz2 rviz2 -d config/slam_3d.rviz

# Add:
# - /waypoints/path (Path)
# - /prius/odom (Odometry)
# - /localization/pose (PoseStamped)
# - /pure_pursuit/lookahead (Marker)
```

---

## ⚠️ Kritik Noktalar

### 1. Coordinate Frame
- **Harita:** Standard X-Y (X sağ, Y yukarı)
- **Gazebo (Prius):** Body frame (Y ileri, -Z yukarı)
- **Fix:** Pure Pursuit node'da `car_yaw_ = raw_yaw - M_PI_2`

### 2. Cmd_vel Yönü
```cpp
// ❌ YANLIŞ (standart)
cmd.linear.x = speed;

// ✅ DOĞRU (Prius)
cmd.linear.y = -speed;  // -Y = ileri
cmd.angular.z = omega;  // standard
```

### 3. Path Kalitesi
- Eğer path çok köşeli → robot sallanır
- **Fix:** Path smoothing (gradient descent)
- Mevcut: `generate_route.py` smoothing yapar

### 4. Lookahead Distance
Çok kısa:
- Robot titrer
- Stability azalır

Çok uzun:
- Keskin köşelerde turns miss edilir
- **Best:** `speed * 1.5` = dynamic lookahead

---

## 🚨 Troubleshooting

### Problem: Robot komut almıyor
```bash
# 1. Topics kontrol et
ros2 topic list | grep cmd_vel
ros2 topic list | grep waypoints

# 2. Pure Pursuit logları kontrol et
ros2 topic echo /prius/cmd_vel
# Eğer zero → pose_ok_, odom_ok_, path_ok_ kontrol et
```

### Problem: Robot yolu takip etmiyor
```bash
# Heading hatası çok mu?
ros2 topic echo /prius/odom
# Yaw değişiyor mu dönership sırasında?

# Eğer yok → localization vs odom mismatch
# Çözüm: heading offset düzelt (-π/2 vs 0)
```

### Problem: ROS IPC (localhost) problemi
```bash
# ROS_DOMAIN_ID kontrol et
echo $ROS_DOMAIN_ID

# Eğer farklı → sync et
export ROS_DOMAIN_ID=0
```

---

## 🎯 Next Steps

1. ✅ Launch files ready
2. ⏳ Gazebo başlat
3. ⏳ Full system launch et
4. ⏳ RViz'de visualize et
5. ⏳ Tuning parametreler
6. ⏳ Real test

---

## 📝 Topic List (Reference)

| Topic | Pub | Sub | Type | Hz |
|-------|-----|-----|------|-----|
| `/waypoints/path` | waypoint_manager | pure_pursuit | nav_msgs/Path | 1 |
| `/waypoints/status` | waypoint_manager | pure_pursuit | std_msgs/String | 1 |
| `/prius/odom` | Gazebo | pure_pursuit | nav_msgs/Odometry | 50 |
| `/prius/cmd_vel` | pure_pursuit | Gazebo | geometry_msgs/Twist | 20 |
| `/localization/pose` | ICP | pure_pursuit | geometry_msgs/PoseStamped | 50 |
| `/prius/nearest_waypoint` | waypoint_tracker | - | geometry_msgs/Point | 50 |
| `/prius/car_matrix_index` | car_index_finder | - | geometry_msgs/Point | 50 |
| `/pure_pursuit/lookahead` | pure_pursuit | RViz | visualization_msgs/Marker | 20 |

