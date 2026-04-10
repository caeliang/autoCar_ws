# 🚀 SIMULATOR'DA PATH TETKİP KONTROL SİSTEMİ

## ✅ Sistem Durumu

```
✓ A* Path Planner (generate_route.py)
✓ Waypoint Tracking (waypoint_tracker.cpp)
✓ Pure Pursuit Controller (pure_pursuit_node.cpp → control package)
✓ ROS2 Node Integration
✓ Launch Files Ready
✓ All packages compile successfully
```

---

## 🎯 Simulator'da Test Etmek

### Hızlı Start (2 Terminal)

**Terminal 1 - Gazebo:**
```bash
cd ~/autoCar_ws
source install/setup.bash
ros2 launch gazebo_traffic_light_plugin gazebo_prius.launch.py
```

**Terminal 2 - Full Autonomous System:**
```bash
cd ~/autoCar_ws
source install/setup.bash
ros2 launch path_planning full_autonomous_system.launch.py \
  waypoint_file:=waypoints/full_road_map.csv
```

✅ Şimdi araba yolu takip etmeye başlamalı

---

## 🔧 Sistem Mimari

### Data Flow
```
A* Planner
    ↓
Path → /waypoints/path
    ↓
Pure Pursuit
    ├─ Reads: /prius/odom (heading)
    ├─ Reads: /localization/pose (position)
    ├─ Reads: /waypoints/path (target)
    └─ Publishes: /prius/cmd_vel → Gazebo
```

### ROS Topics

| Topic | Type | Yön | Frekans | Not |
|-------|------|-----|---------|-----|
| `/waypoints/path` | nav_msgs/Path | pub: waypoint_manager | 1 Hz | A* output |
| `/prius/odom` | nav_msgs/Odometry | pub: Gazebo | 50 Hz | Heading |
| `/prius/cmd_vel` | geometry_msgs/Twist | pub: pure_pursuit | 20 Hz | **Gazebo input** |
| `/localization/pose` | geometry_msgs/PoseStamped | pub: ICP | 50 Hz | Position |

---

## 📊 Kontrol Parametreleri (Runtime Tuning)

### Default
```yaml
max_speed: 2.0        # m/s
max_omega: 1.0        # rad/s
lookahead_ratio: 1.5  # dynamic lookahead = speed * ratio
```

### Slow & Stable
```bash
ros2 param set /pure_pursuit max_speed 1.0
ros2 param set /pure_pursuit max_omega 0.6
```

### Fast & Responsive
```bash
ros2 param set /pure_pursuit max_speed 3.0
ros2 param set /pure_pursuit max_omega 1.2
```

---

## 🐛 Debug Komutları

### Topics Monitor
```bash
# Tüm topics listesi
ros2 topic list

# A* path'i kontrol et
ros2 topic echo /waypoints/path --once

# Pure Pursuit steering output
ros2 topic echo /prius/cmd_vel

# Robot pozisyonu
ros2 topic echo /prius/odom --once

# ICP localization (if available)
ros2 topic echo /localization/pose --once
```

### Nodes Kontrol
```bash
# Running nodes
ros2 node list

# Node info
ros2 node info /pure_pursuit
ros2 node info /waypoint_manager

# Param list
ros2 param list /pure_pursuit
ros2 param get /pure_pursuit max_speed
```

### Logs
```bash
# Pure Pursuit logs (20 Hz debug output her 3 saniye)
ros2 run control pure_pursuit

# Waypoint Tracker logs
ros2 run path_planning waypoint_tracker
```

---

## ⚠️ Common Issues

### Issue #1: "No messages received"
```bash
# Kontrol et
ros2 topic list | grep waypoints
ros2 topic list | grep cmd_vel

# Eğer boş → nodes başlamadı
ros2 node list
```

**Fix:** Full system launch'ı tekrar başlat
```bash
ros2 launch path_planning full_autonomous_system.launch.py \
  waypoint_file:=waypoints/full_road_map.csv
```

### Issue #2: Robot hareket etmiyor
```bash
# cmd_vel yayınlanıyor mu?
ros2 topic echo /prius/cmd_vel | head -5

# Gazebo listening mi?
# (Gazebo terminalde error var mı?)
```

**Fix:** 
1. Gazebo tamamen yeniden başlat
2. ROS_DOMAIN_ID kontrol et: `echo $ROS_DOMAIN_ID` (0 olmalı)

### Issue #3: Robot yolu takip etmiyor (zig-zag)
**Problem:** Pure Pursuit parametreleri uygun değil

**Çözüm:**
```bash
# Daha stabil
ros2 param set /pure_pursuit max_speed 1.0
ros2 param set /pure_pursuit lookahead_ratio 1.0

# Deneyelim...
```

### Issue #4: Path generate edilmiyor
```bash
# Check A* script
python3 scripts/path_planning/generate_route.py \
  --start 3 3 --goal 45 55 --output test_path.csv

# Output var mı?
head -5 test_path.csv
```

---

## 🎮 Manual Path Test (A* only)

A* path'inin doğru üretilip üretilmediğini test et:

```bash
cd ~/autoCar_ws
python3 scripts/path_planning/generate_route.py \
  --start 10 10 \
  --goal 40 50 \
  --output test_route.csv

# Visualize
python3 scripts/visualize_path.py \
  matrices/road_grid_4wide.txt \
  test_route.csv \
  10 10 40 50
```

---

## 📈 Sistem Performansı

### Beklenen Davranış

1️⃣ **Başlama (ilk 2 saniye)**
   - Pure Pursuit başlanıyor
   - `/prius/odom` ve `/waypoints/path` bekleniyor
   - Henüz cmd_vel yayınlanmıyor

2️⃣ **Bağlantı kurma (2-5 saniye)**
   - Path alındı
   - Odometry alındı
   - İlk steering command oluşturuluyor

3️⃣ **Hareket (5+ saniye)**
   ```
   Robot şöyle çalışıyor:
   - Heading check: |error| > 60° ise DUR & DÖN
   - Heading OK (<30°): İLERİ GİT + Pure Pursuit
   - Lookahead: dynamic (hıza orantılı)
   - Update freq: 20 Hz
   ```

### Loglar (her 3 saniye)
```
Pos(10.5,12.3) yaw=45° path_yaw=42° h_err=3° alpha=15° v=2.0 w=0.35
```

Açıklama:
- **Pos:** Robot pozisyonu (m)
- **yaw:** Robot heading (°)
- **path_yaw:** Hedef heading (°)
- **h_err:** Heading hatası (°)
- **alpha:** Lateral offset açısı (°)
- **v:** Hız (m/s)
- **w:** Angular velocity (rad/s)

---

## 🔍 Advanced Tuning

### Heading Gain (Pure Pursuit'ta hardcoded)
Şu andaki: `0.8 * heading_error`

Değiştirmek için `src/control/src/pure_pursuit_node.cpp`'de:
```cpp
// Line ~260
omega = speed * curvature - 0.8 * heading_err;  // ← burası
```

Denemeler:
- 0.2 → smooth but slow turning
- 0.8 → balanced (default)
- 1.5 → aggressive, may oscillate

### Lookahead Distance
Şu andaki: dinamik `speed * 1.5` (1.5-5.0m limit)

Değiştirmek için:
```cpp
// Line ~50-53
declare_parameter<double>("lookahead_ratio", 1.5);  // ← ratio
declare_parameter<double>("min_lookahead", 1.5);    // ← min
declare_parameter<double>("max_lookahead", 5.0);    // ← max
```

---

## 📝 Files Reference

```
autoCar_ws/
├── SIMULATOR_INTEGRATION_GUIDE.md         # Detaylı kılavuz
├── SYSTEM_OVERVIEW.md                     # Bu dosya
├── test_autonomous_system.sh               # Setup verifier
│
├── src/control/
│   ├── launch/
│   │   ├── pure_pursuit.launch.py         # Pure Pursuit standalone
│   │   └── lane_control.launch.py         # Lane controller
│   └── src/
│       └── pure_pursuit_node.cpp          # ✨ Main controller
│
└── src/path_planning/
    ├── launch/
    │   ├── full_autonomous_system.launch.py # ⭐ USE THIS
    │   ├── waypoint_follow.launch.py
    │   └── ...
    ├── scripts/path_planning/
    │   ├── generate_route.py               # A* planner
    │   └── ...
    └── src/
        ├── nodes/
        │   ├── waypoint_manager_node.cpp
        │   ├── waypoint_tracker.cpp
        │   └── ...
        └── core/
            └── waypoint_utils.cpp
```

---

## 🎯 Next Steps

1. ✅ Simulator başlat (Gazebo)
2. ✅ Full system launch et
3. ✅ Robot yolu takip etmeye başla
4. 🔄 Parametreleri tune et
5. 📈 Performance monitor et (RViz)
6. 🧪 Real-world test (optional)

---

## 📞 Support

- **Pure Pursuit output check:** `ros2 topic echo /prius/cmd_vel`
- **Path check:** `ros2 topic echo /waypoints/path --once`
- **Robot odom:** `ros2 topic echo /prius/odom --once`
- **Full verification:** `bash test_autonomous_system.sh`

---

**System Ready! 🚀**
