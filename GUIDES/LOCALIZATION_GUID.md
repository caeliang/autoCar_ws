# Map-Based Localization Module

## 🚦 Hızlı Başlangıç — Lokalizasyonu Nasıl Çalıştırırım?

### Adım 1 — Gazebo Simülasyonunu Başlat
```bash
cd ~/autoCar_ws
./scripts/run_compact_city.sh
```

### Adım 2 — Harita Hazırla (ilk kez veya yeni harita gerekiyorsa)
```bash
# Yeni terminalde SLAM'ı başlat
./scripts/run_slam_3d.sh

# Başka terminalde arabayı klavye ile sür
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/prius/cmd_vel

# Haritayı kaydet (harita adı seçilebilir)
./scripts/save_pcd_map.sh test2
```
> Kaydedilen harita: `~/autoCar_ws/maps/test2.pcd`

### Adım 3 — Lokalizasyonu Başlat
```bash
# Yeni terminalde
cd ~/autoCar_ws
source install/setup.bash
ros2 launch localization simple_localization.launch.py \
    map_pcd_path:=$HOME/autoCar_ws/maps/test2.pcd
```

### Adım 4 — RViz ile Görselleştir
```bash
rviz2 -d ~/autoCar_ws/config/localization.rviz
```
RViz'de manuel eklemek istersen:
| Display Tipi | Topic | Renk |
|---|---|---|
| PointCloud2 | `/localization/map` | Mavi (harita) |
| PointCloud2 | `/localization/scan_in_map` | Kırmızı (canlı tarama) |
| Pose | `/localization/pose` | — |

> **Fixed Frame: `map` olmalı** — `odom` veya başka bir frame seçilirse hiçbir şey görünmez.

---

## 🎯 Modular 3D Localization System

Bu sistem **kolayca değiştirilebilir ve devre dışı bırakılabilir** şekilde tasarlanmıştır.

---

## 📦 Dosyalar

### Core Files (Ana Dosyalar)
- **[map_based_localizer.py](src/localization/scripts/map_based_localizer.py)** - Localization node (ICP-based)
- **[map_localization.launch.py](src/localization/launch/map_localization.launch.py)** - Launch dosyası
- **[run_localization.sh](scripts/run_localization.sh)** - Başlatma scripti

### Modular Design (Modüler Yapı)
- `LocalizationBackend` base class → Kolayca yeni algoritmalar eklenebilir
- `ICPBackend` (şu an aktif) → Point-to-Point ICP
- **Gelecekte eklenebilir:** `NDTBackend`, `GICPBackend`, etc.

---

## 🚀 Kullanım

### 1. Harita Oluştur (Bir kez yapılır)
```bash
# 1. Gazebo'yu başlat
./scripts/run_compact_city.sh

# 2. SLAM'ı başlat (yeni terminalde)
./scripts/run_slam_3d.sh

# 3. Arabayı sür ve harita oluştur
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/prius/cmd_vel

# 4. Haritayı kaydet
./scripts/save_pcd_map.sh my_city_map
```

### 2. Localization Kullan
```bash
# 1. Gazebo çalışıyor olmalı
./scripts/run_compact_city.sh

# 2. Localization'ı başlat (harita adını belirt)
./scripts/run_localization.sh my_city_map
```

### 3. Görselleştir
```bash
rviz2
# Add:
# - Fixed Frame: map
# - PointCloud2 → /localization/map_cloud (map - kırmızı)
# - PointCloud2 → /prius/scan (current scan - cyan)
# - PoseWithCovariance → /localization/pose
```

---

## ⚙️ Parametreler (Kolayca Değiştirilebilir)

[map_localization.launch.py](src/localization/launch/map_localization.launch.py) içinde:

```python
parameters=[{
    'localization_method': 'icp',           # icp, ndt, gicp (sadece icp implement)
    'max_correspondence_distance': 0.5,     # ICP max mesafe (metre)
    'voxel_size': 0.2,                      # Downsampling boyutu
    'scan_topic': '/prius/scan',            # Input point cloud
    'odom_topic': '/odometry/filtered',     # Odometry (initial guess)
}]
```

---

## 🔄 Algoritma Değiştirme (Gelecek)

### NDT Eklemek İçin:
```python
# map_based_localizer.py içine ekle:

class NDTBackend(LocalizationBackend):
    def __init__(self, map_cloud):
        super().__init__(map_cloud)
        # NDT implementation
    
    def localize(self, scan_cloud, initial_pose):
        # NDT matching
        return transformation, fitness
```

Sonra launch file'da:
```bash
ros2 launch localization map_localization.launch.py \
    localization_method:=ndt
```

---

## ❌ Localization'ı Devre Dışı Bırakma

**Çok basit:** `run_localization.sh` scriptini çalıştırma!

Sistem geri kalan her şeyi (SLAM, EKF, görselleştirme) bağımsız olarak çalışır.

```bash
# Sadece SLAM + EKF (localization olmadan)
./scripts/run_compact_city.sh
./scripts/run_slam_3d.sh
./scripts/run_rviz_3d.sh
```

---

## 📊 Topics

### Input (Girdi)
- `/prius/scan` - 3D LiDAR point cloud
- `/odometry/filtered` - EKF odometry (initial guess)

### Output (Çıktı)
- `/localization/pose` - Localized pose (PoseWithCovarianceStamped)
- `/localization/map_cloud` - Map visualization (PointCloud2)
- `map → chassis` TF transform

---

## 🔧 Bağımlılıklar

Python paketi:
```bash
pip3 install open3d scipy
```

ROS2 paketleri (zaten yüklü):
- `rclpy`
- `sensor_msgs`
- `nav_msgs`
- `geometry_msgs`
- `tf2_ros`

---

## 🎯 Avantajlar

✅ **Modüler**: Algoritma kolayca değiştirilebilir  
✅ **Bağımsız**: Devre dışı bırakılabilir, sistem çalışmaya devam eder  
✅ **Lightweight**: hdl_localization'dan daha hafif  
✅ **Özelleştirilebilir**: Tüm parametreler açık  
✅ **ROS2 Native**: Başka paket kurulumu gerekmez  

---

## 📝 Notlar

- **İlk kullanımda**: `open3d` ve `scipy` install edilmezse script otomatik yükler
- **Map güncellemesi**: Yeni harita oluşturduğunuzda aynı script ile çalışır
- **Performans**: Voxel size'ı artırarak hızlandırabilirsiniz (0.2 → 0.3)
- **Doğruluk**: Max correspondence distance'ı azaltarak iyileştirebilirsiniz (0.5 → 0.3)

---

## 🔮 Gelecek Geliştirmeler

- [ ] NDT backend ekle
- [ ] GICP backend ekle  
- [ ] Loop closure detection
- [ ] Multi-map support
- [ ] Dynamic reconfigure

---

## 📧 Sorun mu var?

1. **Map yüklenmiyor**: `maps/` klasöründe `.pcd` dosyası var mı kontrol et
2. **Localization çalışmıyor**: Gazebo ve sensörler çalışıyor mu kontrol et
3. **Pose yanlış**: `max_correspondence_distance` parametresini ayarla

