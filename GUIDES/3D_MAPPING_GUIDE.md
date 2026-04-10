# 3D Point Cloud Mapping Guide

## Genel Bakış
Bu paket, 3D lidar verilerinden **Octomap** kullanarak 3D point cloud haritalar oluşturur ve kaydeder.

## Özellikler
- ✅ 3D Voxel tabanlı haritalama (Octomap)
- ✅ Gerçek zamanlı point cloud işleme
- ✅ Yükseklik bilgisi ile 3D ortam temsili
- ✅ Verimli sıkıştırma (.bt formatı)
- ✅ RViz görselleştirme
- ✅ 2D ve 3D SLAM birlikte çalışma

## Kurulum

### 1. Gerekli paketleri yükleyin:
```bash
sudo apt update
sudo apt install ros-humble-octomap-server \
                 ros-humble-octomap-ros \
                 ros-humble-octomap-msgs \
                 ros-humble-pcl-ros \
                 ros-humble-pcl-conversions \
                 octovis
```

### 2. Workspace'i derleyin:
```bash
cd /home/ranim/autoCar_ws
colcon build --packages-select slam_3d
source install/setup.bash
```

## Kullanım

### 1. Simülasyonu Başlatın
```bash
./run_compact_city.sh
```

### 2. 3D SLAM'i Çalıştırın
Yeni bir terminal açın:
```bash
./run_slam_3d.sh
```

Bu şunları başlatır:
- Octomap Server (3D haritalama)
- EKF sensör füzyonu
- SLAM Toolbox (2D karşılaştırma için)

### 3. RViz'i Başlatın (3D Görselleştirme)
Yeni bir terminal açın:
```bash
./run_rviz_3d.sh
```

Bu özel olarak 3D SLAM için hazırlanmış RViz konfigürasyonunu açar:
- 3D Octomap görselleştirmesi (renkli voxel'ler)
- Raw 3D Point Cloud (lidar verisi)
- Robot pozisyonu ve yörüngesi
- Dinamik kamera kontrolü

### 4. Haritayı Kaydedin (PCD Format - Önerilen) ⭐
En az 30 saniye harita oluşturduktan sonra:
```bash
./save_pcd_map.sh my_building
```

**Neden PCD?**
- ⚡ Çok hızlı (1 saniyede kaydeder)
- 📊 Her programda açılır (CloudCompare, PCL Viewer, RViz)
- ✅ Sorunsuz çalışır
- 📦 Makul dosya boyutu

Harita `/home/ranim/autoCar_ws/maps/` klasörüne kaydedilir.

### 5. Haritayı Optimize Edin (Localization için) 🔧
Kaydedilen ham harita localization için optimize edilmelidir:
```bash
./optimize_map.sh maps/my_building.pcd my_building_clean
```

**Optimizasyon Adımları:**
1. ✅ **Voxel Grid Filter** - Nokta yoğunluğunu düşürür (downsampling)
2. ✅ **Outlier Removal** - Gürültülü noktaları temizler
3. ✅ **Ground Segmentation** - Zemin noktalarını ayırır
4. ✅ **Height Filtering** - Sadece sürüş için gerekli yüksekliği tutar (0.3m - 3.0m)

**Faydaları:**
- 🚀 Localization daha hızlı çalışır
- 🎯 Daha stabil ve doğru konum tespiti
- 💾 Daha küçük dosya boyutu (60-80% azalma)
- 🧹 Gürültüsüz, temiz harita

**Karşılaştırma:**
```bash
# Önce - Ham harita
pcl_viewer maps/my_building.pcd

# Sonra - Optimize edilmiş harita  
pcl_viewer maps/my_building_clean.pcd
```

## RViz'de Görselleştirme

### Otomatik (Önerilen):
```bash
./run_rviz_3d.sh
```
Bu komut önceden yapılandırılmış 3D SLAM görselleştirmesini açar.

### Manuel RViz Konfigürasyonu:
Eğer RViz'i kendiniz ayarlamak isterseniz:

1. RViz'i açın: `rviz2`
2. Fixed Frame: `odom` olarak ayarlayın
3. "Add" → "By topic" → `/occupied_cells_vis_array` → "MarkerArray"
4. "Add" → "By topic" → `/free_cells_vis_array` → "MarkerArray" (opsiyonel)
5. "Add" → "By topic" → `/prius/scan` → "PointCloud2"
6. "Add" → "By topic" → `/odometry/filtered` → "Odometry"

### Görselleştirme Topic'leri:
- `/occupied_cells_vis_array` ⭐ - 3D Voxel harita (ana görselleştirme)
- `/free_cells_vis_array` - Boş alan görselleştirmesi
- `/octomap_binary` - Binary octomap
- `/octomap_full` - Full octomap
- `/octomap_point_cloud_centers` - Point cloud representation
- `/projected_map` - 2D projeksiyon
- `/prius/scan` - Raw 3D point cloud (cyan renk)
- `/odometry/filtered` - Robot pozisyonu (yeşil ok)
- `/path` - Yörünge

## Dosya Formatları

### .pcd (Point Cloud Data) - ⭐ Önerilen
- ⚡ Çok hızlı kayıt (1-2 saniye)
- 📂 Universal format (her yerde açılır)
- ✅ Sorunsuz çalışır
- 📊 ASCII veya binary
```bash
# Görüntüle
pcl_viewer maps/my_map.pcd
cloudcompare maps/my_map.pcd

# RViz'de aç
# Add → PointCloud2 → File → maps/my_map.pcd
```

### .bt (Binary Octree) - İsteğe bağlı
- 💾 En küçük dosya boyutu
- 🐌 Kayıt yavaş (servis sorunları olabilir)
- 🤖 Robotik navigasyon için
- Sadece octomap için kullanılır

**Tavsiye**: PCD kullan, daha pratik! 🎯

## Konfigürasyon

### Çözünürlük Ayarı
[config/octomap_params.yaml](config/octomap_params.yaml) dosyasında:
```yaml
resolution: 0.1  # 10cm voxel boyutu
```

Daha yüksek detay için: `0.05` (5cm)
Daha hızlı işleme için: `0.2` (20cm)

### Yükseklik Filtreleme
```yaml
height_min: -0.5  # Yerden 0.5m aşağı
height_max: 3.0   # Yerden 3m yukarı
```

### Sensör Menzili
```yaml
sensor_model:
  max_range: 30.0  # 30 metre maksimum
```

## Komutlar Özeti

| Komut | Açıklama |
|-------|----------|
| `./run_compact_city.sh` | Gazebo simülasyonunu başlat |
| `./run_slam_3d.sh` | 3D SLAM sistemini başlat |
| `./run_rviz_3d.sh` ⭐ | 3D RViz görselleştirmesini aç |
| `./test_3d_slam.sh` 🔍 | Sistemin çalışıp çalışmadığını kontrol et |
| `./save_pcd_map.sh [isim]` ⚡ | Haritayı PCD olarak kaydet (ÖNERİLEN) |
| `./optimize_map.sh [input.pcd] [output_name]` 🔧 | Haritayı localization için optimize et |
| `./debug_3d_slam.sh` 🐛 | Detaylı sorun tespiti |
| `pcl_viewer maps/my_map.pcd` | Kaydedilmiş haritayı görüntüle |

## Topic'ler

### Subscribed:
- `/prius/scan` (sensor_msgs/PointCloud2) - 3D Lidar verisi

### Published:
- `/octomap_binary` - Binary octomap
- `/octomap_full` - Full octomap  
- `/occupied_cells_vis_array` - Görselleştirme için marker'lar
- `/map` - 2D occupancy grid projection

## Karşılaştırma: 2D vs 3D

| Özellik | 2D SLAM | 3D SLAM (Octomap) |
|---------|---------|-------------------|
| Dosya boyutu | ~1 MB | ~5-10 MB |
| Yükseklik bilgisi | ❌ | ✅ |
| Çok katlı haritalama | ❌ | ✅ |
| İşlem gücü | Düşük | Orta |
| Bellek kullanımı | Az | Orta |
| Engel tespiti | 2D düzlem | 3D tam |

## Sorun Giderme

### ⚠️ Point cloud kalıcı değil, taradığım yerler kaybolyor:

**Sorun**: RViz'de sadece anlık point cloud görünüyor, Octomap oluşmuyor.

**Çözüm**:
```bash
# 1. Sistemin çalıştığını kontrol et
./test_3d_slam.sh

# 2. Her şey çalışıyorsa, 5-10 saniye bekle
# Octomap biriktirme yapıyor, hemen görünmeyebilir

# 3. RViz'de doğru topic'i ekle:
#    Add → MarkerArray → /octomap_server/occupied_cells_vis_array
```

**Neden olur**: Octomap server birkaç saniye veri biriktirdikten sonra yayın yapar. Sabırlı ol!

### ⚠️ "Waiting for octomap service..." hatası:

**Sorun**: Harita kaydetmeye çalışınca sürekli bekliyor.

**Çözüm**:
```bash
# 1. 3D SLAM'in çalıştığından emin ol
ros2 node list | grep octomap_server

# Çıktı yoksa:
./run_slam_3d.sh

# 2. En az 30 saniye harita oluştur (robotla dolaş)
# 3. Sonra kaydet:
./save_3d_map.sh my_map
```

### Harita oluşturulmuyor:
```bash
# Hızlı test
./test_3d_slam.sh

# Detaylı debug
./debug_3d_slam.sh

# Manuel kontrol
ros2 service list | grep octomap
ros2 topic echo /prius/scan --once
ros2 topic hz /octomap_server/occupied_cells_vis_array
```

### RViz'de görünmüyor:
1. **Fixed Frame**: `odom` olmalı (artık otomatik)
2. **Topic**: `/octomap_server/occupied_cells_vis_array` ekle
3. **Display Type**: MarkerArray
4. **Bekle**: 5-10 saniye veri birikiyor
5. **Robot hareket ettir**: Duruyorsa harita güncellenmiyor

## İleri Düzey

### Haritayı temizle:
```bash
ros2 service call /octomap_server/reset std_srvs/srv/Empty
```

### Harita güncellemelerini durdur:
```bash
ros2 service call /octomap_server/pause_updates std_srvs/srv/Empty
```

### Harita güncellemelerini devam ettir:
```bash
ros2 service call /octomap_server/resume_updates std_srvs/srv/Empty
```

## Map Optimization Detayları

### Voxel Grid Filter
```yaml
voxel_size: 0.1  # 10cm küpler halinde birleştir
```
- Yakın noktaları tek noktaya indirir
- Dosya boyutunu küçültür
- İşlem hızını artırır

### Outlier Removal
```yaml
nb_neighbors: 20     # Her nokta için komşu sayısı
std_ratio: 2.0       # Standart sapma eşiği
```
- Gürültülü ölçümleri temizler
- Sensör hatalarını giderir
- Localization doğruluğunu artırır

### Ground Segmentation
```yaml
distance_threshold: 0.2  # Zemin düzleminden mesafe (metre)
```
- Yol yüzeyini kaldırır
- Sadece duvar, bina gibi landmark'ları tutar
- Localization için kritik!

### Height Filtering
```yaml
min_height: 0.3  # Minimum yükseklik
max_height: 3.0  # Maksimum yükseklik
```
- Arabanın görebildiği yükseklikteki engelleri tutar
- Gökyüzü, yer altı noktalarını atar

### Manuel Optimizasyon Ayarları
[src/slam_3d/scripts/optimize_map.py](src/slam_3d/scripts/optimize_map.py) dosyasında:
```python
# Daha agresif downsampling
voxel_size=0.15  # Default: 0.1

# Daha fazla gürültü temizliği
std_ratio=1.5    # Default: 2.0

# Daha geniş yükseklik aralığı
min_height=0.2   # Default: 0.3
max_height=4.0   # Default: 3.0
```

## Performans İpuçları

1. **Düşük çözünürlük** kullanın (0.1-0.2m) geniş alanlar için
2. **Yüksek çözünürlük** kullanın (0.05m) detaylı iç mekan için
3. **Yükseklik filtresi** kullanarak gereksiz verileri azaltın
4. **Compression** aktif tutun (config'de `compress_map: true`)
5. **Her zaman optimize edin** - Ham haritayı localization'da kullanmayın!

## Sonraki Adımlar

- [ ] Navigation için 3D haritaları kullan
- [ ] Çoklu katman haritalar oluştur
- [ ] RGB point cloud desteği ekle
- [ ] Real-time collision detection ekle
