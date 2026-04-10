# 🛣️ Path Planning — Waypoint Sistemi

Otonom araç için **waypoint kayıt**, **yönetim** ve **Pure Pursuit takip** sistemi.

---

## 📋 İçindekiler

1. [Genel Bakış](#genel-bakış)
2. [Mimari](#mimari)
3. [Kurulum & Build](#kurulum--build)
4. [Kullanım](#kullanım)
   - [Adım 1: Waypoint Kayıt](#adım-1-waypoint-kayıt)
   - [Adım 2: Waypoint Takip](#adım-2-waypoint-takip)
5. [Node'lar](#nodelar)
   - [waypoint_recorder (Python)](#waypoint_recorder-python)
   - [waypoint_manager (C++)](#waypoint_manager-c)
   - [pure_pursuit (C++)](#pure_pursuit-c)
6. [Topic & Servis Haritası](#topic--servis-haritası)
7. [Waypoint Dosya Formatı](#waypoint-dosya-formatı)
8. [Parametreler](#parametreler)
9. [RViz Görselleştirme](#rviz-görselleştirme)
10. [Dosya Yapısı](#dosya-yapısı)

---

## Genel Bakış

Sistem iki aşamalı çalışır:

| Aşama | Ne Yaparsın | Araçlar |
|-------|-------------|---------|
| **Kayıt** | Localization açıkken aracı sür, ENTER ile waypoint kaydet | `waypoint_recorder` (Python) |
| **Takip** | Kaydedilen rotayı otomatik takip et | `waypoint_manager` + `pure_pursuit` (C++) |

> **Neden C++?**  Hesaplama yoğun kısımlar (waypoint yönetimi, Pure Pursuit kontrol döngüsü, marker yayını) tamamen C++ ile yazıldı. Sadece interaktif klavye girişi gerektiren recorder Python'da.

---

## Mimari

```
┌──────────────────────────────────────────────────────────────────┐
│                         RViz                                     │
│  /waypoints/markers (küre + ok + çizgi)                         │
│  /pure_pursuit/lookahead (hedef çizgisi + daire)                │
└──────────────────────────────────────────────────────────────────┘
         ▲                              ▲
         │                              │
┌────────┴─────────┐          ┌─────────┴──────────┐
│ waypoint_manager │          │   pure_pursuit     │
│      (C++)       │          │      (C++)         │
│                  │          │                    │
│ • YAML yükle     │ /goal    │ • Dynamic lookahead│
│ • Hedefe mesafe  ├─────────►│ • Curvature hesabı │
│ • Marker yayını  │          │ • Steering komutu  │
│ • Servisler      │          │ • Hız kontrolü     │
└────────┬─────────┘          └─────────┬──────────┘
         │                              │
         │ /localization/pose           │ /cmd_vel
         │ (map frame)                  │ (Twist)
         │                              ▼
┌────────┴──────────────────────────────────────────┐
│              Araç (Prius Gazebo)                   │
│  /prius/odom ──► pure_pursuit (hız bilgisi)       │
└───────────────────────────────────────────────────┘
```

---

## Kurulum & Build

```bash
cd ~/autoCar_ws
colcon build --packages-select path_planning
source install/setup.bash
```

### Bağımlılıklar

Standart ROS2 Humble paketleri — ek kütüphane gerekmez:

- `rclcpp`, `rclpy`
- `geometry_msgs`, `nav_msgs`, `visualization_msgs`
- `std_msgs`, `std_srvs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`

---

## Kullanım

### Ön Koşullar

1. **Gazebo** çalışıyor olmalı (`./scripts/run_compact_city.sh`)
2. **Localization** çalışıyor olmalı (`./scripts/run_localization.sh`)
3. `/localization/pose` topic'i `map` frame'inde yayın yapıyor olmalı

### Adım 1: Waypoint Kayıt

```bash
# Otomatik isimli dosya (waypoints/route_YYYYMMDD_HHMMSS.yaml)
./scripts/record_waypoints.sh

# Belirli bir dosyaya kaydet
./scripts/record_waypoints.sh waypoints/my_route.yaml
```

**Tuş komutları:**

| Tuş | İşlem |
|-----|-------|
| `ENTER` | Mevcut konumu waypoint olarak ekle |
| `d` | Son eklenen waypoint'i sil |
| `l` | Tüm waypoint'leri listele |
| `s` | Dosyaya kaydet |
| `q` | Kaydet ve çık |
| `Ctrl+C` | Kaydet ve çık |

**İpuçları:**
- Aracı klavyeyle sürerken keskin dönüşlere ekstra waypoint koy
- Uzun düzlüklerde 5–10m aralıkla yeterli
- Minimum waypoint mesafesi varsayılan 0.5m (çift kayıt önlenir)

### Adım 2: Waypoint Takip

```bash
# Temel kullanım
./scripts/follow_waypoints.sh waypoints/my_route.yaml

# Döngü modunda (son waypoint'ten sonra başa döner)
./scripts/follow_waypoints.sh waypoints/my_route.yaml --loop

# Hız ayarlı
./scripts/follow_waypoints.sh waypoints/my_route.yaml --speed 2.0

# Takip olmadan sadece görselleştirme
./scripts/follow_waypoints.sh waypoints/my_route.yaml --no-pursuit
```

**Veya launch dosyasıyla:**

```bash
# Kayıt modu
ros2 launch path_planning waypoint_record.launch.py

# Takip modu
ros2 launch path_planning waypoint_follow.launch.py \
    waypoint_file:=/home/ranim/autoCar_ws/waypoints/my_route.yaml \
    loop:=true \
    max_speed:=2.5
```

### Runtime Servisler

Çalışırken terminal'den:

```bash
# Mevcut konumu waypoint olarak ekle
ros2 service call /waypoints/add std_srvs/srv/Trigger

# Waypoint'leri dosyaya kaydet
ros2 service call /waypoints/save std_srvs/srv/Trigger

# Takibi başa al
ros2 service call /waypoints/reset std_srvs/srv/Trigger
```

---

## Node'lar

### waypoint_recorder (Python)

> `src/path_planning/scripts/waypoint_recorder.py`

Hafif, interaktif kayıt aracı. Terminal'den klavye girişi alır, `/localization/pose` dinler.

| Özellik | Değer |
|---------|-------|
| Dil | Python |
| Subscribe | `/localization/pose` (PoseStamped, map frame) |
| Çıktı | YAML dosyası |
| Parametre | `output_file`, `min_distance` |

### waypoint_manager (C++)

> `src/path_planning/src/waypoint_manager_node.cpp`

Ana yönetim node'u. Waypoint'leri yükler, hedefe olan mesafeyi hesaplar, sırayla ilerler, RViz'de gösterir.

| Özellik | Değer |
|---------|-------|
| Dil | C++ |
| Subscribe | `/localization/pose` |
| Publish | `/waypoints/goal`, `/waypoints/markers`, `/waypoints/status` |
| Servisler | `/waypoints/add`, `/waypoints/save`, `/waypoints/reset` |
| Parametre | `waypoint_file`, `frame_id`, `goal_tolerance`, `loop`, `publish_rate` |

**Durum makinesi:**
```
[YÜKLE] → [AKTİF: wp #0] → mesafe < tol → [AKTİF: wp #1] → ... → [BİTTİ]
                                                                        │
                                                              loop=true │
                                                                        ▼
                                                              [AKTİF: wp #0]
```

### pure_pursuit (C++)

> `src/path_planning/src/pure_pursuit_node.cpp`

Pure Pursuit geometrik yol takip algoritması. Ackermann steering modeli kullanır.

| Özellik | Değer |
|---------|-------|
| Dil | C++ |
| Subscribe | `/waypoints/goal`, `/localization/pose`, `/prius/odom` |
| Publish | `/cmd_vel` (Twist), `/pure_pursuit/lookahead` (Marker) |
| Algoritma | Pure Pursuit + dynamic lookahead |

**Algoritma detayı:**

1. **Dynamic Lookahead:** $L_d = \text{clamp}(v \times k, L_{min}, L_{max})$
2. **Hedefi araç koordinatlarına dönüştür** (local frame: x ileri, y sola)
3. **Eğrilik hesabı:** $\kappa = \frac{2 \sin(\alpha)}{L_d}$
4. **Direksiyon açısı:** $\delta = \arctan(\kappa \times L_{wb})$
5. **Hız kontrolü:** Keskin dönüşte yavaşla, hedefe yakınken yavaşla

---

## Topic & Servis Haritası

### Topic'ler

| Topic | Tip | Yön | Frame | Açıklama |
|-------|-----|-----|-------|----------|
| `/localization/pose` | PoseStamped | ← Input | `map` | Aracın konumu |
| `/prius/odom` | Odometry | ← Input | `odom` | Hız bilgisi |
| `/waypoints/goal` | PoseStamped | → Output | `map` | Aktif hedef waypoint |
| `/waypoints/markers` | MarkerArray | → Output | `map` | RViz marker'ları |
| `/waypoints/status` | String | → Output | — | `ACTIVE:2/10`, `FINISHED`, `NO_WAYPOINTS` |
| `/cmd_vel` | Twist | → Output | — | Hız + steering komutu |
| `/pure_pursuit/lookahead` | Marker | → Output | `map` | Lookahead çizgisi + daire |

### Servisler

| Servis | Tip | Açıklama |
|--------|-----|----------|
| `/waypoints/add` | Trigger | Mevcut konumu waypoint olarak ekle |
| `/waypoints/save` | Trigger | Waypoint'leri dosyaya kaydet |
| `/waypoints/reset` | Trigger | Takibi sıfırla (ilk waypoint'e dön) |

---

## Waypoint Dosya Formatı

YAML formatında, `~/autoCar_ws/waypoints/` dizininde saklanır:

```yaml
# Waypoint file — generated by path_planning
# Frame: map
# Total: 5 waypoints

waypoints:
  - x: -2.345600
    y: 12.789000
    z: 0.010000
    qx: 0.000000
    qy: 0.000000
    qz: 0.707107
    qw: 0.707107
    speed: 1.0

  - x: 5.123400
    y: 18.456000
    z: 0.010000
    qx: 0.000000
    qy: 0.000000
    qz: 0.000000
    qw: 1.000000
    speed: 1.5
```

| Alan | Tip | Açıklama |
|------|-----|----------|
| `x, y, z` | double | Pozisyon (map frame, metre) |
| `qx, qy, qz, qw` | double | Oryantasyon (quaternion) |
| `speed` | double | Hedef hız (m/s) — şu an kullanılmıyor, ileride aktif |

> ⚠️ `yaml-cpp` bağımlılığı yok — özel minimal parser kullanılıyor.

---

## Parametreler

### waypoint_manager

| Parametre | Varsayılan | Açıklama |
|-----------|-----------|----------|
| `waypoint_file` | `""` | YAML dosya yolu |
| `frame_id` | `"map"` | Koordinat frame'i |
| `goal_tolerance` | `1.5` | Waypoint'e ulaşıldı kabul mesafesi (m) |
| `loop` | `false` | Son waypoint'ten sonra başa dön |
| `publish_rate` | `10.0` | Yayın frekansı (Hz) |

### pure_pursuit

| Parametre | Varsayılan | Açıklama |
|-----------|-----------|----------|
| `lookahead_distance` | `3.0` | Temel lookahead mesafesi (m) |
| `min_lookahead` | `1.5` | Minimum lookahead (m) |
| `max_lookahead` | `6.0` | Maximum lookahead (m) |
| `lookahead_ratio` | `1.5` | Hız × oran = dinamik lookahead |
| `wheelbase` | `2.7` | Prius aks mesafesi (m) |
| `max_speed` | `3.0` | Maksimum hız (m/s) |
| `max_steering` | `0.6` | Maksimum direksiyon açısı (rad, ~34°) |
| `goal_tolerance` | `1.0` | Hedefe varış mesafesi (m) |
| `enabled` | `true` | Pure pursuit aktif/pasif |
| `publish_rate` | `20.0` | Kontrol döngüsü frekansı (Hz) |

### waypoint_recorder

| Parametre | Varsayılan | Açıklama |
|-----------|-----------|----------|
| `output_file` | `""` (otomatik) | Çıktı dosya yolu |
| `min_distance` | `0.5` | Min waypoint arası mesafe (m) |

Parametreler `config/waypoint_params.yaml` dosyasından veya launch argument olarak verilebilir.

---

## RViz Görselleştirme

Waypoint'ler RViz'de otomatik olarak gösterilir:

| Öğe | Renk | Açıklama |
|-----|------|----------|
| 🔵 Küre | Mavi | Gelecek waypoint'ler |
| 🟢 Küre | Yeşil (büyük) | Aktif hedef waypoint |
| ⚫ Küre | Gri (soluk) | Geçilmiş waypoint'ler |
| 🔢 Yazı | Beyaz | Waypoint numarası |
| ➡️ Ok | Sarı | Waypoint yönü (heading) |
| 〰️ Çizgi | Açık mavi | Waypoint'leri birleştiren rota çizgisi |
| 🟢 Çizgi | Yeşil | Pure pursuit: araçtan hedefe çizgi |
| 🟢 Daire | Yeşil (şeffaf) | Pure pursuit: lookahead yarıçapı |

**RViz'de ekle:**
- Add → By topic → `/waypoints/markers` → MarkerArray
- Add → By topic → `/pure_pursuit/lookahead` → Marker

---

## Dosya Yapısı

```
src/path_planning/
├── CMakeLists.txt                        # Build konfigürasyonu
├── package.xml                           # ROS2 paket tanımı
│
├── config/
│   └── waypoint_params.yaml              # Varsayılan parametreler
│
├── include/path_planning/
│   ├── waypoint.hpp                      # Waypoint struct + yardımcı fonksiyonlar
│   ├── waypoint_io.hpp                   # YAML okuma/yazma arayüzü
│   └── waypoint_visualizer.hpp           # RViz marker yayıncı sınıfı
│
├── launch/
│   ├── waypoint_record.launch.py         # Kayıt modu launch
│   └── waypoint_follow.launch.py         # Takip modu launch
│
├── scripts/
│   └── waypoint_recorder.py              # İnteraktif kayıt aracı (Python)
│
└── src/
    ├── waypoint_manager_node.cpp          # Waypoint yönetim node'u (C++)
    ├── pure_pursuit_node.cpp              # Pure Pursuit takip node'u (C++)
    ├── waypoint_io.cpp                    # YAML parser implementasyonu
    └── waypoint_visualizer.cpp            # Marker yayın implementasyonu

scripts/
├── record_waypoints.sh                   # Kayıt kısayol scripti
└── follow_waypoints.sh                   # Takip kısayol scripti

waypoints/                                # Kaydedilen rota dosyaları
└── route_YYYYMMDD_HHMMSS.yaml
```

---

## Hızlı Başlangıç

```bash
# 1. Build
cd ~/autoCar_ws && colcon build --packages-select path_planning && source install/setup.bash

# 2. Gazebo + Localization başlat (ayrı terminallerde)
./scripts/run_compact_city.sh
./scripts/run_localization.sh

# 3. Waypoint kaydet (aracı sürerken ENTER ile kaydet)
./scripts/record_waypoints.sh waypoints/route1.yaml

# 4. Kayıtlı rotayı takip et
./scripts/follow_waypoints.sh waypoints/route1.yaml
```
