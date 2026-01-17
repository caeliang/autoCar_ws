# RViz + Harita Kaydetme Düzenlemeleri

## ✓ Yapılan Değişiklikler

### 1. Harita Numaralandırma (save_map.py)

**Problem:** Aynı isimle harita kaydedilince eski harita üzerine yazılıyor

**Çözüm:** Haritalar otomatik numaralandırılır

**Kullanım:**
```bash
python3 /home/ranim/autoCar_ws/scripts/save_map.py /home/ranim/autoCar_ws/maps/city_map
```

**Oluşturulan Dosyalar:**
- İlk kez: `city_map_001.pgm` + `city_map_001.yaml`
- İkinci kez: `city_map_002.pgm` + `city_map_002.yaml`
- Üçüncü kez: `city_map_003.pgm` + `city_map_003.yaml`
- ...

**Avantajlar:**
- Eski haritalar kaybolmaz
- Farklı bölgelerin haritalarını kaydetebilirsin
- Harita versiyonu kolayca takip edilir

---

### 2. RViz Görselleştirmesi

#### Seçenek 1: Hazır Konfigürasyon (Önerilen)

```bash
rviz2 -d /home/ranim/autoCar_ws/src/slam_mapping/rviz/slam_ekf.rviz
```

Bu otomatik olarak ekler:
- ✓ Map (/map) - SLAM haritası
- ✓ LaserScan (/prius/scan) - LiDAR verisi
- ✓ Odometry Local (/odometry/filtered/local) - EKF local (sarı ok)
- ✓ Odometry Global (/odometry/filtered/global) - EKF global (kırmızı ok)
- ✓ TF Tree - Transform ağacı

#### Seçenek 2: Manuel Konfigürasyon

Detaylı adımlar için:
```bash
python3 /home/ranim/autoCar_ws/scripts/setup_rviz_config.py
```

---

## RViz'de Görülmesi Gereken Şeyler

| Eleman | Açıklama | Renk |
|--------|----------|------|
| **Map Grid** | SLAM haritası | Gri |
| **LaserScan** | LiDAR noktaları | Kırmızı |
| **Local Arrow** | Odometry + IMU | Sarı |
| **Global Arrow** | GPS + EKF | Kırmızı |
| **TF Tree** | Transform ağacı | Beyaz çizgiler |

---

## Adım Adım Kullanım

### 1. Gazebo Başlat
```bash
ros2 launch prius_model simulation.launch.py
```

### 2. SLAM + EKF Başlat
```bash
bash /home/ranim/autoCar_ws/scripts/start_slam_ekf.sh
```

### 3. RViz Aç
```bash
rviz2 -d /home/ranim/autoCar_ws/src/slam_mapping/rviz/slam_ekf.rviz
```

### 4. Robot Hareket Ettir
```bash
python3 /home/ranim/autoCar_ws/scripts/keyboard_control.py
```

**W** = İleri | **S** = Geri | **A** = Sol | **D** = Sağ | **X** = Fren

### 5. Harita Kaydet
```bash
python3 /home/ranim/autoCar_ws/scripts/save_map.py /home/ranim/autoCar_ws/maps/city_map
```

Kontrol et:
```bash
ls -lah /home/ranim/autoCar_ws/maps/
```

---

## Sorun Giderme

### "RViz boş açılıyor"

**Çözüm 1: Fixed Frame kontrolü**
- RViz'in sol üst köşesinde "Fixed Frame" seç
- "map" olarak ayarla
- Tüm displays'ler görünmeli

**Çözüm 2: Topics kontrolü**
```bash
# Hangi topics yayınlanıyor check et
ros2 topic list | grep -E "(map|scan|odometry|filtered)"
```

**Çözüm 3: Nodes kontrolü**
```bash
# Nodes çalışıyor mu?
ros2 node list | grep -E "(slam|ekf)"
```

### "Harita görünmüyor"
- Alpha değerini 0.7 → 0 yap (daha opak)
- 15-20 saniye bekle (SLAM harita oluştururken)
- View → Reset View

### "LiDAR noktaları görünmüyor"
- /prius/scan'ı check et: `ros2 topic hz /prius/scan`
- LaserScan'ın Size (Pixels)'ını 5 yap

### "Oklar (Odometry) görünmüyor"
- EKF nodes'ları başlatıldı mı?
- /odometry/filtered/local topic'i check et
- Odometry displays'inin Position Tolerance'ı 0.1 olarak ayarla

### "Harita kaydedilemiyor"
```bash
# maps/ klasörü var mı?
mkdir -p /home/ranim/autoCar_ws/maps

# Tekrar dene
python3 /home/ranim/autoCar_ws/scripts/save_map.py /home/ranim/autoCar_ws/maps/test_map
```

---

## Komut Özeti

| Komut | Açıklama |
|-------|----------|
| `rviz2 -d .../slam_ekf.rviz` | Önceden yapılandırılmış RViz aç |
| `python3 setup_rviz_config.py` | RViz setup rehberi göster |
| `ros2 topic hz /map` | Map publish oranı check et |
| `ros2 run tf2_ros tf2_echo map odom` | map→odom TF check et |
| `python3 save_map.py /path/to/map` | Harita numaralı olarak kaydet |
| `ls -lah /home/ranim/autoCar_ws/maps/` | Kaydedilmiş haritaları listele |

---

## Dosyalar

| Dosya | Amaç | Durum |
|-------|------|-------|
| `scripts/save_map.py` | ✓ Harita kaydet (numaralandırma ile) | ✅ **AKTIF** |
| `src/slam_mapping/scripts/save_map.sh` | Eski harita kaydetme scripti | ⚠️ **DEPRECATED** |
| `src/slam_mapping/rviz/slam_ekf.rviz` | Önceden yapılandırılmış RViz config | ✅ AKTIF |
| `scripts/setup_rviz_config.py` | RViz setup rehberi | ✅ AKTIF |

### ⚠️ Uyarı

**`save_map.sh` artık kullanılmıyor!**

Harita kaydetme için **sadece `save_map.py` kullan:**

```bash
# ✅ DOĞRU (numaralandırma ile):
python3 /home/ranim/autoCar_ws/scripts/save_map.py /home/ranim/autoCar_ws/maps/city_map

# ❌ YANLIŞ (eski, kullanılmıyor):
bash /home/ranim/autoCar_ws/src/slam_mapping/scripts/save_map.sh
```

---

## Örnek Workflow

```bash
# Terminal 1: Gazebo
$ ros2 launch prius_model simulation.launch.py

# Terminal 2: SLAM + EKF
$ bash /home/ranim/autoCar_ws/scripts/start_slam_ekf.sh

# Terminal 3: RViz (YENI - hazır config ile)
$ rviz2 -d /home/ranim/autoCar_ws/src/slam_mapping/rviz/slam_ekf.rviz

# Terminal 4: Robot Kontrolü
$ python3 /home/ranim/autoCar_ws/scripts/keyboard_control.py

# Robot hareket ettir, harita oluşturuluyor

# Terminal 5: Harita Kaydet (mapping tamamlandıktan sonra)
$ python3 /home/ranim/autoCar_ws/scripts/save_map.py /home/ranim/autoCar_ws/maps/city_map

# Sonuç: city_map_001.pgm ve city_map_001.yaml oluşturulur

# Tekrar mapping yapıp kaydetersen:
$ python3 /home/ranim/autoCar_ws/scripts/save_map.py /home/ranim/autoCar_ws/maps/city_map
# city_map_002.pgm ve city_map_002.yaml oluşturulur
```

---

**Tarih:** 2026-01-16  
**Durum:** ✓ Harita Numaralandırma ve RViz Setup Tamamlandı
