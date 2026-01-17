#!/usr/bin/env python3
"""
RViz Configuration Guide and Setup Tool for SLAM + EKF
"""

import os
import subprocess
import sys

def print_header(title):
    print(f"\n{'='*70}")
    print(f"  {title:^66}")
    print(f"{'='*70}")

def print_section(title):
    print(f"\n{title}")
    print(f"{'-' * 70}")

def main():
    workspace = "/home/ranim/autoCar_ws"
    rviz_config = os.path.join(workspace, 'src/slam_mapping/rviz/slam_ekf.rviz')
    
    print_header("RViz Setup for SLAM + EKF")
    
    print("""
Bu araç SLAM + EKF sistemi RViz'de görselleştirmek için yardım eder.
""")
    
    print_section("SEÇENEK 1: Önceden yapılandırılmış config kullan")
    
    if os.path.exists(rviz_config):
        print(f"""
✓ Hazır config mevcut: {rviz_config}

Kullan:
  rviz2 -d {rviz_config}

Bu konfigürasyon otomatik olarak ekler:
  • Map (/map) - SLAM haritası
  • LaserScan (/prius/scan) - LiDAR verisi
  • Odometry Local (/odometry/filtered/local) - EKF local
  • Odometry Global (/odometry/filtered/global) - EKF global
  • TF Tree - Transform hiyerarşisi
""")
    else:
        print(f"""
✗ Config dosyası bulunamadı: {rviz_config}
Lütfen SLAM_EKF başlat ve tekrar dene.""")
    
    print_section("SEÇENEK 2: Manuel olarak RViz konfigüre et")
    
    print("""
1. RViz'i başlat:
   rviz2

2. "Fixed Frame" seç (sol üstte):
   ✓ "map" seçin

3. Topics ekle (sol paneldeki + tuşuna tıkla):
   
   a) Map ekle:
      • "+ By topic" → /map → Map
      • Color Scheme: costmap
      • Alpha: 0.7
   
   b) LaserScan ekle:
      • "+ By topic" → /prius/scan → LaserScan
      • Size: 0.05m
      • Color: Red (255; 0; 0)
   
   c) Odometry Local ekle:
      • "+ By topic" → /odometry/filtered/local → Odometry
      • Head Color: Blue (0; 0; 255)
      • Shaft Color: Yellow (255; 255; 0)
   
   d) Odometry Global ekle:
      • "+ By topic" → /odometry/filtered/global → Odometry
      • Head Color: Green (0; 255; 0)
      • Shaft Color: Red (255; 0; 0)
   
   e) TF ekle:
      • "+ By display type" → TF
      • Marker Scale: 1
      • Show Names: true
      • Show Axes: true

4. Görüntü ayarla:
   • Mouse sağ tuşu + oynat = döndür
   • Mouse ortası + oynat = kaydır
   • Scroll = zoom

5. Konfigürasyonu kaydet:
   • File → Save Config → "slam_ekf.rviz"
""")
    
    print_section("SEÇENEK 3: Python script ile RViz konfigürasyonu oluştur")
    
    print("""
Otomatik olarak setup yapmak için:
  python3 /home/ranim/autoCar_ws/scripts/setup_rviz_config.py
  
(Bu script henüz yazılacak)
""")
    
    print_section("RViz'de Görülmesi Gereken İçerik")
    
    print("""
SLAM + EKF sistemi çalışırken RViz'de şunu görmelisiniz:

✓ Harita Grid'i (gri renkli)
  └─ SLAM Toolbox tarafından real-time olarak oluşturuluyor
  └─ Robot dönerken genişliyor

✓ LiDAR Noktaları (kırmızı noktalar)
  └─ /prius/scan'dan gelen verileri gösterir
  └─ Haritanın üzerine overlay olarak görünür

✓ Local EKF Arrow (sarı ok)
  └─ Odometry + IMU füzyonunun sonucu
  └─ Odom frame'inde konumu gösterir

✓ Global EKF Arrow (kırmızı ok)
  └─ GPS + Local EKF + IMU füzyonu
  └─ Map frame'inde gerçek konum tahmini

✓ TF Tree (çizgiler)
  └─ map → odom → chassis → sensörler
  └─ Transform hiyerarşisini gösterir

EĞER BİR ŞEYLER GÖRMÜYORSAN:

1. Tüm nodes çalışıyor mu?
   ros2 node list | grep -E "(slam|ekf|odom_to_tf)"

2. Topics yayınlanıyor mu?
   ros2 topic list | grep -E "(map|scan|odometry|filtered)"

3. TF yayınlanıyor mu?
   ros2 run tf2_ros tf2_echo map odom

4. Gazebo ve tüm launcher çalışıyor mu?
   ps aux | grep ros2

5. RViz'in fixed frame'i "map" olarak ayarlı mı?
""")
    
    print_section("Sık Sorulan Sorular")
    
    print("""
S: Harita görünmüyor ama topic'ler yayınlanıyor?
C: • Fixed Frame'i "map" olarak ayarla
  • Map'in alpha değerini 0.7 olarak ayarla
  • 10-20 saniye bekle (SLAM harita oluştururken)
  • RViz view'ı reset et: View → Default

S: LiDAR noktaları görünmüyor?
C: • /prius/scan topic'ini check et
  • LaserScan display'inin "Fixed Frame" = "map" olup olmadığını check et
  • Size'ı 0.05m olarak ayarla

S: Oklar (Odometry) görünmüyor?
C: • /odometry/filtered/local ve /odometry/filtered/global topics'ini check et
  • EKF nodes'ları başlatıldı mı? (ekf_filter_node_odom, ekf_filter_node_map)
  • Odometry displays'inin Position Tolerance'ı 0.1 olarak ayarla

S: TF ağacı (çizgiler) görünmüyor?
C: • TF display'ını enable et
  • Show Axes ve Show Names'i check et
  • Marker Scale'i 1'e ayarla

S: Her şey görünüyor ama çok küçük / çok büyük?
C: • View → Reset View
  • Mouse scroll ile zoom yap
  • Camera → Orbit mode seçin

S: Konfigürasyon dosyasını kaydedip sonra yükleyebilir miyim?
C: Evet!
  • File → Save Config As → "slam_ekf.rviz"
  • Sonra: rviz2 -d slam_ekf.rviz
""")
    
    print_section("Ayrintılı Topic Bilgileri")
    
    print("""
/map (OccupancyGrid)
  • SLAM Toolbox tarafından yayınlanır
  • Haritanın grid temsili
  • 0-100 değerler: 0=boş, 100=işgal, -1=bilinmeyen
  • Frequency: ~0.33 Hz (her 3 saniyede)

/prius/scan (LaserScan)
  • Gazebo simülasyonunda LiDAR sensöründen
  • Açı, mesafe, yoğunluk bilgisi
  • Frequency: ~10 Hz

/odometry/filtered/local (Odometry)
  • EKF local node tarafından yayınlanır
  • Odometry + IMU füzyonu
  • Odom frame'inde
  • Frequency: 50 Hz

/odometry/filtered/global (Odometry)
  • EKF global node tarafından yayınlanır
  • GPS + Local EKF + IMU füzyonu
  • Map frame'inde
  • Frequency: 50 Hz

/tf (TransformStamped)
  • Tüm transform'lar
  • map → odom (SLAM)
  • odom → chassis (odom_to_tf.py)
  • chassis → sensörler (static)
  • Frequency: 50 Hz
""")
    
    print_section("İleri Ayarlar")
    
    print("""
Daha iyi görsel için:

1. Map'in transparency'sini azalt:
   Display → Map → Transparency: 0

2. LiDAR noktalarını daha belirgin yap:
   Display → LaserScan → Size (Pixels): 5

3. Rotation'ı daha net görmek için:
   • Perspective view yerine Top-down view kullan
   • View → Presets → Top-Down

4. Real-time performans iyileştirmek için:
   • Global Options → Frame Rate: 30 (default)
   • Daha fazla FPS için: 60

5. Renkli harita görmek için:
   Display → Map → Color Scheme: map değişi
""")
    
    print_section("Kullanışlı Komutlar")
    
    print("""
# RViz'i önceden yapılandırılmış config ile aç
rviz2 -d /home/ranim/autoCar_ws/src/slam_mapping/rviz/slam_ekf.rviz

# Hangi nodes çalışıyor check et
ros2 node list

# Hangi topics yayınlanıyor check et
ros2 topic list

# Spesifik topic hakkında bilgi al
ros2 topic info /map

# Topic'in aktif olup olmadığını kontrol et
ros2 topic hz /map

# TF tree'yi göster
ros2 run tf2_tools view_frames

# Spesifik transform kontrol et
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom chassis
""")
    
    print_section("İşte Hazır!")
    
    print(f"""
RViz'i şu komutla başlat:

  rviz2 -d {rviz_config}

veya manuel kurulum için yukarıdaki adımları takip et.

Sistem çalışırken RViz'de hareket eden bir harita ve EKF 
tahminlerini göreceksin!
""")

if __name__ == '__main__':
    main()
