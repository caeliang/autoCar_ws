# 🚗 Yol Planlama (Path Planning) Kullanım Kılavuzu

Bu kılavuz, yeni düzenlenmiş rota planlayıcısı (`generate_route.py`) ve yenilenmiş görselleştiriciyi (`plot_waypoints.py`) kullanarak nasıl rota üreteceğinizi ve sonuçları nasıl inceleyeceğinizi açıklar.

## 1. Rota Oluşturma (A* + Smoother)

Araçların şerit (yaw) kurallarına uyarak ve yolu yumuşatarak ilerlemesini sağlayan planlayıcıyı çalıştırmak için aşağıdaki komutu kullanın:

```bash
cd ~/autoCar_ws
python3 src/path_planning/scripts/generate_route.py <start_x> <start_y> <goal_x> <goal_y>
```

**Örnek Kullanım:**
```bash
# (3, 3) noktasından (45, 55) noktasına bir rota çıkartır
python3 src/path_planning/scripts/generate_route.py 3 3 45 55
```

**Çıktılar:**
- İlgili komut arka planda rotayı (A* ile) çözer, sonrasında Gradient Descent (Yumuşatma) sisteminden geçirerek `waypoints/` dizinine kaydeder.
- Başarılı olursa `waypoints/planned_route.csv` (x, y, z, yaw, step vb.) dosyasını üretir.

---

## 2. Çıkan Rotayı Görselleştirme (Plotting)

Oluşturulan rotayı veya waypoint noktalarınızı incelemek için tek ve çok amaçlı görselleştirme dosyası olan `plot_waypoints.py`'i kullanacaksınız. 

Çizimde arkaplan gridini, asıl harita (map) pakedini ve hesaplanan rotayı hepsini bir arada görmek için:

```bash
cd ~/autoCar_ws
python3 scripts/plot_waypoints.py waypoints/planned_route.csv --grid matrices/road_grid_4wide.txt --map waypoints/full_road_map.csv
```

**Opsiyonlar:**
- `csv_file`: Üretilen rota dosyası veya incelemek istediğiniz kendi oluşturduğunuz herhangi bir .csv noktası (Zorunlu).
- `--grid`: Matrix grid konumu (Örn: `matrices/road_grid_4wide.txt`). Haritanın fiziksel grid hatlarını gösterir.
- `--map`: Bütün map waypoints konumu (Örn: `waypoints/full_road_map.csv`). Bu, tüm yollardaki noktaları ince açık mavi ile arkadan gösterir.
- `--no-arrows`: Yön oklarını kapatmak isterseniz bunu komuta ekleyebilirsiniz.

**Çıktı:**
Betik başarıyla çalıştığında ekranda bir şey açılmakla vakit kaybetmez; doğrudan `.csv` ile aynı dizine (`waypoints/planned_route.png` gibi) **yüksek çözünürlüklü PNG** haritasını okları, haritayı ve rotayı üst üste çizerek kaydeder.
