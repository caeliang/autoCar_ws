#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────
#  Otomatik Rota Oluşturucu + Yaw Hesaplaması
#  Harita asphalt bloklarından 2-şeritli waypoint üretir.
#  YAW DEĞERLERİ OTOMATİK OLARAK HESAPLANIR!
#
#  Kullanım:
#    ./scripts/generate_route.sh
#
#  İşlem:
#    1. RoadNetwork'ten waypoint oluştur
#    2. atan2(Δy, Δx) formülü ile yaw hesapla
#    3. Görselle göster
# ──────────────────────────────────────────────────────────────────────
set -e

WS=~/autoCar_ws
GENERATE_SCRIPT="$WS/src/path_planning/scripts/generate_full_map.py"
VISUALIZE_SCRIPT="$WS/scripts/visualize_waypoints_yaw.py"
WAYPOINTS_FILE="$WS/waypoints/full_road_map.csv"
GRID_FILE="$WS/matrices/road_grid_4wide.txt"

if [ ! -f "$GENERATE_SCRIPT" ]; then
    echo "✗ Script bulunamadı: $GENERATE_SCRIPT"
    exit 1
fi

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  🗺️  OTOMATİK ROTA OLUŞTURUCU + YAW HESAPLAMASı"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo

# Step 1: Waypoints + Yaw Hesaplama (tek script)
echo "[1/2] Waypoints oluşturuluyor ve yaw hesaplanıyor..."
python3 "$GENERATE_SCRIPT"

# Step 2: Görselleştirme (tüm waypoints)
if [ -f "$VISUALIZE_SCRIPT" ] && [ -f "$WAYPOINTS_FILE" ]; then
    echo "[2/2] Görselleştirme oluşturuluyor (tüm waypoints)..."
    if [ -f "$GRID_FILE" ]; then
        OUTPUT_VIZ="$WS/waypoints/waypoints_all_points.png"
        timeout 30 python3 "$VISUALIZE_SCRIPT" "$WAYPOINTS_FILE" "$GRID_FILE" "$OUTPUT_VIZ" 1 2>/dev/null || true
        echo "✓ Görselleştirme: $OUTPUT_VIZ"
    else
        echo "⚠ Grid dosyası bulunamadı, görselleştirme atlanıyor"
    fi
fi

echo
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✓ TAMAMLANDI!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo
echo "Waypoints dosyası: $WAYPOINTS_FILE"
[ -f "$WS/waypoints/waypoints_all_points.png" ] && echo "Görselleştirme: $WS/waypoints/waypoints_all_points.png"
