#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────
#  Waypoint kayıt scripti
#  Localization çalışırken waypoint kaydet.
#
#  Kullanım:
#    ./scripts/record_waypoints.sh [output_file]
#
#  Örnekler:
#    ./scripts/record_waypoints.sh                          # otomatik isim
#    ./scripts/record_waypoints.sh waypoints/my_route.yaml  # belirli dosya
# ──────────────────────────────────────────────────────────────────────
set -e

WS=~/autoCar_ws
OUTPUT_FILE="${1:-}"

cd "$WS"
source install/setup.bash

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  WAYPOINT RECORDER"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Localization çalışıyor mu kontrol et
if ! ros2 topic list 2>/dev/null | grep -q "/localization/pose"; then
    echo "⚠  /localization/pose bulunamadı"
    echo "   Önce localization başlat: ./scripts/run_localization.sh"
    echo ""
fi

# Çalıştır
if [ -n "$OUTPUT_FILE" ]; then
    ros2 run path_planning waypoint_recorder.py \
        --ros-args -p output_file:="$OUTPUT_FILE"
else
    ros2 run path_planning waypoint_recorder.py
fi
