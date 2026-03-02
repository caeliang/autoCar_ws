#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────
#  Waypoint takip scripti
#  Kayıtlı waypoint'leri sırayla takip et.
#
#  Kullanım:
#    ./scripts/follow_waypoints.sh <waypoint_file> [options]
#
#  Örnekler:
#    ./scripts/follow_waypoints.sh waypoints/route_20260225.yaml
#    ./scripts/follow_waypoints.sh waypoints/route.yaml --loop
#    ./scripts/follow_waypoints.sh waypoints/route.yaml --speed 2.0
# ──────────────────────────────────────────────────────────────────────
set -e

WS=~/autoCar_ws
LOOP="false"
MAX_SPEED="3.0"
PURSUIT="true"

# İlk argüman waypoint dosyası
WP_FILE="${1:-}"
shift 2>/dev/null || true

# Opsiyonel argümanları parse et
while [[ $# -gt 0 ]]; do
    case "$1" in
        --loop)    LOOP="true";    shift ;;
        --speed)   MAX_SPEED="$2"; shift 2 ;;
        --no-pursuit) PURSUIT="false"; shift ;;
        *) echo "Bilinmeyen argüman: $1"; exit 1 ;;
    esac
done

if [ -z "$WP_FILE" ]; then
    echo "Kullanım: $0 <waypoint_file> [--loop] [--speed X] [--no-pursuit]"
    echo ""
    echo "Mevcut waypoint dosyaları:"
    ls -1 "$WS"/waypoints/*.yaml 2>/dev/null || echo "  (yok — önce kayıt yap)"
    exit 1
fi

# Mutlak yol
if [[ "$WP_FILE" != /* ]]; then
    WP_FILE="$WS/$WP_FILE"
fi

if [ ! -f "$WP_FILE" ]; then
    echo "✗ Dosya bulunamadı: $WP_FILE"
    exit 1
fi

WP_COUNT=$(grep -c "^  - x:" "$WP_FILE" 2>/dev/null || echo "?")

cd "$WS"
source install/setup.bash

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  WAYPOINT FOLLOWER"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Dosya:    $WP_FILE"
echo "  Waypoint: $WP_COUNT adet"
echo "  Loop:     $LOOP"
echo "  Hız:      $MAX_SPEED m/s"
echo "  Pursuit:  $PURSUIT"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

ros2 launch path_planning waypoint_follow.launch.py \
    waypoint_file:="$WP_FILE" \
    loop:="$LOOP" \
    max_speed:="$MAX_SPEED" \
    enable_pursuit:="$PURSUIT"
