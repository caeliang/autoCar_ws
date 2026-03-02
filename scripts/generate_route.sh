#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────
#  Otomatik Rota Oluşturucu
#  Harita asphalt bloklarından 2-şeritli waypoint üretir.
#  ARTIK ARACI SÜRMENİZE GEREK YOK!
#
#  Kullanım:
#    ./scripts/generate_route.sh                    # interaktif mod
#    ./scripts/generate_route.sh --route west_south  # hazır rota
#    ./scripts/generate_route.sh --list-routes       # rotaları listele
#    ./scripts/generate_route.sh --generate-all      # tüm rotaları üret
#    ./scripts/generate_route.sh --custom -28 32 27 -22  # özel rota
#
#  Seçenekler:
#    --spacing 1.0    Waypoint arası mesafe (m)
#    --speed 1.5      Hedef hız (m/s)
#    --output file    Çıktı dosyası
# ──────────────────────────────────────────────────────────────────────
set -e

WS=~/autoCar_ws
SCRIPT="$WS/src/path_planning/scripts/route_generator.py"

if [ ! -f "$SCRIPT" ]; then
    echo "✗ Script bulunamadı: $SCRIPT"
    exit 1
fi

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  🗺️  OTOMATİK ROTA OLUŞTURUCU"
echo "  Asphalt bloklarından 2-şeritli waypoint"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

python3 "$SCRIPT" "$@"
