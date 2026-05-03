#!/bin/bash
#===============================================================================
# Lokalizasyon + RViz Başlatıcı
#===============================================================================
# PCD harita tabanlı lokalizasyonu ve RViz görselleştirmesini birlikte başlatır.
#
# Kullanım:
#   ./scripts/run_localization.sh              # test2 haritası (varsayılan)
#   ./scripts/run_localization.sh my_city_map  # özel harita adı
#
# Ön koşul: Gazebo çalışıyor olmalı → ./scripts/run_compact_city.sh
#===============================================================================

set -e

# Renkler
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${BLUE}"
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║         Lokalizasyon + RViz Başlatıcı                         ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

cd ~/autoCar_ws

# ── 1. Workspace source ──────────────────────────────────────────────
echo -e "${YELLOW}[1/4] Sourcing workspace...${NC}"
source install/setup.bash
echo -e "${GREEN}  ✓ Workspace sourced${NC}"

# ── 2. Gazebo kontrolü ───────────────────────────────────────────────
echo -e "${YELLOW}[2/4] Checking Gazebo...${NC}"
if ! pgrep -x "gzserver" > /dev/null; then
    echo -e "${RED}  ✗ Gazebo çalışmıyor!${NC}"
    echo -e "${YELLOW}  Önce simülasyonu başlat: ./scripts/run_compact_city.sh${NC}"
    exit 1
fi
echo -e "${GREEN}  ✓ Gazebo is running${NC}"

# ── 3. Harita kontrolü ───────────────────────────────────────────────
MAP_NAME="${1:-scaled_build}"
MAP_PATH="$HOME/autoCar_ws/maps/${MAP_NAME}.pcd"

echo -e "${YELLOW}[3/4] Checking map: ${MAP_NAME}${NC}"
if [ ! -f "$MAP_PATH" ]; then
    echo -e "${RED}  ✗ Harita bulunamadı: $MAP_PATH${NC}"
    echo -e "${YELLOW}  Mevcut haritalar:${NC}"
    ls -1 maps/*.pcd 2>/dev/null | sed 's/maps\//    - /' | sed 's/\.pcd//' || echo "    Harita yok!"
    exit 1
fi
echo -e "${GREEN}  ✓ Map found: $MAP_PATH${NC}"

# ── 4. Eski işlemleri temizle ─────────────────────────────────────────
echo -e "${YELLOW}[4/4] Starting localization + RViz...${NC}"
pkill -f simple_localizer 2>/dev/null || true
pkill -f rviz2 2>/dev/null || true
sleep 1

# ── 5. Lokalizasyonu arka planda başlat ──────────────────────────────
echo -e "${CYAN}  → Launching localization (map: ${MAP_NAME})...${NC}"
ros2 launch localization simple_localization.launch.py \
    map_pcd_path:="$MAP_PATH" &
LOC_PID=$!
sleep 2

# ── 6. RViz'i arka planda başlat ─────────────────────────────────────
echo -e "${CYAN}  → Launching RViz...${NC}"
rviz2 -d ~/autoCar_ws/config/localization.rviz &
RVIZ_PID=$!

echo ""
echo -e "${GREEN}════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  ✓ Lokalizasyon ve RViz başlatıldı!${NC}"
echo -e "${GREEN}════════════════════════════════════════════════════════════════${NC}"
echo -e "  Harita       : ${CYAN}${MAP_NAME}${NC}"
echo -e "  Localizer PID: ${CYAN}${LOC_PID}${NC}"
echo -e "  RViz PID     : ${CYAN}${RVIZ_PID}${NC}"
echo ""
echo -e "${YELLOW}  Durdurmak için: Ctrl+C${NC}"
echo ""

# ── Ctrl+C ile temiz kapatma ──────────────────────────────────────────
cleanup() {
    echo ""
    echo -e "${YELLOW}Kapatılıyor...${NC}"
    kill $LOC_PID 2>/dev/null || true
    kill $RVIZ_PID 2>/dev/null || true
    wait $LOC_PID 2>/dev/null || true
    wait $RVIZ_PID 2>/dev/null || true
    echo -e "${GREEN}✓ Temiz kapatıldı.${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM

# Her iki process bitene kadar bekle
wait $LOC_PID $RVIZ_PID 2>/dev/null
