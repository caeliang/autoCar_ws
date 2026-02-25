#!/bin/bash
#===============================================================================
# 3D SLAM + RViz  —  Tek Script
#===============================================================================
# Gazebo çalışırken 3D SLAM (Octomap + EKF) ve RViz'i birlikte başlatır.
#
# Kullanım:
#   ./scripts/run_slam_rviz.sh          # Varsayılan slam_3d.rviz config ile
#   ./scripts/run_slam_rviz.sh --no-rviz   # Sadece SLAM, RViz yok
#
# Ön koşul: Gazebo çalışıyor olmalı → ./scripts/run_compact_city.sh
#===============================================================================

set -e

# ── Renkler ───────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m'

# ── Argüman parse ─────────────────────────────────────────────────────
LAUNCH_RVIZ=true
for arg in "$@"; do
    case "$arg" in
        --no-rviz) LAUNCH_RVIZ=false ;;
    esac
done

echo -e "${CYAN}"
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║           3D SLAM + RViz  (Octomap + EKF)                     ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

cd ~/autoCar_ws

# ── 1. Source workspace ───────────────────────────────────────────────
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

# ── 3. RViz config kontrolü ──────────────────────────────────────────
RVIZ_CONFIG="$HOME/autoCar_ws/config/slam_3d.rviz"

if [ "$LAUNCH_RVIZ" = true ]; then
    echo -e "${YELLOW}[3/4] Checking RViz config...${NC}"
    if [ ! -f "$RVIZ_CONFIG" ]; then
        echo -e "${RED}  ✗ RViz config bulunamadı: $RVIZ_CONFIG${NC}"
        echo -e "${YELLOW}  RViz varsayılan ayarlarla açılacak.${NC}"
        RVIZ_CONFIG=""
    else
        echo -e "${GREEN}  ✓ RViz config: $RVIZ_CONFIG${NC}"
    fi
else
    echo -e "${YELLOW}[3/4] RViz devre dışı (--no-rviz)${NC}"
fi

# ── Bilgi ─────────────────────────────────────────────────────────────
echo ""
echo -e "${MAGENTA}  Topics:${NC}"
echo -e "    • ${GREEN}/occupied_cells_vis_array${NC}  — 3D Voxel Map"
echo -e "    • ${GREEN}/prius/scan${NC}               — Raw 3D Point Cloud"
echo -e "    • ${GREEN}/odometry/filtered${NC}         — EKF Odometry"
echo -e "    • ${GREEN}/path${NC}                      — Trajectory"
echo ""

# ── Eski process temizliği ────────────────────────────────────────────
pkill -f "slam_3d.launch" 2>/dev/null || true
pkill -f rviz2 2>/dev/null || true
sleep 1

# ── 4. SLAM + RViz başlat ────────────────────────────────────────────
echo -e "${YELLOW}[4/4] Starting SLAM...${NC}"

ros2 launch slam_3d slam_3d.launch.py &
SLAM_PID=$!
sleep 2

RVIZ_PID=""
if [ "$LAUNCH_RVIZ" = true ]; then
    echo -e "${CYAN}  → Launching RViz...${NC}"
    if [ -n "$RVIZ_CONFIG" ]; then
        rviz2 -d "$RVIZ_CONFIG" &
    else
        rviz2 &
    fi
    RVIZ_PID=$!
fi

echo ""
echo -e "${GREEN}════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  ✓ 3D SLAM başlatıldı!${NC}"
echo -e "${GREEN}════════════════════════════════════════════════════════════════${NC}"
echo -e "  SLAM PID : ${CYAN}${SLAM_PID}${NC}"
[ -n "$RVIZ_PID" ] && echo -e "  RViz PID : ${CYAN}${RVIZ_PID}${NC}"
echo ""
echo -e "${YELLOW}  Arabayı sürmek için başka terminalde:${NC}"
echo -e "  ${CYAN}ros2 run teleop_twist_keyboard teleop_twist_keyboard \\${NC}"
echo -e "  ${CYAN}    --ros-args -r cmd_vel:=/prius/cmd_vel${NC}"
echo ""
echo -e "${YELLOW}  Haritayı kaydetmek için:${NC}"
echo -e "  ${CYAN}./scripts/save_pcd_map.sh my_map${NC}"
echo ""
echo -e "${YELLOW}  Durdurmak için: Ctrl+C${NC}"
echo ""

# ── Ctrl+C ile temiz kapatma ──────────────────────────────────────────
cleanup() {
    echo ""
    echo -e "${YELLOW}Kapatılıyor...${NC}"
    kill $SLAM_PID 2>/dev/null || true
    [ -n "$RVIZ_PID" ] && kill $RVIZ_PID 2>/dev/null || true
    wait $SLAM_PID 2>/dev/null || true
    [ -n "$RVIZ_PID" ] && wait $RVIZ_PID 2>/dev/null || true
    echo -e "${GREEN}✓ Temiz kapatıldı.${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM

# Her iki process bitene kadar bekle
if [ -n "$RVIZ_PID" ]; then
    wait $SLAM_PID $RVIZ_PID 2>/dev/null
else
    wait $SLAM_PID 2>/dev/null
fi
