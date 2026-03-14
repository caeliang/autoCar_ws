#!/usr/bin/env python3
"""
Dış CCW (counter-clockwise) döngü rotası üretici.
Kavşaklarda düzgün ark waypoint'leri ekler.

Sol dönüşler: geniş kavisli (R=3.0m)
Sağ dönüşler: dar açılı  (R=1.5m)

Kullanım:
  python3 generate_loop_route.py
  → waypoints/waypoints.csv üretir
"""

import math
import csv
import os

# ═══════════════════════════════════════════════════════════════
#  PARAMETRELER
# ═══════════════════════════════════════════════════════════════

LEFT_TURN_R  = 3.0    # metre — geniş sol dönüş
RIGHT_TURN_R = 1.5    # metre — dar sağ dönüş
WP_SPACING   = 1.0    # metre — düz yolda waypoint aralığı
ARC_SPACING  = 0.3    # metre — ark üzerinde waypoint aralığı (daha yoğun)

# Şerit pozisyonları (sağ el trafiği)
SOUTH_LANE_X    = -28.75   # batı yolda güneye giden şerit
NORTH_LANE_X_E  =  28.75   # doğu yolda kuzeye giden şerit
EAST_LANE_Y_S   = -23.75   # güney yolda doğuya giden şerit
WEST_LANE_Y_N   =  33.75   # kuzey yolda batıya giden şerit

# İç yol şeritleri
SOUTH_LANE_INNER = -13.75  # iç batı güneye giden
EAST_LANE_MID    =  16.25  # orta doğuya giden

# ═══════════════════════════════════════════════════════════════
#  YARDIMCI FONKSİYONLAR
# ═══════════════════════════════════════════════════════════════

def normalize_angle(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def gen_straight(x0, y0, x1, y1, heading, spacing=WP_SPACING):
    """Düz yol boyunca waypoint üret."""
    dx = x1 - x0
    dy = y1 - y0
    length = math.sqrt(dx*dx + dy*dy)
    if length < 0.01:
        return [(x0, y0, heading)]
    n = max(1, int(round(length / spacing)))
    pts = []
    for i in range(n + 1):
        t = i / n
        pts.append((x0 + t*dx, y0 + t*dy, heading))
    return pts


def gen_left_arc(corner_x, corner_y, R, entry_heading, arc_spacing=ARC_SPACING):
    """
    Sol dönüş (CCW) ark waypoint'leri üret.

    corner: iki şerit çizgisinin kesişim noktası
    R: dönüş yarıçapı
    entry_heading: giriş yönü (radyan)

    Giriş noktası: corner + R geri (giriş yönünün tersinde)
    Çıkış noktası: corner + R ileri (çıkış yönünde)
    """
    h = entry_heading

    # Ark merkezi: köşeden R uzakta, dönüşün iç tarafında
    # Sol dönüş → merkez sola (gidiş yönüne göre)
    # Merkez formülü: her yön için ayrı hesaplanmış

    if abs(h - (-math.pi/2)) < 0.1:
        # Güney → Doğu
        cx = corner_x + R
        cy = corner_y + R
        t_start = math.pi
        t_end   = 3*math.pi/2
    elif abs(h - 0) < 0.1:
        # Doğu → Kuzey
        cx = corner_x - R
        cy = corner_y + R
        t_start = -math.pi/2
        t_end   = 0
    elif abs(h - math.pi/2) < 0.1:
        # Kuzey → Batı
        cx = corner_x - R
        cy = corner_y - R
        t_start = 0
        t_end   = math.pi/2
    elif abs(normalize_angle(h - math.pi)) < 0.1:
        # Batı → Güney
        cx = corner_x + R
        cy = corner_y - R
        t_start = math.pi/2
        t_end   = math.pi
    else:
        raise ValueError(f"Desteklenmeyen giriş açısı: {h}")

    arc_len = R * math.pi / 2
    n_arc = max(5, int(round(arc_len / arc_spacing)))

    pts = []
    for i in range(n_arc + 1):
        t = t_start + i * (t_end - t_start) / n_arc
        x = cx + R * math.cos(t)
        y = cy + R * math.sin(t)
        yaw = normalize_angle(t + math.pi/2)  # CCW ark için teğet yön
        pts.append((x, y, yaw))

    return pts


def gen_right_arc(corner_x, corner_y, R, entry_heading, arc_spacing=ARC_SPACING):
    """
    Sağ dönüş (CW) ark waypoint'leri üret.

    Sağ dönüş: heading h → h − π/2
    Merkez: sağ tarafta
    """
    h = entry_heading

    if abs(h - (-math.pi/2)) < 0.1:
        # Güney → Batı
        cx = corner_x - R
        cy = corner_y + R
        t_start = 0
        t_end   = -math.pi/2
    elif abs(h - 0) < 0.1:
        # Doğu → Güney
        cx = corner_x - R
        cy = corner_y - R
        t_start = math.pi/2
        t_end   = 0
    elif abs(h - math.pi/2) < 0.1:
        # Kuzey → Doğu
        cx = corner_x + R
        cy = corner_y - R
        t_start = math.pi
        t_end   = math.pi/2
    elif abs(normalize_angle(h - math.pi)) < 0.1:
        # Batı → Kuzey
        cx = corner_x + R
        cy = corner_y + R
        t_start = -math.pi/2
        t_end   = 0  # = -π/2 + π/2
        # Actually: west→north is CW, t goes -π/2 → 0? No.
        # Let me recalculate...
        # Heading west (π), turning right → heading north (π/2)
        # Right of west = north direction
        # Center at (+R, +R) from corner
        cx = corner_x + R
        cy = corner_y + R
        t_start = math.pi  # ← corresponds to heading west
        t_end   = 3*math.pi/2  # wait, CW should decrease angle
        # For CW: t decreases
        # At t=π: heading = π - π/2 = π/2 (north)? No, CW heading = -(t - π/2)?
        # OK this is getting confusing. Let me use a different approach for CW.
        # For CW motion: P(t) = C + R*(cos(t), sin(t)), t DECREASING
        # Velocity = R*(-sin(t), cos(t)) * dt/dt = negative for CW
        # So heading = atan2(-cos(t), sin(t)) = t - π/2
        raise ValueError("Batı→Kuzey sağ dönüş henüz desteklenmiyor")
    else:
        raise ValueError(f"Desteklenmeyen giriş açısı: {h}")

    arc_len = R * math.pi / 2
    n_arc = max(4, int(round(arc_len / arc_spacing)))

    pts = []
    for i in range(n_arc + 1):
        t = t_start + i * (t_end - t_start) / n_arc  # t decreases for CW
        x = cx + R * math.cos(t)
        y = cy + R * math.sin(t)
        # CW ark için heading: t − π/2
        yaw = normalize_angle(t - math.pi/2)
        pts.append((x, y, yaw))

    return pts


# ═══════════════════════════════════════════════════════════════
#  DIŞ DÖNGÜ (CCW — tüm dönüşler SOL)
# ═══════════════════════════════════════════════════════════════

def generate_outer_loop():
    """
    Dış CCW döngü:
      Güney → SOL@GB → Doğu → SOL@GD → Kuzey → SOL@KD → Batı → SOL@KB → tekrar

    Tüm dönüşler SOL (geniş kavisli).
    """
    R = LEFT_TURN_R
    wps = []

    # Köşe noktaları (iki şerit çizgisinin kesişimi)
    sw = (SOUTH_LANE_X, EAST_LANE_Y_S)      # (-28.75, -23.75)
    se = (NORTH_LANE_X_E, EAST_LANE_Y_S)    # ( 28.75, -23.75)
    ne = (NORTH_LANE_X_E, WEST_LANE_Y_N)    # ( 28.75,  33.75)
    nw = (SOUTH_LANE_X, WEST_LANE_Y_N)      # (-28.75,  33.75)

    # Segment sınırları (ark giriş/çıkış noktaları)
    # NW ark çıkışı = güney düz başlangıcı
    south_start_y = nw[1] - R   # 33.75 − 3.0 = 30.75
    south_end_y   = sw[1] + R   # −23.75 + 3.0 = −20.75

    east_start_x  = sw[0] + R   # −28.75 + 3.0 = −25.75
    east_end_x    = se[0] - R   # 28.75 − 3.0  = 25.75

    north_start_y = se[1] + R   # −23.75 + 3.0 = −20.75
    north_end_y   = ne[1] - R   # 33.75 − 3.0  = 30.75

    west_start_x  = ne[0] - R   # 28.75 − 3.0  = 25.75
    west_end_x    = nw[0] + R   # −28.75 + 3.0 = −25.75

    # ── 1. DÜZGÜN GÜNEY ──────────────────────────────────────────
    wps += gen_straight(SOUTH_LANE_X, south_start_y,
                        SOUTH_LANE_X, south_end_y,
                        -math.pi/2)
    wps.pop()  # son nokta = ark girişi (çakışma olmasın)

    # ── 2. SOL ARK GB (güney → doğu) ─────────────────────────────
    wps += gen_left_arc(sw[0], sw[1], R, -math.pi/2)
    wps.pop()

    # ── 3. DÜZGÜN DOĞU ──────────────────────────────────────────
    wps += gen_straight(east_start_x, EAST_LANE_Y_S,
                        east_end_x,   EAST_LANE_Y_S,
                        0)
    wps.pop()

    # ── 4. SOL ARK GD (doğu → kuzey) ────────────────────────────
    wps += gen_left_arc(se[0], se[1], R, 0)
    wps.pop()

    # ── 5. DÜZGÜN KUZEY ─────────────────────────────────────────
    wps += gen_straight(NORTH_LANE_X_E, north_start_y,
                        NORTH_LANE_X_E, north_end_y,
                        math.pi/2)
    wps.pop()

    # ── 6. SOL ARK KD (kuzey → batı) ────────────────────────────
    wps += gen_left_arc(ne[0], ne[1], R, math.pi/2)
    wps.pop()

    # ── 7. DÜZGÜN BATI ──────────────────────────────────────────
    wps += gen_straight(west_start_x,  WEST_LANE_Y_N,
                        west_end_x,    WEST_LANE_Y_N,
                        math.pi)
    wps.pop()

    # ── 8. SOL ARK KB (batı → güney) ────────────────────────────
    wps += gen_left_arc(nw[0], nw[1], R, math.pi)
    # Son noktayı koru — döngüde ilk noktayla aynı konuma döner

    return wps


# ═══════════════════════════════════════════════════════════════
#  CSV KAYDETME
# ═══════════════════════════════════════════════════════════════

def save_csv(waypoints, filepath):
    """Waypoint listesini CSV olarak kaydet."""
    os.makedirs(os.path.dirname(filepath), exist_ok=True)

    with open(filepath, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['index', 'x', 'y', 'yaw'])
        for i, (x, y, yaw) in enumerate(waypoints):
            writer.writerow([i, f'{x:.4f}', f'{y:.4f}', f'{yaw:.6f}'])

    print(f"✓ {len(waypoints)} waypoint → {filepath}")


# ═══════════════════════════════════════════════════════════════
#  DOĞRULAMA
# ═══════════════════════════════════════════════════════════════

def validate_route(wps):
    """Rota waypoint'lerini doğrula: mesafe ve yaw sürekliliği."""
    errors = 0

    for i in range(len(wps) - 1):
        x0, y0, yaw0 = wps[i]
        x1, y1, yaw1 = wps[i+1]

        dist = math.sqrt((x1-x0)**2 + (y1-y0)**2)
        yaw_diff = abs(normalize_angle(yaw1 - yaw0))

        if dist > 3.0:
            print(f"  ⚠ WP {i}→{i+1}: mesafe={dist:.2f}m (>3m)")
            errors += 1

        if yaw_diff > 0.5:
            print(f"  ⚠ WP {i}→{i+1}: yaw farkı={math.degrees(yaw_diff):.1f}° (>28°)")
            errors += 1

    # Döngü kontrolü (son → ilk)
    x0, y0, _ = wps[-1]
    x1, y1, _ = wps[0]
    loop_gap = math.sqrt((x1-x0)**2 + (y1-y0)**2)

    if errors == 0:
        print(f"  ✓ Tüm waypoint'ler sürekliliği doğrulandı")
    print(f"  Döngü boşluğu: {loop_gap:.2f}m")
    print(f"  Toplam waypoint: {len(wps)}")

    # Bölüm istatistikleri
    total_len = sum(
        math.sqrt((wps[i+1][0]-wps[i][0])**2 + (wps[i+1][1]-wps[i][1])**2)
        for i in range(len(wps)-1)
    )
    print(f"  Toplam rota uzunluğu: {total_len:.1f}m")


# ═══════════════════════════════════════════════════════════════
#  ANA
# ═══════════════════════════════════════════════════════════════

if __name__ == '__main__':
    print("=" * 50)
    print("Dış döngü rotası üretiliyor...")
    print(f"  Sol dönüş yarıçapı: {LEFT_TURN_R}m")
    print(f"  Waypoint aralığı: düz={WP_SPACING}m, ark={ARC_SPACING}m")
    print("=" * 50)

    wps = generate_outer_loop()

    # Doğrulama
    validate_route(wps)

    # Kaydet
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ws_dir = os.path.dirname(script_dir)
    csv_path = os.path.join(ws_dir, 'waypoints', 'waypoints.csv')

    save_csv(wps, csv_path)

    print(f"\nKullanım:")
    print(f"  ros2 launch path_planning waypoint_follow.launch.py \\")
    print(f"      waypoint_file:={csv_path}")
