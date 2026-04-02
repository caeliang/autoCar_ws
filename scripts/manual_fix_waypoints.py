import csv
import math

def generate_manual_waypoints():
    # Şerit pozisyonları (generate_loop_route.py'den alınan değerler)
    SOUTH_LANE_X = -28.75
    SOUTH_LANE_INNER = -13.75
    NORTH_LANE_X_E = 28.75
    
    EAST_LANE_Y_S = -23.75
    EAST_LANE_MID = 16.25
    WEST_LANE_Y_N = 33.75

    x_lanes = [SOUTH_LANE_X, SOUTH_LANE_INNER, NORTH_LANE_X_E]
    y_lanes = [EAST_LANE_Y_S, EAST_LANE_MID, WEST_LANE_Y_N]

    waypoints = []
    
    # Grid aralığı (1m)
    spacing = 1.0

    # 1. Tüm YATAY yolları oluştur (Sürekli hatlar)
    for y in y_lanes:
        # X range: -35 to 35
        n_points = int((35.0 - (-35.0)) / spacing)
        for i in range(n_points + 1):
            x = -35.0 + i * spacing
            waypoints.append((x, y, 0.0))

    # 2. Tüm DİKEY yolları oluştur (Sürekli hatlar)
    for x in x_lanes:
        # Y range: -30 to 40
        n_points = int((40.0 - (-30.0)) / spacing)
        for i in range(n_points + 1):
            y = -30.0 + i * spacing
            waypoints.append((x, y, math.pi/2))

    # CSV olarak kaydet
    file_path = '/home/ranim/autoCar_ws/waypoints/full_road_map.csv'
    with open(file_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['index', 'x', 'y', 'yaw'])
        for i, (x, y, yaw) in enumerate(waypoints):
            writer.writerow([i, f'{x:.4f}', f'{y:.4f}', f'{yaw:.6f}'])
    
    print(f"✓ {len(waypoints)} waypoint manually generated and saved to {file_path}")

if __name__ == "__main__":
    generate_manual_waypoints()
