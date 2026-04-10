import csv
import math
import os
import numpy as np
import cv2

def generate_grid_from_csv(input_csv, output_txt, output_npy=None, resolution=1.0, width_mode='thin'):
    points = []
    with open(input_csv, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append((float(row['x']), float(row['y'])))
            
    if not points:
        return
        
    # generate_route.py icindeki grid haritasiyla tam uyumlu olmasi icin fixed boyutlar
    min_x, max_x = -28.8 * 1.5, 28.8 * 1.5
    min_y, max_y = -32.5 * 1.5, 33.8 * 1.5
    
    width = int(math.ceil((max_x - min_x) / resolution)) + 1
    height = int(math.ceil((max_y - min_y) / resolution)) + 1
    
    grid = np.zeros((height, width), dtype=np.uint8)
    
    # Ardisik waypointlerin mesafesi threshold'dan kucukse birlestir
    threshold = 8.0 
    
    for i in range(len(points)):
        px, py = points[i]
        gx = int(round((px - min_x) / resolution))
        gy = int(round((py - min_y) / resolution))
        gx = max(0, min(gx, width - 1))
        gy = max(0, min(gy, height - 1))
        # Points only, no line yet
        if i == 0:
            grid[gy, gx] = 1
        
        if i > 0:
            ppx, ppy = points[i-1]
            dist = math.hypot(px - ppx, py - ppy)
            if dist <= threshold:
                pgx = int(round((ppx - min_x) / resolution))
                pgy = int(round((ppy - min_y) / resolution))
                pgx = max(0, min(pgx, width - 1))
                pgy = max(0, min(pgy, height - 1))
                # Opencv line cizici Bresenham kullaniyor
                cv2.line(grid, (pgx, pgy), (gx, gy), 1, 1)

    if width_mode == '4-wide':
        # OpenCV Dilation (Genisletme) ile kusursuz kalinlik elde et
        # Seritler arasi boslugu kapatmak ve tam 4 piksel yol saglamak icin karesel 4x4 kernel
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))
        grid = cv2.dilate(grid, kernel, iterations=1)

    os.makedirs(os.path.dirname(output_txt), exist_ok=True)
    with open(output_txt, 'w') as f:
        # Orijinal matris duzeninde satirlari ters cevir
        for row in reversed(grid.tolist()):
            line = "".join(str(cell) for cell in row)
            f.write(line + "\n")
            
    if output_npy:
        os.makedirs(os.path.dirname(output_npy), exist_ok=True)
        np.save(output_npy, np.flipud(grid))
            
    print(f"Izgara Matrisi ({width_mode}) (Boyutlar: {width}x{height}, Cozunurluk: {resolution}m) olusturuldu: {output_txt}")

if __name__ == '__main__':
    # Rota algoritmasi 68x59 matris bekliyor, bu sebeple resolution=1.5 olmak zorunda
    generate_grid_from_csv("waypoints/full_road_map.csv", "matrices/road_grid.txt", "matrices/road_grid.npy", resolution=1.5, width_mode='thin')
    generate_grid_from_csv("waypoints/full_road_map.csv", "matrices/road_grid_4wide.txt", "matrices/road_grid_4wide.npy", resolution=1.5, width_mode='4-wide')
