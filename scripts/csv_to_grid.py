import csv
import math
import os
import numpy as np

def generate_grid_from_csv(input_csv, output_txt, output_npy=None, resolution=1.0, width_mode='thin'):
    points = []
    with open(input_csv, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append((float(row['x']), float(row['y'])))
            
    if not points:
        return
        
    min_x = min(p[0] for p in points)
    max_x = max(p[0] for p in points)
    min_y = min(p[1] for p in points)
    max_y = max(p[1] for p in points)
    
    width = int(math.ceil((max_x - min_x) / resolution)) + 1
    height = int(math.ceil((max_y - min_y) / resolution)) + 1
    
    grid = [[0 for _ in range(width)] for _ in range(height)]
    
    def draw_line(x0, y0, x1, y1):
        x0, y0 = int(x0), int(y0)
        x1, y1 = int(x1), int(y1)
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            if 0 <= x0 < width and 0 <= y0 < height:
                grid[y0][x0] = 1
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    # Ardışık waypointlerin mesafesi thresholddan küçükse birleştir
    threshold = 8.0 
    
    for i in range(len(points)):
        px, py = points[i]
        gx = int(round((px - min_x) / resolution))
        gy = int(round((py - min_y) / resolution))
        gx = max(0, min(gx, width - 1))
        gy = max(0, min(gy, height - 1))
        grid[gy][gx] = 1
        
        if i > 0:
            ppx, ppy = points[i-1]
            dist = math.hypot(px - ppx, py - ppy)
            if dist <= threshold:
                pgx = int(round((ppx - min_x) / resolution))
                pgy = int(round((ppy - min_y) / resolution))
                pgx = max(0, min(pgx, width - 1))
                pgy = max(0, min(pgy, height - 1))
                draw_line(pgx, pgy, gx, gy)

    temp_grid = [row[:] for row in grid]
    for y in range(height):
        for x in range(width):
            if temp_grid[y][x] == 1:
                if width_mode == 'thin':
                    if y+1 < height and x+1 < width and temp_grid[y+1][x+1] == 1 and temp_grid[y][x+1] == 0 and temp_grid[y+1][x] == 0:
                        grid[y][x+1] = 1
                    if y-1 >= 0 and x+1 < width and temp_grid[y-1][x+1] == 1 and temp_grid[y][x+1] == 0 and temp_grid[y-1][x] == 0:
                        grid[y][x+1] = 1
                elif width_mode == '4-wide':
                    # Sag ve asagi dogru 2x2 genisletme yaparak yolun genisligini tam 4 birim yapiyoruz. 
                    # Cunku -1 ve +1'de noktalar var. (-1, 0) ve (1, 2) olusturarak -1,0,1,2 genisligini tam 4 piksele tamamlar.
                    for dy, dx in [(0,0), (0,1), (1,0), (1,1)]:
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < height and 0 <= nx < width:
                             grid[ny][nx] = 1

    os.makedirs(os.path.dirname(output_txt), exist_ok=True)
    with open(output_txt, 'w') as f:
        for row in reversed(grid):
            line = "".join(str(cell) for cell in row)
            f.write(line + "\n")
            
    if output_npy:
        os.makedirs(os.path.dirname(output_npy), exist_ok=True)
        np_grid = np.array(grid)
        np.save(output_npy, np.flipud(np_grid))
            
    print(f"Izgara Matrisi ({width_mode}) (Boyutlar: {width}x{height}, Çözünürlük: {resolution}m) oluşturuldu: {output_txt}")

if __name__ == '__main__':
    # Onceki ince hali (ayri seritler)
    generate_grid_from_csv("waypoints/full_road_map.csv", "matrices/road_grid.txt", "matrices/road_grid.npy", resolution=1.0, width_mode='thin')
    # Yeni islenen tam 4 kalinlik (ortasi dolmus tek parca yol)
    generate_grid_from_csv("waypoints/full_road_map.csv", "matrices/road_grid_4wide.txt", "matrices/road_grid_4wide.npy", resolution=1.0, width_mode='4-wide')
