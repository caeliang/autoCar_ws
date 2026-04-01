import sys
import re

def scale_pose(pose_val, scale):
    parts = pose_val.strip().split()
    if len(parts) != 6:
        return pose_val
    try:
        x = float(parts[0]) * scale
        y = float(parts[1]) * scale
        z = float(parts[2]) # Keep Z same or slightly scale? Keep same for 0 height
        r, p, yaw = parts[3], parts[4], parts[5]
        return f"{x} {y} {z} {r} {p} {yaw}"
    except:
        return pose_val

def scale_size(size_val, scale):
    parts = size_val.strip().split()
    try:
        # Scale X and Y, keep Z (height) same for roads and curbs
        sx = float(parts[0]) * scale
        sy = float(parts[1]) * scale
        sz = float(parts[2])
        return f"{sx} {sy} {sz}"
    except:
        return size_val

def scale_world(input_file, output_file, scale):
    print(f"Scaling {input_file} by {scale}, EXCLUDING Prius model...")
    with open(input_file, 'r') as f:
        lines = f.readlines()
    output_lines = []
    in_prius = False
    for line in lines:
        if "<model name='prius'>" in line:
            in_prius = True
        if in_prius:
            output_lines.append(line)
            if "</model>" in line:
                in_prius = False
            continue
        # Scale poses and sizes
        scaled_line = re.sub(r'<pose[^>]*>([^<]+)</pose>', 
                           lambda m: f"<pose>{scale_pose(m.group(1), scale)}</pose>", line)
        scaled_line = re.sub(r'<size[^>]*>([^<]+)</size>', 
                           lambda m: f"<size>{scale_size(m.group(1), scale)}</size>", scaled_line)
        output_lines.append(scaled_line)
    with open(output_file, 'w') as f:
        f.writelines(output_lines)
    print(f"Done! Scaled world saved to {output_file}")

if __name__ == "__main__":
    scale_world(sys.argv[1], sys.argv[2], float(sys.argv[3]))
