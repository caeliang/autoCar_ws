#!/usr/bin/env python3
"""
Quick verification script for SLAM + EKF setup
"""

import subprocess
import sys
import time

def check_file_exists(path):
    """Check if a file exists"""
    try:
        with open(path, 'r'):
            return True
    except:
        return False

def run_cmd(cmd):
    """Run command and return output"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=2)
        return result.stdout.strip(), result.returncode
    except:
        return "", 1

def main():
    print("\n" + "="*60)
    print("  SLAM + EKF Setup Verification")
    print("="*60 + "\n")
    
    workspace = "/home/ranim/autoCar_ws"
    checks = [
        ("Install directory exists", f"test -d {workspace}/install"),
        ("slam_mapping package installed", f"test -d {workspace}/install/slam_mapping"),
        ("localization package installed", f"test -d {workspace}/install/localization"),
        ("slam_with_ekf.launch.py exists", f"test -f {workspace}/src/slam_mapping/launch/slam_with_ekf.launch.py"),
        ("slam_toolbox_params.yaml exists", f"test -f {workspace}/src/slam_mapping/config/slam_toolbox_params.yaml"),
        ("ekf.yaml exists", f"test -f {workspace}/src/localization/config/ekf.yaml"),
        ("odom_to_tf.py exists", f"test -f {workspace}/scripts/odom_to_tf.py"),
        ("save_map.py exists", f"test -f {workspace}/scripts/save_map.py"),
        ("keyboard_control.py exists", f"test -f {workspace}/scripts/keyboard_control.py"),
    ]
    
    all_passed = True
    
    print("Configuration Files:")
    print("-" * 60)
    
    for check_name, cmd in checks:
        output, returncode = run_cmd(cmd)
        status = "✓" if returncode == 0 else "✗"
        print(f"  {status} {check_name}")
        if returncode != 0:
            all_passed = False
    
    print("\n" + "="*60)
    
    if all_passed:
        print("\n✓ All configuration files are in place!")
        print("\nQuick Start:")
        print("  Terminal 1: ros2 launch prius_model simulation.launch.py")
        print("  Terminal 2: bash /home/ranim/autoCar_ws/scripts/start_slam_ekf.sh")
        print("  Terminal 3: rviz2")
        print("  Terminal 4: python3 /home/ranim/autoCar_ws/scripts/keyboard_control.py")
        print("\nDiagnostics:")
        print("  python3 /home/ranim/autoCar_ws/scripts/diagnose_slam_ekf.py")
        print("\nGuide:")
        print("  cat /home/ranim/autoCar_ws/SLAM_EKF_GUIDE.md")
        return 0
    else:
        print("\n✗ Some configuration files are missing!")
        print("Run: colcon build --packages-select slam_mapping localization")
        return 1

if __name__ == '__main__':
    sys.exit(main())
