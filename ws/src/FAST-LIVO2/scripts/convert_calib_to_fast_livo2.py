#!/usr/bin/env python3
"""
Script để convert kết quả calibration từ direct_visual_lidar_calibration sang format của FAST-LIVO2

Usage:
    python3 convert_calib_to_fast_livo2.py <path_to_calib.json> [--output <output_yaml_file>]

Input:
    - calib.json từ direct_visual_lidar_calibration chứa T_lidar_camera: [x, y, z, qx, qy, qz, qw]
    
Output:
    - Rcl và Pcl cho FAST-LIVO2 config file
    - Rcl: rotation matrix từ lidar sang camera (9 values, row-major)
    - Pcl: translation vector từ lidar sang camera (3 values)
"""

import json
import sys
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R


def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """Convert quaternion [qx, qy, qz, qw] to rotation matrix (3x3)"""
    quat = [qw, qx, qy, qz]  # scipy uses [w, x, y, z] format
    r = R.from_quat(quat)
    return r.as_matrix()


def convert_calib_to_fast_livo2(calib_json_path):
    """
    Convert T_lidar_camera từ direct_visual_lidar_calibration sang Rcl và Pcl cho FAST-LIVO2
    
    Args:
        calib_json_path: Đường dẫn đến file calib.json
        
    Returns:
        tuple: (Rcl_list, Pcl_list) - Rcl là list 9 values, Pcl là list 3 values
    """
    # Đọc file calib.json
    with open(calib_json_path, 'r') as f:
        calib_data = json.load(f)
    
    # Lấy T_lidar_camera từ results
    if "results" not in calib_data or "T_lidar_camera" not in calib_data["results"]:
        raise ValueError("Không tìm thấy T_lidar_camera trong calib.json")
    
    T_lidar_camera_values = calib_data["results"]["T_lidar_camera"]
    
    if len(T_lidar_camera_values) != 7:
        raise ValueError(f"T_lidar_camera phải có 7 giá trị [x, y, z, qx, qy, qz, qw], nhưng có {len(T_lidar_camera_values)}")
    
    # Extract translation và quaternion
    trans_lidar_camera = np.array(T_lidar_camera_values[0:3])  # [x, y, z]
    quat_lidar_camera = T_lidar_camera_values[3:7]  # [qx, qy, qz, qw]
    
    # Tạo rotation matrix từ quaternion
    R_lidar_camera = quaternion_to_rotation_matrix(
        quat_lidar_camera[0], quat_lidar_camera[1], 
        quat_lidar_camera[2], quat_lidar_camera[3]
    )
    
    # Tạo transformation matrix T_lidar_camera (4x4)
    T_lidar_camera_4x4 = np.eye(4)
    T_lidar_camera_4x4[0:3, 0:3] = R_lidar_camera
    T_lidar_camera_4x4[0:3, 3] = trans_lidar_camera
    
    # Inverse để có T_camera_lidar (từ lidar sang camera)
    T_camera_lidar_4x4 = np.linalg.inv(T_lidar_camera_4x4)
    
    # Extract Rcl (rotation matrix từ lidar sang camera)
    Rcl = T_camera_lidar_4x4[0:3, 0:3]
    
    # Extract Pcl (translation vector từ lidar sang camera)
    Pcl = T_camera_lidar_4x4[0:3, 3]
    
    # Convert sang list format cho YAML (row-major cho rotation matrix)
    Rcl_list = Rcl.flatten().tolist()  # [r11, r12, r13, r21, r22, r23, r31, r32, r33]
    Pcl_list = Pcl.tolist()  # [x, y, z]
    
    return Rcl_list, Pcl_list


def format_yaml_output(Rcl_list, Pcl_list):
    """Format output thành YAML format cho FAST-LIVO2"""
    yaml_output = f"""    extrin_calib:
      # Calibration từ direct_visual_lidar_calibration
      # Rcl: rotation matrix từ lidar sang camera (row-major)
      Rcl: [{Rcl_list[0]:.8f}, {Rcl_list[1]:.8f}, {Rcl_list[2]:.8f},
            {Rcl_list[3]:.8f}, {Rcl_list[4]:.8f}, {Rcl_list[5]:.8f},
            {Rcl_list[6]:.8f}, {Rcl_list[7]:.8f}, {Rcl_list[8]:.8f}]
      # Pcl: translation vector từ lidar sang camera
      Pcl: [{Pcl_list[0]:.8f}, {Pcl_list[1]:.8f}, {Pcl_list[2]:.8f}]"""
    
    return yaml_output


def main():
    parser = argparse.ArgumentParser(
        description='Convert calibration từ direct_visual_lidar_calibration sang FAST-LIVO2 format'
    )
    parser.add_argument('calib_json', type=str, help='Đường dẫn đến file calib.json')
    parser.add_argument('--output', '-o', type=str, help='Đường dẫn file output YAML (optional)')
    
    args = parser.parse_args()
    
    try:
        # Convert calibration
        Rcl_list, Pcl_list = convert_calib_to_fast_livo2(args.calib_json)
        
        # Format output
        yaml_output = format_yaml_output(Rcl_list, Pcl_list)
        
        # In ra console
        print("=" * 70)
        print("Kết quả conversion cho FAST-LIVO2:")
        print("=" * 70)
        print(yaml_output)
        print("=" * 70)
        
        # Ghi vào file nếu có output path
        if args.output:
            with open(args.output, 'w') as f:
                f.write(yaml_output)
            print(f"\nĐã lưu kết quả vào: {args.output}")
        
        # In thông tin chi tiết
        print("\nChi tiết:")
        print(f"  Rcl (rotation matrix từ lidar -> camera):")
        print(f"    [{Rcl_list[0]:.8f}, {Rcl_list[1]:.8f}, {Rcl_list[2]:.8f}]")
        print(f"    [{Rcl_list[3]:.8f}, {Rcl_list[4]:.8f}, {Rcl_list[5]:.8f}]")
        print(f"    [{Rcl_list[6]:.8f}, {Rcl_list[7]:.8f}, {Rcl_list[8]:.8f}]")
        print(f"  Pcl (translation từ lidar -> camera):")
        print(f"    [{Pcl_list[0]:.8f}, {Pcl_list[1]:.8f}, {Pcl_list[2]:.8f}]")
        
    except Exception as e:
        print(f"Lỗi: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()

