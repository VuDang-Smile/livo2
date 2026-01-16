#!/usr/bin/env python3
"""
Script xoay PCD file quanh trục Oz (trục thẳng đứng) để hướng đường hầm về trục +x.

Script này đọc file PCD và file JSON chứa direction vector của đường hầm,
tính toán góc xoay cần thiết quanh trục Oy (trục thẳng đứng) để hướng
đường hầm về trục +x, sau đó áp dụng transformation và lưu kết quả.

Usage:
    python rotate_pcd_to_positive_x.py --pcd_path ws/src/FAST-LIVO2/Log/merged_map/merged_all.pcd
    python rotate_pcd_to_positive_x.py --pcd_path merged_all.pcd --direction_json merged_all_tunnel_direction.json
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

try:
    import open3d as o3d
except ImportError as e:
    raise SystemExit(
        "Thiếu thư viện 'open3d'. Hãy cài trước khi chạy:\n"
        "    pip install open3d\n"
    ) from e


def load_pcd(path: Path) -> o3d.geometry.PointCloud:
    """
    Load PCD file và trả về Open3D point cloud.
    
    Args:
        path: Đường dẫn tới file PCD
    
    Returns:
        Point cloud object
    
    Raises:
        FileNotFoundError: Nếu file không tồn tại
        RuntimeError: Nếu PCD rỗng
    """
    if not path.exists():
        raise FileNotFoundError(f"Không tìm thấy file PCD: {path}")
    
    print(f"[INFO] Đang đọc file PCD: {path}")
    pcd = o3d.io.read_point_cloud(str(path))
    
    if pcd.is_empty():
        raise RuntimeError(f"PCD rỗng: {path}")
    
    num_points = len(pcd.points)
    print(f"[INFO] Đã đọc {num_points:,} điểm từ PCD")
    
    return pcd


def load_direction_json(json_path: Path) -> Tuple[np.ndarray, Dict]:
    """
    Đọc direction vector từ file JSON.
    
    Args:
        json_path: Đường dẫn tới file JSON chứa direction vector
    
    Returns:
        (direction_vector, metadata_dict)
        - direction_vector: numpy array [dx, dy, dz] đã normalize
        - metadata_dict: toàn bộ nội dung JSON
    
    Raises:
        FileNotFoundError: Nếu file không tồn tại
        ValueError: Nếu format JSON không đúng hoặc thiếu direction
    """
    if not json_path.exists():
        raise FileNotFoundError(f"Không tìm thấy file JSON: {json_path}")
    
    print(f"[INFO] Đang đọc file JSON: {json_path}")
    
    with open(json_path, 'r', encoding='utf-8') as f:
        metadata = json.load(f)
    
    # Tìm direction vector trong JSON
    # Có thể ở tunnel_direction.direction hoặc direction
    direction = None
    if 'tunnel_direction' in metadata and 'direction' in metadata['tunnel_direction']:
        direction = metadata['tunnel_direction']['direction']
    elif 'direction' in metadata:
        direction = metadata['direction']
    else:
        raise ValueError(
            f"Không tìm thấy 'direction' trong JSON. "
            f"Các keys có sẵn: {list(metadata.keys())}"
        )
    
    # Convert sang numpy array và normalize
    direction_vec = np.array(direction, dtype=np.float64)
    
    if len(direction_vec) != 3:
        raise ValueError(f"Direction vector phải có 3 phần tử, nhận được {len(direction_vec)}")
    
    # Normalize vector
    norm = np.linalg.norm(direction_vec)
    if norm < 1e-6:
        raise ValueError(f"Direction vector quá nhỏ (norm={norm}), không thể normalize")
    
    direction_vec = direction_vec / norm
    
    print(f"[INFO] Direction vector: [{direction_vec[0]:.6f}, {direction_vec[1]:.6f}, {direction_vec[2]:.6f}]")
    print(f"[INFO] Norm: {np.linalg.norm(direction_vec):.6f}")
    
    return direction_vec, metadata


def compute_rotation_angle_around_z(direction: np.ndarray) -> float:
    """
    Tính góc xoay quanh trục Oz (trục thẳng đứng) để hướng direction vector về trục +x.
    
    Trong ROS convention (PCD gốc): X=forward, Y=left, Z=up (trục thẳng đứng là Z).
    Xoay quanh trục Oz nghĩa là chỉ thay đổi góc trong mặt phẳng XY, giữ nguyên thành phần Z.
    
    Args:
        direction: Direction vector [dx, dy, dz] đã normalize (ROS convention)
    
    Returns:
        Góc xoay (radian). Góc dương theo quy ước right-hand rule.
        Sau khi xoay, direction sẽ hướng về [1, 0, dz] (dy = 0).
    """
    dx, dy, dz = direction[0], direction[1], direction[2]
    
    # Projection của direction lên mặt phẳng XY
    # Chúng ta muốn xoay để projection này hướng về [1, 0]
    proj_xy_norm = np.sqrt(dx**2 + dy**2)
    
    if proj_xy_norm < 1e-6:
        # Direction vector gần như song song với trục Oz
        # Không thể xác định góc xoay, trả về 0
        print("[WARNING] Direction vector gần như song song với trục Oz, góc xoay = 0")
        return 0.0
    
    # Tính góc hiện tại của projection trong mặt phẳng XY
    # atan2(dy, dx) cho góc từ trục +x đến vector (dx, dy) trong mặt phẳng XY
    current_angle = np.arctan2(dy, dx)
    
    # Để hướng về +x (làm cho dy' = 0 sau khi xoay quanh Z), ta cần xoay một góc
    # sao cho: dy' = sin(θ)*dx + cos(θ)*dy = 0
    # Giải phương trình: sin(θ)*dx + cos(θ)*dy = 0
    # => tan(θ) = -dy/dx
    # => θ = arctan2(-dy, dx) = -arctan2(dy, dx) = -current_angle
    # 
    # Tuy nhiên, nếu direction đang hướng về -x (dx < 0), ta cần xoay thêm 180 độ
    # để đảo chiều về +x
    if dx < 0:
        # Direction đang hướng về -x, cần xoay 180 độ để đảo chiều
        angle = -current_angle + np.pi
    else:
        # Direction đang hướng về +x, xoay góc -current_angle để làm dy = 0
        angle = -current_angle
    
    # Normalize góc về range [-pi, pi]
    angle = np.arctan2(np.sin(angle), np.cos(angle))
    
    print(f"[INFO] Góc hiện tại trong mặt phẳng XY: {np.rad2deg(current_angle):.2f}°")
    print(f"[INFO] Góc xoay quanh trục Oz (trục thẳng đứng): {np.rad2deg(angle):.2f}° ({angle:.6f} rad)")
    
    return angle


def create_rotation_matrix_z(angle: float) -> np.ndarray:
    """
    Tạo rotation matrix 4x4 cho phép xoay quanh trục Oz (trục thẳng đứng).
    
    Rotation matrix quanh trục Oz:
        R_z(θ) = [[cos(θ), -sin(θ), 0, 0],
                  [sin(θ),  cos(θ), 0, 0],
                  [0,       0,      1, 0],
                  [0,       0,      0, 1]]
    
    Args:
        angle: Góc xoay (radian)
    
    Returns:
        Transformation matrix 4x4 (numpy array)
    """
    cos_a = np.cos(angle)
    sin_a = np.sin(angle)
    
    # Rotation matrix quanh trục Oz
    R = np.array([
        [cos_a, -sin_a, 0, 0],
        [sin_a,  cos_a, 0, 0],
        [0,      0,     1, 0],
        [0,      0,     0, 1]
    ], dtype=np.float64)
    
    return R


def apply_rotation_to_direction(direction: np.ndarray, rotation_matrix: np.ndarray) -> np.ndarray:
    """
    Áp dụng rotation matrix lên direction vector để kiểm tra kết quả.
    
    Args:
        direction: Direction vector gốc [dx, dy, dz]
        rotation_matrix: Rotation matrix 4x4
    
    Returns:
        Direction vector sau khi xoay [dx', dy', dz']
    """
    # Chuyển direction thành vector 4D (homogeneous coordinates)
    dir_homogeneous = np.array([direction[0], direction[1], direction[2], 0.0])
    
    # Áp dụng rotation (chỉ rotation, không translation)
    dir_rotated_homogeneous = rotation_matrix @ dir_homogeneous
    
    # Lấy 3 thành phần đầu và normalize
    dir_rotated = dir_rotated_homogeneous[:3]
    dir_rotated = dir_rotated / np.linalg.norm(dir_rotated)
    
    return dir_rotated


def rotate_pointcloud(pcd: o3d.geometry.PointCloud, rotation_matrix: np.ndarray) -> o3d.geometry.PointCloud:
    """
    Áp dụng rotation matrix lên point cloud.
    
    Args:
        pcd: Point cloud gốc
        rotation_matrix: Transformation matrix 4x4
    
    Returns:
        Point cloud đã xoay (copy mới)
    """
    print("[INFO] Đang áp dụng rotation matrix lên point cloud...")
    
    # Tạo copy để không thay đổi point cloud gốc
    pcd_rotated = o3d.geometry.PointCloud(pcd)
    
    # Áp dụng transformation
    pcd_rotated.transform(rotation_matrix)
    
    print("[INFO] Đã hoàn thành rotation")
    
    return pcd_rotated


def save_pcd(pcd: o3d.geometry.PointCloud, output_path: Path, binary: bool = True) -> None:
    """
    Lưu point cloud ra file PCD.
    
    Args:
        pcd: Point cloud cần lưu
        output_path: Đường dẫn file output
        binary: Lưu dạng binary (True) hay ASCII (False)
    
    Raises:
        RuntimeError: Nếu không thể lưu file
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    print(f"[INFO] Đang lưu PCD: {output_path}")
    
    success = o3d.io.write_point_cloud(
        str(output_path),
        pcd,
        write_ascii=not binary,
        compressed=False
    )
    
    if not success:
        raise RuntimeError(f"Không thể lưu PCD file: {output_path}")
    
    print(f"[INFO] Đã lưu {len(pcd.points):,} điểm vào {output_path}")


def save_rotation_metadata(
    rotation_angle_rad: float,
    rotation_angle_deg: float,
    rotation_matrix: np.ndarray,
    direction_before: np.ndarray,
    direction_after: np.ndarray,
    input_pcd_path: Path,
    output_pcd_path: Path,
    output_json_path: Path,
    original_metadata: Optional[Dict] = None
) -> None:
    """
    Lưu metadata về rotation vào file JSON.
    
    Args:
        rotation_angle_rad: Góc xoay (radian)
        rotation_angle_deg: Góc xoay (độ)
        rotation_matrix: Rotation matrix 4x4
        direction_before: Direction vector trước khi xoay
        direction_after: Direction vector sau khi xoay
        input_pcd_path: Đường dẫn file PCD input
        output_pcd_path: Đường dẫn file PCD output
        output_json_path: Đường dẫn file JSON output
        original_metadata: Metadata gốc từ file JSON input (optional)
    """
    # Tạo metadata
    metadata = {
        "rotation_info": {
            "rotation_angle_rad": float(rotation_angle_rad),
            "rotation_angle_deg": float(rotation_angle_deg),
            "rotation_axis": [0.0, 0.0, 1.0],  # Trục Oz (trục thẳng đứng trong ROS convention)
            "rotation_matrix": rotation_matrix.tolist(),
            "method": "align_to_positive_x_around_z_axis"
        },
        "direction_before": direction_before.tolist(),
        "direction_after": direction_after.tolist(),
        "input_pcd": str(input_pcd_path),
        "output_pcd": str(output_pcd_path),
        "timestamp": datetime.now().isoformat(),
        "original_metadata": original_metadata if original_metadata else None
    }
    
    # Lưu file JSON
    output_json_path.parent.mkdir(parents=True, exist_ok=True)
    
    print(f"[INFO] Đang lưu metadata: {output_json_path}")
    
    with open(output_json_path, 'w', encoding='utf-8') as f:
        json.dump(metadata, f, ensure_ascii=False, indent=2)
    
    print(f"[INFO] Đã lưu metadata vào {output_json_path}")


def find_direction_json(pcd_path: Path) -> Optional[Path]:
    """
    Tự động tìm file JSON chứa direction vector trong cùng thư mục với PCD.
    
    Tìm kiếm các pattern:
    - {pcd_stem}_tunnel_direction.json
    - tunnel_direction.json
    - {pcd_stem}_direction.json
    
    Args:
        pcd_path: Đường dẫn file PCD
    
    Returns:
        Đường dẫn file JSON nếu tìm thấy, None nếu không
    """
    pcd_dir = pcd_path.parent
    pcd_stem = pcd_path.stem
    
    # Các pattern tên file có thể
    candidates = [
        pcd_dir / f"{pcd_stem}_tunnel_direction.json",
        pcd_dir / f"tunnel_direction.json",
        pcd_dir / f"{pcd_stem}_direction.json",
    ]
    
    for candidate in candidates:
        if candidate.exists():
            print(f"[INFO] Tìm thấy file JSON: {candidate}")
            return candidate
    
    return None


def main():
    """Hàm main của script."""
    parser = argparse.ArgumentParser(
        description="Xoay PCD file quanh trục Oz (trục thẳng đứng) để hướng đường hầm về trục +x",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Sử dụng với file PCD và tự động tìm JSON
  python rotate_pcd_to_positive_x.py --pcd_path ws/src/FAST-LIVO2/Log/merged_map/merged_all.pcd
  
  # Chỉ định cả PCD và JSON
  python rotate_pcd_to_positive_x.py --pcd_path merged_all.pcd --direction_json merged_all_tunnel_direction.json
  
  # Chỉ định thư mục output
  python rotate_pcd_to_positive_x.py --pcd_path merged_all.pcd --output_dir ./output
        """
    )
    
    parser.add_argument(
        '--pcd_path',
        type=str,
        required=True,
        help='Đường dẫn đến file PCD cần xoay'
    )
    parser.add_argument(
        '--direction_json',
        type=str,
        default=None,
        help='Đường dẫn đến file JSON chứa direction vector (nếu không chỉ định, sẽ tự tìm)'
    )
    parser.add_argument(
        '--output_dir',
        type=str,
        default=None,
        help='Thư mục output (mặc định: cùng thư mục với file PCD input)'
    )
    parser.add_argument(
        '--output_name',
        type=str,
        default=None,
        help='Tên file output (không bao gồm extension, mặc định: {input_name}_rotated)'
    )
    parser.add_argument(
        '--binary',
        type=bool,
        default=True,
        help='Lưu PCD dạng binary (True) hay ASCII (False), mặc định: True'
    )
    
    args = parser.parse_args()
    
    # Xác định script directory và project root
    script_dir = Path(__file__).parent.resolve()
    project_root = script_dir.parent
    
    # Xử lý đường dẫn PCD
    pcd_path = Path(args.pcd_path)
    if not pcd_path.is_absolute():
        pcd_path = project_root / pcd_path
    
    if not pcd_path.exists():
        print(f"[ERROR] File PCD không tồn tại: {pcd_path}")
        sys.exit(1)
    
    # Xử lý đường dẫn JSON direction
    if args.direction_json:
        direction_json_path = Path(args.direction_json)
        if not direction_json_path.is_absolute():
            direction_json_path = project_root / direction_json_path
    else:
        # Tự động tìm file JSON
        direction_json_path = find_direction_json(pcd_path)
        if direction_json_path is None:
            print(f"[ERROR] Không tìm thấy file JSON chứa direction vector")
            print(f"[INFO] Hãy chỉ định bằng --direction_json hoặc đặt file JSON trong cùng thư mục với PCD")
            sys.exit(1)
    
    # Xác định thư mục output
    if args.output_dir:
        output_dir = Path(args.output_dir)
        if not output_dir.is_absolute():
            output_dir = project_root / output_dir
    else:
        output_dir = pcd_path.parent
    
    # Xác định tên file output
    if args.output_name:
        output_pcd_name = args.output_name + ".pcd"
        output_json_name = args.output_name + "_rotation_metadata.json"
    else:
        output_pcd_name = f"{pcd_path.stem}_rotated.pcd"
        output_json_name = f"{pcd_path.stem}_rotation_metadata.json"
    
    output_pcd_path = output_dir / output_pcd_name
    output_json_path = output_dir / output_json_name
    
    print(f"\n{'='*70}")
    print(f"[INFO] Xoay PCD để hướng đường hầm về trục +x")
    print(f"{'='*70}")
    print(f"[INFO] Input PCD: {pcd_path}")
    print(f"[INFO] Direction JSON: {direction_json_path}")
    print(f"[INFO] Output PCD: {output_pcd_path}")
    print(f"[INFO] Output JSON: {output_json_path}")
    print(f"{'='*70}\n")
    
    try:
        # 1. Load PCD
        pcd = load_pcd(pcd_path)
        
        # 2. Load direction vector
        direction_vec, original_metadata = load_direction_json(direction_json_path)
        
        # 3. Tính góc xoay quanh trục Oz (trục thẳng đứng trong ROS convention)
        rotation_angle = compute_rotation_angle_around_z(direction_vec)
        rotation_angle_deg = np.rad2deg(rotation_angle)
        
        # 4. Tạo rotation matrix
        rotation_matrix = create_rotation_matrix_z(rotation_angle)
        
        # 5. Kiểm tra direction sau khi xoay (để verify)
        direction_after = apply_rotation_to_direction(direction_vec, rotation_matrix)
        print(f"[INFO] Direction sau khi xoay: [{direction_after[0]:.6f}, {direction_after[1]:.6f}, {direction_after[2]:.6f}]")
        
        # Tính góc giữa direction sau xoay và [1, 0, 0] để verify
        target_direction = np.array([1.0, 0.0, 0.0])
        dot_product = np.clip(np.dot(direction_after, target_direction), -1.0, 1.0)
        angle_to_target = np.arccos(dot_product)
        angle_to_target_deg = np.rad2deg(angle_to_target)
        
        # Kiểm tra projection lên mặt phẳng XY
        proj_xy_after = np.array([direction_after[0], direction_after[1], 0.0])
        proj_xy_after = proj_xy_after / np.linalg.norm(proj_xy_after) if np.linalg.norm(proj_xy_after) > 1e-6 else proj_xy_after
        angle_proj_to_x = np.rad2deg(np.arccos(np.clip(np.dot(proj_xy_after, target_direction), -1.0, 1.0)))
        
        print(f"[INFO] Góc lệch tổng thể so với trục +x: {angle_to_target_deg:.2f}°")
        print(f"[INFO] Góc lệch projection XY so với trục +x: {angle_proj_to_x:.2f}°")
        
        if abs(direction_after[1]) < 0.01:
            print(f"[INFO] ✓ Projection lên mặt phẳng XY đã hướng về +x (dy ≈ 0)")
        else:
            print(f"[WARNING] Projection XY chưa hướng về +x (dy = {direction_after[1]:.6f})")
        
        if angle_to_target_deg > 5.0:
            print(f"[INFO] Góc lệch tổng thể ({angle_to_target_deg:.2f}°) là do thành phần Z không thay đổi khi xoay quanh Oz")
            print(f"[INFO] Đây là hành vi đúng và mong đợi. Projection XY đã hướng về +x.")
        
        # 6. Áp dụng rotation lên point cloud
        pcd_rotated = rotate_pointcloud(pcd, rotation_matrix)
        
        # 7. Lưu PCD đã xoay
        save_pcd(pcd_rotated, output_pcd_path, binary=args.binary)
        
        # 8. Lưu metadata
        save_rotation_metadata(
            rotation_angle_rad=rotation_angle,
            rotation_angle_deg=rotation_angle_deg,
            rotation_matrix=rotation_matrix,
            direction_before=direction_vec,
            direction_after=direction_after,
            input_pcd_path=pcd_path,
            output_pcd_path=output_pcd_path,
            output_json_path=output_json_path,
            original_metadata=original_metadata
        )
        
        print(f"\n{'='*70}")
        print(f"[DONE] Hoàn thành!")
        print(f"[INFO] File PCD đã xoay: {output_pcd_path}")
        print(f"[INFO] File metadata: {output_json_path}")
        print(f"[INFO] Góc xoay: {rotation_angle_deg:.2f}° ({rotation_angle:.6f} rad)")
        print(f"{'='*70}\n")
        
    except Exception as e:
        print(f"\n[ERROR] Lỗi khi xử lý: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
