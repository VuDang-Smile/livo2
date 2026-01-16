#!/usr/bin/env python3
"""
Tính vector hướng của đường hầm từ trajectory file.

Phương án 1: Tính từ trajectory file (đơn giản, nhanh, đáng tin cậy)
- Đọc trajectory file từ Log/result/{seq_name}.txt
- Tính vector hướng giữa các pose liên tiếp
- Lọc nhiễu và tính vector tổng hợp
- Lưu kết quả vào metadata JSON

Usage:
    python compute_tunnel_direction.py --pcd_path ws/src/FAST-LIVO2/Log/merged_map/merged_all.pcd
    python compute_tunnel_direction.py --pcd_dir ws/src/FAST-LIVO2/Log/merged_map
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


def load_trajectory(traj_path: Path) -> List[Dict]:
    """
    Đọc trajectory file.
    
    Format: timestamp x y z qx qy qz qw (mỗi dòng)
    
    Returns:
        List of pose dicts với keys: timestamp, x, y, z, qx, qy, qz, qw
    """
    if not traj_path.exists():
        raise FileNotFoundError(f"Trajectory file not found: {traj_path}")
    
    poses = []
    with open(traj_path, 'r') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            parts = line.split()
            if len(parts) < 8:
                print(f"[WARNING] Line {line_num} has invalid format, skipping: {line[:50]}")
                continue
            
            try:
                timestamp = float(parts[0])
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
                
                poses.append({
                    'timestamp': timestamp,
                    'x': x,
                    'y': y,
                    'z': z,
                    'qx': qx,
                    'qy': qy,
                    'qz': qz,
                    'qw': qw,
                })
            except ValueError as e:
                print(f"[WARNING] Line {line_num} parse error: {e}, skipping")
                continue
    
    if not poses:
        raise ValueError(f"No valid poses found in trajectory file: {traj_path}")
    
    print(f"[INFO] Loaded {len(poses)} poses from trajectory file")
    return poses


def compute_direction_vectors(poses: List[Dict], min_distance: float = 0.1) -> np.ndarray:
    """
    Tính vector hướng giữa các pose liên tiếp.
    
    Args:
        poses: List of pose dicts
        min_distance: Khoảng cách tối thiểu giữa 2 pose để tính vector (m)
    
    Returns:
        Array of direction vectors (N-1, 3)
    """
    if len(poses) < 2:
        raise ValueError("Need at least 2 poses to compute direction vectors")
    
    directions = []
    
    for i in range(len(poses) - 1):
        p1 = poses[i]
        p2 = poses[i + 1]
        
        # Vector từ pose i đến pose i+1
        dx = p2['x'] - p1['x']
        dy = p2['y'] - p1['y']
        dz = p2['z'] - p1['z']
        
        distance = np.sqrt(dx**2 + dy**2 + dz**2)
        
        # Chỉ tính vector nếu khoảng cách đủ lớn (lọc nhiễu)
        if distance >= min_distance:
            direction = np.array([dx, dy, dz]) / distance  # Normalize
            directions.append(direction)
        else:
            print(f"[DEBUG] Skipping pose pair {i}-{i+1}: distance too small ({distance:.4f}m)")
    
    if not directions:
        raise ValueError(f"No valid direction vectors found (all distances < {min_distance}m)")
    
    return np.array(directions)


def filter_directions(
    directions: np.ndarray,
    angle_threshold_deg: float = 45.0
) -> np.ndarray:
    """
    Lọc các vector hướng có góc lệch quá lớn so với trung bình.
    
    Args:
        directions: Array of direction vectors (N, 3)
        angle_threshold_deg: Góc lệch tối đa so với trung bình (degrees)
    
    Returns:
        Filtered direction vectors
    """
    if len(directions) == 0:
        return directions
    
    # Tính vector trung bình
    mean_direction = np.mean(directions, axis=0)
    mean_direction = mean_direction / np.linalg.norm(mean_direction)  # Normalize
    
    angle_threshold_rad = np.deg2rad(angle_threshold_deg)
    filtered = []
    
    for i, direction in enumerate(directions):
        # Tính góc giữa vector này và vector trung bình
        dot_product = np.clip(np.dot(direction, mean_direction), -1.0, 1.0)
        angle = np.arccos(dot_product)
        
        if angle <= angle_threshold_rad:
            filtered.append(direction)
        else:
            angle_deg = np.rad2deg(angle)
            print(f"[DEBUG] Filtering out direction {i}: angle {angle_deg:.1f}° > threshold {angle_threshold_deg}°")
    
    if not filtered:
        print(f"[WARNING] All directions filtered out, using original directions")
        return directions
    
    return np.array(filtered)


def compute_principal_direction(directions: np.ndarray, method: str = 'mean') -> np.ndarray:
    """
    Tính vector hướng tổng hợp từ các direction vectors.
    
    Args:
        directions: Array of direction vectors (N, 3)
        method: 'mean' hoặc 'pca'
            - 'mean': Trung bình đơn giản các vector
            - 'pca': PCA để tìm principal direction
    
    Returns:
        Normalized direction vector (3,)
    """
    if len(directions) == 0:
        raise ValueError("No directions provided")
    
    if method == 'mean':
        # Trung bình đơn giản
        mean_dir = np.mean(directions, axis=0)
        mean_dir = mean_dir / np.linalg.norm(mean_dir)
        return mean_dir
    
    elif method == 'pca':
        # PCA để tìm principal direction
        # Covariance matrix
        cov = np.cov(directions.T)
        # Eigenvalues và eigenvectors
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        # Principal direction là eigenvector tương ứng với eigenvalue lớn nhất
        principal_idx = np.argmax(eigenvalues)
        principal_dir = eigenvectors[:, principal_idx]
        # Đảm bảo hướng nhất quán (luôn cùng chiều với mean)
        mean_dir = np.mean(directions, axis=0)
        if np.dot(principal_dir, mean_dir) < 0:
            principal_dir = -principal_dir
        return principal_dir / np.linalg.norm(principal_dir)
    
    else:
        raise ValueError(f"Unknown method: {method}. Use 'mean' or 'pca'")


def find_trajectory_file(pcd_path: Path, log_result_dir: Path) -> Optional[Path]:
    """
    Tìm trajectory file tương ứng với PCD file.
    
    Tìm kiếm trong log_result_dir:
    - {seq_name}.txt
    - {seq_name}_manual_save.txt
    
    Args:
        pcd_path: Đường dẫn đến PCD file
        log_result_dir: Thư mục chứa trajectory files (Log/trajectory_file/)
    
    Returns:
        Path đến trajectory file hoặc None nếu không tìm thấy
    """
    if not log_result_dir.exists():
        print(f"[WARNING] Trajectory directory not found: {log_result_dir}")
        return None
    
    pcd_stem = pcd_path.stem
    
    # Thử các pattern tên file
    candidates = [
        log_result_dir / f"{pcd_stem}.txt",
        log_result_dir / f"{pcd_stem}_manual_save.txt",
    ]
    
    # Nếu không tìm thấy, thử tìm tất cả .txt files và chọn file gần nhất về thời gian
    for candidate in candidates:
        if candidate.exists():
            print(f"[INFO] Found trajectory file: {candidate}")
            return candidate
    
    # Tìm tất cả trajectory files
    all_traj_files = list(log_result_dir.glob("*.txt"))
    if not all_traj_files:
        print(f"[WARNING] No trajectory files found in {log_result_dir}")
        return None
    
    # Chọn file mới nhất (có thể là file tương ứng)
    latest_file = max(all_traj_files, key=lambda p: p.stat().st_mtime)
    print(f"[INFO] Using latest trajectory file: {latest_file}")
    return latest_file


def compute_tunnel_direction(
    traj_path: Path,
    min_distance: float = 0.1,
    angle_threshold_deg: float = 45.0,
    method: str = 'mean'
) -> Dict:
    """
    Tính vector hướng đường hầm từ trajectory file.
    
    Args:
        traj_path: Đường dẫn đến trajectory file
        min_distance: Khoảng cách tối thiểu giữa 2 pose (m)
        angle_threshold_deg: Góc lệch tối đa để lọc (degrees)
        method: 'mean' hoặc 'pca'
    
    Returns:
        Dict chứa:
        - direction: [x, y, z] normalized vector
        - num_poses: Số lượng poses
        - num_vectors: Số lượng direction vectors
        - total_distance: Tổng quãng đường (m)
        - stats: Thống kê khác
    """
    # Đọc trajectory
    poses = load_trajectory(traj_path)
    
    # Tính direction vectors
    directions = compute_direction_vectors(poses, min_distance=min_distance)
    print(f"[INFO] Computed {len(directions)} direction vectors")
    
    # Lọc nhiễu
    filtered_directions = filter_directions(directions, angle_threshold_deg=angle_threshold_deg)
    print(f"[INFO] After filtering: {len(filtered_directions)} direction vectors")
    
    # Tính vector tổng hợp
    principal_dir = compute_principal_direction(filtered_directions, method=method)
    
    # Tính tổng quãng đường
    total_distance = 0.0
    for i in range(len(poses) - 1):
        p1 = poses[i]
        p2 = poses[i + 1]
        dx = p2['x'] - p1['x']
        dy = p2['y'] - p1['y']
        dz = p2['z'] - p1['z']
        total_distance += np.sqrt(dx**2 + dy**2 + dz**2)
    
    # Tính thống kê
    direction_magnitudes = np.linalg.norm(directions, axis=1) if len(directions) > 0 else []
    
    result = {
        'direction': principal_dir.tolist(),
        'num_poses': len(poses),
        'num_vectors': len(directions),
        'num_filtered_vectors': len(filtered_directions),
        'total_distance_m': float(total_distance),
        'stats': {
            'min_vector_length': float(np.min(direction_magnitudes)) if len(direction_magnitudes) > 0 else 0.0,
            'max_vector_length': float(np.max(direction_magnitudes)) if len(direction_magnitudes) > 0 else 0.0,
            'mean_vector_length': float(np.mean(direction_magnitudes)) if len(direction_magnitudes) > 0 else 0.0,
        },
        'method': method,
        'trajectory_file': str(traj_path),
    }
    
    return result


def save_direction_metadata(
    direction_data: Dict,
    output_path: Path,
    pcd_path: Optional[Path] = None
) -> None:
    """
    Lưu metadata về vector hướng vào file JSON.
    
    Args:
        direction_data: Dict chứa thông tin vector hướng
        output_path: Đường dẫn file JSON output
        pcd_path: Đường dẫn PCD file (optional, để thêm vào metadata)
    """
    metadata = {
        'tunnel_direction': direction_data,
        'pcd_file': str(pcd_path) if pcd_path else None,
    }
    
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(metadata, f, ensure_ascii=False, indent=2)
    
    print(f"[INFO] Saved direction metadata to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Tính vector hướng đường hầm từ trajectory file",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Tính từ PCD file cụ thể
  python compute_tunnel_direction.py --pcd_path ws/src/FAST-LIVO2/Log/merged_map/merged_all.pcd
  
  # Tính từ thư mục merged_map (tìm tất cả PCD files)
  python compute_tunnel_direction.py --pcd_dir ws/src/FAST-LIVO2/Log/merged_map
  
  # Chỉ định trajectory file cụ thể
  python compute_tunnel_direction.py --traj_path ws/src/FAST-LIVO2/Log/result/01.txt
        """
    )
    
    parser.add_argument(
        '--pcd_path',
        type=str,
        help='Đường dẫn đến PCD file'
    )
    parser.add_argument(
        '--pcd_dir',
        type=str,
        help='Thư mục chứa PCD files (sẽ tìm tất cả .pcd files)'
    )
    parser.add_argument(
        '--traj_path',
        type=str,
        help='Đường dẫn đến trajectory file (nếu không chỉ định, sẽ tự tìm)'
    )
    parser.add_argument(
        '--log_result_dir',
        type=str,
        default='ws/src/FAST-LIVO2/Log/trajectory_file',
        help='Thư mục chứa trajectory files (default: ws/src/FAST-LIVO2/Log/trajectory_file)'
    )
    parser.add_argument(
        '--output_dir',
        type=str,
        help='Thư mục output (default: cùng thư mục với PCD file)'
    )
    parser.add_argument(
        '--min_distance',
        type=float,
        default=0.1,
        help='Khoảng cách tối thiểu giữa 2 pose để tính vector (m, default: 0.1)'
    )
    parser.add_argument(
        '--angle_threshold',
        type=float,
        default=45.0,
        help='Góc lệch tối đa để lọc vector (degrees, default: 45.0)'
    )
    parser.add_argument(
        '--method',
        type=str,
        choices=['mean', 'pca'],
        default='mean',
        help='Phương pháp tính vector tổng hợp: mean hoặc pca (default: mean)'
    )
    
    args = parser.parse_args()
    
    # Xác định script directory và project root
    script_dir = Path(__file__).parent.resolve()
    project_root = script_dir.parent
    
    # Xử lý input
    pcd_files = []
    if args.pcd_path:
        pcd_path = Path(args.pcd_path)
        if not pcd_path.is_absolute():
            pcd_path = project_root / pcd_path
        if not pcd_path.exists():
            print(f"[ERROR] PCD file not found: {pcd_path}")
            sys.exit(1)
        pcd_files.append(pcd_path)
    elif args.pcd_dir:
        pcd_dir = Path(args.pcd_dir)
        if not pcd_dir.is_absolute():
            pcd_dir = project_root / pcd_dir
        if not pcd_dir.exists():
            print(f"[ERROR] PCD directory not found: {pcd_dir}")
            sys.exit(1)
        pcd_files = list(pcd_dir.glob("*.pcd"))
        if not pcd_files:
            print(f"[ERROR] No PCD files found in: {pcd_dir}")
            sys.exit(1)
        print(f"[INFO] Found {len(pcd_files)} PCD files")
    else:
        parser.print_help()
        sys.exit(1)
    
    # Xác định log_result_dir
    log_result_dir = Path(args.log_result_dir)
    if not log_result_dir.is_absolute():
        log_result_dir = project_root / log_result_dir
    
    # Xử lý từng PCD file
    for pcd_path in pcd_files:
        print(f"\n{'='*70}")
        print(f"[INFO] Processing PCD: {pcd_path}")
        print(f"{'='*70}")
        
        # Tìm trajectory file
        if args.traj_path:
            traj_path = Path(args.traj_path)
            if not traj_path.is_absolute():
                traj_path = project_root / traj_path
        else:
            traj_path = find_trajectory_file(pcd_path, log_result_dir)
        
        if traj_path is None or not traj_path.exists():
            print(f"[ERROR] Trajectory file not found for {pcd_path.name}")
            print(f"[INFO] Skipping...")
            continue
        
        try:
            # Tính vector hướng
            direction_data = compute_tunnel_direction(
                traj_path,
                min_distance=args.min_distance,
                angle_threshold_deg=args.angle_threshold,
                method=args.method
            )
            
            # In kết quả
            print(f"\n[RESULT] Tunnel Direction Vector:")
            print(f"  Direction: [{direction_data['direction'][0]:.6f}, "
                  f"{direction_data['direction'][1]:.6f}, "
                  f"{direction_data['direction'][2]:.6f}]")
            print(f"  Number of poses: {direction_data['num_poses']}")
            print(f"  Number of vectors: {direction_data['num_vectors']}")
            print(f"  Filtered vectors: {direction_data['num_filtered_vectors']}")
            print(f"  Total distance: {direction_data['total_distance_m']:.2f} m")
            print(f"  Method: {direction_data['method']}")
            
            # Lưu metadata
            if args.output_dir:
                output_dir = Path(args.output_dir)
                if not output_dir.is_absolute():
                    output_dir = project_root / output_dir
            else:
                output_dir = pcd_path.parent
            
            output_path = output_dir / f"{pcd_path.stem}_tunnel_direction.json"
            save_direction_metadata(direction_data, output_path, pcd_path)
            
            print(f"[DONE] Successfully computed tunnel direction for {pcd_path.name}")
            
        except Exception as e:
            print(f"[ERROR] Failed to compute tunnel direction for {pcd_path.name}: {e}")
            import traceback
            traceback.print_exc()
            continue


if __name__ == "__main__":
    main()
