#!/usr/bin/env python3
"""
Phân tích chất lượng trajectory để đánh giá độ tin cậy của tunnel direction.
"""

import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt

# Thêm project root vào path
script_dir = Path(__file__).parent.resolve()
project_root = script_dir.parent
sys.path.insert(0, str(script_dir))

from compute_tunnel_direction import load_trajectory, compute_direction_vectors, filter_directions


def analyze_trajectory_quality(traj_path: Path, min_distance: float = 0.01):
    """
    Phân tích chi tiết chất lượng trajectory.
    """
    print(f"\n{'='*70}")
    print(f"[ANALYSIS] Analyzing trajectory: {traj_path.name}")
    print(f"{'='*70}\n")
    
    # Load trajectory
    poses = load_trajectory(traj_path)
    
    # Tính toán khoảng cách giữa các pose liên tiếp
    distances = []
    valid_pairs = []
    skipped_pairs = []
    
    for i in range(len(poses) - 1):
        p1 = poses[i]
        p2 = poses[i + 1]
        
        dx = p2['x'] - p1['x']
        dy = p2['y'] - p1['y']
        dz = p2['z'] - p1['z']
        
        distance = np.sqrt(dx**2 + dy**2 + dz**2)
        distances.append(distance)
        
        if distance >= min_distance:
            valid_pairs.append(i)
        else:
            skipped_pairs.append(i)
    
    distances = np.array(distances)
    
    # Thống kê khoảng cách
    print("[STATISTICS] Distance Statistics:")
    print(f"  Total pose pairs: {len(distances)}")
    print(f"  Valid pairs (>= {min_distance}m): {len(valid_pairs)} ({100*len(valid_pairs)/len(distances):.1f}%)")
    print(f"  Skipped pairs (< {min_distance}m): {len(skipped_pairs)} ({100*len(skipped_pairs)/len(distances):.1f}%)")
    print(f"  Min distance: {np.min(distances):.6f} m")
    print(f"  Max distance: {np.max(distances):.6f} m")
    print(f"  Mean distance: {np.mean(distances):.6f} m")
    print(f"  Median distance: {np.median(distances):.6f} m")
    print(f"  Std deviation: {np.std(distances):.6f} m")
    
    # Phân tích phân bố
    print(f"\n[ANALYSIS] Distance Distribution:")
    bins = [0, 0.001, 0.005, 0.01, 0.05, 0.1, 0.5, np.inf]
    hist, _ = np.histogram(distances, bins=bins)
    for i in range(len(bins)-1):
        if bins[i+1] == np.inf:
            label = f">= {bins[i]:.3f}m"
        else:
            label = f"{bins[i]:.3f}-{bins[i+1]:.3f}m"
        print(f"  {label:15s}: {hist[i]:4d} pairs ({100*hist[i]/len(distances):5.1f}%)")
    
    # Kiểm tra các đoạn liên tiếp bị skip
    print(f"\n[ANALYSIS] Consecutive Skipped Pairs:")
    if skipped_pairs:
        consecutive_skips = []
        current_skip_start = skipped_pairs[0]
        current_skip_count = 1
        
        for i in range(1, len(skipped_pairs)):
            if skipped_pairs[i] == skipped_pairs[i-1] + 1:
                current_skip_count += 1
            else:
                if current_skip_count > 1:
                    consecutive_skips.append((current_skip_start, current_skip_count))
                current_skip_start = skipped_pairs[i]
                current_skip_count = 1
        
        if current_skip_count > 1:
            consecutive_skips.append((current_skip_start, current_skip_count))
        
        if consecutive_skips:
            print(f"  Found {len(consecutive_skips)} regions with consecutive skips:")
            for start, count in sorted(consecutive_skips, key=lambda x: x[1], reverse=True)[:10]:
                print(f"    Poses {start}-{start+count}: {count} consecutive pairs skipped")
        else:
            print("  No consecutive skipped pairs (good!)")
    else:
        print("  No skipped pairs")
    
    # Tính direction vectors
    try:
        directions = compute_direction_vectors(poses, min_distance=min_distance)
        print(f"\n[ANALYSIS] Direction Vectors:")
        print(f"  Total valid vectors: {len(directions)}")
        
        # Tính vector trung bình
        mean_dir = np.mean(directions, axis=0)
        mean_dir = mean_dir / np.linalg.norm(mean_dir)
        print(f"  Mean direction: [{mean_dir[0]:.6f}, {mean_dir[1]:.6f}, {mean_dir[2]:.6f}]")
        
        # Tính độ lệch góc
        angles = []
        for direction in directions:
            dot_product = np.clip(np.dot(direction, mean_dir), -1.0, 1.0)
            angle = np.arccos(dot_product)
            angles.append(np.rad2deg(angle))
        
        angles = np.array(angles)
        print(f"  Angle deviation from mean:")
        print(f"    Mean: {np.mean(angles):.2f}°")
        print(f"    Median: {np.median(angles):.2f}°")
        print(f"    Max: {np.max(angles):.2f}°")
        print(f"    Std: {np.std(angles):.2f}°")
        
        # Phân tích phân bố vectors trong trajectory
        print(f"\n[ANALYSIS] Vector Distribution in Trajectory:")
        # Chia trajectory thành 10 đoạn
        n_segments = 10
        segment_size = len(valid_pairs) // n_segments
        vectors_per_segment = []
        
        for seg in range(n_segments):
            start_idx = seg * segment_size
            end_idx = (seg + 1) * segment_size if seg < n_segments - 1 else len(valid_pairs)
            segment_vectors = len([i for i in valid_pairs if start_idx <= i < end_idx])
            vectors_per_segment.append(segment_vectors)
            print(f"  Segment {seg+1:2d} (poses {start_idx:4d}-{end_idx:4d}): {segment_vectors:4d} vectors")
        
        # Kiểm tra xem có đoạn nào thiếu vectors không
        min_vectors = min(vectors_per_segment)
        max_vectors = max(vectors_per_segment)
        if max_vectors > 0:
            ratio = min_vectors / max_vectors
            print(f"\n  Distribution uniformity: {ratio:.2%} (1.0 = perfect)")
            if ratio < 0.5:
                print(f"  [WARNING] Uneven distribution! Some segments have very few vectors.")
            elif ratio < 0.7:
                print(f"  [CAUTION] Somewhat uneven distribution.")
            else:
                print(f"  [OK] Distribution is relatively uniform.")
        
        # Tính tổng quãng đường
        total_distance = np.sum([distances[i] for i in valid_pairs])
        print(f"\n[ANALYSIS] Total Distance:")
        print(f"  Total distance covered: {total_distance:.2f} m")
        print(f"  Average distance per valid vector: {total_distance/len(valid_pairs):.4f} m")
        
        # Đánh giá độ tin cậy
        print(f"\n[RELIABILITY ASSESSMENT]")
        reliability_score = 1.0
        
        # 1. Tỷ lệ valid pairs
        valid_ratio = len(valid_pairs) / len(distances)
        if valid_ratio < 0.3:
            print(f"  [ISSUE] Only {valid_ratio:.1%} pairs are valid (too many skips)")
            reliability_score *= 0.5
        elif valid_ratio < 0.5:
            print(f"  [CAUTION] {valid_ratio:.1%} pairs are valid (many skips)")
            reliability_score *= 0.7
        else:
            print(f"  [OK] {valid_ratio:.1%} pairs are valid")
        
        # 2. Số lượng vectors
        if len(directions) < 50:
            print(f"  [ISSUE] Only {len(directions)} vectors (too few for reliable result)")
            reliability_score *= 0.5
        elif len(directions) < 200:
            print(f"  [CAUTION] {len(directions)} vectors (moderate sample size)")
            reliability_score *= 0.8
        else:
            print(f"  [OK] {len(directions)} vectors (good sample size)")
        
        # 3. Độ nhất quán của hướng
        mean_angle_dev = np.mean(angles)
        if mean_angle_dev > 30:
            print(f"  [ISSUE] High angle deviation: {mean_angle_dev:.1f}° (inconsistent directions)")
            reliability_score *= 0.5
        elif mean_angle_dev > 15:
            print(f"  [CAUTION] Moderate angle deviation: {mean_angle_dev:.1f}°")
            reliability_score *= 0.8
        else:
            print(f"  [OK] Low angle deviation: {mean_angle_dev:.1f}° (consistent directions)")
        
        # 4. Phân bố đều
        if ratio < 0.5:
            print(f"  [ISSUE] Uneven distribution across trajectory")
            reliability_score *= 0.6
        elif ratio < 0.7:
            print(f"  [CAUTION] Somewhat uneven distribution")
            reliability_score *= 0.8
        else:
            print(f"  [OK] Even distribution across trajectory")
        
        print(f"\n  Overall Reliability Score: {reliability_score:.2%}")
        if reliability_score >= 0.8:
            print(f"  [VERDICT] Result is RELIABLE ✓")
        elif reliability_score >= 0.6:
            print(f"  [VERDICT] Result is MODERATELY RELIABLE ⚠")
        else:
            print(f"  [VERDICT] Result may be UNRELIABLE ✗")
        
    except Exception as e:
        print(f"\n[ERROR] Failed to compute direction vectors: {e}")
        return
    
    print(f"\n{'='*70}\n")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Analyze trajectory quality")
    parser.add_argument(
        '--traj_path',
        type=str,
        required=True,
        help='Path to trajectory file'
    )
    parser.add_argument(
        '--min_distance',
        type=float,
        default=0.01,
        help='Minimum distance threshold (m)'
    )
    
    args = parser.parse_args()
    
    script_dir = Path(__file__).parent.resolve()
    project_root = script_dir.parent
    
    traj_path = Path(args.traj_path)
    if not traj_path.is_absolute():
        traj_path = project_root / traj_path
    
    analyze_trajectory_quality(traj_path, args.min_distance)
