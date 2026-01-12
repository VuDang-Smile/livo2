#!/usr/bin/env python3
"""
Normalize map using HBA and merge into merge_all_hba.pcd.
"""

import argparse
import os
import shutil
import subprocess
from pathlib import Path
import numpy as np
import open3d as o3d

def load_pose(pose_path):
    poses = []
    with open(pose_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line: continue
            vals = list(map(float, line.split()))
            if len(vals) < 7: continue
            tx, ty, tz, qw, qx, qy, qz = vals[:7]
            poses.append({
                't': np.array([tx, ty, tz]),
                'q': np.array([qw, qx, qy, qz])
            })
    return poses

def get_transformation_matrix(t, q):
    # q is [w, x, y, z]
    r = o3d.geometry.get_rotation_matrix_from_quaternion(q)
    T = np.eye(4)
    T[:3, :3] = r
    T[:3, 3] = t
    return T

def find_hba_binary(project_root):
    """Find hba_standalone binary in project_root"""
    print(f"[INFO] Searching for hba_standalone binary in {project_root}...")
    # Thử tìm trong các vị trí phổ biến
    search_paths = [
        project_root / "dependencies" / "HBA" / "build_standalone" / "hba_standalone",
        project_root / "dependencies" / "HBA" / "hba_standalone",
        project_root / "build" / "hba_standalone",
    ]
    
    for path in search_paths:
        if path.exists():
            return path
            
    # If not found, use find command
    try:
        result = subprocess.run(
            ["find", str(project_root), "-name", "hba_standalone", "-type", "f", "-not", "-path", "*/.*"],
            capture_output=True, text=True
        )
        if result.returncode == 0 and result.stdout.strip():
            # Lấy kết quả đầu tiên
            return Path(result.stdout.strip().split('\n')[0])
    except Exception:
        pass
        
    return None

def main():
    # Determine default HBA binary path relative to this script
    script_dir = Path(__file__).parent.resolve()
    project_root = script_dir.parent
    
    hba_bin_found = find_hba_binary(project_root)
    default_hba_bin = hba_bin_found if hba_bin_found else (project_root / "dependencies" / "HBA" / "build_standalone" / "hba_standalone")

    parser = argparse.ArgumentParser(description="Normalize map using HBA.")
    parser.add_argument("--input_dir", type=str, required=True, help="FAST-LIVO2 Log/PCD directory.")
    parser.add_argument("--hba_binary", type=str, default=str(default_hba_bin), help="Path to hba_standalone binary.")
    parser.add_argument("--layers", type=int, default=None, help="Number of layers for HBA (auto if not specified).")
    parser.add_argument("--threads", type=int, default=None, help="Number of threads for HBA (auto if not specified).")
    parser.add_argument("--output_pcd", type=str, default="merge_all_hba.pcd", help="Output merged PCD file.")
    parser.add_argument("--voxel_size", type=float, default=0.0, help="Voxel size for downsampling after merge (0 = disabled).")
    parser.add_argument("--adaptive_layers", action="store_true", help="Adaptively choose layers based on number of poses.")
    args = parser.parse_args()

    input_dir = Path(args.input_dir).resolve()
    hba_bin = Path(args.hba_binary).resolve()
    
    if not input_dir.exists():
        print(f"[ERROR] Input directory {input_dir} not found.")
        return

    # 1. Prepare directory structure for HBA
    # HBA expects data_path/pose.json and data_path/pcd/*.pcd
    
    # Check if we have enough data for HBA (WIN_SIZE is usually 10)
    livo_pose = input_dir / "scans_pos.json"
    if not livo_pose.exists():
        print(f"[ERROR] scans_pos.json not found in {input_dir}")
        return
    
    poses = load_pose(livo_pose)
    num_poses = len(poses)
    min_hba_poses = 15 # HBA usually needs at least one full window + some extra
    
    if num_poses < min_hba_poses:
        print(f"[INFO] Map too short ({num_poses} poses). Skipping HBA optimization.")
        merged_all = input_dir / "merged_all.pcd"
        output_path = input_dir / args.output_pcd
        
        if merged_all.exists():
            print(f"[INFO] Copying {merged_all.name} to {args.output_pcd} as fallback.")
            shutil.copy(merged_all, output_path)
            print(f"[DONE] Fallback map saved to {output_path}")
        else:
            print(f"[ERROR] Could not find {merged_all}. Please run 'Merge PCD' first.")
        return

    # Adaptive layers based on number of poses
    if args.layers is None or args.adaptive_layers:
        if num_poses < 50:
            layers = 2
        elif num_poses < 200:
            layers = 3
        elif num_poses < 500:
            layers = 4
        else:
            layers = 5
        if args.layers is None:
            args.layers = layers
            print(f"[INFO] Auto-selected {layers} layers based on {num_poses} poses")
    
    # Adaptive threads based on CPU cores
    if args.threads is None:
        try:
            import os
            cpu_count = os.cpu_count() or 8
            # Use 75% of CPU cores, minimum 4, maximum 32
            args.threads = max(4, min(32, int(cpu_count * 0.75)))
            print(f"[INFO] Auto-selected {args.threads} threads based on {cpu_count} CPU cores")
        except:
            args.threads = 16
            print(f"[INFO] Using default {args.threads} threads")

    hba_work_dir = input_dir / "hba_work"
    hba_work_dir.mkdir(parents=True, exist_ok=True)
    
    # Create pcd subdirectory for HBA
    pcd_subdir = hba_work_dir / "pcd"
    pcd_subdir.mkdir(parents=True, exist_ok=True)

    # Link/Copy PCDs
    print("[INFO] Linking PCD files...")
    pcd_files = sorted(list(input_dir.glob("*.pcd")))
    # Filter only numbered pcds like 0.pcd, 1.pcd...
    linked_count = 0
    for p in pcd_files:
        if p.stem.isdigit():
            # Use symlink to save space
            target = pcd_subdir / p.name
            if target.exists(): 
                target.unlink()
            try:
                os.symlink(p, target)
                linked_count += 1
            except Exception as e:
                # Fallback to copy if symlink fails
                shutil.copy(p, target)
                linked_count += 1
    
    print(f"[INFO] Linked/copied {linked_count} PCD files to {pcd_subdir}")
    
    # Copy pose.json to hba_work_dir
    pose_json = hba_work_dir / "pose.json"
    shutil.copy(livo_pose, pose_json)
    print(f"[INFO] Copied pose.json to {pose_json}")

    # 2. Run HBA
    print(f"[INFO] Running HBA: {hba_bin} {hba_work_dir} {args.layers} {args.threads}")
    try:
        subprocess.run([str(hba_bin), str(hba_work_dir), str(args.layers), str(args.threads)], check=True)
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] HBA failed: {e}")
        return

    # 3. Read optimized poses and merge PCDs
    print("[INFO] Merging optimized PCDs...")
    opt_pose_path = hba_work_dir / "pose.json"
    if not opt_pose_path.exists():
        print(f"[ERROR] Optimized pose.json not found at {opt_pose_path}")
        return
    
    poses = load_pose(opt_pose_path)
    print(f"[INFO] Loaded {len(poses)} optimized poses")
    
    merged_cloud = o3d.geometry.PointCloud()
    total_points_before = 0
    
    for i, pose in enumerate(poses):
        pcd_path = pcd_subdir / f"{i}.pcd"
        if not pcd_path.exists():
            print(f"[WARNING] PCD {pcd_path} missing, skipping.")
            continue
        
        try:
            pcd = o3d.io.read_point_cloud(str(pcd_path))
            if pcd.is_empty():
                print(f"[WARNING] PCD {pcd_path} is empty, skipping.")
                continue
            
            total_points_before += len(pcd.points)
            T = get_transformation_matrix(pose['t'], pose['q'])
            pcd.transform(T)
            merged_cloud += pcd
            
            if (i + 1) % 10 == 0 or (i + 1) == len(poses):
                print(f"  - Merged {i + 1}/{len(poses)} scans ({len(merged_cloud.points):,} points)")
        except Exception as e:
            print(f"[WARNING] Error processing PCD {pcd_path}: {e}")
            continue

    if merged_cloud.is_empty():
        print(f"[ERROR] Merged cloud is empty!")
        return
    
    print(f"[INFO] Total points before downsampling: {len(merged_cloud.points):,}")
    
    # Optional downsampling to reduce noise and improve quality
    if args.voxel_size > 0:
        print(f"[INFO] Downsampling merged cloud with voxel_size={args.voxel_size}m...")
        merged_cloud = merged_cloud.voxel_down_sample(voxel_size=args.voxel_size)
        print(f"[INFO] Total points after downsampling: {len(merged_cloud.points):,}")
    
    output_path = input_dir / args.output_pcd
    o3d.io.write_point_cloud(str(output_path), merged_cloud)
    
    # Calculate file size
    file_size_mb = output_path.stat().st_size / (1024 * 1024)
    print(f"[DONE] Optimized map saved to {output_path}")
    print(f"[INFO] File size: {file_size_mb:.2f} MB")
    print(f"[INFO] Total points: {len(merged_cloud.points):,}")

if __name__ == "__main__":
    main()

