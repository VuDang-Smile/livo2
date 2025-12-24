#!/usr/bin/env python3
"""
Merge individual PCD scans into aggregated downsampled map
Usage: python3 merge_pcd_maps.py <map_directory>
Example: python3 merge_pcd_maps.py ../Log/map_20251223_164801
"""

import sys
import os
import glob
from pathlib import Path

try:
    import open3d as o3d
    import numpy as np
except ImportError:
    print("ERROR: open3d not installed")
    print("Install with: pip3 install open3d")
    sys.exit(1)


def merge_pcd_scans(map_dir, voxel_size=0.12, output_dir=None):
    """
    Merge all individual PCD scans into aggregated downsampled map
    
    Args:
        map_dir: Path to map_<timestamp> directory
        voxel_size: Voxel size for downsampling (default: 0.12m)
        output_dir: Output directory (default: <map_dir>/../PCD/)
    """
    map_path = Path(map_dir).resolve()
    pcd_dir = map_path / "pcd"
    
    if not pcd_dir.exists():
        print(f"ERROR: PCD directory not found: {pcd_dir}")
        return False
    
    # Get all PCD files sorted by number
    pcd_files = sorted(pcd_dir.glob("*.pcd"), key=lambda x: int(x.stem) if x.stem.isdigit() else 0)
    
    if not pcd_files:
        print(f"ERROR: No PCD files found in {pcd_dir}")
        return False
    
    print(f"Found {len(pcd_files)} PCD files")
    print(f"Voxel size: {voxel_size}m")
    print(f"Loading and merging...")
    
    # Merge all point clouds
    combined_pcd = o3d.geometry.PointCloud()
    loaded_count = 0
    
    for i, pcd_file in enumerate(pcd_files, 1):
        try:
            pcd = o3d.io.read_point_cloud(str(pcd_file))
            if len(pcd.points) > 0:
                combined_pcd += pcd
                loaded_count += 1
            
            # Progress indicator
            if i % 50 == 0 or i == len(pcd_files):
                print(f"  Loaded {i}/{len(pcd_files)} scans, {len(combined_pcd.points):,} points so far...")
        
        except Exception as e:
            print(f"  Warning: Failed to load {pcd_file.name}: {e}")
            continue
    
    if len(combined_pcd.points) == 0:
        print("ERROR: No points loaded from PCD files")
        return False
    
    print(f"\nTotal points before downsampling: {len(combined_pcd.points):,}")
    
    # Determine output directory
    if output_dir is None:
        output_dir = map_path.parent / "PCD"
    else:
        output_dir = Path(output_dir)
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Save raw map (no downsampling) - PRIORITY
    raw_output_file = output_dir / "all_raw_points.pcd"
    print(f"\nSaving raw map (no downsampling) to: {raw_output_file}")
    o3d.io.write_point_cloud(str(raw_output_file), combined_pcd, write_ascii=False)
    raw_file_size_mb = raw_output_file.stat().st_size / (1024 * 1024)
    print(f"✓ Raw map saved: {len(combined_pcd.points):,} points, {raw_file_size_mb:.2f} MB")
    
    # Voxel downsampling
    print(f"\nDownsampling with voxel size {voxel_size}m...")
    downsampled_pcd = combined_pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f"Points after downsampling: {len(downsampled_pcd.points):,}")
    
    # Save downsampled map
    output_file = output_dir / "all_downsampled_points.pcd"
    print(f"\nSaving downsampled map to: {output_file}")
    o3d.io.write_point_cloud(str(output_file), downsampled_pcd, write_ascii=False)
    
    # Get file size
    file_size_mb = output_file.stat().st_size / (1024 * 1024)
    
    print(f"\n{'='*60}")
    print(f"✓ SUCCESS!")
    print(f"{'='*60}")
    print(f"Input:  {loaded_count} scans from {map_path.name}/pcd/")
    print(f"Output files:")
    print(f"  ├── {raw_output_file.name}")
    print(f"  │   Points: {len(combined_pcd.points):,}")
    print(f"  │   Size:   {raw_file_size_mb:.2f} MB")
    print(f"  └── {output_file.name}")
    print(f"      Points: {len(downsampled_pcd.points):,}")
    print(f"      Size:   {file_size_mb:.2f} MB")
    print(f"{'='*60}")
    
    return True


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 merge_pcd_maps.py <map_directory> [voxel_size] [output_dir]")
        print("\nExample:")
        print("  python3 merge_pcd_maps.py ../Log/map_20251223_164801")
        print("  python3 merge_pcd_maps.py ../Log/map_20251223_164801 0.15")
        print("  python3 merge_pcd_maps.py ../Log/map_20251223_164801 0.12 /tmp/output")
        sys.exit(1)
    
    map_dir = sys.argv[1]
    voxel_size = float(sys.argv[2]) if len(sys.argv) > 2 else 0.12
    output_dir = sys.argv[3] if len(sys.argv) > 3 else None
    
    success = merge_pcd_scans(map_dir, voxel_size, output_dir)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

