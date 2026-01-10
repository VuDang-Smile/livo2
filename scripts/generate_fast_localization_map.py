#!/usr/bin/env python3
"""
Sinh bộ tile bản đồ PCD kiểu ScanContext để dùng cho FAST-LOCALIZATION.

Pipeline mong muốn:
    1) FAST-LIVO2 mapping -> xuất PCD tổng (vd: all_raw_points.pcd hoặc merge_all.pcd)
    2) Chạy HBA ngoài (https://github.com/hku-mars/HBA) -> PCD đã chuẩn hoá (merge_all_hba.pcd)
    3) Chạy script này trên PCD đã chuẩn hoá để:
         - (tuỳ chọn) downsample
         - chia map thành các tile theo lưới XY
         - lưu từng tile .pcd + file index.json mô tả các tile

Lưu ý:
    - Script này cố gắng KHÔNG phụ thuộc nhiều vào cấu trúc nội bộ của FAST-LOCALIZATION,
      chỉ chuẩn bị dữ liệu map dạng PCD tiles + index, để sau đó bạn cấu hình vào repo FAST-LOCALIZATION.
    - Script dùng open3d. Cần cài:
          pip install open3d
"""

import argparse
import json
import math
import os
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

try:
    import open3d as o3d
except ImportError as e:
    raise SystemExit(
        "Missing library 'open3d'. Please install before running:\n"
        "    pip install open3d\n"
    ) from e


def load_pcd(path: Path) -> o3d.geometry.PointCloud:
    if not path.exists():
        raise FileNotFoundError(f"Input PCD file not found: {path}")
    print(f"[INFO] Reading PCD: {path}")
    pcd = o3d.io.read_point_cloud(str(path))
    if pcd.is_empty():
        raise RuntimeError(f"PCD is empty: {path}")
    print(f"[INFO]  - Number of points: {np.asarray(pcd.points).shape[0]}")
    return pcd


def voxel_downsample(pcd: o3d.geometry.PointCloud, voxel_size: float) -> o3d.geometry.PointCloud:
    if voxel_size <= 0:
        print("[INFO] Skipping downsample (voxel_size <= 0)")
        return pcd
    print(f"[INFO] Downsampling with voxel_size = {voxel_size} m")
    ds = pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f"[INFO]  - After downsample: {np.asarray(ds.points).shape[0]} points")
    return ds


def strip_colors(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    """
    Remove color data: keep only XYZ coordinates.
    Note: open3d can still write PCD with empty color fields; FAST-LOCALIZATION
    typically only cares about XYZ (and possibly intensity).
    """
    if not pcd.has_colors():
        print("[INFO] PCD has no colors, skipping strip_colors step")
        return pcd

    print("[INFO] Removing color (RGB) from point cloud, keeping only XYZ")
    pts = np.asarray(pcd.points)
    out = o3d.geometry.PointCloud()
    out.points = o3d.utility.Vector3dVector(pts)
    return out


def compute_xy_bounds(pcd: o3d.geometry.PointCloud) -> Tuple[float, float, float, float]:
    pts = np.asarray(pcd.points)
    x_min = float(pts[:, 0].min())
    x_max = float(pts[:, 0].max())
    y_min = float(pts[:, 1].min())
    y_max = float(pts[:, 1].max())
    return x_min, x_max, y_min, y_max


def generate_tiles(
    pcd: o3d.geometry.PointCloud,
    tile_size: float,
    overlap: float,
    min_points_per_tile: int,
) -> Tuple[List[o3d.geometry.PointCloud], List[Dict]]:
    """
    Grid Partitioning mechanism - Matches @smile-lidar-recorder:
    Divide map into square tiles WITHOUT OVERLAPPING.
    Each tile will be saved in Local coordinate system (center at 0,0,0).
    Global position of tile center will be saved in pose.json.
    """
    if tile_size <= 0:
        raise ValueError("tile_size must be > 0")

    pts = np.asarray(pcd.points)
    x_min, x_max, y_min, y_max = compute_xy_bounds(pcd)
    
    # Grid steps - don't use overlap to avoid map overlap
    nx = int(math.ceil((x_max - x_min) / tile_size))
    ny = int(math.ceil((y_max - y_min) / tile_size))

    print(f"[INFO] Dividing map grid: {nx}x{ny} cells, size {tile_size:.2f}m")

    tiles: List[o3d.geometry.PointCloud] = []
    metas: List[Dict] = []

    tile_index = 0
    for ix in range(nx):
        x0 = x_min + ix * tile_size
        x1 = x0 + tile_size
        
        # Mask X
        mask_x = (pts[:, 0] >= x0) & (pts[:, 0] < x1)
        if not np.any(mask_x): continue
        
        pts_x = pts[mask_x]
        
        for iy in range(ny):
            y0 = y_min + iy * tile_size
            y1 = y0 + tile_size
            
            # Mask Y trên tập điểm đã lọc X
            mask_y = (pts_x[:, 1] >= y0) & (pts_x[:, 1] < y1)
            tile_pts = pts_x[mask_y]
            
            if len(tile_pts) < min_points_per_tile:
                continue

            # Tính tâm của tile (centroid)
            cx = (x0 + x1) / 2.0
            cy = (y0 + y1) / 2.0
            cz = float(tile_pts[:, 2].mean())

            # CONVERT TO LOCAL COORDINATES (Important!)
            # When loading, C++ node will use pose.json to translate this position back.
            tile_pts_local = tile_pts - np.array([cx, cy, cz])

            tile_pcd = o3d.geometry.PointCloud()
            tile_pcd.points = o3d.utility.Vector3dVector(tile_pts_local)

            meta = {
                "id": tile_index,
                "centroid": {
                    "x": float(cx),
                    "y": float(cy),
                    "z": float(cz),
                },
                "num_points": len(tile_pts)
            }

            tiles.append(tile_pcd)
            metas.append(meta)
            tile_index += 1

    print(f"[INFO] Divided into {len(tiles)} non-overlapping tiles.")
    return tiles, metas


def save_tiles(
    tiles: List[o3d.geometry.PointCloud],
    metas: List[Dict],
    output_dir: Path,
    base_name: str,
) -> None:

    tiles_dir = output_dir / "pcd"
    tiles_dir.mkdir(parents=True, exist_ok=True)

    index_data = {
        "base_name": base_name,
        "num_tiles": len(tiles),
        "tiles": [],
    }

    pose_data = []

    print(f"[INFO] Saving tiles to directory: {tiles_dir}")
    for idx, (tile_pcd, meta) in enumerate(zip(tiles, metas)):
        # FAST_LIO_LOCALIZATION2 dùng số thứ tự làm tên file
        fname = f"{idx}.pcd"
        fpath = tiles_dir / fname
        ok = o3d.io.write_point_cloud(str(fpath), tile_pcd, write_ascii=False, compressed=False)
        if not ok:
            raise RuntimeError(f"Error writing tile: {fpath}")

        meta_with_file = dict(meta)
        meta_with_file["file"] = str(fpath.relative_to(output_dir))
        index_data["tiles"].append(meta_with_file)

        # Chuẩn bị pose cho pose.json
        c = meta["centroid"]
        # Format: tx ty tz qw qx qy qz
        pose_data.append(f"{c['x']:.6f} {c['y']:.6f} {c['z']:.6f} 1.000000 0.000000 0.000000 0.000000")

    # Ghi index.json (cho script này quản lý)
    index_path = output_dir / "index.json"
    with index_path.open("w", encoding="utf-8") as f:
        json.dump(index_data, f, ensure_ascii=False, indent=2)

    # Ghi pose.json (cho FAST_LIO_LOCALIZATION2)
    pose_path = output_dir / "pose.json"
    with pose_path.open("w", encoding="utf-8") as f:
        f.write("\n".join(pose_data) + "\n")

    print(f"[INFO] Wrote tile index: {index_path}")
    print(f"[INFO] Wrote pose.json (FAST_LIO_LOCALIZATION2 format): {pose_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate ScanContext-style PCD map tiles for FAST-LOCALIZATION from a large PCD file (after HBA)."
    )
    parser.add_argument(
        "--input_pcd",
        type=str,
        required=True,
        help="Path to input PCD file (e.g., merge_all_hba.pcd).",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        required=True,
        help="Output directory containing tiles + index.json (e.g., livo2/maps/fast_localization_map).",
    )
    parser.add_argument(
        "--base_name",
        type=str,
        default="fastloc_map",
        help="Prefix for tile file names (default: fastloc_map).",
    )
    parser.add_argument(
        "--voxel_size",
        type=float,
        default=0.0,
        help="Voxel size (m) for downsampling before tiling. 0 = skip.",
    )
    parser.add_argument(
        "--tile_size",
        type=float,
        default=80.0,
        help="Tile size in XY (m), default 80m.",
    )
    parser.add_argument(
        "--overlap",
        type=float,
        default=5.0,
        help="Overlap between tiles in XY (m), default 5m.",
    )
    parser.add_argument(
        "--min_points_per_tile",
        type=int,
        default=2000,
        help="Minimum number of points to accept a tile (default 2000).",
    )
    parser.add_argument(
        "--strip_color",
        action="store_true",
        help="If set, script will remove RGB color (keep only XYZ).",
    )
    parser.add_argument(
        "--input_pose",
        type=str,
        help="Path to scans_pos.json file from FAST-LIVO2 (if available) to extract pose for each tile.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    input_pcd = Path(args.input_pcd).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    print("==============================================")
    print("  FAST-LOCALIZATION Map Tile Generator")
    print("==============================================")
    print(f"[CONFIG] input_pcd          = {input_pcd}")
    print(f"[CONFIG] output_dir         = {output_dir}")
    print(f"[CONFIG] base_name          = {args.base_name}")
    print(f"[CONFIG] voxel_size         = {args.voxel_size}")
    print(f"[CONFIG] tile_size          = {args.tile_size}")
    print(f"[CONFIG] overlap            = {args.overlap}")
    print(f"[CONFIG] min_points_per_tile= {args.min_points_per_tile}")
    print(f"[CONFIG] strip_color        = {args.strip_color}")
    print("==============================================")

    pcd = load_pcd(input_pcd)

    if args.strip_color:
        pcd = strip_colors(pcd)

    pcd = voxel_downsample(pcd, args.voxel_size)

    tiles, metas = generate_tiles(
        pcd=pcd,
        tile_size=args.tile_size,
        overlap=args.overlap,
        min_points_per_tile=args.min_points_per_tile,
    )

    if len(tiles) == 0:
        print("[ERROR] No tiles created, stopping.")
        return

    save_tiles(tiles, metas, output_dir, base_name=args.base_name)

    print("[DONE] Map tiles generated for FAST-LOCALIZATION.")
    print("       Please point FAST-LOCALIZATION config to the output_dir above.")


if __name__ == "__main__":
    main()


