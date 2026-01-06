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
        "Thiếu thư viện 'open3d'. Hãy cài trước khi chạy:\n"
        "    pip install open3d\n"
    ) from e


def load_pcd(path: Path) -> o3d.geometry.PointCloud:
    if not path.exists():
        raise FileNotFoundError(f"Không tìm thấy file PCD đầu vào: {path}")
    print(f"[INFO] Đang đọc PCD: {path}")
    pcd = o3d.io.read_point_cloud(str(path))
    if pcd.is_empty():
        raise RuntimeError(f"PCD rỗng: {path}")
    print(f"[INFO]  - Số điểm: {np.asarray(pcd.points).shape[0]}")
    return pcd


def voxel_downsample(pcd: o3d.geometry.PointCloud, voxel_size: float) -> o3d.geometry.PointCloud:
    if voxel_size <= 0:
        print("[INFO] Bỏ qua downsample (voxel_size <= 0)")
        return pcd
    print(f"[INFO] Đang downsample với voxel_size = {voxel_size} m")
    ds = pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f"[INFO]  - Sau downsample: {np.asarray(ds.points).shape[0]} điểm")
    return ds


def strip_colors(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    """
    Loại bỏ màu trong dữ liệu sử dụng: chỉ giữ toạ độ XYZ.
    Lưu ý: open3d vẫn có thể ghi PCD với field màu trống; FAST-LOCALIZATION
    thường chỉ quan tâm XYZ/X, Y, Z (và có thể intensity).
    """
    if not pcd.has_colors():
        print("[INFO] PCD không có màu, bỏ qua bước strip_colors")
        return pcd

    print("[INFO] Đang bỏ màu (RGB) khỏi point cloud, chỉ giữ XYZ")
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
    Cơ chế VIRTUAL SCANCONTEXT (Giống smile-lidar-recorder):
    Thay vì cắt theo tile vuông (grid), ta sẽ lấy các 'Virtual Scans' tại các điểm node.
    Mỗi node sẽ lấy các điểm xung quanh nó trong bán kính (tile_size/2).
    """
    if tile_size <= 0:
        raise ValueError("tile_size phải > 0")

    pts = np.asarray(pcd.points)
    x_min, x_max, y_min, y_max = compute_xy_bounds(pcd)
    print(f"[INFO] Bounds XY tổng: x[{x_min:.2f}, {x_max:.2f}], y[{y_min:.2f}, {y_max:.2f}]")

    # Khoảng cách giữa các điểm quét ảo (spacing). 
    # Mặc định lấy tile_size/2 để đảm bảo độ phủ (overlap 50%)
    spacing = tile_size / 2.0 
    
    nx = max(1, int(math.ceil((x_max - x_min) / spacing)))
    ny = max(1, int(math.ceil((y_max - y_min) / spacing)))

    print(f"[INFO] Tạo Virtual ScanContext: Lưới {nx}x{ny}, khoảng cách {spacing:.2f} m")

    tiles: List[o3d.geometry.PointCloud] = []
    metas: List[Dict] = []

    # Dùng KDTree để trích xuất điểm xung quanh nhanh hơn
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    radius = tile_size / 2.0

    tile_index = 0
    for ix in range(nx):
        for iy in range(ny):
            cx = x_min + ix * spacing
            cy = y_min + iy * spacing
            
            # Tìm cao độ trung bình tại vị trí này để đặt tâm scan ảo
            # (Đơn giản nhất là lấy trung bình Z của toàn map hoặc 0)
            cz = (pts[:, 2].min() + pts[:, 2].max()) / 2.0 

            # Trích xuất các điểm trong bán kính radius quanh tâm (cx, cy, cz)
            [k, idx, _] = pcd_tree.search_radius_vector_3d([cx, cy, cz], radius)
            
            if k < min_points_per_tile:
                continue

            tile_pts = pts[idx]
            
            # Quan trọng: Dịch chuyển cụm điểm về tâm (0,0,0) của virtual scan
            # Vì ScanContext descriptor được tính toán dựa trên sensor-centric coordinates
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
                "num_points": int(k)
            }

            tiles.append(tile_pcd)
            metas.append(meta)
            tile_index += 1

    print(f"[INFO] Đã tạo {len(tiles)} Virtual Scans.")
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

    print(f"[INFO] Lưu tile vào thư mục: {tiles_dir}")
    for idx, (tile_pcd, meta) in enumerate(zip(tiles, metas)):
        # FAST_LIO_LOCALIZATION2 dùng số thứ tự làm tên file
        fname = f"{idx}.pcd"
        fpath = tiles_dir / fname
        ok = o3d.io.write_point_cloud(str(fpath), tile_pcd, write_ascii=False, compressed=False)
        if not ok:
            raise RuntimeError(f"Lỗi khi ghi tile: {fpath}")

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

    print(f"[INFO] Đã ghi index tile: {index_path}")
    print(f"[INFO] Đã ghi pose.json (định dạng FAST_LIO_LOCALIZATION2): {pose_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sinh map tile PCD kiểu ScanContext cho FAST-LOCALIZATION từ 1 file PCD lớn (đã qua HBA)."
    )
    parser.add_argument(
        "--input_pcd",
        type=str,
        required=True,
        help="Đường dẫn tới file PCD đầu vào (ví dụ merge_all_hba.pcd).",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        required=True,
        help="Thư mục output chứa tiles + index.json (ví dụ livo2/maps/fast_localization_map).",
    )
    parser.add_argument(
        "--base_name",
        type=str,
        default="fastloc_map",
        help="Prefix tên file tile (mặc định: fastloc_map).",
    )
    parser.add_argument(
        "--voxel_size",
        type=float,
        default=0.0,
        help="Voxel size (m) để downsample trước khi chia tile. 0 = bỏ qua.",
    )
    parser.add_argument(
        "--tile_size",
        type=float,
        default=80.0,
        help="Kích thước tile theo XY (m), mặc định 80m.",
    )
    parser.add_argument(
        "--overlap",
        type=float,
        default=5.0,
        help="Overlap giữa các tile theo XY (m), mặc định 5m.",
    )
    parser.add_argument(
        "--min_points_per_tile",
        type=int,
        default=2000,
        help="Số điểm tối thiểu để chấp nhận 1 tile (mặc định 2000).",
    )
    parser.add_argument(
        "--strip_color",
        action="store_true",
        help="Nếu đặt cờ này, script sẽ bỏ màu RGB (chỉ giữ XYZ).",
    )
    parser.add_argument(
        "--input_pose",
        type=str,
        help="Đường dẫn tới file scans_pos.json từ FAST-LIVO2 (nếu có) để trích xuất pose cho từng tile.",
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
        print("[ERROR] Không tạo được tile nào, dừng.")
        return

    save_tiles(tiles, metas, output_dir, base_name=args.base_name)

    print("[DONE] Đã sinh map tile cho FAST-LOCALIZATION.")
    print("       Hãy trỏ cấu hình FAST-LOCALIZATION tới thư mục output_dir ở trên.")


if __name__ == "__main__":
    main()


