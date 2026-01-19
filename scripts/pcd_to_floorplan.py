"""
Utility to generate 2 floorplan views (top, side_x) and rich metadata
from a PCD point cloud.

Used by GUI through function `pcd_to_2_views` in `bag_mapping_tab.py`.

Output:
- 2 PNG files: <stem>_top.png, <stem>_side_x.png
- 1 JSON metadata file: <stem>_metadata.json (format matches frontend MapMetadata)
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, Any, Tuple

import numpy as np

try:
    import open3d as o3d
except ImportError as e:  # pragma: no cover - sẽ được log ở GUI
    raise ImportError(
        "Module 'open3d' is not installed. "
        "Please run: pip install open3d"
    ) from e

try:
    from PIL import Image
except ImportError as e:  # pragma: no cover
    raise ImportError(
        "Module 'Pillow' is not installed. "
        "Please run: pip install pillow"
    ) from e


def _compute_bounds(points: np.ndarray) -> Tuple[float, float, float, float, float, float]:
    """Tính bounding box (min/max cho X, Y, Z)."""
    x_min, y_min, z_min = points.min(axis=0)
    x_max, y_max, z_max = points.max(axis=0)
    return float(x_min), float(x_max), float(y_min), float(y_max), float(z_min), float(z_max)


def _apply_outlier_filter(points: np.ndarray, percentile: float) -> Tuple[np.ndarray, Dict[str, float]]:
    """
    Lọc outlier đơn giản theo percentile cho từng trục.
    Giữ lại các điểm nằm trong [p, 100-p] cho mỗi trục.
    """
    if percentile <= 0.0 or percentile >= 50.0:
        return points, {}

    p_low = percentile
    p_high = 100.0 - percentile

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    x_min_o, x_max_o = np.percentile(x, [p_low, p_high])
    y_min_o, y_max_o = np.percentile(y, [p_low, p_high])
    z_min_o, z_max_o = np.percentile(z, [p_low, p_high])

    mask = (
        (x >= x_min_o) & (x <= x_max_o) &
        (y >= y_min_o) & (y <= y_max_o) &
        (z >= z_min_o) & (z <= z_max_o)
    )

    filtered = points[mask]
    original_bounds = {
        "x_min": float(points[:, 0].min()),
        "x_max": float(points[:, 0].max()),
        "y_min": float(points[:, 1].min()),
        "y_max": float(points[:, 1].max()),
    }
    return filtered, original_bounds


def _make_occupancy_image(
    u: np.ndarray,
    v: np.ndarray,
    resolution: float,
    border_margin: int,
    auto_crop: bool,
    crop_margin: int,
    invert_colors: bool,
) -> Dict[str, Any]:
    """
    Tạo ảnh occupancy 2D từ tọa độ (u, v) với:
    - resolution (m/pixel)
    - border_margin (viền trắng xung quanh)
    """
    u_min = float(u.min())
    u_max = float(u.max())
    v_min = float(v.min())
    v_max = float(v.max())

    # Nếu auto_crop thì dùng min/max đã lọc; crop_margin (m) sẽ nới rộng thêm một chút
    if auto_crop:
        u_min_c = u_min - crop_margin
        u_max_c = u_max + crop_margin
        v_min_c = v_min - crop_margin
        v_max_c = v_max + crop_margin
    else:
        u_min_c, u_max_c, v_min_c, v_max_c = u_min, u_max, v_min, v_max

    # Kích thước vùng content (không tính border), tính theo pixel
    width_m = max(u_max_c - u_min_c, 1e-3)
    height_m = max(v_max_c - v_min_c, 1e-3)

    content_width = int(np.ceil(width_m / resolution))
    content_height = int(np.ceil(height_m / resolution))

    # Lưới occupancy (0 = nền, 1 = có điểm)
    grid_mask = np.zeros((content_height, content_width), dtype=np.uint8)

    # Chuyển (u, v) → chỉ số grid
    u_norm = (u - u_min_c) / resolution
    v_norm = (v - v_min_c) / resolution

    iu = np.clip(u_norm.astype(int), 0, content_width - 1)
    iv = np.clip(v_norm.astype(int), 0, content_height - 1)

    # Lưu ý: ảnh gốc coi (0,0) ở góc dưới trái ⇒ ta cần flip theo trục v
    # Ở đây grid[y, x] với y=0 là hàng trên cùng, nên dùng (content_height - 1 - iv)
    grid_mask[content_height - 1 - iv, iu] = 255

    # Tạo ảnh RGB với màu xanh biển cho các điểm có dữ liệu
    # Màu xanh biển: RGB(30, 144, 255) - Dodger Blue
    blue_color = np.array([30, 144, 255], dtype=np.uint8)
    
    # Tạo grid RGB với nền trắng
    grid_rgb = np.ones((content_height, content_width, 3), dtype=np.uint8) * 255
    # Áp dụng màu xanh biển cho các điểm có dữ liệu
    mask = grid_mask > 0
    grid_rgb[mask] = blue_color

    # Thêm border_margin
    h_with_border = content_height + 2 * border_margin
    w_with_border = content_width + 2 * border_margin
    canvas = np.zeros((h_with_border, w_with_border, 3), dtype=np.uint8)

    # Luôn dùng nền trắng
    canvas[:, :] = [255, 255, 255]  # Nền trắng

    # Dán content vào giữa
    canvas[
        border_margin:border_margin + content_height,
        border_margin:border_margin + content_width,
    ] = grid_rgb

    image = Image.fromarray(canvas, mode="RGB")  # RGB với màu xanh biển

    processing = {
        "border_margin": int(border_margin),
        "content_area": {
            "x": 0,
            "y": 0,
            "width": int(content_width),
            "height": int(content_height),
        },
        "auto_crop": bool(auto_crop),
        "crop_applied": bool(auto_crop),
        "outlier_filter": None,          # sẽ set ở ngoài nếu cần
        "outlier_percentile": None,      # sẽ set ở ngoài nếu cần
        "colormap": "binary",
        "inverted": bool(invert_colors),
        "original_bounds": None,         # sẽ set ở ngoài nếu cần
    }

    bounds_world = {
        "u_min": float(u_min_c),
        "u_max": float(u_max_c),
        "v_min": float(v_min_c),
        "v_max": float(v_max_c),
    }

    return {
        "image": image,
        "image_width": int(w_with_border),
        "image_height": int(h_with_border),
        "processing": processing,
        "bounds_world": bounds_world,
    }


def pcd_to_2_views(
    input_pcd: str,
    output_dir: str,
    resolution: float = 0.05,
    colormap: str = "binary",
    invert_colors: bool = True,
    auto_crop: bool = True,
    crop_margin: int = 5,
    border_margin: int = 20,
    outlier_filter: bool = True,
    outlier_percentile: float = 1.0,
) -> Dict[str, Any]:
    """
    Hàm chính được GUI gọi để sinh 2 ảnh (top, side_x) + metadata.

    Parameters khớp với cách gọi từ GUI:
    - input_pcd: đường dẫn file PCD
    - output_dir: thư mục output
    - resolution: kích thước 1 pixel (m)
    - colormap, invert_colors, auto_crop, crop_margin, border_margin
    - outlier_filter, outlier_percentile
    """
    input_pcd_path = Path(input_pcd)
    output_dir_path = Path(output_dir)
    output_dir_path.mkdir(parents=True, exist_ok=True)

    if not input_pcd_path.exists():
        raise FileNotFoundError(f"PCD file does not exist: {input_pcd_path}")

    cloud = o3d.io.read_point_cloud(str(input_pcd_path))
    if cloud.is_empty():
        raise ValueError(f"PCD file is empty: {input_pcd_path}")

    pts = np.asarray(cloud.points, dtype=np.float32)
    if pts.shape[1] != 3:
        raise ValueError("PCD is not a valid 3D point cloud (needs Nx3)")

    # Bounds trước khi lọc
    x_min, x_max, y_min, y_max, z_min, z_max = _compute_bounds(pts)

    original_bounds_xy = {
        "x_min": x_min,
        "x_max": x_max,
        "y_min": y_min,
        "y_max": y_max,
    }

    # Lọc outlier nếu bật
    if outlier_filter:
        pts_filtered, original_bounds_for_meta = _apply_outlier_filter(pts, outlier_percentile)
    else:
        pts_filtered = pts
        original_bounds_for_meta = original_bounds_xy

    if pts_filtered.shape[0] == 0:
        # Nếu lọc xong trống, dùng lại toàn bộ điểm
        pts_filtered = pts

    x = pts_filtered[:, 0]
    y = pts_filtered[:, 1]
    z = pts_filtered[:, 2]

    # ===== View TOP (XY, trục chiếu: Z) =====
    top_data = _make_occupancy_image(
        u=x,
        v=y,
        resolution=resolution,
        border_margin=border_margin,
        auto_crop=auto_crop,
        crop_margin=crop_margin,
        invert_colors=invert_colors,
    )

    # ===== View SIDE_X (XZ, trục chiếu: Y) – đổi sang chiếu theo trục Y cho đúng side_y =====
    side_x_data = _make_occupancy_image(
        u=x,
        v=z,
        resolution=resolution,
        border_margin=border_margin,
        auto_crop=auto_crop,
        crop_margin=crop_margin,
        invert_colors=invert_colors,
    )

    stem = input_pcd_path.stem
    top_png = output_dir_path / f"{stem}_top.png"
    side_x_png = output_dir_path / f"{stem}_side_x.png"
    meta_json = output_dir_path / f"{stem}_metadata.json"

    # Ghi ảnh
    top_data["image"].save(top_png)
    side_x_data["image"].save(side_x_png)

    # Điều chỉnh processing cho metadata (thêm info filter, original_bounds)
    for view_data in (top_data, side_x_data):
        view_data["processing"]["outlier_filter"] = bool(outlier_filter)
        view_data["processing"]["outlier_percentile"] = float(outlier_percentile) if outlier_filter else None
        view_data["processing"]["colormap"] = colormap if colormap in ("binary", "density", "height") else "binary"
        view_data["processing"]["original_bounds"] = original_bounds_for_meta

    # Xây dựng MapMetadata cho frontend/backend
    # Lưu ý: các field này được thiết kế để tương thích với
    # frontend/src/types/mapMetadata.ts và coordinateTransform.ts
    metadata: Dict[str, Any] = {
        "input_file": str(input_pcd_path.name),
        "resolution_m_per_pixel": float(resolution),
        # Hệ trục mặc định của dữ liệu PCD/floorplan là ROS (X forward, Y left, Z up)
        "coordinate_system": {
            "convention": "ros"
        },
        "views": {
            "top": {
                "id": "top",
                "projection": {
                    "axis": "Z",
                    "plane": "XY",
                    "world_axes": {
                        "horizontal": "X",
                        "vertical": "Y",
                    },
                    "image_axes": {
                        "x_direction": "+X",
                        "y_direction": "-Y",
                    },
                },
                "bounds": {
                    "world": {
                        "X": {
                            "min": float(top_data["bounds_world"]["u_min"]),
                            "max": float(top_data["bounds_world"]["u_max"]),
                        },
                        "Y": {
                            "min": float(top_data["bounds_world"]["v_min"]),
                            "max": float(top_data["bounds_world"]["v_max"]),
                        },
                    },
                },
                "image": {
                    "width": int(top_data["image_width"]),
                    "height": int(top_data["image_height"]),
                },
                "processing": top_data["processing"],
                "orientation": {
                    "source_frame": "base_link",
                    "forward_vector_in_body": [1.0, 0.0, 0.0],
                    "plane": "XY",
                    "world_u_axis": "X",
                    "world_v_axis": "Y",
                    "image_mapping": {
                        "u_to_image_x": 1.0,
                        "v_to_image_y": 1.0,
                    },
                    "angle_unit": "radian",
                },
            },
            "side_x": {
                "id": "side_x",
                "projection": {
                    "axis": "Y",
                    "plane": "XZ",
                    "world_axes": {
                        "horizontal": "X",
                        "vertical": "Z",
                    },
                    "image_axes": {
                        "x_direction": "+X",
                        "y_direction": "-Z",
                    },
                },
                "bounds": {
                    "world": {
                        "X": {
                            "min": float(side_x_data["bounds_world"]["u_min"]),
                            "max": float(side_x_data["bounds_world"]["u_max"]),
                        },
                        "Z": {
                            "min": float(side_x_data["bounds_world"]["v_min"]),
                            "max": float(side_x_data["bounds_world"]["v_max"]),
                        },
                    },
                },
                "image": {
                    "width": int(side_x_data["image_width"]),
                    "height": int(side_x_data["image_height"]),
                },
                "processing": side_x_data["processing"],
                "orientation": {
                    "source_frame": "base_link",
                    "forward_vector_in_body": [1.0, 0.0, 0.0],
                    "plane": "XZ",
                    "world_u_axis": "X",
                    "world_v_axis": "Z",
                    "image_mapping": {
                        "u_to_image_x": 1.0,
                        "v_to_image_y": 1.0,
                    },
                    "angle_unit": "radian",
                },
            },
        },
    }

    # Ghi metadata JSON
    with meta_json.open("w", encoding="utf-8") as f:
        json.dump(metadata, f, ensure_ascii=False, indent=2)

    return {
        "metadata_path": str(meta_json),
        "top_png": str(top_png),
        "side_x_png": str(side_x_png),
    }


__all__ = ["pcd_to_2_views"]


