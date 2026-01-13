#!/usr/bin/env python3
"""
Script để downscale PCD file bằng voxel downsampling
"""

import sys
from pathlib import Path

try:
    import open3d as o3d
except ImportError as e:
    raise SystemExit(
        "Thiếu thư viện 'open3d'. Hãy cài trước khi chạy:\n"
        "    pip install open3d\n"
    ) from e


def downscale_pcd(input_path: Path, output_path: Path = None, voxel_size: float = 0.1):
    """
    Downscale PCD file bằng voxel downsampling
    
    Args:
        input_path: Đường dẫn tới file PCD input
        output_path: Đường dẫn tới file PCD output (nếu None, sẽ tạo tên tự động)
        voxel_size: Kích thước voxel để downsample (m, mặc định: 0.1)
    """
    input_path = Path(input_path).expanduser().resolve()
    
    if not input_path.exists():
        raise FileNotFoundError(f"Không tìm thấy file PCD: {input_path}")
    
    if output_path is None:
        # Tạo tên file output tự động: merged_all_downscaled.pcd
        output_path = input_path.parent / f"{input_path.stem}_downscaled{input_path.suffix}"
    else:
        output_path = Path(output_path).expanduser().resolve()
    
    print(f"Đang đọc file PCD: {input_path}")
    print(f"Kích thước file: {input_path.stat().st_size / (1024*1024):.2f} MB")
    
    # Load PCD
    pcd = o3d.io.read_point_cloud(str(input_path))
    
    if pcd.is_empty():
        raise RuntimeError(f"PCD rỗng: {input_path}")
    
    original_points = len(pcd.points)
    print(f"Số điểm ban đầu: {original_points:,}")
    
    # Downsample
    print(f"Đang downscale với voxel_size = {voxel_size}m...")
    pcd_downscaled = pcd.voxel_down_sample(voxel_size=voxel_size)
    
    downscaled_points = len(pcd_downscaled.points)
    reduction_ratio = (1 - downscaled_points / original_points) * 100
    
    print(f"Số điểm sau downscale: {downscaled_points:,}")
    print(f"Giảm: {reduction_ratio:.1f}%")
    
    # Save
    print(f"Đang lưu file: {output_path}")
    o3d.io.write_point_cloud(str(output_path), pcd_downscaled)
    
    output_size = output_path.stat().st_size / (1024*1024)
    print(f"Kích thước file sau downscale: {output_size:.2f} MB")
    print(f"Hoàn thành! File đã được lưu tại: {output_path}")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Downscale PCD file bằng voxel downsampling",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ví dụ:
  # Downscale với voxel_size mặc định (0.1m)
  python downscale_pcd.py input.pcd
  
  # Downscale với voxel_size tùy chỉnh
  python downscale_pcd.py input.pcd --voxel-size 0.05
  
  # Chỉ định file output
  python downscale_pcd.py input.pcd -o output.pcd
        """
    )
    
    parser.add_argument(
        "input_pcd",
        type=str,
        help="Đường dẫn tới file PCD cần downscale"
    )
    
    parser.add_argument(
        "-o", "--output",
        type=str,
        default=None,
        help="Đường dẫn tới file PCD output (mặc định: input_downscaled.pcd)"
    )
    
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.1,
        help="Kích thước voxel để downsample (m, mặc định: 0.1)"
    )
    
    args = parser.parse_args()
    
    try:
        downscale_pcd(
            input_path=args.input_pcd,
            output_path=args.output,
            voxel_size=args.voxel_size
        )
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
