#!/usr/bin/env python3
"""
PCD Comparison Tool - Standalone Script
So sánh 2 file PCD hoặc OBJ vs PCD để phát hiện drift và tính tỷ lệ tương đồng
"""

import argparse
import json
import sys
from pathlib import Path
from datetime import datetime
from typing import Optional

# Import utilities
sys.path.insert(0, str(Path(__file__).parent))
from pcd_comparison_utils import (
    load_pcd,
    load_obj_as_pointcloud,
    preprocess_pcd,
    compute_similarity_metrics,
    calculate_similarity_percentage,
    detect_drift,
    DEFAULT_VOXEL_SIZE,
    DEFAULT_ICP_MAX_ITERATIONS,
    DEFAULT_DRIFT_THRESHOLDS,
    DEFAULT_MAX_DISTANCES,
    DEFAULT_ICP_THRESHOLD_MULTIPLIER
)


class ComparisonConfig:
    """Configuration class cho comparison"""
    
    def __init__(self, args):
        self.reference_file = Path(args.reference_file).expanduser().resolve()
        self.test_file = Path(args.test_pcd).expanduser().resolve()
        self.reference_type = self._detect_reference_type(args)
        self.voxel_size = args.voxel_size
        self.remove_outliers = not args.no_remove_outliers
        self.use_icp = not args.no_icp
        self.icp_threshold = args.icp_threshold or (self.voxel_size * DEFAULT_ICP_THRESHOLD_MULTIPLIER)
        self.icp_max_iterations = args.icp_max_iterations
        
        # OBJ settings
        self.obj_sampling = args.obj_sampling
        self.obj_points = args.obj_points
        self.match_density = args.match_density
        self.save_obj_pcd_path = self._determine_save_path(args)
        
        # Drift detection thresholds
        self.drift_thresholds = {
            "hausdorff": args.hausdorff_threshold,
            "chamfer": args.chamfer_threshold,
            "centroid": args.centroid_threshold,
            "similarity": args.similarity_threshold
        }
        
        # Normalization
        self.max_distances = {
            "hausdorff": args.max_hausdorff,
            "chamfer": args.max_chamfer,
            "centroid": args.max_centroid
        }
        
        # Output
        self.output_file = self._determine_output_file(args)
    
    def _detect_reference_type(self, args) -> str:
        """Auto-detect reference type từ extension"""
        reference_type = args.reference_type.lower()
        if reference_type == "pcd" and self.reference_file.suffix.lower() == ".obj":
            reference_type = "obj"
            print(f"INFO: Auto-detected OBJ file, using --reference-type obj")
        return reference_type
    
    def _determine_save_path(self, args) -> Optional[Path]:
        """Determine save path cho converted OBJ PCD"""
        if args.save_obj_pcd:
            return Path(args.save_obj_pcd).expanduser().resolve()
        elif self.reference_type == "obj":
            # Auto-generate
            return self.reference_file.parent / f"{self.reference_file.stem}_converted.pcd"
        return None
    
    def _determine_output_file(self, args) -> Path:
        """Determine output JSON file path"""
        if args.output:
            return Path(args.output).expanduser().resolve()
        return self.test_file.parent / f"{self.test_file.stem}_comparison.json"
    
    def validate(self):
        """Validate configuration"""
        if not self.reference_file.exists():
            raise FileNotFoundError(f"Reference file không tồn tại: {self.reference_file}")
        
        if not self.test_file.exists():
            raise FileNotFoundError(f"Test PCD file không tồn tại: {self.test_file}")
        
        if self.reference_type == "obj" and self.reference_file.suffix.lower() != ".obj":
            print(f"WARNING: Reference file không phải .obj: {self.reference_file}", file=sys.stderr)
        
        if self.test_file.suffix.lower() != ".pcd":
            print(f"WARNING: Test file không phải .pcd: {self.test_file}", file=sys.stderr)


def print_results(
    config: ComparisonConfig,
    metrics: dict,
    similarity: float,
    drift_detected: bool,
    drift_reasons: list
):
    """In kết quả ra console"""
    ref_type_label = "OBJ" if config.reference_type == "obj" else "PCD"
    
    print("=" * 50)
    print("PCD Comparison Results")
    print("=" * 50)
    print(f"Reference {ref_type_label}: {config.reference_file}")
    print(f"Test PCD: {config.test_file}")
    print()
    
    print("Point Cloud Info:")
    print(f"  Reference: {metrics['reference_points']:,} points")
    print(f"  Test: {metrics['test_points']:,} points")
    print()
    
    print("Metrics:")
    print(f"  ICP Fitness Score: {metrics['icp_fitness']:.4f}")
    print(f"  ICP Inlier RMSE: {metrics['icp_inlier_rmse']:.6f} m")
    print(f"  Hausdorff Distance: {metrics['hausdorff_distance']:.6f} m")
    print(f"  Chamfer Distance: {metrics['chamfer_distance']:.6f} m")
    print(f"  Bounding Box Overlap: {metrics['bbox_overlap']:.4f}")
    print(f"  Centroid Distance: {metrics['centroid_distance']:.6f} m")
    print(f"  Point Density Ratio: {metrics['density_ratio']:.4f}")
    print()
    
    print(f"Similarity: {similarity:.2f}%")
    print(f"Drift Detected: {'Yes' if drift_detected else 'No'}")
    
    if drift_detected and drift_reasons:
        print()
        print("Drift Reasons:")
        for reason in drift_reasons:
            print(f"  - {reason}")
    
    print("=" * 50)


def save_json_results(
    config: ComparisonConfig,
    metrics: dict,
    similarity: float,
    drift_detected: bool,
    drift_reasons: list
):
    """Lưu kết quả ra file JSON"""
    result = {
        "timestamp": datetime.now().isoformat(),
        "reference_file": str(config.reference_file),
        "test_file": str(config.test_file),
        "reference_type": config.reference_type,
        "reference_points": metrics["reference_points"],
        "test_points": metrics["test_points"],
        "metrics": {
            "icp_fitness": metrics["icp_fitness"],
            "icp_inlier_rmse": metrics["icp_inlier_rmse"],
            "hausdorff_distance": metrics["hausdorff_distance"],
            "chamfer_distance": metrics["chamfer_distance"],
            "bbox_overlap": metrics["bbox_overlap"],
            "centroid_distance": metrics["centroid_distance"],
            "density_ratio": metrics["density_ratio"]
        },
        "similarity_percentage": similarity,
        "drift_detected": drift_detected,
        "drift_reasons": drift_reasons
    }
    
    with open(config.output_file, 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    
    print(f"\nResults saved to: {config.output_file}")


def load_reference_file(config: ComparisonConfig):
    """Load reference file (PCD or OBJ)"""
    if config.reference_type == "obj":
        print(f"\nConverting OBJ to point cloud...")
        print(f"  Sampling method: {config.obj_sampling}")
        
        # Determine number of points
        num_points = config.obj_points
        match_density = None
        
        if config.match_density:
            # Load test PCD first to get point count
            pcd_test_temp = load_pcd(config.test_file)
            match_density = len(pcd_test_temp.points)
            print(f"  Matching PCD density: {match_density:,} points")
        
        pcd_ref = load_obj_as_pointcloud(
            config.reference_file,
            sampling_method=config.obj_sampling,
            num_points=num_points,
            match_density=match_density
        )
        print(f"  Converted OBJ: {len(pcd_ref.points):,} points")
        return pcd_ref
    else:
        return load_pcd(config.reference_file)


def create_argument_parser() -> argparse.ArgumentParser:
    """Tạo argument parser với tất cả options"""
    parser = argparse.ArgumentParser(
        description="So sánh 2 file PCD hoặc OBJ vs PCD để phát hiện drift và tính tỷ lệ tương đồng",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # So sánh 2 PCD files cơ bản
  python compare_pcd.py reference.pcd test.pcd

  # So sánh OBJ vs PCD
  python compare_pcd.py design.obj map.pcd --reference-type obj

  # So sánh với preprocessing tùy chỉnh
  python compare_pcd.py ref.pcd test.pcd --voxel-size 0.1 --no-remove-outliers

  # So sánh và lưu kết quả JSON
  python compare_pcd.py ref.pcd test.pcd -o results.json

  # Tùy chỉnh thresholds cho drift detection
  python compare_pcd.py ref.pcd test.pcd --hausdorff-threshold 1.0 --similarity-threshold 70
        """
    )
    
    # Required arguments
    parser.add_argument(
        "reference_file",
        type=str,
        help="Đường dẫn tới file chuẩn (reference) - PCD hoặc OBJ"
    )
    
    parser.add_argument(
        "test_pcd",
        type=str,
        help="Đường dẫn tới file PCD cần kiểm tra (test)"
    )
    
    # Output
    parser.add_argument(
        "-o", "--output",
        type=str,
        default=None,
        help="Đường dẫn file JSON để lưu kết quả (mặc định: auto-generate)"
    )
    
    # File type
    parser.add_argument(
        "--reference-type",
        type=str,
        choices=["pcd", "obj"],
        default="pcd",
        help="Loại file reference: 'pcd' hoặc 'obj' (mặc định: pcd, auto-detect từ extension)"
    )
    
    # Preprocessing options
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=DEFAULT_VOXEL_SIZE,
        help=f"Kích thước voxel để downsample (m, mặc định: {DEFAULT_VOXEL_SIZE}, khuyến nghị: 0.15-0.5)"
    )
    
    parser.add_argument(
        "--no-remove-outliers",
        action="store_true",
        help="Không remove outliers trong preprocessing"
    )
    
    # ICP options
    parser.add_argument(
        "--no-icp",
        action="store_true",
        help="Không thực hiện ICP alignment (chỉ so sánh trực tiếp)"
    )
    
    parser.add_argument(
        "--icp-threshold",
        type=float,
        default=None,
        help=f"Threshold cho ICP alignment (m, mặc định: auto = voxel_size * {DEFAULT_ICP_THRESHOLD_MULTIPLIER})"
    )
    
    parser.add_argument(
        "--icp-max-iterations",
        type=int,
        default=DEFAULT_ICP_MAX_ITERATIONS,
        help=f"Số iteration tối đa cho ICP (mặc định: {DEFAULT_ICP_MAX_ITERATIONS})"
    )
    
    # OBJ-specific options
    parser.add_argument(
        "--obj-sampling",
        type=str,
        choices=["uniform", "poisson", "vertex"],
        default="uniform",
        help="Method sampling cho OBJ: 'uniform', 'poisson', hoặc 'vertex' (mặc định: uniform)"
    )
    
    parser.add_argument(
        "--obj-points",
        type=int,
        default=None,
        help="Số điểm để sample từ OBJ (optional, auto nếu không chỉ định)"
    )
    
    parser.add_argument(
        "--match-density",
        action="store_true",
        help="Match số điểm OBJ với số điểm của PCD test"
    )
    
    parser.add_argument(
        "--save-obj-pcd",
        type=str,
        default=None,
        help="Lưu PCD đã convert từ OBJ ra file (optional)"
    )
    
    # Drift detection thresholds
    parser.add_argument(
        "--hausdorff-threshold",
        type=float,
        default=DEFAULT_DRIFT_THRESHOLDS["hausdorff"],
        help=f"Threshold cho Hausdorff distance để phát hiện drift (m, mặc định: {DEFAULT_DRIFT_THRESHOLDS['hausdorff']})"
    )
    
    parser.add_argument(
        "--chamfer-threshold",
        type=float,
        default=DEFAULT_DRIFT_THRESHOLDS["chamfer"],
        help=f"Threshold cho Chamfer distance để phát hiện drift (m, mặc định: {DEFAULT_DRIFT_THRESHOLDS['chamfer']})"
    )
    
    parser.add_argument(
        "--centroid-threshold",
        type=float,
        default=DEFAULT_DRIFT_THRESHOLDS["centroid"],
        help=f"Threshold cho Centroid distance để phát hiện drift (m, mặc định: {DEFAULT_DRIFT_THRESHOLDS['centroid']})"
    )
    
    parser.add_argument(
        "--similarity-threshold",
        type=float,
        default=DEFAULT_DRIFT_THRESHOLDS["similarity"],
        help=f"Threshold cho Similarity percentage để phát hiện drift (%%, mặc định: {DEFAULT_DRIFT_THRESHOLDS['similarity']})"
    )
    
    # Normalization parameters
    parser.add_argument(
        "--max-hausdorff",
        type=float,
        default=DEFAULT_MAX_DISTANCES["hausdorff"],
        help=f"Max Hausdorff distance để normalize (m, mặc định: {DEFAULT_MAX_DISTANCES['hausdorff']})"
    )
    
    parser.add_argument(
        "--max-chamfer",
        type=float,
        default=DEFAULT_MAX_DISTANCES["chamfer"],
        help=f"Max Chamfer distance để normalize (m, mặc định: {DEFAULT_MAX_DISTANCES['chamfer']})"
    )
    
    parser.add_argument(
        "--max-centroid",
        type=float,
        default=DEFAULT_MAX_DISTANCES["centroid"],
        help=f"Max Centroid distance để normalize (m, mặc định: {DEFAULT_MAX_DISTANCES['centroid']})"
    )
    
    return parser


def run_comparison(config: ComparisonConfig):
    """Chạy comparison process"""
    print("Loading files...")
    print(f"  Reference ({config.reference_type.upper()}): {config.reference_file}")
    print(f"  Test (PCD): {config.test_file}")
    
    # Load reference file (PCD or OBJ)
    pcd_ref = load_reference_file(config)
    
    # Load test PCD
    pcd_test = load_pcd(config.test_file)
    
    print(f"  Reference: {len(pcd_ref.points):,} points")
    print(f"  Test: {len(pcd_test.points):,} points")
    
    # Preprocess
    print("\nPreprocessing point clouds...")
    print(f"  Voxel size: {config.voxel_size} m")
    print(f"  Remove outliers: {config.remove_outliers}")
    
    pcd_ref = preprocess_pcd(
        pcd_ref,
        voxel_size=config.voxel_size,
        remove_outliers=config.remove_outliers
    )
    
    pcd_test = preprocess_pcd(
        pcd_test,
        voxel_size=config.voxel_size,
        remove_outliers=config.remove_outliers
    )
    
    print(f"  After preprocessing - Reference: {len(pcd_ref.points):,} points")
    print(f"  After preprocessing - Test: {len(pcd_test.points):,} points")
    
    # Compute metrics
    print("\nComputing similarity metrics...")
    print(f"  ICP alignment: {config.use_icp}")
    if config.save_obj_pcd_path and config.reference_type == "obj":
        print(f"  Saving converted PCD to: {config.save_obj_pcd_path}")
    
    metrics = compute_similarity_metrics(
        pcd_ref,
        pcd_test,
        perform_icp=config.use_icp,
        icp_threshold=config.icp_threshold,
        icp_max_iterations=config.icp_max_iterations,
        save_obj_pcd=config.save_obj_pcd_path if config.reference_type == "obj" else None
    )
    
    if config.save_obj_pcd_path and config.reference_type == "obj":
        print(f"\n✅ Converted PCD saved to: {config.save_obj_pcd_path}")
    
    # Calculate similarity percentage
    similarity = calculate_similarity_percentage(
        metrics,
        max_hausdorff=config.max_distances["hausdorff"],
        max_chamfer=config.max_distances["chamfer"],
        max_centroid=config.max_distances["centroid"]
    )
    
    # Detect drift
    drift_detected, drift_reasons = detect_drift(
        metrics,
        similarity,
        hausdorff_threshold=config.drift_thresholds["hausdorff"],
        chamfer_threshold=config.drift_thresholds["chamfer"],
        centroid_threshold=config.drift_thresholds["centroid"],
        similarity_threshold=config.drift_thresholds["similarity"]
    )
    
    # Print results
    print()
    print_results(config, metrics, similarity, drift_detected, drift_reasons)
    
    # Save JSON
    save_json_results(config, metrics, similarity, drift_detected, drift_reasons)
    
    return drift_detected


def main():
    """Main function"""
    parser = create_argument_parser()
    args = parser.parse_args()
    
    try:
        # Create and validate config
        config = ComparisonConfig(args)
        config.validate()
        
        # Run comparison
        drift_detected = run_comparison(config)
        
        # Exit code based on drift detection
        sys.exit(1 if drift_detected else 0)
        
    except (FileNotFoundError, RuntimeError, ValueError) as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
