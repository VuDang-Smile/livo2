#!/usr/bin/env python3
"""
PCD Comparison Utilities
Các hàm utility để so sánh 2 point cloud và tính toán metrics
"""

import numpy as np
import copy
from typing import Dict, List, Tuple, Optional
from pathlib import Path

try:
    import open3d as o3d
except ImportError as e:
    raise SystemExit(
        "Thiếu thư viện 'open3d'. Hãy cài trước khi chạy:\n"
        "    pip install open3d\n"
    ) from e

# Constants
DEFAULT_VOXEL_SIZE = 0.2
DEFAULT_ICP_THRESHOLD_MULTIPLIER = 2.0
DEFAULT_ICP_MAX_ITERATIONS = 2000
DEFAULT_ICP_THRESHOLD = 0.02
MIN_POINTS_FOR_OUTLIER_REMOVAL = 20
DEFAULT_STD_RATIO = 2.0
MIN_AUTO_POINTS = 10000

# Similarity calculation weights
SIMILARITY_WEIGHTS = {
    "icp_fitness": 0.4,
    "hausdorff": 0.2,
    "chamfer": 0.2,
    "bbox_overlap": 0.1,
    "centroid": 0.1
}

# Default drift detection thresholds
DEFAULT_DRIFT_THRESHOLDS = {
    "hausdorff": 0.5,
    "chamfer": 0.3,
    "centroid": 1.0,
    "similarity": 80.0
}

# Default normalization max distances
DEFAULT_MAX_DISTANCES = {
    "hausdorff": 10.0,
    "chamfer": 10.0,
    "centroid": 10.0
}


def load_pcd(path: Path) -> o3d.geometry.PointCloud:
    """
    Load PCD file và trả về Open3D point cloud
    
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
    
    pcd = o3d.io.read_point_cloud(str(path))
    if pcd.is_empty():
        raise RuntimeError(f"PCD rỗng: {path}")
    
    return pcd


def load_obj_as_pointcloud(
    path: Path,
    sampling_method: str = "uniform",
    num_points: Optional[int] = None,
    match_density: Optional[int] = None
) -> o3d.geometry.PointCloud:
    """
    Load OBJ file và convert sang point cloud bằng sampling
    
    Args:
        path: Đường dẫn tới file OBJ
        sampling_method: Method sampling ("uniform", "poisson", "vertex")
        num_points: Số điểm để sample (None = auto)
        match_density: Match số điểm với PCD này (optional)
    
    Returns:
        Point cloud từ OBJ mesh
    
    Raises:
        FileNotFoundError: Nếu file không tồn tại
        RuntimeError: Nếu OBJ rỗng hoặc conversion thất bại
        ValueError: Nếu sampling method không hợp lệ
    """
    if not path.exists():
        raise FileNotFoundError(f"Không tìm thấy file OBJ: {path}")
    
    # Load OBJ mesh
    mesh = o3d.io.read_triangle_mesh(str(path))
    if len(mesh.vertices) == 0:
        raise RuntimeError(f"OBJ file rỗng hoặc không có vertices: {path}")
    
    # Determine number of points
    if match_density is not None:
        num_points = match_density
    elif num_points is None:
        # Auto: use number of vertices or estimate from mesh
        num_points = max(len(mesh.vertices), MIN_AUTO_POINTS)
    
    # Convert mesh to point cloud based on sampling method
    pcd = _sample_mesh_to_pointcloud(mesh, sampling_method, num_points)
    
    if pcd.is_empty():
        raise RuntimeError(f"Failed to convert OBJ to point cloud: {path}")
    
    return pcd


def _sample_mesh_to_pointcloud(
    mesh: o3d.geometry.TriangleMesh,
    sampling_method: str,
    num_points: int
) -> o3d.geometry.PointCloud:
    """
    Sample mesh thành point cloud theo method được chọn
    
    Args:
        mesh: Triangle mesh
        sampling_method: "uniform", "poisson", hoặc "vertex"
        num_points: Số điểm để sample
    
    Returns:
        Point cloud từ mesh
    """
    if sampling_method == "vertex":
        # Use vertices directly (fastest)
        pcd = o3d.geometry.PointCloud()
        pcd.points = mesh.vertices
        if mesh.has_vertex_normals():
            pcd.normals = mesh.vertex_normals
        if mesh.has_vertex_colors():
            pcd.colors = mesh.vertex_colors
    
    elif sampling_method == "uniform":
        # Uniform sampling on surface
        pcd = mesh.sample_points_uniformly(number_of_points=num_points)
    
    elif sampling_method == "poisson":
        # Poisson disk sampling
        pcd = mesh.sample_points_poisson_disk(number_of_points=num_points)
    
    else:
        raise ValueError(
            f"Unknown sampling method: {sampling_method}. "
            f"Use 'uniform', 'poisson', or 'vertex'"
        )
    
    return pcd


def save_pcd(pcd: o3d.geometry.PointCloud, output_path: Path, binary: bool = True) -> bool:
    """
    Lưu point cloud ra file PCD
    
    Args:
        pcd: Point cloud to save
        output_path: Đường dẫn file output
        binary: Lưu dạng binary (True) hay ASCII (False)
    
    Returns:
        True nếu thành công
    
    Raises:
        RuntimeError: Nếu không thể lưu file
    """
    try:
        # Create parent directory if needed
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        # Save point cloud
        success = o3d.io.write_point_cloud(
            str(output_path),
            pcd,
            write_ascii=not binary,
            compressed=False
        )
        
        if not success:
            raise RuntimeError(f"Failed to write PCD file: {output_path}")
        
        return True
    except Exception as e:
        raise RuntimeError(f"Error saving PCD to {output_path}: {e}") from e


def preprocess_pcd(
    pcd: o3d.geometry.PointCloud,
    voxel_size: float = DEFAULT_VOXEL_SIZE,
    remove_outliers: bool = True,
    nb_neighbors: int = MIN_POINTS_FOR_OUTLIER_REMOVAL,
    std_ratio: float = DEFAULT_STD_RATIO
) -> o3d.geometry.PointCloud:
    """
    Preprocess point cloud: downsample và remove outliers
    
    Args:
        pcd: Input point cloud
        voxel_size: Kích thước voxel để downsample (m)
        remove_outliers: Có remove outliers không
        nb_neighbors: Số neighbors để statistical outlier removal
        std_ratio: Standard deviation ratio threshold
    
    Returns:
        Processed point cloud
    """
    # Downsample
    if voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    
    # Remove outliers
    if remove_outliers and len(pcd.points) > nb_neighbors:
        pcd, _ = pcd.remove_statistical_outlier(
            nb_neighbors=nb_neighbors,
            std_ratio=std_ratio
        )
    
    return pcd


def compute_icp_alignment(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    threshold: float = DEFAULT_ICP_THRESHOLD,
    max_iterations: int = DEFAULT_ICP_MAX_ITERATIONS,
    use_initial_guess: bool = True
) -> Tuple[o3d.geometry.PointCloud, Dict]:
    """
    Thực hiện ICP alignment giữa source và target point cloud
    
    Args:
        source: Source point cloud
        target: Target point cloud
        threshold: Distance threshold
        max_iterations: Maximum iterations
        use_initial_guess: Use centroid-based initial guess
    
    Returns:
        (aligned_source, registration_result)
    """
    # Initial alignment - estimate translation from centroid difference
    trans_init = _get_initial_transformation(source, target) if use_initial_guess else np.identity(4)
    
    # Point-to-point ICP
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=max_iterations
        )
    )
    
    # Apply transformation (create a copy using deepcopy)
    source_aligned = copy.deepcopy(source)
    source_aligned.transform(reg_p2p.transformation)
    
    result = {
        "fitness": reg_p2p.fitness,
        "inlier_rmse": reg_p2p.inlier_rmse,
        "transformation": reg_p2p.transformation,
        "correspondence_set": reg_p2p.correspondence_set
    }
    
    return source_aligned, result


def _get_initial_transformation(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud
) -> np.ndarray:
    """
    Tính initial transformation từ centroid difference
    
    Args:
        source: Source point cloud
        target: Target point cloud
    
    Returns:
        4x4 transformation matrix
    """
    source_center = source.get_center()
    target_center = target.get_center()
    translation = target_center - source_center
    
    # Create initial transformation matrix
    trans_init = np.identity(4)
    trans_init[0:3, 3] = translation
    
    return trans_init


def compute_hausdorff_distance(
    pcd1: o3d.geometry.PointCloud,
    pcd2: o3d.geometry.PointCloud
) -> float:
    """
    Tính Hausdorff distance (symmetric) giữa 2 point clouds
    
    Hausdorff distance = max(h(pcd1, pcd2), h(pcd2, pcd1))
    trong đó h(A, B) = max(min(distance(a, b)) for a in A) for b in B
    
    Args:
        pcd1: First point cloud
        pcd2: Second point cloud
    
    Returns:
        Hausdorff distance (m)
    """
    # Build KDTree cho pcd2
    pcd2_tree = o3d.geometry.KDTreeFlann(pcd2)
    
    # Tính h(pcd1, pcd2)
    max_dist_1_to_2 = _compute_max_distance_to_cloud(pcd1, pcd2_tree)
    
    # Build KDTree cho pcd1
    pcd1_tree = o3d.geometry.KDTreeFlann(pcd1)
    
    # Tính h(pcd2, pcd1)
    max_dist_2_to_1 = _compute_max_distance_to_cloud(pcd2, pcd1_tree)
    
    # Symmetric Hausdorff distance
    return max(max_dist_1_to_2, max_dist_2_to_1)


def _compute_max_distance_to_cloud(
    source_pcd: o3d.geometry.PointCloud,
    target_tree: o3d.geometry.KDTreeFlann
) -> float:
    """
    Tính max distance từ source points đến target cloud
    
    Args:
        source_pcd: Source point cloud
        target_tree: KDTree của target cloud
    
    Returns:
        Maximum distance
    """
    max_dist = 0.0
    points = np.asarray(source_pcd.points)
    
    for point in points:
        [_, idx, dist] = target_tree.search_knn_vector_3d(point, 1)
        max_dist = max(max_dist, np.sqrt(dist[0]))
    
    return max_dist


def compute_chamfer_distance(
    pcd1: o3d.geometry.PointCloud,
    pcd2: o3d.geometry.PointCloud
) -> float:
    """
    Tính Chamfer distance (symmetric) giữa 2 point clouds
    
    Chamfer distance = mean(min(distance(a, b)) for a in pcd1) + 
                       mean(min(distance(b, a)) for b in pcd2)
    
    Args:
        pcd1: First point cloud
        pcd2: Second point cloud
    
    Returns:
        Chamfer distance (m)
    """
    # Build KDTree cho pcd2
    pcd2_tree = o3d.geometry.KDTreeFlann(pcd2)
    
    # Tính mean distance từ pcd1 đến pcd2
    mean_dist_1_to_2 = _compute_mean_distance_to_cloud(pcd1, pcd2_tree)
    
    # Build KDTree cho pcd1
    pcd1_tree = o3d.geometry.KDTreeFlann(pcd1)
    
    # Tính mean distance từ pcd2 đến pcd1
    mean_dist_2_to_1 = _compute_mean_distance_to_cloud(pcd2, pcd1_tree)
    
    # Symmetric Chamfer distance
    return (mean_dist_1_to_2 + mean_dist_2_to_1) / 2.0


def _compute_mean_distance_to_cloud(
    source_pcd: o3d.geometry.PointCloud,
    target_tree: o3d.geometry.KDTreeFlann
) -> float:
    """
    Tính mean distance từ source points đến target cloud
    
    Args:
        source_pcd: Source point cloud
        target_tree: KDTree của target cloud
    
    Returns:
        Mean distance
    """
    distances = []
    points = np.asarray(source_pcd.points)
    
    for point in points:
        [_, idx, dist] = target_tree.search_knn_vector_3d(point, 1)
        distances.append(np.sqrt(dist[0]))
    
    return np.mean(distances) if distances else 0.0


def compute_bbox_overlap(
    pcd1: o3d.geometry.PointCloud,
    pcd2: o3d.geometry.PointCloud
) -> float:
    """
    Tính tỷ lệ overlap giữa 2 bounding boxes (IoU - Intersection over Union)
    
    Args:
        pcd1: First point cloud
        pcd2: Second point cloud
    
    Returns:
        IoU value (0-1)
    """
    bbox1 = pcd1.get_axis_aligned_bounding_box()
    bbox2 = pcd2.get_axis_aligned_bounding_box()
    
    min1 = bbox1.get_min_bound()
    max1 = bbox1.get_max_bound()
    min2 = bbox2.get_min_bound()
    max2 = bbox2.get_max_bound()
    
    # Tính intersection
    inter_min = np.maximum(min1, min2)
    inter_max = np.minimum(max1, max2)
    
    # Kiểm tra có overlap không
    if np.any(inter_max < inter_min):
        return 0.0
    
    # Volume của intersection
    inter_volume = np.prod(inter_max - inter_min)
    
    # Volume của union
    volume1 = np.prod(max1 - min1)
    volume2 = np.prod(max2 - min2)
    union_volume = volume1 + volume2 - inter_volume
    
    if union_volume <= 0:
        return 0.0
    
    # IoU (Intersection over Union)
    return inter_volume / union_volume


def compute_centroid_distance(
    pcd1: o3d.geometry.PointCloud,
    pcd2: o3d.geometry.PointCloud
) -> float:
    """
    Tính khoảng cách giữa centroids của 2 point clouds
    
    Args:
        pcd1: First point cloud
        pcd2: Second point cloud
    
    Returns:
        Centroid distance (m)
    """
    centroid1 = pcd1.get_center()
    centroid2 = pcd2.get_center()
    
    return float(np.linalg.norm(centroid1 - centroid2))


def compute_density_ratio(
    pcd1: o3d.geometry.PointCloud,
    pcd2: o3d.geometry.PointCloud
) -> float:
    """
    Tính tỷ lệ mật độ điểm giữa 2 point clouds (sau khi normalize theo volume)
    
    Args:
        pcd1: First point cloud
        pcd2: Second point cloud
    
    Returns:
        Density ratio (0-1)
    """
    num_points1 = len(pcd1.points)
    num_points2 = len(pcd2.points)
    
    # Volume của bounding box
    bbox1 = pcd1.get_axis_aligned_bounding_box()
    bbox2 = pcd2.get_axis_aligned_bounding_box()
    
    volume1 = np.prod(bbox1.get_max_bound() - bbox1.get_min_bound())
    volume2 = np.prod(bbox2.get_max_bound() - bbox2.get_min_bound())
    
    # Mật độ điểm (points per unit volume)
    if volume1 > 0 and volume2 > 0:
        density1 = num_points1 / volume1
        density2 = num_points2 / volume2
        
        # Tỷ lệ (normalize về 0-1)
        if density1 > 0:
            return min(density2 / density1, density1 / density2)
        return 0.0
    
    # Nếu volume = 0, so sánh số điểm
    if num_points1 > 0:
        return min(num_points2 / num_points1, num_points1 / num_points2)
    
    return 0.0


def compute_similarity_metrics(
    pcd1: o3d.geometry.PointCloud,
    pcd2: o3d.geometry.PointCloud,
    perform_icp: bool = True,
    icp_threshold: float = DEFAULT_ICP_THRESHOLD,
    icp_max_iterations: int = DEFAULT_ICP_MAX_ITERATIONS,
    use_initial_guess: bool = True,
    save_obj_pcd: Optional[Path] = None
) -> Dict:
    """
    Tính tất cả metrics để so sánh 2 point clouds
    
    Args:
        pcd1: Reference point cloud
        pcd2: Test point cloud
        perform_icp: Có thực hiện ICP alignment không
        icp_threshold: Threshold cho ICP
        icp_max_iterations: Số iteration tối đa cho ICP
        use_initial_guess: Use initial guess từ centroid
        save_obj_pcd: Path để lưu converted PCD (nếu có)
    
    Returns:
        Dictionary chứa tất cả metrics
    """
    metrics = {}
    
    # Save converted PCD if requested (typically from OBJ)
    if save_obj_pcd is not None:
        save_pcd(pcd1, save_obj_pcd)
    
    # ICP Alignment và Fitness Score
    if perform_icp:
        pcd2_aligned, icp_result = compute_icp_alignment(
            pcd2, pcd1,
            threshold=icp_threshold,
            max_iterations=icp_max_iterations,
            use_initial_guess=use_initial_guess
        )
        metrics["icp_fitness"] = icp_result["fitness"]
        metrics["icp_inlier_rmse"] = icp_result["inlier_rmse"]
        pcd2_for_metrics = pcd2_aligned
    else:
        metrics["icp_fitness"] = 0.0
        metrics["icp_inlier_rmse"] = 0.0
        pcd2_for_metrics = pcd2
    
    # Compute all distance metrics
    metrics["hausdorff_distance"] = compute_hausdorff_distance(pcd1, pcd2_for_metrics)
    metrics["chamfer_distance"] = compute_chamfer_distance(pcd1, pcd2_for_metrics)
    metrics["bbox_overlap"] = compute_bbox_overlap(pcd1, pcd2_for_metrics)
    metrics["centroid_distance"] = compute_centroid_distance(pcd1, pcd2_for_metrics)
    metrics["density_ratio"] = compute_density_ratio(pcd1, pcd2_for_metrics)
    
    # Thông tin về số điểm
    metrics["reference_points"] = len(pcd1.points)
    metrics["test_points"] = len(pcd2.points)
    
    return metrics


def normalize_distance(distance: float, max_distance: float) -> float:
    """
    Normalize distance về range 0-1
    
    Args:
        distance: Distance value
        max_distance: Maximum distance để normalize
    
    Returns:
        Normalized distance (0-1)
    """
    return min(distance / max_distance, 1.0)


def calculate_similarity_percentage(
    metrics: Dict,
    max_hausdorff: float = DEFAULT_MAX_DISTANCES["hausdorff"],
    max_chamfer: float = DEFAULT_MAX_DISTANCES["chamfer"],
    max_centroid: float = DEFAULT_MAX_DISTANCES["centroid"]
) -> float:
    """
    Tính similarity percentage từ các metrics
    
    Formula:
    similarity = (
        0.4 * ICP_fitness_score +
        0.2 * (1 - normalized_hausdorff) +
        0.2 * (1 - normalized_chamfer) +
        0.1 * bbox_overlap +
        0.1 * (1 - normalized_centroid_dist)
    ) * 100
    
    Args:
        metrics: Dictionary chứa các metrics
        max_hausdorff: Max Hausdorff distance để normalize
        max_chamfer: Max Chamfer distance để normalize
        max_centroid: Max Centroid distance để normalize
    
    Returns:
        Similarity percentage (0-100)
    """
    icp_fitness = metrics.get("icp_fitness", 0.0)
    hausdorff = metrics.get("hausdorff_distance", 0.0)
    chamfer = metrics.get("chamfer_distance", 0.0)
    bbox_overlap = metrics.get("bbox_overlap", 0.0)
    centroid_dist = metrics.get("centroid_distance", 0.0)
    
    # Normalize distances
    norm_hausdorff = normalize_distance(hausdorff, max_hausdorff)
    norm_chamfer = normalize_distance(chamfer, max_chamfer)
    norm_centroid = normalize_distance(centroid_dist, max_centroid)
    
    # Tính similarity với weights
    similarity = (
        SIMILARITY_WEIGHTS["icp_fitness"] * icp_fitness +
        SIMILARITY_WEIGHTS["hausdorff"] * (1.0 - norm_hausdorff) +
        SIMILARITY_WEIGHTS["chamfer"] * (1.0 - norm_chamfer) +
        SIMILARITY_WEIGHTS["bbox_overlap"] * bbox_overlap +
        SIMILARITY_WEIGHTS["centroid"] * (1.0 - norm_centroid)
    ) * 100.0
    
    return max(0.0, min(100.0, similarity))  # Clamp to 0-100


def detect_drift(
    metrics: Dict,
    similarity: float,
    hausdorff_threshold: float = DEFAULT_DRIFT_THRESHOLDS["hausdorff"],
    chamfer_threshold: float = DEFAULT_DRIFT_THRESHOLDS["chamfer"],
    centroid_threshold: float = DEFAULT_DRIFT_THRESHOLDS["centroid"],
    similarity_threshold: float = DEFAULT_DRIFT_THRESHOLDS["similarity"]
) -> Tuple[bool, List[str]]:
    """
    Phát hiện drift dựa trên metrics và thresholds
    
    Args:
        metrics: Dictionary chứa các metrics
        similarity: Similarity percentage
        hausdorff_threshold: Threshold cho Hausdorff distance
        chamfer_threshold: Threshold cho Chamfer distance
        centroid_threshold: Threshold cho Centroid distance
        similarity_threshold: Threshold cho Similarity percentage
    
    Returns:
        (drift_detected, drift_reasons)
    """
    drift_detected = False
    reasons = []
    
    hausdorff = metrics.get("hausdorff_distance", 0.0)
    chamfer = metrics.get("chamfer_distance", 0.0)
    centroid_dist = metrics.get("centroid_distance", 0.0)
    
    # Check each threshold
    if hausdorff > hausdorff_threshold:
        drift_detected = True
        reasons.append(
            f"Hausdorff distance ({hausdorff:.3f}m) > threshold ({hausdorff_threshold:.3f}m)"
        )
    
    if chamfer > chamfer_threshold:
        drift_detected = True
        reasons.append(
            f"Chamfer distance ({chamfer:.3f}m) > threshold ({chamfer_threshold:.3f}m)"
        )
    
    if centroid_dist > centroid_threshold:
        drift_detected = True
        reasons.append(
            f"Centroid distance ({centroid_dist:.3f}m) > threshold ({centroid_threshold:.3f}m)"
        )
    
    if similarity < similarity_threshold:
        drift_detected = True
        reasons.append(
            f"Similarity ({similarity:.1f}%) < threshold ({similarity_threshold:.1f}%)"
        )
    
    return drift_detected, reasons
