/**
 * Type definitions for map metadata and coordinate transformations
 * Supports 2 views: top (XY plane), side_x (YZ plane)
 */

/**
 * 3D pose with position and orientation (quaternion)
 */
export interface Pose3D {
  position: {
    x: number;
    y: number;
    z: number;
  };
  orientation: {
    x: number;
    y: number;
    z: number;
    w: number;
  };
  // Optional metadata provided by some MQTT pose messages
  frame_id?: string;
  timestamp?: string | number;
}

/**
 * 2D pixel coordinates with orientation and world coordinates
 */
export interface Pose2DPixel {
  pixel_x: number;
  pixel_y: number;
  yaw: number; // Orientation angle in radians
  world_x: number; // Original world X coordinate
  world_y: number; // Original world Y coordinate (or Z for side views)
  is_clamped: boolean; // Whether pixel was clamped to image bounds
  is_out_of_bounds: boolean; // Whether pose was outside map bounds
}

/**
 * Rotation metadata structure from merged_all_rotated_rotation_metadata.json
 */
export interface RotationMetadata {
  rotation_info: {
    rotation_angle_rad: number;
    rotation_angle_deg: number;
    rotation_axis: [number, number, number];
    rotation_matrix: number[][]; // 4x4 matrix
    method: string;
  };
  direction_before: [number, number, number];
  direction_after: [number, number, number];
  input_pcd?: string;
  output_pcd?: string;
  timestamp?: string;
  original_metadata?: any;
}

/**
 * Coordinate system convention definition
 * Dùng để mô tả hệ trục của dữ liệu bản đồ/PCD so với chuẩn ROS hoặc Three.js.
 */
export interface CoordinateSystemConfig {
  /**
   * Tên quy ước:
   * - 'ros': X=forward, Y=left, Z=up (chuẩn ROS / RViz)
   * - 'threejs': X=right, Y=up, Z=backward (chuẩn Three.js mặc định)
   * - 'custom': cấu hình tuỳ chỉnh qua transform phía dưới
   */
  convention: 'ros' | 'threejs' | 'custom';
  /**
   * Cấu hình transform tuỳ chỉnh nếu convention = 'custom'.
   * Có thể dùng rotation Euler hoặc axis mapping đơn giản.
   */
  transform?: {
    /** Rotation Euler (rad) XYZ, áp dụng lên dữ liệu trước khi render */
    rotation?: [number, number, number];
    /**
     * Ánh xạ trục: chọn trục world nào map sang trục scene X/Y/Z.
     * Ví dụ: sceneX='X', sceneY='Z', sceneZ='Y'
     */
    axisMapping?: {
      sceneX: 'X' | 'Y' | 'Z';
      sceneY: 'X' | 'Y' | 'Z';
      sceneZ: 'X' | 'Y' | 'Z';
    };
    /** Dấu của từng trục sau mapping: 1 hoặc -1 cho [X, Y, Z] */
    axisSigns?: [number, number, number];
  };
}

/**
 * Projection information: which axis is projected and which plane is used
 */
export interface ProjectionInfo {
  axis: 'X' | 'Y' | 'Z'; // Axis along which projection is done
  plane: 'XY' | 'YZ' | 'XZ'; // Plane onto which points are projected
  world_axes: {
    horizontal: 'X' | 'Y' | 'Z'; // World axis mapped to image X (horizontal)
    vertical: 'X' | 'Y' | 'Z'; // World axis mapped to image Y (vertical, before flip)
  };
  image_axes: {
    x_direction: '+X' | '-X' | '+Y' | '-Y' | '+Z' | '-Z'; // Direction of image X axis
    y_direction: '+X' | '-X' | '+Y' | '-Y' | '+Z' | '-Z'; // Direction of image Y axis (before flip)
  };
}

/**
 * Configuration for computing 2D orientation from 3D quaternion
 */
export interface OrientationConfig {
  source_frame: string; // Frame name (e.g., "base_link")
  forward_vector_in_body: [number, number, number]; // Forward vector in body frame (e.g., [1, 0, 0])
  plane: 'XY' | 'YZ' | 'XZ'; // Plane for projection
  world_u_axis: 'X' | 'Y' | 'Z'; // Horizontal axis in world frame
  world_v_axis: 'X' | 'Y' | 'Z'; // Vertical axis in world frame
  image_mapping: {
    u_to_image_x: number; // +1 or -1: mapping from u to image X
    v_to_image_y: number; // +1 or -1: mapping from v to image Y (before flip)
  };
  angle_unit: 'radian' | 'degree'; // Unit for angle output
}

/**
 * Bounds for a specific axis
 */
export interface AxisBounds {
  min: number;
  max: number;
}

/**
 * World bounds for a view (2 axes)
 */
export interface WorldBounds {
  [key: string]: AxisBounds; // e.g., { X: {min, max}, Y: {min, max} }
}

/**
 * Image dimensions
 */
export interface ImageDimensions {
  width: number;
  height: number;
}

/**
 * Content area within image (excluding border)
 */
export interface ContentArea {
  x: number;
  y: number;
  width: number;
  height: number;
}

/**
 * Processing parameters for map generation
 */
export interface ProcessingParams {
  border_margin: number;
  content_area: ContentArea;
  auto_crop: boolean;
  crop_applied: boolean;
  outlier_filter?: boolean;
  outlier_percentile?: number | null;
  colormap: 'binary' | 'density' | 'height';
  inverted: boolean;
  original_bounds?: {
    x_min: number;
    x_max: number;
    y_min: number;
    y_max: number;
  } | null;
}

/**
 * Metadata for a single view (top or side_x)
 */
export interface ViewMetadata {
  id: 'top' | 'side_x';
  projection: ProjectionInfo;
  bounds: {
    world: WorldBounds;
  };
  image: ImageDimensions;
  processing: ProcessingParams;
  orientation: OrientationConfig;
}

/**
 * Complete map metadata with 2 views (top and side_x)
 */
export interface MapMetadata {
  input_file: string;
  resolution_m_per_pixel: number;
  /**
   * Thông tin hệ trục của bản đồ/PCD.
   * Nếu không có, frontend sẽ mặc định coi dữ liệu ở chuẩn ROS.
   */
  coordinate_system?: CoordinateSystemConfig;
  views: {
    top: ViewMetadata;
    side_x: ViewMetadata;
  };
}

/**
 * Legacy MapMetadata format (for backward compatibility)
 * Used by existing coordinateTransform.ts
 */
export interface LegacyMapMetadata {
  bounds: {
    x: AxisBounds;
    y: AxisBounds;
  };
  resolution_m_per_pixel: number;
  processing: {
    border_margin: number;
    content_area: ContentArea;
  };
  image_width: number;
  image_height: number;
}
