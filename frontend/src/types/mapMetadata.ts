/**
 * Type definitions for map metadata and coordinate transformations
 * Supports 3 views: top (XY plane), side_x (YZ plane), side_y (XZ plane)
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
 * Metadata for a single view (top, side_x, or side_y)
 */
export interface ViewMetadata {
  id: 'top' | 'side_x' | 'side_y';
  projection: ProjectionInfo;
  bounds: {
    world: WorldBounds;
  };
  image: ImageDimensions;
  processing: ProcessingParams;
  orientation: OrientationConfig;
}

/**
 * Complete map metadata with all 3 views
 */
export interface MapMetadata {
  input_file: string;
  resolution_m_per_pixel: number;
  views: {
    top: ViewMetadata;
    side_x: ViewMetadata;
    side_y: ViewMetadata;
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

