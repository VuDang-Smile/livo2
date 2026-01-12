import { Pose3D, Pose2DPixel, MapMetadata, OrientationConfig } from '../types/mapMetadata';

/**
 * Transform 3D pose (world coordinates) to 2D pixel coordinates on floorplan image
 * Supports multiple views: top (XY), side_x (YZ), side_y (XZ)
 * 
 * @param pose - 3D pose from localization (in map frame)
 * @param metadata - Complete map metadata with all views
 * @param view - View to use: 'top', 'side_x', or 'side_y'
 * @param debugMode - Enable console logging for debugging
 * @returns Pixel coordinates and orientation, or null if transformation fails
 */
export function transformPoseToPixel(
  pose: Pose3D,
  metadata: MapMetadata,
  view: 'top' | 'side_x' | 'side_y' = 'top',
  debugMode: boolean = false
): Pose2DPixel | null {
  
  if (!pose || !metadata) {
    console.error('transformPoseToPixel: Missing pose or metadata');
    return null;
  }
  
  const viewMetadata = metadata.views[view];
  if (!viewMetadata) {
    console.error(`transformPoseToPixel: View '${view}' not found in metadata`);
    return null;
  }
  
  const { position } = pose;
  const { projection, bounds, image, processing, orientation } = viewMetadata;
  const resolution = metadata.resolution_m_per_pixel;
  
  // Get the two axes used for this view
  const u_axis = projection.world_axes.horizontal; // e.g., 'X' for top view
  const v_axis = projection.world_axes.vertical;   // e.g., 'Y' for top view
  
  // Extract coordinates from pose based on axes
  const getAxisValue = (axis: string): number => {
    switch (axis) {
      case 'X': return position.x;
      case 'Y': return position.y;
      case 'Z': return position.z;
      default: return 0;
    }
  };
  
  const u_value = getAxisValue(u_axis);
  const v_value = getAxisValue(v_axis);
  
  // Get bounds for the two axes
  const u_bounds = bounds.world[u_axis];
  const v_bounds = bounds.world[v_axis];
  
  if (!u_bounds || !v_bounds) {
    console.error(`transformPoseToPixel: Invalid bounds for view '${view}'`);
    return null;
  }
  
  const u_min = u_bounds.min;
  const u_max = u_bounds.max;
  const v_min = v_bounds.min;
  const v_max = v_bounds.max;
  
  // Check if pose is within bounds
  const is_out_of_bounds = (
    u_value < u_min || u_value > u_max ||
    v_value < v_min || v_value > v_max
  );
  
  if (debugMode && is_out_of_bounds) {
    console.warn(`Pose outside map bounds for view '${view}':`, {
      pose: position,
      bounds: { [u_axis]: [u_min, u_max], [v_axis]: [v_min, v_max] }
    });
  }
  
  // Transform world coordinates → grid cell (relative to bounds)
  const grid_u = (u_value - u_min) / resolution;
  const grid_v = (v_value - v_min) / resolution;
  
  // Flip V-axis (image Y increases downward, world V increases upward)
  const content_height = processing.content_area.height;
  const pixel_u_content = Math.round(grid_u);
  const pixel_v_content = Math.round(content_height - grid_v - 1);
  
  // Add border offset
  const border_margin = processing.border_margin;
  const pixel_u_with_border = pixel_u_content + border_margin;
  const pixel_v_with_border = pixel_v_content + border_margin;
  
  // Clamp to image bounds
  const pixel_x = Math.max(0, Math.min(image.width - 1, pixel_u_with_border));
  const pixel_y = Math.max(0, Math.min(image.height - 1, pixel_v_with_border));
  
  const is_clamped = (
    pixel_x !== pixel_u_with_border ||
    pixel_y !== pixel_v_with_border
  );
  
  // Calculate 2D orientation from quaternion
  const yaw = compute2DOrientation(pose.orientation, orientation);
  
  // Debug output
  if (debugMode) {
    console.log(`🔍 Coordinate Transform (${view}):`, {
      input: {
        world: `(${u_value.toFixed(2)}, ${v_value.toFixed(2)})`,
        bounds: { [u_axis]: [u_min, u_max], [v_axis]: [v_min, v_max] },
        resolution
      },
      steps: {
        grid: `(${grid_u.toFixed(2)}, ${grid_v.toFixed(2)})`,
        content: `(${pixel_u_content}, ${pixel_v_content})`,
        with_border: `(${pixel_u_with_border}, ${pixel_v_with_border})`,
        final: `(${pixel_x}, ${pixel_y})`
      },
      output: {
        pixel: `(${pixel_x}, ${pixel_y})`,
        yaw_deg: `${(yaw * 180 / Math.PI).toFixed(1)}°`,
        is_clamped,
        is_out_of_bounds
      }
    });
  }
  
  return {
    pixel_x,
    pixel_y,
    yaw,
    world_x: u_value,
    world_y: v_value,
    is_clamped,
    is_out_of_bounds
  };
}

/**
 * Compute 2D orientation angle from 3D quaternion based on view configuration
 * 
 * @param q - Quaternion {x, y, z, w}
 * @param orientationConfig - Orientation configuration from view metadata
 * @returns Orientation angle in radians
 */
export function compute2DOrientation(
  q: { x: number; y: number; z: number; w: number },
  orientationConfig: OrientationConfig
): number {
  // Convert quaternion to rotation matrix (3x3)
  // Forward vector in body frame
  const forward_body = orientationConfig.forward_vector_in_body;
  
  // Rotate forward vector from body to world frame using quaternion
  // q * v * q^-1 where v is pure quaternion (0, x, y, z)
  const qw = q.w;
  const qx = q.x;
  const qy = q.y;
  const qz = q.z;
  
  // Rotation matrix from quaternion
  const R = [
    [
      1 - 2 * (qy * qy + qz * qz),
      2 * (qx * qy - qw * qz),
      2 * (qx * qz + qw * qy)
    ],
    [
      2 * (qx * qy + qw * qz),
      1 - 2 * (qx * qx + qz * qz),
      2 * (qy * qz - qw * qx)
    ],
    [
      2 * (qx * qz - qw * qy),
      2 * (qy * qz + qw * qx),
      1 - 2 * (qx * qx + qy * qy)
    ]
  ];
  
  // Apply rotation to forward vector
  const fw_x = R[0][0] * forward_body[0] + R[0][1] * forward_body[1] + R[0][2] * forward_body[2];
  const fw_y = R[1][0] * forward_body[0] + R[1][1] * forward_body[1] + R[1][2] * forward_body[2];
  const fw_z = R[2][0] * forward_body[0] + R[2][1] * forward_body[1] + R[2][2] * forward_body[2];
  
  // Get forward vector in world frame
  const f_world = [fw_x, fw_y, fw_z];
  
  // Extract components for the two axes used in this view
  const getAxisComponent = (axis: string): number => {
    switch (axis) {
      case 'X': return f_world[0];
      case 'Y': return f_world[1];
      case 'Z': return f_world[2];
      default: return 0;
    }
  };
  
  const u = getAxisComponent(orientationConfig.world_u_axis);
  const v = getAxisComponent(orientationConfig.world_v_axis);
  
  // Apply image mapping (flip directions)
  const u_prime = u * orientationConfig.image_mapping.u_to_image_x;
  const v_prime = v * orientationConfig.image_mapping.v_to_image_y;
  
  // Calculate angle in image coordinates
  const angle = Math.atan2(v_prime, u_prime);
  
  return angle;
}

/**
 * Convert quaternion to yaw angle (rotation around Z-axis)
 * Legacy function for backward compatibility
 * 
 * @param q - Quaternion {x, y, z, w}
 * @returns Yaw angle in radians
 */
export function quaternionToYaw(q: { x: number; y: number; z: number; w: number }): number {
  // Yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))
  return Math.atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  );
}

/**
 * Reverse transform: 2D pixel → 3D world coordinates (useful for debugging)
 * 
 * @param pixel_x - Pixel X coordinate
 * @param pixel_y - Pixel Y coordinate
 * @param metadata - Complete map metadata
 * @param view - View to use: 'top', 'side_x', or 'side_y'
 * @returns World coordinates for the two axes used in this view
 */
export function pixelToWorld(
  pixel_x: number,
  pixel_y: number,
  metadata: MapMetadata,
  view: 'top' | 'side_x' | 'side_y' = 'top'
): { [key: string]: number } {
  const viewMetadata = metadata.views[view];
  if (!viewMetadata) {
    console.error(`pixelToWorld: View '${view}' not found in metadata`);
    return {};
  }
  
  const { projection, bounds, processing } = viewMetadata;
  const resolution = metadata.resolution_m_per_pixel;
  
  // Remove border offset
  const pixel_u_content = pixel_x - processing.border_margin;
  const pixel_v_content = pixel_y - processing.border_margin;
  
  // Un-flip V-axis
  const content_height = processing.content_area.height;
  const grid_v = content_height - pixel_v_content - 1;
  const grid_u = pixel_u_content;
  
  // Transform grid → world
  const u_axis = projection.world_axes.horizontal;
  const v_axis = projection.world_axes.vertical;
  
  const u_bounds = bounds.world[u_axis];
  const v_bounds = bounds.world[v_axis];
  
  if (!u_bounds || !v_bounds) {
    console.error(`pixelToWorld: Invalid bounds for view '${view}'`);
    return {};
  }
  
  const world_u = grid_u * resolution + u_bounds.min;
  const world_v = grid_v * resolution + v_bounds.min;
  
  return {
    [u_axis]: world_u,
    [v_axis]: world_v
  };
}

/**
 * Validate coordinate transformation (round-trip test)
 * 
 * @param pose - Original 3D pose
 * @param pixel - Transformed pixel coordinates
 * @param metadata - Complete map metadata
 * @param view - View to use: 'top', 'side_x', or 'side_y'
 * @returns Validation result with error distance
 */
export function validateTransform(
  pose: Pose3D,
  pixel: Pose2DPixel,
  metadata: MapMetadata,
  view: 'top' | 'side_x' | 'side_y' = 'top'
): { isValid: boolean; error: number } {
  const viewMetadata = metadata.views[view];
  if (!viewMetadata) {
    console.error(`validateTransform: View '${view}' not found in metadata`);
    return { isValid: false, error: Infinity };
  }
  
  const world_back = pixelToWorld(pixel.pixel_x, pixel.pixel_y, metadata, view);
  
  const { projection } = viewMetadata;
  const u_axis = projection.world_axes.horizontal;
  const v_axis = projection.world_axes.vertical;
  
  const getAxisValue = (axis: string): number => {
    switch (axis) {
      case 'X': return pose.position.x;
      case 'Y': return pose.position.y;
      case 'Z': return pose.position.z;
      default: return 0;
    }
  };
  
  const original_u = getAxisValue(u_axis);
  const original_v = getAxisValue(v_axis);
  
  const error_u = Math.abs((world_back[u_axis] || 0) - original_u);
  const error_v = Math.abs((world_back[v_axis] || 0) - original_v);
  const error = Math.sqrt(error_u * error_u + error_v * error_v);
  
  const tolerance = metadata.resolution_m_per_pixel * 2;  // 2 pixels tolerance
  const isValid = error <= tolerance;
  
  if (!isValid) {
    console.error(`Transform validation failed for view '${view}':`, {
      original: { [u_axis]: original_u, [v_axis]: original_v },
      pixel: { x: pixel.pixel_x, y: pixel.pixel_y },
      reverse: world_back,
      error: { [u_axis]: error_u, [v_axis]: error_v, total: error },
      tolerance
    });
  }
  
  return { isValid, error };
}

