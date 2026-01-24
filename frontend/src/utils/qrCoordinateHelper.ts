import { MapMetadata } from '../types/mapMetadata';
import { transformSceneToWorld, transformWorldToScene } from './coordinateTransformer';

/**
 * Get default Y coordinate for a given surface
 * @param surface - Surface type: 'floor', 'ceiling', 'left', or 'right'
 * @returns Default Y coordinate value
 */
export function getDefaultYForSurface(surface: 'floor' | 'ceiling' | 'left' | 'right'): number {
  switch (surface) {
    case 'floor':
      return -2;
    case 'ceiling':
      return 8;
    case 'left':
    case 'right':
      return 3;
    default:
      return -2; // Default to floor
  }
}

/**
 * Extract 2D position from 3D scene coordinates (Three.js)
 * Converts scene coordinates -> world coordinates -> 2D position
 * @param position3DScene - 3D position in scene coordinates (Three.js) as [x, y, z]
 * @param mapMetadata - Map metadata containing projection configuration
 * @param view - View type: 'top' or 'side_x' (default: 'top')
 * @returns 2D position as [u, v] tuple in world coordinates
 */
export function extract2DFrom3DScene(
  position3DScene: [number, number, number],
  mapMetadata: MapMetadata | null | undefined,
  view: 'top' | 'side_x' = 'top'
): [number, number] {
  // Transform from scene coordinates to world coordinates
  const coordinateConfig = mapMetadata?.coordinate_system;
  const worldPos = transformSceneToWorld(position3DScene, coordinateConfig);
  
  // Extract 2D from world coordinates
  return extract2DFrom3DWorld(worldPos, mapMetadata, view);
}

/**
 * Extract 2D position from 3D world coordinates using map metadata projection configuration
 * @param position3D - 3D position in world coordinates as [x, y, z] or {x, y, z}
 * @param mapMetadata - Map metadata containing projection configuration
 * @param view - View type: 'top' or 'side_x' (default: 'top')
 * @returns 2D position as [u, v] tuple
 */
export function extract2DFrom3DWorld(
  position3D: [number, number, number] | { x: number; y: number; z: number },
  mapMetadata: MapMetadata | null | undefined,
  view: 'top' | 'side_x' = 'top'
): [number, number] {
  // Normalize input to object format
  const pos3d = Array.isArray(position3D)
    ? { x: position3D[0], y: position3D[1], z: position3D[2] }
    : position3D;

  // Fallback if no metadata
  if (!mapMetadata?.views?.[view]?.projection?.world_axes) {
    return [pos3d.x, pos3d.z];
  }

  const projection = mapMetadata.views[view].projection.world_axes;
  
  const getAxis = (axis: 'X' | 'Y' | 'Z'): number => {
    switch (axis) {
      case 'X': return pos3d.x;
      case 'Y': return pos3d.y;
      case 'Z': return pos3d.z;
      default: return 0;
    }
  };

  return [getAxis(projection.horizontal), getAxis(projection.vertical)];
}

/**
 * Legacy function - kept for backward compatibility
 * @deprecated Use extract2DFrom3DScene or extract2DFrom3DWorld instead
 */
export function extract2DFrom3D(
  position3D: [number, number, number] | { x: number; y: number; z: number },
  mapMetadata: MapMetadata | null | undefined,
  view: 'top' | 'side_x' = 'top'
): [number, number] {
  // Assume it's world coordinates for backward compatibility
  return extract2DFrom3DWorld(position3D, mapMetadata, view);
}

/**
 * Convert 2D world position to 3D scene coordinates (Three.js)
 * Converts 2D world coordinates -> 3D world coordinates -> scene coordinates
 * @param position2D - 2D position in world coordinates as [u, v] tuple
 * @param surface - Surface type: 'floor', 'ceiling', 'left', or 'right' (default: 'floor')
 * @param mapMetadata - Map metadata containing projection configuration
 * @param view - View type: 'top' or 'side_x' (default: 'top')
 * @returns 3D position in scene coordinates as [x, y, z] tuple
 */
export function convert2DTo3DScene(
  position2D: [number, number],
  surface: 'floor' | 'ceiling' | 'left' | 'right' = 'floor',
  mapMetadata: MapMetadata | null | undefined,
  view: 'top' | 'side_x' = 'top'
): [number, number, number] {
  // Convert 2D to 3D world coordinates
  const worldPos3D = convert2DTo3DWorld(position2D, surface, mapMetadata, view);
  
  // Transform from world coordinates to scene coordinates
  const coordinateConfig = mapMetadata?.coordinate_system;
  return transformWorldToScene(worldPos3D, coordinateConfig);
}

/**
 * Convert 2D position to 3D world position based on surface and map metadata
 * @param position2D - 2D position in world coordinates as [u, v] tuple
 * @param surface - Surface type: 'floor', 'ceiling', 'left', or 'right' (default: 'floor')
 * @param mapMetadata - Map metadata containing projection configuration
 * @param view - View type: 'top' or 'side_x' (default: 'top')
 * @returns 3D position in world coordinates as [x, y, z] tuple
 */
export function convert2DTo3DWorld(
  position2D: [number, number],
  surface: 'floor' | 'ceiling' | 'left' | 'right' = 'floor',
  mapMetadata: MapMetadata | null | undefined,
  view: 'top' | 'side_x' = 'top'
): [number, number, number] {
  const [u, v] = position2D;
  const base = { x: 0, y: 0, z: 0 };

  // Fallback if no metadata
  if (!mapMetadata?.views?.[view]?.projection?.world_axes) {
    const defaultY = getDefaultYForSurface(surface);
    return [u, defaultY, v];
  }

  const projection = mapMetadata.views[view].projection.world_axes;
  
  const setAxis = (axis: 'X' | 'Y' | 'Z', val: number) => {
    switch (axis) {
      case 'X': base.x = val; break;
      case 'Y': base.y = val; break;
      case 'Z': base.z = val; break;
      default: break;
    }
  };

  // Set horizontal and vertical axes from 2D position
  setAxis(projection.horizontal, u);
  setAxis(projection.vertical, v);

  // Set Y coordinate based on surface (if Y is not already set by projection)
  // If Y is the vertical axis, it's already set, otherwise use surface default
  if (projection.vertical !== 'Y') {
    base.y = getDefaultYForSurface(surface);
  } else {
    // If Y is the vertical axis, adjust based on surface if needed
    // For now, keep the value from projection but could adjust if needed
  }

  return [base.x, base.y, base.z];
}

/**
 * Legacy function - kept for backward compatibility
 * @deprecated Use convert2DTo3DScene or convert2DTo3DWorld instead
 */
export function convert2DTo3D(
  position2D: [number, number],
  surface: 'floor' | 'ceiling' | 'left' | 'right' = 'floor',
  mapMetadata: MapMetadata | null | undefined,
  view: 'top' | 'side_x' = 'top'
): [number, number, number] {
  // Return world coordinates for backward compatibility
  return convert2DTo3DWorld(position2D, surface, mapMetadata, view);
}

/**
 * Convert 2D position to 3D position object format in scene coordinates
 * @param position2D - 2D position in world coordinates as [u, v] tuple
 * @param surface - Surface type: 'floor', 'ceiling', 'left', or 'right' (default: 'floor')
 * @param mapMetadata - Map metadata containing projection configuration
 * @param view - View type: 'top' or 'side_x' (default: 'top')
 * @returns 3D position in scene coordinates as {x, y, z} object
 */
export function convert2DTo3DObject(
  position2D: [number, number],
  surface: 'floor' | 'ceiling' | 'left' | 'right' = 'floor',
  mapMetadata: MapMetadata | null | undefined,
  view: 'top' | 'side_x' = 'top'
): { x: number; y: number; z: number } {
  const [x, y, z] = convert2DTo3DScene(position2D, surface, mapMetadata, view);
  return { x, y, z };
}
