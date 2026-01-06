/**
 * Configuration for 2D map
 * Includes URL of map image and world bounds for coordinate mapping
 */

export const MAP_2D_IMAGE_URL = '/2Dmap.png';

/**
 * World bounds for mapping coordinates from 3D world space to 2D canvas
 * These values must match the mapping logic in Map2D and Image2DPreview components
 */
export const MAP_WORLD_BOUNDS = {
  minX: -100,
  maxX: 100,
  minZ: -50,
  maxZ: 50,
  width: 200,  // maxX - minX
  height: 100, // maxZ - minZ
} as const;

/**
 * Get backend URL based on current hostname
 */
export function getBackendUrl(): string {
  const hostname = window.location.hostname;
  // If running on frontend.lidar.tm or lidar.tm, use backend.lidar.tm
  if (hostname === 'frontend.lidar.tm' || hostname === 'lidar.tm' || hostname.includes('lidar.tm')) {
    return 'http://backend.lidar.tm';
  }
  // For localhost or IP addresses, use same hostname with port 8000
  return `http://${hostname}:8000`;
}

/**
 * Build URL for map image based on upload_id and view
 */
export function getMapImageUrl(uploadId: string, view: 'top' | 'side_x' | 'side_y'): string {
  const backendUrl = getBackendUrl();
  return `${backendUrl}/maps/${uploadId}/images/${view}`;
}

/**
 * Build URL for map metadata JSON
 */
export function getMapMetadataUrl(uploadId: string): string {
  const backendUrl = getBackendUrl();
  return `${backendUrl}/maps/${uploadId}/metadata`;
}

/**
 * Get current map from backend
 */
export async function getCurrentMap(): Promise<any> {
  const backendUrl = getBackendUrl();
  const response = await fetch(`${backendUrl}/maps/current`, {
    method: 'GET',
    headers: {
      'Content-Type': 'application/json',
    },
  });
  
  if (!response.ok) {
    throw new Error(`Failed to get current map: ${response.statusText}`);
  }
  
  return await response.json();
}

