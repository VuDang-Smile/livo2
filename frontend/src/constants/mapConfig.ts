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
 * Get MQTT WebSocket URL based on current hostname
 */
export function getMQTTWebSocketUrl(): string {
  const hostname = window.location.hostname;
  // If running on frontend.lidar.tm or lidar.tm, use mqtt.lidar.tm via nginx proxy
  if (hostname === 'frontend.lidar.tm' || hostname === 'lidar.tm' || hostname.includes('lidar.tm')) {
    return 'ws://mqtt.lidar.tm';
  }
  // For localhost or IP addresses, use same hostname with port 9001
  return `ws://${hostname}:9001`;
}

/**
 * Build URL for map image based on view
 * Uses local public folder for testing (uploadId parameter is ignored)
 */
export function getMapImageUrl(uploadId: string | undefined, view: 'top' | 'side_x' | 'side_y'): string {
  // Use local public folder for testing - uploadId is not needed
  const imageMap: Record<'top' | 'side_x' | 'side_y', string> = {
    top: '/floorplan_2d/merge_all_hba_top.png',
    side_x: '/floorplan_2d/merge_all_hba_side_x.png',
    side_y: '/floorplan_2d/merge_all_hba_side_y.png'
  };
  return imageMap[view];
}

/**
 * Build URL for map metadata JSON
 * Uses local public folder for testing
 */
export function getMapMetadataUrl(uploadId: string): string {
  // Use local public folder for testing
  return '/floorplan_2d/merge_all_hba_metadata.json';
}

/**
 * Get current map from backend
 * @deprecated API calls removed - using local files from /floorplan_2d/ instead
 */
// export async function getCurrentMap(): Promise<any> {
//   const backendUrl = getBackendUrl();
//   const response = await fetch(`${backendUrl}/maps/current`, {
//     method: 'GET',
//     headers: {
//       'Content-Type': 'application/json',
//     },
//   });
//   
//   if (!response.ok) {
//     throw new Error(`Failed to get current map: ${response.statusText}`);
//   }
//   
//   return await response.json();
// }

