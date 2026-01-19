import {
  MAP_FLOORPLAN_IMAGE_URLS,
  MAP_FLOORPLAN_METADATA_URL,
} from '../config/dataSources';
/**
 * Configuration for 2D map
 * Includes URL of map image and world bounds for coordinate mapping
 */

// Ảnh 2D mặc định dùng view top từ floorplan thực tế
export const MAP_2D_IMAGE_URL = MAP_FLOORPLAN_IMAGE_URLS.top;

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
  const { protocol, host } = window.location;

  // Use secure WebSocket when running over HTTPS, otherwise use plain WS.
  const wsProtocol = protocol === 'https:' ? 'wss' : 'ws';

  // Always connect to the same host as the frontend (domain hoặc IP),
  // đi qua nginx reverse proxy tại đường dẫn /mqtt.
  // Ví dụ:
  //   http://frontend.lidar.tm -> ws://frontend.lidar.tm/mqtt
  //   https://frontend.lidar.tm -> wss://frontend.lidar.tm/mqtt
  //   http://192.168.x.x -> ws://192.168.x.x/mqtt
  return `${wsProtocol}://${host}/mqtt`;
}

/**
 * Build URL for map image based on view
 * Sử dụng URL thực tế từ storage (uploadId hiện tại không dùng)
 */
export function getMapImageUrl(uploadId: string | undefined, view: 'top' | 'side_x'): string {
  return MAP_FLOORPLAN_IMAGE_URLS[view];
}

/**
 * Build URL for map metadata JSON
 * Sử dụng URL thực tế từ storage
 */
export function getMapMetadataUrl(uploadId: string): string {
  return MAP_FLOORPLAN_METADATA_URL;
}

/**
 * Get current map from backend
 * @deprecated hiện tại sử dụng metadata/floorplan từ storage.lidar.tm
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

