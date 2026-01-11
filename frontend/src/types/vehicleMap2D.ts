/**
 * Types for 2D vehicle map
 */

/**
 * Vehicle data for 2D map
 */
export interface Vehicle {
  id: string;
  name: string;
  type: 'scanner' | 'worker';
  status: 'active' | 'inactive' | 'offline';
  position: { x: number; y: number; z: number };
  timestamp: string;
  source?: 'api' | 'mqtt';
}

/**
 * Response from vehicle map API
 */
export interface VehicleMapResponse {
  modelUrl: string;
  unit: string;
  mapType: '2d';
  source: 'converted' | 'original';
  pcd_file_id?: string;
}
