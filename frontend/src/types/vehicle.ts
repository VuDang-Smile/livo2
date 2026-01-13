/**
 * Types for vehicle-related data structures
 */

/**
 * Vehicle marker for 3D map display
 */
export interface VehicleMarker3D {
  id: string;
  position: [number, number, number];
  orientation?: [number, number, number, number]; // Quaternion [w, x, y, z]
  color: string;
  showOrientation: boolean;
  status?: 'online' | 'offline';
}

/**
 * Vehicle marker for 2D map display
 */
export interface VehicleMarker2D {
  id: string;
  position: [number, number, number]; // [x, y, z] - z is usually 0 for 2D
  orientation: [number, number, number, number]; // Quaternion [w, x, y, z]
  color: string;
  showOrientation: boolean;
  label: string;
  lastUpdate: Date;
}

/**
 * Vehicle metadata structure
 * Các field bổ sung có thể được lưu trong metadata
 */
export interface VehicleMetadata {
  licensePlate?: string;
  driver?: string;
  mission?: string;
  [key: string]: any; // Allow additional metadata fields
}

/**
 * Vehicle data from API
 */
export interface ApiVehicle {
  vehicle_id: string;
  name?: string;
  description?: string;
  type?: 'scanner' | 'worker';
  type_category?: 'scanner' | 'worker';
  vehicle_type?: string;
  status?: 'online' | 'offline';
  current_pose?: {
    frame_id: string;
    position: { x: number; y: number; z: number };
    orientation: { w: number; x: number; y: number; z: number };
    timestamp: string;
  };
  current_position?: {
    x: number;
    y: number;
    z: number;
    timestamp?: string;
  };
  metadata?: VehicleMetadata;
  created_at?: string;
  updated_at?: string;
  [key: string]: any; // Allow additional fields
}
