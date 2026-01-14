/**
 * Centralized types for all vehicle-related data structures (frontend).
 *
 * Quy ước:
 * - API layer: mirror backend, dùng snake_case (VehicleApi*, VehicleStatus, VehicleCategory, VehicleApiPose, ...).
 * - Domain/UI: dùng camelCase, thuận tiện cho logic & hiển thị (Vehicle, VehicleFormData, ...).
 * - Map/canvas: type riêng cho 2D/3D map, canvas (VehicleMapVehicle, VehicleCanvasPosition, ...).
 * - Marker: view-model cho render (VehicleMarker3D, VehicleMarker2D).
 */

/* =========================
 *  API MODELS (mirror backend)
 * ========================= */

export type VehicleStatus = 'online' | 'offline';

export type VehicleCategory =
  | 'excavator'
  | 'bulldozer'
  | 'truck'
  | 'loader'
  | 'crane'
  | 'other';

export interface VehicleApiPose {
  position: { x: number; y: number; z: number };
  orientation: { x: number; y: number; z: number; w: number };
  timestamp: string; // ISO8601 string from backend
}

export interface VehicleApi {
  vehicle_id: string;
  name?: string;
  description?: string;
  vehicle_type?: string;
  vehicle_category?: VehicleCategory;
  status: VehicleStatus;
  metadata?: Record<string, unknown>;
  latest_pose?: VehicleApiPose;
  created_at?: string;
  updated_at?: string;
}

export interface VehiclesListApiResponse {
  vehicles: VehicleApi[];
  total: number;
}

// Backward compatibility alias (internal FE only)
export type ApiVehicle = VehicleApi;

/* =========================
 *  DOMAIN / UI MODELS
 * ========================= */

export interface Vehicle {
  id: string;
  name?: string;
  description?: string;
  vehicleType?: string;
  vehicleCategory?: VehicleCategory;
  status: VehicleStatus;
  metadata?: Record<string, unknown>;
  latestPose?: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
    timestamp: string;
  };
  createdAt?: string;
  updatedAt?: string;
}

/**
 * View-model cho form & bảng quản lý phương tiện (Vehicles.tsx, EditVehicle.tsx).
 */
export interface VehicleFormData {
  id: string;
  licensePlate: string;
  driver: string;
  vehicleType: string;
  vehicleCategory?: VehicleCategory;
  mission: string;
  status: VehicleStatus;
}

/* =========================
 *  MAP & CANVAS MODELS
 * ========================= */

/**
 * Vehicle xuất hiện trên bản đồ 2D (normalized domain cho map).
 */
export interface VehicleMapVehicle {
  id: string;
  name: string;
  vehicleType?: string;
  vehicleCategory?: VehicleCategory;
  status: VehicleStatus;
  position: { x: number; y: number; z: number };
  timestamp: string;
  source?: 'api' | 'mqtt';
}

/**
 * Response from vehicle map API (2D).
 * Giữ nguyên structure cũ.
 */
export interface VehicleMapResponse {
  modelUrl: string;
  unit: string;
  mapType: '2d';
  source: 'converted' | 'original';
  pcd_file_id?: string;
}

/**
 * Vehicle dùng cho canvas 2D (upload/preview, v.v.).
 */
export interface VehicleCanvasPosition {
  id: string;
  licensePlate?: string;
  driver?: string;
  vehicleType?: string;
  vehicleCategory?: VehicleCategory;
  mission?: string;
  position: [number, number, number];
  position2D?: [number, number]; // Normalized [0-1,0-1] for 2D rendering
  status?: VehicleStatus;
  lastUpdate?: string;
}

/* =========================
 *  METADATA
 * ========================= */

export interface VehicleMetadata {
  licensePlate?: string;
  driver?: string;
  mission?: string;
  // Mở rộng thêm các key khác tuỳ nhu cầu
  [key: string]: unknown;
}

/* =========================
 *  MARKER VIEW-MODELS (3D / 2D)
 * ========================= */

/**
 * Vehicle marker for 3D map display
 */
export interface VehicleMarker3D {
  id: string;
  position: [number, number, number];
  orientation?: [number, number, number, number]; // Quaternion [w, x, y, z]
  color: string;
  showOrientation: boolean;
  status?: VehicleStatus;
  name?: string;
  vehicleType?: string;
  vehicleCategory?: VehicleCategory;
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
  name?: string;
  vehicleType?: string;
  vehicleCategory?: VehicleCategory;
  status?: VehicleStatus;
}
