/**
 * Type guards and validation utilities for nullable checks
 * Provides safe validation functions for API responses, data structures, and complex types
 */

import { VehicleApi, VehicleApiPose, ApiVehicle, MapVehicle, VehicleStatus } from '../types/vehicle';
import { MapMetadata, Pose3D, ViewMetadata, RotationMetadata } from '../types/mapMetadata';
import { VehiclesListApiResponse } from '../types/vehicle';

/**
 * Type guard: Check if value is not null or undefined
 */
export function isNotNull<T>(value: T | null | undefined): value is T {
  return value !== null && value !== undefined;
}

/**
 * Type guard: Check if value is a valid array
 */
export function isValidArray<T>(value: unknown): value is T[] {
  return Array.isArray(value);
}

/**
 * Type guard: Check if value is a valid object (not null, not array, not primitive)
 */
export function isValidObject(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null && !Array.isArray(value);
}

/**
 * Type guard: Check if value is a valid number (not NaN, not Infinity)
 */
export function isValidNumber(value: unknown): value is number {
  return typeof value === 'number' && isFinite(value);
}

/**
 * Type guard: Check if value is a valid string (not empty)
 */
export function isValidString(value: unknown): value is string {
  return typeof value === 'string' && value.length > 0;
}

/**
 * Validate API response structure
 */
export function validateApiResponse<T>(response: unknown): response is T {
  return response !== null && response !== undefined;
}

/**
 * Validate VehiclesListApiResponse
 */
export function validateVehiclesListResponse(
  response: unknown
): response is VehiclesListApiResponse {
  if (!isValidObject(response)) {
    return false;
  }

  const data = response as Partial<VehiclesListApiResponse>;
  
  // Check if vehicles is an array
  if (!isValidArray(data.vehicles)) {
    return false;
  }

  // Check if total is a valid number
  if (typeof data.total !== 'number' || !isFinite(data.total)) {
    return false;
  }

  return true;
}

/**
 * Validate VehicleApi structure
 */
export function validateVehicleApi(vehicle: unknown): vehicle is VehicleApi {
  if (!isValidObject(vehicle)) {
    return false;
  }

  const v = vehicle as Partial<VehicleApi>;

  // Required fields
  if (!isValidString(v.vehicle_id)) {
    return false;
  }

  if (v.status !== 'online' && v.status !== 'offline') {
    return false;
  }

  // Validate latest_pose if present
  if (v.latest_pose !== undefined && !validateVehicleApiPose(v.latest_pose)) {
    return false;
  }

  return true;
}

/**
 * Validate VehicleApiPose structure
 */
export function validateVehicleApiPose(pose: unknown): pose is VehicleApiPose {
  if (!isValidObject(pose)) {
    return false;
  }

  const p = pose as Partial<VehicleApiPose>;

  // Validate position
  if (!isValidObject(p.position)) {
    return false;
  }

  const pos = p.position as Partial<{ x: number; y: number; z: number }>;
  if (
    !isValidNumber(pos.x) ||
    !isValidNumber(pos.y) ||
    !isValidNumber(pos.z)
  ) {
    return false;
  }

  // Validate orientation
  if (!isValidObject(p.orientation)) {
    return false;
  }

  const orient = p.orientation as Partial<{ x: number; y: number; z: number; w: number }>;
  if (
    !isValidNumber(orient.x) ||
    !isValidNumber(orient.y) ||
    !isValidNumber(orient.z) ||
    !isValidNumber(orient.w)
  ) {
    return false;
  }

  // Validate timestamp if present
  if (p.timestamp !== undefined && !isValidString(p.timestamp)) {
    return false;
  }

  return true;
}

/**
 * Validate Pose3D structure
 */
export function validatePose3D(pose: unknown): pose is Pose3D {
  if (!isValidObject(pose)) {
    return false;
  }

  const p = pose as Partial<Pose3D>;

  // Validate position
  if (!isValidObject(p.position)) {
    return false;
  }

  const pos = p.position as Partial<{ x: number; y: number; z: number }>;
  if (
    !isValidNumber(pos.x) ||
    !isValidNumber(pos.y) ||
    !isValidNumber(pos.z)
  ) {
    return false;
  }

  // Validate orientation
  if (!isValidObject(p.orientation)) {
    return false;
  }

  const orient = p.orientation as Partial<{ x: number; y: number; z: number; w: number }>;
  if (
    !isValidNumber(orient.x) ||
    !isValidNumber(orient.y) ||
    !isValidNumber(orient.z) ||
    !isValidNumber(orient.w)
  ) {
    return false;
  }

  return true;
}

/**
 * Validate MapMetadata structure
 */
export function validateMapMetadata(metadata: unknown): metadata is MapMetadata {
  if (!isValidObject(metadata)) {
    return false;
  }

  const m = metadata as Partial<MapMetadata>;

  // Required fields
  if (!isValidString(m.input_file)) {
    return false;
  }

  if (!isValidNumber(m.resolution_m_per_pixel)) {
    return false;
  }

  // Validate views
  if (!isValidObject(m.views)) {
    return false;
  }

  const views = m.views as Partial<{ top?: ViewMetadata; side_x?: ViewMetadata }>;
  
  if (!validateViewMetadata(views.top)) {
    return false;
  }

  if (!validateViewMetadata(views.side_x)) {
    return false;
  }

  return true;
}

/**
 * Validate ViewMetadata structure
 */
export function validateViewMetadata(view: unknown): view is ViewMetadata {
  if (!isValidObject(view)) {
    return false;
  }

  const v = view as Partial<ViewMetadata>;

  // Required fields
  if (v.id !== 'top' && v.id !== 'side_x') {
    return false;
  }

  if (!isValidObject(v.projection)) {
    return false;
  }

  if (!isValidObject(v.bounds) || !isValidObject(v.bounds.world)) {
    return false;
  }

  if (!isValidObject(v.image)) {
    return false;
  }

  const img = v.image as Partial<{ width: number; height: number }>;
  if (!isValidNumber(img.width) || !isValidNumber(img.height)) {
    return false;
  }

  if (!isValidObject(v.processing)) {
    return false;
  }

  if (!isValidObject(v.orientation)) {
    return false;
  }

  return true;
}

/**
 * Validate RotationMetadata structure
 */
export function validateRotationMetadata(metadata: unknown): metadata is RotationMetadata {
  if (!isValidObject(metadata)) {
    return false;
  }

  const m = metadata as Partial<RotationMetadata>;

  if (!isValidObject(m.rotation_info)) {
    return false;
  }

  const ri = m.rotation_info as Partial<{
    rotation_angle_rad: number;
    rotation_angle_deg: number;
    rotation_axis: [number, number, number];
    rotation_matrix: number[][];
    method: string;
  }>;

  if (
    !isValidNumber(ri.rotation_angle_rad) ||
    !isValidNumber(ri.rotation_angle_deg) ||
    !isValidArray(ri.rotation_axis) ||
    !isValidArray(ri.rotation_matrix) ||
    !isValidString(ri.method)
  ) {
    return false;
  }

  return true;
}

/**
 * Safe array access with validation
 */
export function safeArrayAccess<T>(
  array: unknown,
  index: number
): T | null {
  if (!isValidArray(array) || index < 0 || index >= array.length) {
    return null;
  }
  return array[index] as T;
}

/**
 * Safe property access with default value
 */
export function safeGet<T>(
  obj: unknown,
  path: string,
  defaultValue: T
): T {
  if (!isValidObject(obj)) {
    return defaultValue;
  }

  const keys = path.split('.');
  let current: unknown = obj;

  for (const key of keys) {
    if (!isValidObject(current) || !(key in current)) {
      return defaultValue;
    }
    current = (current as Record<string, unknown>)[key];
  }

  return (current as T) ?? defaultValue;
}

/**
 * Validate and normalize vehicle array from API response
 */
export function validateAndNormalizeVehicles(
  vehicles: unknown
): ApiVehicle[] {
  if (!isValidArray(vehicles)) {
    return [];
  }

  return vehicles
    .filter((v): v is VehicleApi => validateVehicleApi(v))
    .map((v) => v as ApiVehicle);
}

/**
 * Create default position object
 */
export function createDefaultPosition(): { x: number; y: number; z: number } {
  return { x: 0, y: 0, z: 0 };
}

/**
 * Create default orientation object
 */
export function createDefaultOrientation(): {
  x: number;
  y: number;
  z: number;
  w: number;
} {
  return { x: 0, y: 0, z: 0, w: 1 };
}

/**
 * Validate position object
 */
export function validatePosition(
  position: unknown
): position is { x: number; y: number; z: number } {
  if (!isValidObject(position)) {
    return false;
  }

  const pos = position as Partial<{ x: number; y: number; z: number }>;
  return (
    isValidNumber(pos.x) &&
    isValidNumber(pos.y) &&
    isValidNumber(pos.z)
  );
}

/**
 * Validate orientation object
 */
export function validateOrientation(
  orientation: unknown
): orientation is { x: number; y: number; z: number; w: number } {
  if (!isValidObject(orientation)) {
    return false;
  }

  const orient = orientation as Partial<{
    x: number;
    y: number;
    z: number;
    w: number;
  }>;
  return (
    isValidNumber(orient.x) &&
    isValidNumber(orient.y) &&
    isValidNumber(orient.z) &&
    isValidNumber(orient.w)
  );
}

/**
 * Get safe vehicle status with fallback
 */
export function getSafeVehicleStatus(
  status: unknown,
  fallback: VehicleStatus = 'offline'
): VehicleStatus {
  if (status === 'online' || status === 'offline') {
    return status;
  }
  return fallback;
}

/**
 * Validate and get position with fallback
 */
export function getSafePosition(
  position: unknown,
  fallback: { x: number; y: number; z: number } = createDefaultPosition()
): { x: number; y: number; z: number } {
  if (validatePosition(position)) {
    return position;
  }
  return fallback;
}

/**
 * Validate and get orientation with fallback
 */
export function getSafeOrientation(
  orientation: unknown,
  fallback: {
    x: number;
    y: number;
    z: number;
    w: number;
  } = createDefaultOrientation()
): { x: number; y: number; z: number; w: number } {
  if (validateOrientation(orientation)) {
    return orientation;
  }
  return fallback;
}
