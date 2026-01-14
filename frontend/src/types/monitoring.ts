/**
 * Return types for monitoring hooks
 */

import { VehicleMarker3D } from './vehicle';
import { VehicleMarker2D } from './vehicle';
import { MapMetadata } from './mapMetadata';
import { VehicleMapVehicle, VehicleMapResponse } from './vehicle';

/**
 * Return type for useVehicleMarkers3D hook
 */
export interface UseVehicleMarkers3DResult {
  vehicleMarkers: VehicleMarker3D[];
  loading: boolean;
  error: string | null;
  fetchVehicles: () => Promise<void>;
}

/**
 * Return type for useVehiclePose2D hook
 */
export interface UseVehiclePose2DResult {
  vehicleMarkers: VehicleMarker2D[];
  mapMetadata: MapMetadata | null;
  isLoading: boolean;
  error: string | null;
  refreshMetadata: () => Promise<void>;
}

/**
 * Return type for useVehicleMap2D hook
 */
export interface UseVehicleMap2DResult {
  mapData: VehicleMapResponse | null;
  mapVehicles: VehicleMapVehicle[];
  filteredVehicles: VehicleMapVehicle[];
  loading: boolean;
  error: string;
  loadMapData: () => Promise<void>;
}
