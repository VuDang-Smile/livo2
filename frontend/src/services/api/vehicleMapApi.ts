/**
 * Vehicle Map Service
 * 
 * Chức năng: Xử lý các API calls liên quan đến vehicle maps và positions
 * - Lấy latest 2D map từ GLB model
 * - Lấy vehicles với real-time position updates
 * 
 * Sử dụng: Trong các hooks và components cần vehicle map data
 * API Base: /api/v1/pcd và /api/v1/vehicles
 */

import { MapVehicle, VehicleMapResponse } from '../../types/vehicle';
import { normalizeTimestamp } from '../../utils/timestampHelpers';
import {
  isValidArray,
  isValidObject,
  isValidString,
  safeArrayAccess,
  getSafePosition,
  getSafeVehicleStatus,
} from '../../utils/validationUtils';

export class VehicleMapService {
  constructor(private apiBaseUrl: string) {
    // Nhận API base URL qua dependency injection
  }

  /**
   * Get latest 2D map based on the most recent PCD & its floorplan.
   *
   * Flow:
   * - Gọi /api/v1/pcd?limit=1 để lấy PCD mới nhất
   * - Gọi /api/v1/maps/pcd/{pcd_file_id}/floorplan để lấy floorplan mới nhất của PCD đó
   * - Build VehicleMapResponse để frontend 2D sử dụng
   */
  async getLatest2DMap(): Promise<VehicleMapResponse> {
    // Bước 1: Lấy PCD mới nhất
    const pcdRes = await fetch(`${this.apiBaseUrl}/api/v1/pcd?limit=1`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!pcdRes.ok) {
      const text = await pcdRes.text();
      throw new Error(text || `Failed to get latest PCD with status ${pcdRes.status}`);
    }

    const pcdListData = await pcdRes.json();
    
    // Validate pcdList is an array
    if (!isValidArray(pcdListData) || pcdListData.length === 0) {
      throw new Error('No PCD files available to build 2D map');
    }

    // Safely access first element
    const latestPcd = safeArrayAccess<Record<string, unknown>>(pcdListData, 0);
    
    if (!latestPcd || !isValidObject(latestPcd)) {
      throw new Error('Invalid PCD record structure');
    }

    const pcdFileId = latestPcd.file_id;

    // Validate file_id is a string
    if (!isValidString(pcdFileId)) {
      throw new Error('Latest PCD record does not contain a valid file_id');
    }

    // Bước 2: Lấy floorplan cho PCD này
    const floorplanRes = await fetch(
      `${this.apiBaseUrl}/api/v1/maps/pcd/${pcdFileId}/floorplan`,
      {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      }
    );

    if (!floorplanRes.ok) {
      const text = await floorplanRes.text();
      throw new Error(
        text ||
          `Failed to get floorplan for latest PCD with status ${floorplanRes.status}`
      );
    }

    const floorplansData = await floorplanRes.json();
    
    // Validate floorplans is an array
    if (!isValidArray(floorplansData) || floorplansData.length === 0) {
      throw new Error('No floorplan available for latest PCD');
    }

    // Safely access first element
    const floorplan = safeArrayAccess<Record<string, unknown>>(floorplansData, 0);
    
    if (!floorplan || !isValidObject(floorplan)) {
      throw new Error('Invalid floorplan record structure');
    }

    // Validate file_id
    const floorplanFileId = floorplan.file_id;
    if (!isValidString(floorplanFileId)) {
      throw new Error('Floorplan record does not contain a valid file_id');
    }

    // Bước 3: Build VehicleMapResponse cho 2D viewer
    const modelUrl = `${this.apiBaseUrl}/api/v1/maps/${floorplanFileId}/download`;

    const floorplanPcdFileId = floorplan.pcd_file_id;
    const response: VehicleMapResponse = {
      modelUrl,
      unit: 'meters',
      mapType: '2d',
      source: 'converted',
      pcd_file_id: isValidString(floorplanPcdFileId) ? floorplanPcdFileId : pcdFileId,
    };

    return response;
  }

  /**
   * Get vehicles with real-time position updates
   */
  async getVehiclesWithPositions(): Promise<MapVehicle[]> {
    const res = await fetch(`${this.apiBaseUrl}/api/v1/vehicles/`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
      },
    });
    
    if (!res.ok) {
      const text = await res.text();
      throw new Error(text || `Failed to get vehicles with status ${res.status}`);
    }
    
    const data = await res.json();

    // Validate response is not null/undefined
    if (data === null || data === undefined) {
      throw new Error('API returned null or undefined response');
    }

    // Backend hiện trả VehiclesListResponse { vehicles, total }
    let vehicles: unknown[] = [];
    
    if (isValidObject(data) && isValidArray(data.vehicles)) {
      vehicles = data.vehicles;
    } else if (isValidArray(data)) {
      vehicles = data;
    } else {
      console.warn('[VehicleMapService] Invalid vehicles response structure, returning empty array');
      return [];
    }

    // Transform API response to MapVehicle format with position data (dựa trên latest_pose)
    return vehicles
      .filter((vehicle): vehicle is Record<string, unknown> => isValidObject(vehicle))
      .map((vehicle: Record<string, unknown>): MapVehicle => {
        const vehicleId = vehicle.vehicle_id;
        if (!isValidString(vehicleId)) {
          throw new Error('Vehicle missing required vehicle_id field');
        }

      const latest = vehicle.latest_pose;
        const position = isValidObject(latest) && isValidObject(latest.position)
          ? getSafePosition(latest.position)
          : getSafePosition(undefined);

        const rawTimestamp = 
          (isValidObject(latest) && typeof latest.timestamp === 'string' ? latest.timestamp : null) ??
          (typeof vehicle.updated_at === 'string' ? vehicle.updated_at : null) ??
          (typeof vehicle.created_at === 'string' ? vehicle.created_at : null) ??
          new Date().toISOString();

        const status = getSafeVehicleStatus(vehicle.status);

      return {
          id: vehicleId,
          name: typeof vehicle.name === 'string' ? vehicle.name : vehicleId,
          vehicleType: typeof vehicle.vehicle_type === 'string' ? vehicle.vehicle_type : undefined,
          vehicleCategory: vehicle.vehicle_category as MapVehicle['vehicleCategory'] | undefined,
          status,
        position,
        timestamp: normalizeTimestamp(rawTimestamp, true) as string,
      };
    });
  }
}
