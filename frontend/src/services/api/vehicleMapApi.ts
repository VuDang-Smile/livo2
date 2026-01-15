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

import { VehicleMapVehicle, VehicleMapResponse } from '../../types/vehicle';
import { normalizeTimestamp } from '../../utils/timestampHelpers';

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

    const pcdList = (await pcdRes.json()) as any[];
    if (!pcdList || pcdList.length === 0) {
      throw new Error('No PCD files available to build 2D map');
    }

    const latestPcd = pcdList[0];
    const pcdFileId = latestPcd?.file_id;

    if (!pcdFileId) {
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

    const floorplans = (await floorplanRes.json()) as any[];
    if (!floorplans || floorplans.length === 0) {
      throw new Error('No floorplan available for latest PCD');
    }

    const floorplan = floorplans[0];

    // Bước 3: Build VehicleMapResponse cho 2D viewer
    const modelUrl = `${this.apiBaseUrl}/api/v1/maps/${floorplan.file_id}/download`;

    const response: VehicleMapResponse = {
      modelUrl,
      unit: 'meters',
      mapType: '2d',
      source: 'converted',
      pcd_file_id: floorplan.pcd_file_id || pcdFileId,
    };

    return response;
  }

  /**
   * Get vehicles with real-time position updates
   */
  async getVehiclesWithPositions(): Promise<VehicleMapVehicle[]> {
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

    // Backend hiện trả VehiclesListResponse { vehicles, total }
    const vehicles = Array.isArray(data.vehicles) ? data.vehicles : data;

    // Transform API response to Vehicle format with position data (dựa trên latest_pose)
    return vehicles.map((vehicle: any): VehicleMapVehicle => {
      const latest = vehicle.latest_pose;
      const position = latest?.position || { x: 0, y: 0, z: 0 };
      const rawTimestamp = latest?.timestamp || vehicle.updated_at || vehicle.created_at;

      return {
        id: vehicle.vehicle_id,
        name: vehicle.name,
        vehicleType: vehicle.vehicle_type,
        vehicleCategory: vehicle.vehicle_category,
        status: vehicle.status,
        position,
        timestamp: normalizeTimestamp(rawTimestamp, true) as string,
      };
    });
  }
}
