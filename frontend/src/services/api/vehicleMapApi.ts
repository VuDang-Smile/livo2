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

import { Vehicle, VehicleMapResponse } from '../../types/vehicleMap2D';
import { normalizeTimestamp } from '../../utils/timestampHelpers';

// Re-export types for backward compatibility
export type { Vehicle, VehicleMapResponse };

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
  async getVehiclesWithPositions(): Promise<Vehicle[]> {
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
    
    const vehicles = await res.json();
    
    // Transform API response to Vehicle format with position data
    return vehicles.map((vehicle: any) => ({
      id: vehicle.vehicle_id,
      name: vehicle.name,
      type: vehicle.type_category || vehicle.type, // Backend uses type_category
      status: vehicle.status,
      position: vehicle.current_pose?.position ? {
        x: vehicle.current_pose.position.x,
        y: vehicle.current_pose.position.y,
        z: vehicle.current_pose.position.z
      } : vehicle.current_position ? {
        x: vehicle.current_position.x,
        y: vehicle.current_position.y,
        z: vehicle.current_position.z
      } : {
        x: 0,
        y: 0,
        z: 0
      },
      timestamp: normalizeTimestamp(
        vehicle.current_pose?.timestamp || vehicle.current_position?.timestamp,
        true
      ) as string
    }));
  }
}
