/**
 * Map Info Service
 * 
 * Chức năng: Xử lý các API calls liên quan đến map information
 * - Lấy thông tin bản đồ hiện tại từ storage.lidar.tm/metadata_map.json
 * 
 * Sử dụng: Trong các hook và components cần hiển thị thông tin bản đồ
 * Data Source: http://storage.lidar.tm/metadata_map.json
 */

import { MapInfo, MapInfoApiResponse } from '../../types/mapInfo';
import { formatISOToDisplay } from '../../utils/dateFormatters';

export class MapInfoService {
  constructor(private metadataUrl: string) {
    // Nhận metadata URL qua dependency injection
  }

  /**
   * Get current map information from storage
   * @returns MapInfo object with transformed data
   * @throws Error if fetch fails or data is invalid
   */
  async getCurrentMapInfo(): Promise<MapInfo> {
    try {
      const response = await fetch(this.metadataUrl
      //   , {
      //   method: 'GET',
      //   headers: {
      //     'Content-Type': 'application/json',
      //   },
      // }
    );
      
      if (!response.ok) {
        const text = await response.text();
        throw new Error(
          text || `Failed to fetch map info: HTTP ${response.status} ${response.statusText}`
        );
      }

      const data: MapInfoApiResponse = await response.json();
      return this.transformToMapInfo(data);
    } catch (error) {
      if (error instanceof Error) {
        throw error;
      }
      throw new Error(`Failed to fetch map info: ${String(error)}`);
    }
  }

  /**
   * Transform API response to MapInfo interface
   * @param data - Raw API response
   * @returns Transformed MapInfo object
   */
  private transformToMapInfo(data: MapInfoApiResponse): MapInfo {
    // Validate required fields
    if (!data.vehicle_id) {
      throw new Error('Missing required field: vehicle_id');
    }
    if (!data.map_name) {
      throw new Error('Missing required field: map_name');
    }
    if (typeof data.size_bytes !== 'number') {
      throw new Error('Invalid size_bytes: must be a number');
    }

    const nowFormatted = formatISOToDisplay(new Date().toISOString());

    return {
      id: data.vehicle_id || 'unknown',
      name: data.map_name || 'Unknown Map',
      createdAt: data.created_at ? formatISOToDisplay(data.created_at) : nowFormatted,
      uploadedBy: data.vehicle_name || 'Unknown',
      uploadedAt: data.uploaded_at ? formatISOToDisplay(data.uploaded_at) : nowFormatted,
      fileSize: data.size_bytes || 0,
    };
  }
}
