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
import { formatISOToLocalDisplay } from '../../utils/dateFormatters';
import { isValidObject, isValidString, isValidNumber } from '../../utils/validationUtils';

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

      const data = await response.json();
      
      // Validate response is not null/undefined
      if (data === null || data === undefined) {
        throw new Error('API returned null or undefined response');
      }
      
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
  private transformToMapInfo(data: unknown): MapInfo {
    // Validate data is an object
    if (!isValidObject(data)) {
      throw new Error('Invalid data structure: expected an object');
    }

    const d = data as Partial<MapInfoApiResponse>;

    // Validate required fields
    if (!isValidString(d.vehicle_id)) {
      throw new Error('Missing or invalid required field: vehicle_id');
    }
    
    if (!isValidString(d.map_name)) {
      throw new Error('Missing or invalid required field: map_name');
    }
    
    if (!isValidNumber(d.size_bytes)) {
      throw new Error('Invalid size_bytes: must be a valid number');
    }

    const nowFormatted = formatISOToLocalDisplay(new Date().toISOString());

    // Safely extract optional fields
    const created_at = typeof d.created_at === 'string' ? d.created_at : null;
    const uploaded_at = typeof d.uploaded_at === 'string' ? d.uploaded_at : null;
    const vehicle_name = typeof d.vehicle_name === 'string' ? d.vehicle_name : null;

    return {
      id: d.vehicle_id,
      name: d.map_name,
      createdAt: created_at ? formatISOToLocalDisplay(created_at) : nowFormatted,
      uploadedBy: vehicle_name ?? 'Unknown',
      uploadedAt: uploaded_at ? formatISOToLocalDisplay(uploaded_at) : nowFormatted,
      fileSize: d.size_bytes,
    };
  }
}
