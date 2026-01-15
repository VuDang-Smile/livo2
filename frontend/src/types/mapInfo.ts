/**
 * Type definitions for Map Information
 */

/**
 * Map information structure
 */
export interface MapInfo {
  id: string;
  name: string;           // Tên map
  createdAt: string;      // Ngày tạo (format: YYYY-MM-DD HH:mm:ss)
  uploadedBy: string;     // Người tải lên
  uploadedAt: string;     // Ngày tải lên (format: YYYY-MM-DD HH:mm:ss)
  fileSize: number;       // Dung lượng (bytes)
}

/**
 * Raw response from metadata_map.json API
 */
export interface MapInfoApiResponse {
  map_name: string;
  vehicle_name: string;
  vehicle_id: string;
  size_bytes: number;
  size_mb?: number;
  created_at: string; // ISO 8601 format
  uploaded_at: string; // ISO 8601 format
}
