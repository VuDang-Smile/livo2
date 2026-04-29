/**
 * Cấu hình nguồn dữ liệu thực tế cho PCD & floorplan 2D
 * Tất cả URL hardcode nên tập trung tại đây để dễ bảo trì.
 */

/**
 * URL PCD map đã merge từ storage thực tế
 */
export const PCD_SOURCE_URL =
  'http://storage.lidar.tm/merged_map/merged_all_rotated.pcd';

/**
 * URL metadata JSON cho floorplan 2D (top/side_x)
 */
export const MAP_FLOORPLAN_METADATA_URL =
  'http://storage.lidar.tm/floorplan_2d/merged_all_rotated_metadata.json';

/**
 * URL metadata JSON cho thông tin bản đồ hiện tại (map info)
 */
export const MAP_INFO_URL =
  'http://storage.lidar.tm/metadata_map.json';

/**
 * URL JSON cho danh sách QR codes được detect
 */
export const QR_DETECT_URL =
  'http://storage.lidar.tm/QR_detect.json';

/**
 * URL ảnh floorplan 2D cho từng view
 */
export const MAP_FLOORPLAN_IMAGE_URLS: Record<
  'top' | 'side_x',
  string
> = {
  top: 'http://storage.lidar.tm/floorplan_2d/merged_all_rotated_top.png',
  side_x: 'http://storage.lidar.tm/floorplan_2d/merged_all_rotated_side_x.png',
};

/**
 * Get backend API base URL based on current hostname
 * Tái sử dụng logic từ mapConfig.ts để đảm bảo consistency
 */
export function getBackendApiUrl(): string {
  return 'http://192.168.2.1:20128';
}
