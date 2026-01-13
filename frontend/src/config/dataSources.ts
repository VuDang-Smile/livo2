/**
 * Cấu hình nguồn dữ liệu thực tế cho PCD & floorplan 2D
 * Tất cả URL hardcode nên tập trung tại đây để dễ bảo trì.
 */

/**
 * URL PCD map đã merge từ storage thực tế
 */
export const PCD_SOURCE_URL =
  'http://storage.lidar.tm/merged_map/merged_all.pcd';

/**
 * URL metadata JSON cho floorplan 2D (top/side_x/side_y)
 */
export const MAP_FLOORPLAN_METADATA_URL =
  'http://storage.lidar.tm/floorplan_2d/merge_all_hba_metadata.json';

/**
 * URL metadata JSON cho thông tin bản đồ hiện tại (map info)
 */
export const MAP_INFO_URL =
  'http://storage.lidar.tm/metadata_map.json';

/**
 * URL ảnh floorplan 2D cho từng view
 */
export const MAP_FLOORPLAN_IMAGE_URLS: Record<
  'top' | 'side_x' | 'side_y',
  string
> = {
  top: 'http://storage.lidar.tm/floorplan_2d/merge_all_hba_top.png',
  side_x: 'http://storage.lidar.tm/floorplan_2d/merge_all_hba_side_x.png',
  side_y: 'http://storage.lidar.tm/floorplan_2d/merge_all_hba_side_y.png',
};

/**
 * Get backend API base URL based on current hostname
 * Tái sử dụng logic từ mapConfig.ts để đảm bảo consistency
 */
export function getBackendApiUrl(): string {
  const hostname = window.location.hostname;
  // If running on frontend.lidar.tm or lidar.tm, use backend.lidar.tm
  if (hostname === 'frontend.lidar.tm' || hostname === 'lidar.tm' || hostname.includes('lidar.tm')) {
    return 'http://backend.lidar.tm';
  }
  // For localhost or IP addresses, use same hostname with port 8000
  return `http://${hostname}:8000`;
}
