/**
 * Cấu hình cho bản đồ 2D
 * Bao gồm URL của hình ảnh map và world bounds để mapping tọa độ
 */

export const MAP_2D_IMAGE_URL = '/2Dmap.png';

/**
 * World bounds cho mapping tọa độ từ 3D world space sang 2D canvas
 * Các giá trị này phải khớp với logic mapping trong Map2D và Image2DPreview components
 */
export const MAP_WORLD_BOUNDS = {
  minX: -100,
  maxX: 100,
  minZ: -50,
  maxZ: 50,
  width: 200,  // maxX - minX
  height: 100, // maxZ - minZ
} as const;

