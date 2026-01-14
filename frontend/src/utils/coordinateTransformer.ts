import { CoordinateSystemConfig } from '../types/mapMetadata';

/**
 * Transform coordinates từ world frame (ROS / RViz convention) sang Three.js scene frame.
 * Hàm này cố gắng đọc cấu hình hệ trục từ metadata, nếu không có sẽ dùng mặc định ROS.
 *
 * World position luôn hiểu là [x, y, z] theo frame map/odom/backend.
 */
export function transformWorldToScene(
  position: [number, number, number],
  config?: CoordinateSystemConfig
): [number, number, number] {
  const [x, y, z] = position;

  // Nếu không có config, giả định dữ liệu ở chuẩn ROS
  if (!config || config.convention === 'ros') {
    // ROS (X forward, Y left, Z up) + PCD xoay -90° quanh X:
    //   sceneX = worldX
    //   sceneY = worldZ
    //   sceneZ = -worldY
    return [x, z, -y];
  }

  // Nếu dữ liệu đã ở chuẩn Three.js thì không cần đổi trục
  if (config.convention === 'threejs') {
    return position;
  }

  // Trường hợp custom: ưu tiên axisMapping + axisSigns
  if (config.convention === 'custom' && config.transform) {
    const { axisMapping, axisSigns } = config.transform;

    if (axisMapping) {
      const getAxis = (axis: 'X' | 'Y' | 'Z'): number => {
        switch (axis) {
          case 'X':
            return x;
          case 'Y':
            return y;
          case 'Z':
            return z;
        }
      };

      const [sx, sy, sz] = axisSigns ?? [1, 1, 1];

      return [
        getAxis(axisMapping.sceneX) * sx,
        getAxis(axisMapping.sceneY) * sy,
        getAxis(axisMapping.sceneZ) * sz,
      ];
    }
  }

  // Fallback: giữ hành vi cũ (ROS)
  return [x, z, -y];
}

/**
 * Transform coordinates từ Three.js scene frame về world frame (ROS / RViz convention).
 * Dùng khi cần convert ngược (ví dụ pick điểm trên scene để gửi về backend).
 */
export function transformSceneToWorld(
  position: [number, number, number],
  config?: CoordinateSystemConfig
): [number, number, number] {
  const [sx, sy, sz] = position;

  if (!config || config.convention === 'ros') {
    // Nghịch đảo của [x, y, z] -> [x, z, -y]:
    //   sx = x
    //   sy = z
    //   sz = -y
    // => x = sx, y = -sz, z = sy
    return [sx, -sz, sy];
  }

  if (config.convention === 'threejs') {
    return position;
  }

  // Với custom hiện tại chưa cần dùng chiều ngược phức tạp, giữ nguyên
  return [sx, -sz, sy];
}

/**
 * Tính rotation Euler cho component PCDMap dựa trên cấu hình hệ trục.
 * Trả về [rx, ry, rz] theo radian.
 */
export function getPCDRotation(config?: CoordinateSystemConfig): [number, number, number] {
  // Mặc định dữ liệu ở chuẩn ROS nên cần xoay -90° quanh trục X
  if (!config || config.convention === 'ros') {
    return [-Math.PI / 2, 0, 0];
  }

  // Nếu dữ liệu đã ở chuẩn Three.js thì không cần xoay
  if (config.convention === 'threejs') {
    return [0, 0, 0];
  }

  // Nếu là custom và có rotation khai báo sẵn thì dùng luôn
  if (config.convention === 'custom' && config.transform?.rotation) {
    return config.transform.rotation;
  }

  // Fallback: giữ giống ROS
  return [-Math.PI / 2, 0, 0];
}


