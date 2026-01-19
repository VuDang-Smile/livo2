import { RotationMetadata, Pose3D } from '../types/mapMetadata';

/**
 * URL để load rotation metadata từ storage
 */
const ROTATION_METADATA_URL = 'http://storage.lidar.tm/merged_map/merged_all_rotated_rotation_metadata.json';

/**
 * Cache cho rotation metadata để tránh fetch nhiều lần
 */
let rotationMetadataCache: RotationMetadata | null = null;
let rotationMetadataPromise: Promise<RotationMetadata | null> | null = null;

/**
 * Load rotation metadata từ storage
 * Sử dụng cache để tránh fetch nhiều lần
 * 
 * @returns Rotation metadata hoặc null nếu không tìm thấy hoặc có lỗi
 */
export async function loadRotationMetadata(): Promise<RotationMetadata | null> {
  // Nếu đã có cache, trả về ngay
  if (rotationMetadataCache !== null) {
    return rotationMetadataCache;
  }

  // Nếu đang fetch, đợi kết quả
  if (rotationMetadataPromise !== null) {
    return rotationMetadataPromise;
  }

  // Bắt đầu fetch mới
  rotationMetadataPromise = (async () => {
    try {
      const response = await fetch(ROTATION_METADATA_URL);
      
      if (!response.ok) {
        console.warn('⚠️ [rotationUtils] Failed to load rotation metadata:', response.statusText);
        return null;
      }

      const data: RotationMetadata = await response.json();
      
      // Validate structure
      if (!data.rotation_info || !data.rotation_info.rotation_matrix) {
        console.warn('⚠️ [rotationUtils] Invalid rotation metadata structure');
        return null;
      }

      // Cache kết quả
      rotationMetadataCache = data;
      console.log('✅ [rotationUtils] Loaded rotation metadata:', {
        angle_deg: data.rotation_info.rotation_angle_deg,
        axis: data.rotation_info.rotation_axis
      });

      return data;
    } catch (error) {
      console.warn('⚠️ [rotationUtils] Error loading rotation metadata:', error);
      return null;
    } finally {
      // Clear promise để có thể fetch lại nếu cần
      rotationMetadataPromise = null;
    }
  })();

  return rotationMetadataPromise;
}

/**
 * Lấy rotation matrix 3x3 từ metadata (bỏ hàng/cột cuối của matrix 4x4)
 * 
 * @param metadata - Rotation metadata
 * @returns Rotation matrix 3x3 hoặc null nếu không hợp lệ
 */
export function getRotationMatrix(metadata: RotationMetadata | null): number[][] | null {
  if (!metadata || !metadata.rotation_info || !metadata.rotation_info.rotation_matrix) {
    return null;
  }

  const matrix4x4 = metadata.rotation_info.rotation_matrix;
  
  // Validate matrix size
  if (matrix4x4.length !== 4 || matrix4x4.some(row => row.length !== 4)) {
    console.warn('⚠️ [rotationUtils] Invalid rotation matrix size');
    return null;
  }

  // Extract 3x3 rotation matrix (bỏ hàng/cột cuối)
  return [
    [matrix4x4[0][0], matrix4x4[0][1], matrix4x4[0][2]],
    [matrix4x4[1][0], matrix4x4[1][1], matrix4x4[1][2]],
    [matrix4x4[2][0], matrix4x4[2][1], matrix4x4[2][2]]
  ];
}

/**
 * Nhân rotation matrix 3x3 với vector 3D
 * 
 * @param matrix - Rotation matrix 3x3
 * @param vector - Vector [x, y, z]
 * @returns Rotated vector [x, y, z]
 */
function multiplyMatrixVector(matrix: number[][], vector: [number, number, number]): [number, number, number] {
  const [x, y, z] = vector;
  return [
    matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z,
    matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z,
    matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z
  ];
}

/**
 * Convert quaternion sang rotation matrix 3x3
 * 
 * @param q - Quaternion {x, y, z, w}
 * @returns Rotation matrix 3x3
 */
function quaternionToMatrix(q: { x: number; y: number; z: number; w: number }): number[][] {
  const { x, y, z, w } = q;
  
  return [
    [
      1 - 2 * (y * y + z * z),
      2 * (x * y - w * z),
      2 * (x * z + w * y)
    ],
    [
      2 * (x * y + w * z),
      1 - 2 * (x * x + z * z),
      2 * (y * z - w * x)
    ],
    [
      2 * (x * z - w * y),
      2 * (y * z + w * x),
      1 - 2 * (x * x + y * y)
    ]
  ];
}

/**
 * Nhân hai rotation matrix 3x3
 * 
 * @param a - Matrix 3x3
 * @param b - Matrix 3x3
 * @returns Result matrix 3x3
 */
function multiplyMatrices(a: number[][], b: number[][]): number[][] {
  const result: number[][] = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      for (let k = 0; k < 3; k++) {
        result[i][j] += a[i][k] * b[k][j];
      }
    }
  }
  
  return result;
}

/**
 * Convert rotation matrix 3x3 sang quaternion
 * 
 * @param matrix - Rotation matrix 3x3
 * @returns Quaternion {x, y, z, w}
 */
function matrixToQuaternion(matrix: number[][]): { x: number; y: number; z: number; w: number } {
  const m = matrix;
  const trace = m[0][0] + m[1][1] + m[2][2];
  
  let x: number, y: number, z: number, w: number;
  
  if (trace > 0) {
    const s = Math.sqrt(trace + 1.0) * 2; // s = 4 * qw
    w = 0.25 * s;
    x = (m[2][1] - m[1][2]) / s;
    y = (m[0][2] - m[2][0]) / s;
    z = (m[1][0] - m[0][1]) / s;
  } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
    const s = Math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2; // s = 4 * qx
    w = (m[2][1] - m[1][2]) / s;
    x = 0.25 * s;
    y = (m[0][1] + m[1][0]) / s;
    z = (m[0][2] + m[2][0]) / s;
  } else if (m[1][1] > m[2][2]) {
    const s = Math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2; // s = 4 * qy
    w = (m[0][2] - m[2][0]) / s;
    x = (m[0][1] + m[1][0]) / s;
    y = 0.25 * s;
    z = (m[1][2] + m[2][1]) / s;
  } else {
    const s = Math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2; // s = 4 * qz
    w = (m[1][0] - m[0][1]) / s;
    x = (m[0][2] + m[2][0]) / s;
    y = (m[1][2] + m[2][1]) / s;
    z = 0.25 * s;
  }
  
  return { x, y, z, w };
}

/**
 * Áp dụng rotation vào pose (cả position và orientation)
 * 
 * @param pose - Pose gốc từ MQTT/API (world coordinates, chưa xoay)
 * @param rotationMatrix - Rotation matrix 3x3 từ metadata
 * @returns Pose đã được xoay
 */
export function applyRotationToPose(
  pose: Pose3D,
  rotationMatrix: number[][] | null
): Pose3D {
  // Nếu không có rotation matrix, trả về pose gốc
  if (!rotationMatrix) {
    return pose;
  }

  // Áp dụng rotation vào position
  const rotatedPosition = multiplyMatrixVector(rotationMatrix, [
    pose.position.x,
    pose.position.y,
    pose.position.z
  ]);

  // Áp dụng rotation vào orientation (quaternion)
  // Convert quaternion → rotation matrix → nhân với rotation matrix → convert lại về quaternion
  const poseMatrix = quaternionToMatrix(pose.orientation);
  const combinedMatrix = multiplyMatrices(rotationMatrix, poseMatrix);
  const rotatedOrientation = matrixToQuaternion(combinedMatrix);

  return {
    ...pose,
    position: {
      x: rotatedPosition[0],
      y: rotatedPosition[1],
      z: rotatedPosition[2]
    },
    orientation: rotatedOrientation
  };
}

/**
 * Áp dụng rotation vào position vector (không có orientation)
 * 
 * @param position - Position vector [x, y, z]
 * @param rotationMatrix - Rotation matrix 3x3 từ metadata
 * @returns Rotated position [x, y, z]
 */
export function applyRotationToPosition(
  position: [number, number, number],
  rotationMatrix: number[][] | null
): [number, number, number] {
  if (!rotationMatrix) {
    return position;
  }

  return multiplyMatrixVector(rotationMatrix, position);
}

/**
 * Áp dụng rotation vào quaternion orientation
 * 
 * @param orientation - Quaternion {x, y, z, w}
 * @param rotationMatrix - Rotation matrix 3x3 từ metadata
 * @returns Rotated quaternion {x, y, z, w}
 */
export function applyRotationToOrientation(
  orientation: { x: number; y: number; z: number; w: number },
  rotationMatrix: number[][] | null
): { x: number; y: number; z: number; w: number } {
  if (!rotationMatrix) {
    return orientation;
  }

  const poseMatrix = quaternionToMatrix(orientation);
  const combinedMatrix = multiplyMatrices(rotationMatrix, poseMatrix);
  return matrixToQuaternion(combinedMatrix);
}

/**
 * Tính transpose của rotation matrix 3x3 (inverse cho orthogonal matrix)
 * 
 * @param matrix - Rotation matrix 3x3
 * @returns Transpose matrix 3x3
 */
function transposeMatrix(matrix: number[][]): number[][] {
  return [
    [matrix[0][0], matrix[1][0], matrix[2][0]],
    [matrix[0][1], matrix[1][1], matrix[2][1]],
    [matrix[0][2], matrix[1][2], matrix[2][2]]
  ];
}

/**
 * Đảo ngược rotation vào position vector (unrotate)
 * Dùng khi lưu QR code mới được pick trên bản đồ đã xoay
 * 
 * @param position - Position vector [x, y, z] (world coordinates, đã xoay)
 * @param rotationMatrix - Rotation matrix 3x3 từ metadata
 * @returns Unrotated position [x, y, z] (world coordinates, chưa xoay)
 */
export function applyInverseRotationToPosition(
  position: [number, number, number],
  rotationMatrix: number[][] | null
): [number, number, number] {
  if (!rotationMatrix) {
    return position;
  }

  // Inverse rotation matrix = transpose (vì rotation matrix là orthogonal)
  const inverseMatrix = transposeMatrix(rotationMatrix);
  return multiplyMatrixVector(inverseMatrix, position);
}
