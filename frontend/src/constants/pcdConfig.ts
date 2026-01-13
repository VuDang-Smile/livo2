import { PCD_SOURCE_URL } from '../config/dataSources';

/**
 * Cấu hình URL cho file PCD mặc định
 * Sử dụng nguồn dữ liệu thực tế từ storage
 */
export const DEFAULT_PCD_URL = PCD_SOURCE_URL;

/**
 * Cấu hình hiển thị point cloud (PCD)
 * Tăng point size và tắt size attenuation để điểm nhìn rõ ngay từ xa.
 */
export const PCD_POINT_SIZE = 1.5;
export const PCD_POINT_SIZE_ATTENUATION = false;

/**
 * Cấu hình camera/OrbitControls mặc định cho MapView3D
 * Đặt camera gần hơn để thấy cloud ngay sau khi load.
 */
export const PCD_CAMERA_POSITION: [number, number, number] = [0, 30, 60];
export const PCD_CAMERA_FOV = 50;
export const PCD_ORBIT_MIN_DISTANCE = 5;
export const PCD_ORBIT_MAX_DISTANCE = 200;


