/**
 * Type definitions for QR Code related data structures
 */

/**
 * QR Code information for display and management
 */
export interface QRCodeInfo {
  id: string;
  code: string;
  /**
   * Vị trí 2D hiển thị (legacy: [x, z] khi chưa dùng metadata)
   */
  position: [number, number];
  /**
   * Vị trí 3D gốc từ data source (x, y, z) để transform chính xác bằng metadata
   */
  position3D?: [number, number, number];
  /**
   * Surface mà QR code được gắn vào (tunnel surface: floor, ceiling, left, right)
   */
  surface?: 'floor' | 'ceiling' | 'left' | 'right';
  isManual?: boolean; // Đánh dấu QR code được thêm thủ công
}

/**
 * Raw response from QR_detect.json API
 * Format: { "TM:0001": [x, y, z], "TM:0002": [x, y, z], ... }
 */
export interface QRCodeApiResponse {
  [code: string]: [number, number, number]; // [x, y, z]
}
