/**
 * Type definitions for QR Code related data structures
 */

/**
 * QR Code information for display and management
 */
export interface QRCodeInfo {
  id: string;
  code: string;
  position: [number, number]; // [x, z] for 2D map
  isManual?: boolean; // Đánh dấu QR code được thêm thủ công
}

/**
 * Raw response from QR_detect.json API
 * Format: { "TM:0001": [x, y, z], "TM:0002": [x, y, z], ... }
 */
export interface QRCodeApiResponse {
  [code: string]: [number, number, number]; // [x, y, z]
}
