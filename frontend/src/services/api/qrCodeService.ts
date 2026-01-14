/**
 * QR Code Service
 * 
 * Chức năng: Xử lý các API calls liên quan đến QR codes
 * - Lấy danh sách QR codes từ storage.lidar.tm/QR_detect.json
 * 
 * Sử dụng: Trong các hook và components cần hiển thị QR codes
 * Data Source: http://storage.lidar.tm/QR_detect.json
 */

import { QRCodeInfo, QRCodeApiResponse } from '../../types/qrCode';

export class QRCodeService {
  constructor(private qrDetectUrl: string) {
    // Nhận QR detect URL qua dependency injection
  }

  /**
   * Get QR codes from storage
   * @returns Array of QRCodeInfo objects with transformed data
   * @throws Error if fetch fails or data is invalid
   */
  async getQRCodes(): Promise<QRCodeInfo[]> {
    try {
      const response = await fetch(this.qrDetectUrl);
      
      if (!response.ok) {
        const text = await response.text();
        throw new Error(
          text || `Failed to fetch QR codes: HTTP ${response.status} ${response.statusText}`
        );
      }

      const data: QRCodeApiResponse = await response.json();
      return this.transformToQRCodeInfoArray(data);
    } catch (error) {
      if (error instanceof Error) {
        throw error;
      }
      throw new Error(`Failed to fetch QR codes: ${String(error)}`);
    }
  }

  /**
   * Generate unique ID from QR code string
   * @param code - QR code string (e.g., "TM:0001")
   * @returns Generated ID (e.g., "qr-TM-0001")
   */
  private generateId(code: string): string {
    // Replace colons and other special chars with hyphens
    return `qr-${code.replace(/[:]/g, '-')}`;
  }

  /**
   * Transform API response to QRCodeInfo array
   * @param data - Raw API response object
   * @returns Array of transformed QRCodeInfo objects
   */
  private transformToQRCodeInfoArray(data: QRCodeApiResponse): QRCodeInfo[] {
    // Handle empty response
    if (!data || typeof data !== 'object') {
      return [];
    }

    const qrCodes: QRCodeInfo[] = [];

    // Iterate through object entries
    for (const [code, position] of Object.entries(data)) {
      // Validate code
      if (!code || typeof code !== 'string') {
        console.warn(`[QRCodeService] Invalid QR code key: ${code}`);
        continue;
      }

      // Validate position array
      if (!Array.isArray(position) || position.length < 2) {
        console.warn(`[QRCodeService] Invalid position for QR code ${code}:`, position);
        continue;
      }

      const [x, y, z] = position;

      // Validate coordinates are numbers
      if (typeof x !== 'number' || typeof z !== 'number' || !isFinite(x) || !isFinite(z)) {
        console.warn(`[QRCodeService] Invalid coordinates for QR code ${code}:`, position);
        continue;
      }

      // Transform: use [x, z] for 2D map (ignore y)
      qrCodes.push({
        id: this.generateId(code),
        code: code,
        position: [x, z],
        isManual: false, // All QR codes from API are not manual
      });
    }

    return qrCodes;
  }
}
