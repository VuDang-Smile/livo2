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
import { isValidObject, isValidArray, isValidNumber } from '../../utils/validationUtils';

export class QRCodeService {
  constructor(private qrDetectUrl: string) {
    // Nhận QR detect URL qua dependency injection
  }

  /**
   * Get QR codes from storage
   * @param signal - Optional AbortSignal to cancel the request
   * @returns Array of QRCodeInfo objects with transformed data
   * @throws Error if fetch fails or data is invalid
   */
  async getQRCodes(signal?: AbortSignal): Promise<QRCodeInfo[]> {
    try {
      const response = await fetch(this.qrDetectUrl, {
        signal, // Pass abort signal to fetch
      });
      
      if (!response.ok) {
        const text = await response.text();
        throw new Error(
          text || `Failed to fetch QR codes: HTTP ${response.status} ${response.statusText}`
        );
      }

      const data = await response.json();
      
      // Validate response is not null/undefined
      if (data === null || data === undefined) {
        throw new Error('API returned null or undefined response');
      }
      
      return this.transformToQRCodeInfoArray(data);
    } catch (error) {
      // Handle abort errors gracefully
      if (error instanceof Error && error.name === 'AbortError') {
        throw error; // Re-throw abort errors as-is
      }
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
  private transformToQRCodeInfoArray(data: unknown): QRCodeInfo[] {
    // Handle empty or invalid response
    if (!isValidObject(data)) {
      console.warn('[QRCodeService] Invalid data structure, returning empty array');
      return [];
    }

    const qrCodes: QRCodeInfo[] = [];

    // Iterate through object entries
    for (const [code, position] of Object.entries(data)) {
      // Validate code
      if (!code || typeof code !== 'string' || code.length === 0) {
        console.warn(`[QRCodeService] Invalid QR code key: ${code}`);
        continue;
      }

      // Validate position array
      if (!isValidArray(position) || position.length < 2) {
        console.warn(`[QRCodeService] Invalid position for QR code ${code}:`, position);
        continue;
      }

      // Extract coordinates with safe defaults
      const x = position[0];
      const y = position[1] ?? 0;
      const z = position[2] ?? 0;

      // Validate all coordinates are numbers (even though y is not used in output)
      if (
        !isValidNumber(x) ||
        !isValidNumber(y) ||
        !isValidNumber(z)
      ) {
        console.warn(`[QRCodeService] Invalid coordinates for QR code ${code}:`, position);
        continue;
      }

      // Transform: keep 3D gốc để về sau transform theo metadata; position 2D legacy [x, z]
      qrCodes.push({
        id: this.generateId(code),
        code: code,
        position: [x, z],
        position3D: [x, y, z],
        isManual: false, // All QR codes from API are not manual
      });
    }

    return qrCodes;
  }
}
