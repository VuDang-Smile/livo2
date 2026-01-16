/**
 * Type definitions for Upload page related data structures
 */

/**
 * Row being edited in the QR code table
 */
export interface EditingRow {
  tempId: string;
  codeIndex: string;
  position: [number, number];
  position3D?: [number, number, number];
  surface?: 'floor' | 'ceiling' | 'left' | 'right';
  errors: {
    codeIndex?: string;
    position?: string;
  };
}

/**
 * Manual pin for map interaction
 */
export interface ManualPin {
  id: string;
  /**
   * World position theo 2 trục của view (ví dụ top: X/Y hoặc X/Z tuỳ metadata)
   */
  position: [number, number];
  /**
   * Pixel position trong ảnh gốc (theo metadata) - dùng khi render scale ảnh
   */
  pixelPosition?: [number, number];
  /**
   * Trạng thái cảnh báo từ transform
   */
  isOutOfBounds?: boolean;
  isClamped?: boolean;
  isActive?: boolean;
  isDraft?: boolean;
  label?: string;
}
