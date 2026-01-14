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
  position: [number, number];
  isActive?: boolean;
  isDraft?: boolean;
  label?: string;
}
