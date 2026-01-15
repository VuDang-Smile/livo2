import type { VehicleCanvasPosition } from '../types/vehicle';

/**
 * Lấy tọa độ 2D của vehicle trên canvas
 * @param vehicle VehicleCanvasPosition object
 * @param canvasWidth Chiều rộng canvas
 * @param canvasHeight Chiều cao canvas
 * @returns Tọa độ pixel trên canvas {x, y}
 */
export function getVehicle2DPosition(
  vehicle: VehicleCanvasPosition,
  canvasWidth: number,
  canvasHeight: number
): { x: number; y: number } {
  // Nếu có position2D (normalized coordinates 0-1), sử dụng nó
  if (vehicle.position2D && Array.isArray(vehicle.position2D) && vehicle.position2D.length === 2) {
    const [normalizedX, normalizedY] = vehicle.position2D;
    
    // Validate và clamp giá trị trong khoảng [0, 1]
    const clampedX = Math.max(0, Math.min(1, normalizedX));
    const clampedY = Math.max(0, Math.min(1, normalizedY));
    
    // Convert từ normalized [0-1, 0-1] sang canvas pixels
    return {
      x: clampedX * canvasWidth,
      y: clampedY * canvasHeight,
    };
  }
  
  // Fallback: trả về center của canvas nếu không có position2D
  return {
    x: canvasWidth * 0.5,
    y: canvasHeight * 0.5,
  };
}


