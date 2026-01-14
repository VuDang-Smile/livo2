import { VehicleMapVehicle } from '../types/vehicle';

export type VehicleStatusKey = VehicleMapVehicle['status'] | 'total';

export interface VehicleStatusConfigItem {
  key: VehicleStatusKey;
  /**
   * i18n key dùng cho label, ví dụ: 'active_vehicles_count'
   */
  labelKey: string;
  /**
   * Màu nền nhạt cho icon/indicator (Tailwind class)
   */
  bgClass: string;
  /**
   * Màu chính cho dot/icon (Tailwind class)
   */
  colorClass: string;
  /**
   * Có hiển thị trong legend không
   */
  showInLegend?: boolean;
}

export const vehicleStatusConfig: Record<VehicleStatusKey, VehicleStatusConfigItem> = {
  online: {
    key: 'online',
    labelKey: 'active_vehicles_count',
    bgClass: 'bg-green-100',
    colorClass: 'bg-green-500',
    showInLegend: true,
  },
  offline: {
    key: 'offline',
    labelKey: 'offline_vehicles_count',
    bgClass: 'bg-red-100',
    colorClass: 'bg-red-500',
    showInLegend: true,
  },
  // Các trạng thái khác trong tương lai có thể thêm vào đây (idle, warning, ...)
  total: {
    key: 'total',
    labelKey: 'total_vehicles_count',
    bgClass: 'bg-blue-100',
    colorClass: 'text-blue-600',
    showInLegend: false,
  },
};

