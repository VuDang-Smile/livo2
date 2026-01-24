import { Language } from '../contexts/LanguageContext';

export type VehicleType = 'scanner' | 'worker';

/**
 * Lấy label đã dịch của vehicle type (scanner/worker) theo ngôn ngữ hiện tại
 * @param vehicleType - Vehicle type value từ API ('scanner' | 'worker')
 * @param t - Translation function từ useLanguage hook
 * @returns Label đã dịch hoặc giá trị gốc nếu không hợp lệ
 */
export function getVehicleTypeLabel(
  vehicleType: string | undefined | null,
  t: (key: string) => string
): string {
  if (!vehicleType) {
    return '-';
  }

  const normalizedType = vehicleType.toLowerCase() as VehicleType;
  
  // Chỉ hỗ trợ scanner và worker
  if (normalizedType !== 'scanner' && normalizedType !== 'worker') {
    return vehicleType; // Fallback về giá trị gốc nếu không phải scanner/worker
  }
  
  return t(`type.${normalizedType}`);
}
