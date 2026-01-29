import { VehicleCategory } from '../types/vehicle';

/**
 * Lấy label đã dịch của vehicle category theo ngôn ngữ hiện tại
 * @param category - Vehicle category value từ API
 * @param t - Hàm dịch từ useLanguage
 * @returns Label đã dịch hoặc category value nếu không tìm thấy
 */
export function getVehicleCategoryLabel(
  category: VehicleCategory | undefined | null,
  t: (key: string) => string
): string {
  if (!category) {
    return '-'; // Fallback khi không có category
  }
  
  return t(`category.${category}`);
}
