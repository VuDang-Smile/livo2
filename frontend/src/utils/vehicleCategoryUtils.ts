import { VehicleCategory } from '../types/vehicle';
import { VEHICLE_CATEGORY_LABELS } from '../constants/vehicleCategories';
import { Language } from '../contexts/LanguageContext';

/**
 * Lấy label đã dịch của vehicle category theo ngôn ngữ hiện tại
 * @param category - Vehicle category value từ API
 * @param language - Ngôn ngữ hiện tại ('vi' | 'ja' | 'en')
 * @returns Label đã dịch hoặc category value nếu không tìm thấy
 */
export function getVehicleCategoryLabel(
  category: VehicleCategory | undefined | null,
  language: Language = 'vi'
): string {
  if (!category) {
    return '-'; // Fallback khi không có category
  }
  
  const label = VEHICLE_CATEGORY_LABELS[category];
  if (!label) {
    return category; // Fallback về giá trị gốc nếu không có trong constants
  }
  
  return label[language] || label.vi; // Fallback về tiếng Việt nếu thiếu translation
}
