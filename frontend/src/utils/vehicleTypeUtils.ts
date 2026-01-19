import { Language } from '../contexts/LanguageContext';

export type VehicleType = 'scanner' | 'worker';

const VEHICLE_TYPE_LABELS: Record<VehicleType, Record<Language, string>> = {
  scanner: {
    vi: 'Scanner',
    ja: 'スキャナー',
    en: 'Scanner',
  },
  worker: {
    vi: 'Worker',
    ja: 'ワーカー',
    en: 'Worker',
  },
};

/**
 * Lấy label đã dịch của vehicle type (scanner/worker) theo ngôn ngữ hiện tại
 * @param vehicleType - Vehicle type value từ API ('scanner' | 'worker')
 * @param language - Ngôn ngữ hiện tại ('vi' | 'ja' | 'en')
 * @returns Label đã dịch hoặc giá trị gốc nếu không hợp lệ
 */
export function getVehicleTypeLabel(
  vehicleType: string | undefined | null,
  language: Language = 'vi'
): string {
  if (!vehicleType) {
    return '-';
  }

  const normalizedType = vehicleType.toLowerCase() as VehicleType;
  const label = VEHICLE_TYPE_LABELS[normalizedType];
  
  if (!label) {
    return vehicleType; // Fallback về giá trị gốc nếu không phải scanner/worker
  }
  
  return label[language] || label.vi;
}
