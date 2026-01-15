import { VehicleCategory } from '../types/vehicle';

export interface VehicleCategoryLabel {
  vi: string;
  ja: string;
  en: string;
}

export interface VehicleCategoryOption {
  key: VehicleCategory;
  label: VehicleCategoryLabel;
}

export const VEHICLE_CATEGORIES: VehicleCategoryOption[] = [
  {
    key: 'roadheader',
    label: {
      vi: 'Máy đào gương hầm',
      ja: '自由断面掘削機',
      en: 'Roadheader',
    },
  },
  {
    key: 'drill_jumbo',
    label: {
      vi: 'Máy khoan Jumbo',
      ja: 'ドリルジャンボ',
      en: 'Drill jumbo',
    },
  },
  {
    key: 'shotcrete_machine',
    label: {
      vi: 'Máy phun bê tông',
      ja: '吹付機',
      en: 'Shotcrete machine',
    },
  },
  {
    key: 'concrete_mixer_truck',
    label: {
      vi: 'Xe bồn trộn bê tông',
      ja: 'ミキサー車',
      en: 'Concrete mixer truck',
    },
  },
  {
    key: 'wheel_loader',
    label: {
      vi: 'Máy xúc lật',
      ja: 'ホイールローダー',
      en: 'Wheel loader',
    },
  },
  {
    key: 'dump_truck',
    label: {
      vi: 'Xe ben / Xe đổ',
      ja: 'ダンプカー',
      en: 'Dump truck',
    },
  },
  {
    key: 'backhoe',
    label: {
      vi: 'Máy xúc đào',
      ja: 'バックホウ',
      en: 'Backhoe excavator',
    },
  },
  {
    key: 'rock_breaker',
    label: {
      vi: 'Máy đục phá đá',
      ja: 'ブレーカー',
      en: 'Rock breaker',
    },
  },
  {
    key: 'other_named',
    label: {
      vi: 'Khác (Ghi tên cụ thể)',
      ja: 'その他(名前付き)',
      en: 'Other (named)',
    },
  },
];

export const VEHICLE_CATEGORY_LABELS: Record<VehicleCategory, VehicleCategoryLabel> =
  VEHICLE_CATEGORIES.reduce((acc, item) => {
    acc[item.key] = item.label;
    return acc;
  }, {} as Record<VehicleCategory, VehicleCategoryLabel>);
