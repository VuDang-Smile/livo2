import { useLanguage } from '../contexts/LanguageContext';

export interface VehiclePosition {
  id: string;
  licensePlate: string;
  driver: string;
  vehicleType: string;
  mission: string;
  position: [number, number, number];
  position2D?: [number, number]; // Normalized coordinates [0-1, 0-1] for 2D map rendering
  status: 'active' | 'maintenance' | 'inactive';
  lastUpdate: string;
}

// Hàm dùng trong code React thông qua hook
export const useMockVehiclePositions = (): VehiclePosition[] => {
  const { t } = useLanguage();
  return getMockVehiclePositions(t);
};

// Hàm thuần để có thể dùng từ window hoặc các nơi khác
export const getMockVehiclePositions = (t: (key: string) => string): VehiclePosition[] => [
  {
    id: '1',
    licensePlate: '30A-12345',
    driver: t('driver_name_1'),
    vehicleType: t('tbm_tunnel'),
    mission: t('tunnel_line_1'),
    position: [2, 0.5, 0],
    position2D: [0.65, 0.24], // Normalized coordinates [0-1, 0-1] - to be adjusted manually
    status: 'active',
    lastUpdate: '2024-01-15 14:30:00'
  },
  {
    id: '2',
    licensePlate: '30B-67890',
    driver: t('driver_name_2'),
    vehicleType: t('transport_vehicle'),
    mission: t('material_transport'),
    position: [15, 0.5, 10],
    position2D: [0.72, 0.28], // Normalized coordinates [0-1, 0-1] - to be adjusted manually
    status: 'active',
    lastUpdate: '2024-01-15 14:28:00'
  },
  {
    id: '3',
    licensePlate: '30C-11111',
    driver: t('driver_name_3'),
    vehicleType: t('concrete_pump'),
    mission: t('concrete_wall_pump'),
    position: [35, 0, 28],
    position2D: [0.85, 0.37], // Normalized coordinates [0-1, 0-1] - to be adjusted manually
    status: 'maintenance',
    lastUpdate: '2024-01-15 14:25:00'
  },
  {
    id: '4',
    licensePlate: '30D-22222',
    driver: t('driver_name_4'),
    vehicleType: t('crane'),
    mission: t('equipment_installation'),
    position: [-95, -1, 50],
    position2D: [0.2, 0.46], // Normalized coordinates [0-1, 0-1] - to be adjusted manually
    status: 'active',
    lastUpdate: '2024-01-15 14:32:00'
  }
];



