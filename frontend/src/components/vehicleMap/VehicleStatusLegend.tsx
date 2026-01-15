import React from 'react';
import { useLanguage } from '../../contexts/LanguageContext';
import { vehicleStatusConfig, VehicleStatusKey } from '../../constants/vehicleStatusConfig';

interface VehicleStatusLegendProps {
  statuses?: VehicleStatusKey[];
}

export const VehicleStatusLegend: React.FC<VehicleStatusLegendProps> = ({ statuses }) => {
  const { t } = useLanguage();

  const items = (statuses && statuses.length > 0
    ? statuses
    : (Object.keys(vehicleStatusConfig) as VehicleStatusKey[])
  )
    .map((key) => vehicleStatusConfig[key])
    .filter((item) => item.showInLegend);

  if (items.length === 0) return null;

  return (
    <div className="mt-2 flex flex-wrap items-center gap-4 text-xs text-gray-600">
      {items.map((item) => (
        <div key={item.key} className="flex items-center space-x-2">
          <div className={`w-3 h-3 rounded-full ${item.colorClass}`} />
          <span>{t(item.labelKey) || t(item.key) || item.labelKey}</span>
        </div>
      ))}
    </div>
  );
};

export default VehicleStatusLegend;

