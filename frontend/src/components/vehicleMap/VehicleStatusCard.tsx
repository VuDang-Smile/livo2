import React from 'react';
import { useLanguage } from '../../contexts/LanguageContext';
import { VehicleStatusKey, vehicleStatusConfig } from '../../constants/vehicleStatusConfig';

interface VehicleStatusCardProps {
  statusKey: VehicleStatusKey;
  value: number;
}

export const VehicleStatusCard: React.FC<VehicleStatusCardProps> = ({ statusKey, value }) => {
  const { t } = useLanguage();
  const config = vehicleStatusConfig[statusKey];

  const label = t(config.labelKey) || t(statusKey) || config.labelKey;

  const isTotal = statusKey === 'total';

  return (
    <div className="bg-white px-6 py-3 rounded-lg shadow-sm border border-gray-200 min-w-[160px]">
      <div className="flex items-center">
        <div className={`p-2 rounded-lg ${config.bgClass}`}>
          {isTotal ? (
            <svg
              className={`w-6 h-6 ${config.colorClass}`}
              fill="none"
              stroke="currentColor"
              viewBox="0 0 24 24"
            >
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={2}
                d="M9 20l-5.447-2.724A1 1 0 013 16.382V5.618a1 1 0 011.447-.894L9 7m0 13l6-3m-6 3V7m6 10l4.553 2.276A1 1 0 0021 18.382V7.618a1 1 0 00-1.447-.894L15 4m0 13V4m-6 3l6-3"
              />
            </svg>
          ) : (
            <div className={`w-4 h-4 rounded-full ${config.colorClass}`} />
          )}
        </div>
        <div className="ml-3">
          <p className="text-xs font-medium text-gray-600 leading-tight">{label}</p>
          <p className="text-xl font-bold text-gray-900 leading-tight mt-1">{value}</p>
        </div>
      </div>
    </div>
  );
};

export default VehicleStatusCard;

