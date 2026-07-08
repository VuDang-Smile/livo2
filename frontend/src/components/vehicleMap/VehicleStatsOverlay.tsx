import React from 'react';
import { useLanguage } from '../../contexts/LanguageContext';
import VehicleStatusCard from './VehicleStatusCard';

interface VehicleStatsOverlayProps {
  total: number;
  online: number;
  offline: number;
  isVisible?: boolean;
}

const VehicleStatsOverlay = React.memo<VehicleStatsOverlayProps>(({ 
  total, online, offline, isVisible = true
}) => {
  const { t } = useLanguage();

  if (!isVisible) return null;
  
  return (
    <div className="bg-white/80 backdrop-blur-md rounded-lg shadow-lg border border-gray-200/50 p-3">
      <div className="grid grid-cols-1 md:grid-cols-3 gap-3">
        <VehicleStatusCard
          statusKey="total"
          value={total}
        />
        <VehicleStatusCard
          statusKey="online"
          value={online}
        />
        <VehicleStatusCard
          statusKey="offline"
          value={offline}
        />
      </div>
    </div>
  );
}, (prev, next) => {
  // Custom comparison để tránh rerender khi props không đổi
  return prev.total === next.total && 
         prev.online === next.online && 
         prev.offline === next.offline &&
         prev.isVisible === next.isVisible;
});

VehicleStatsOverlay.displayName = 'VehicleStatsOverlay';

export default VehicleStatsOverlay;
