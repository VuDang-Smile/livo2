import { useMemo } from 'react';
import { VehicleMapService } from '../../services/api/vehicleMapApi';
import { getBackendUrl } from '../../constants/mapConfig';

/**
 * Hook to get VehicleMapService instance
 * Returns null if service is not ready
 */
export function useVehicleMapService(): VehicleMapService | null {
  const service = useMemo(() => {
    try {
      const apiBaseUrl = getBackendUrl();
      return new VehicleMapService(apiBaseUrl);
    } catch (error) {
      console.error('Failed to create VehicleMapService:', error);
      return null;
    }
  }, []);

  return service;
}
