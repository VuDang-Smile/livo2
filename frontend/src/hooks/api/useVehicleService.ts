import { useMemo } from 'react';
import { VehicleService } from '../../services/api/vehicleService';
import { getBackendUrl } from '../../constants/mapConfig';

/**
 * Hook to get VehicleService instance
 * Returns null if service is not ready
 */
export function useVehicleService(): VehicleService | null {
  const service = useMemo(() => {
    try {
      const apiBaseUrl = getBackendUrl();
      return new VehicleService(apiBaseUrl);
    } catch (error) {
      console.error('Failed to create VehicleService:', error);
      return null;
    }
  }, []);

  return service;
}
