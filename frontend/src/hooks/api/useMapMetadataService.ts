import { useMemo } from 'react';
import { MapMetadataService } from '../../services/api/mapMetadataService';
import { getBackendUrl } from '../../constants/mapConfig';

/**
 * Hook to get MapMetadataService instance
 * Returns null if service is not ready
 */
export function useMapMetadataService(): MapMetadataService | null {
  const service = useMemo(() => {
    try {
      const apiBaseUrl = getBackendUrl();
      return new MapMetadataService(apiBaseUrl);
    } catch (error) {
      console.error('Failed to create MapMetadataService:', error);
      return null;
    }
  }, []);

  return service;
}
