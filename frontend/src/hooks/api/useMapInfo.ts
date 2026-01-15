import { useState, useEffect, useCallback, useRef } from 'react';
import { MapInfoService } from '../../services/api/mapInfoService';
import { MapInfo } from '../../types/mapInfo';
import { MAP_INFO_URL } from '../../config/dataSources';

/**
 * Hook result interface
 */
export interface UseMapInfoResult {
  mapInfo: MapInfo | null;
  isLoading: boolean;
  error: string | null;
  refetch: () => Promise<void>;
}

/**
 * Hook to fetch and manage map information from storage
 * 
 * Features:
 * - Automatic fetch on mount
 * - Loading and error state management
 * - Proper cleanup on unmount
 * - Manual refetch capability
 * 
 * @returns UseMapInfoResult with mapInfo, isLoading, error, and refetch function
 */
export function useMapInfo(): UseMapInfoResult {
  const [mapInfo, setMapInfo] = useState<MapInfo | null>(null);
  const [isLoading, setIsLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);
  
  // Use ref to track mounted state and prevent state updates after unmount
  const isMountedRef = useRef<boolean>(true);
  const abortControllerRef = useRef<AbortController | null>(null);
  
  // Create service instance
  const service = useRef<MapInfoService | null>(null);
  
  useEffect(() => {
    // Initialize service
    service.current = new MapInfoService(MAP_INFO_URL);
    
    // Reset mounted flag
    isMountedRef.current = true;
    
    return () => {
      // Cleanup: mark as unmounted
      isMountedRef.current = false;
      
      // Abort any pending fetch
      if (abortControllerRef.current) {
        abortControllerRef.current.abort();
      }
    };
  }, []);
  
  /**
   * Fetch map info from storage
   */
  const fetchMapInfo = useCallback(async () => {
    if (!service.current) {
      console.warn('[useMapInfo] Service not initialized');
      return;
    }
    
    // Abort previous request if exists
    if (abortControllerRef.current) {
      abortControllerRef.current.abort();
    }
    
    // Create new abort controller
    const abortController = new AbortController();
    abortControllerRef.current = abortController;
    
    // Only update loading state if component is still mounted
    if (isMountedRef.current) {
      setIsLoading(true);
      setError(null);
    }
    
    try {
      const info = await service.current.getCurrentMapInfo();
      
      // Guard: only update state if component is still mounted and not aborted
      if (isMountedRef.current && !abortController.signal.aborted) {
        setMapInfo(info);
        setIsLoading(false);
        setError(null);
      }
    } catch (err: any) {
      // Guard: only update state if component is still mounted and not aborted
      if (isMountedRef.current && !abortController.signal.aborted) {
        const errorMessage = err?.message || 'Failed to fetch map information';
        setError(errorMessage);
        setIsLoading(false);
        setMapInfo(null);
        console.error('[useMapInfo] Failed to fetch map info:', err);
      }
    } finally {
      // Clear abort controller reference
      if (abortControllerRef.current === abortController) {
        abortControllerRef.current = null;
      }
    }
  }, []);
  
  // Fetch on mount
  useEffect(() => {
    fetchMapInfo();
  }, [fetchMapInfo]);
  
  return {
    mapInfo,
    isLoading,
    error,
    refetch: fetchMapInfo,
  };
}
