import { useState, useEffect, useCallback, useMemo } from 'react';
import { useMQTT } from '../../contexts/MQTTContext';
import { Vehicle, VehicleMapResponse } from '../../types/vehicleMap2D';
import { UseVehicleMap2DResult } from '../../types/monitoring';
import { useMQTTPositionHandler } from '../shared/useMQTTPositionHandler';
import { useVehicleTimeout } from '../shared/useVehicleTimeout';
import {
  extractTimestampStringFromMQTT,
  extractTimestampMsFromMQTT
} from '../../utils/timestampHelpers';
import { useVehicleService } from '../api/useVehicleService';
import { ApiVehicle } from '../../types/vehicle';

const DEBUG = process.env.REACT_APP_DEBUG_LOGS === '1';

/**
 * Hook to manage vehicles and map data for 2D map mode
 * Handles MQTT position updates and timeout cleanup
 * Note: API calls removed - using local files instead
 */
export function useVehicleMap2D(): UseVehicleMap2DResult {
  const [mapData, setMapData] = useState<VehicleMapResponse | null>(null);
  const [mapVehicles, setMapVehicles] = useState<Vehicle[]>([]);
  const [loading, setLoading] = useState<boolean>(false);
  const [error, setError] = useState<string>('');
  const [vehicleLastSeen, setVehicleLastSeen] = useState<Map<string, number>>(new Map());

  const { lastPositionUpdate, lastVehicleStatus } = useMQTT();
  const { extractPosition, isExcludedVehicle } = useMQTTPositionHandler();
  const vehicleService = useVehicleService();
  // Store vehicles from API for merging with MQTT updates
  const [apiVehicles, setApiVehicles] = useState<Map<string, ApiVehicle>>(new Map());

  // Track last position update timestamp for timeout check
  const lastAnyPositionUpdate = useMemo(() => {
    if (!lastPositionUpdate) return null;
    const timestampMs = extractTimestampMsFromMQTT(lastPositionUpdate);
    if (timestampMs === null && DEBUG) {
      console.warn('⚠️ [useVehicleMap2D] No valid timestamp in position update, timeout check may be inaccurate');
    }
    return timestampMs;
  }, [lastPositionUpdate]);

  /**
   * Load map data - API calls removed, hiện dùng metadata/floorplan từ storage.lidar.tm
   * Hàm giữ lại cho compatibility nhưng không thực hiện gọi API.
   */
  const loadMapData = useCallback(async () => {
    if (DEBUG) {
      console.log('ℹ️ [useVehicleMap2D] loadMapData called but API calls are disabled - using storage.lidar.tm metadata');
    }
    setMapData(null); // Clear mapData as we're not using API
    setError('');
    setLoading(false);
  }, []);
  
  /**
   * Fetch vehicles from API and transform to Vehicle format
   */
  const fetchVehiclesFromAPI = useCallback(async () => {
    if (!vehicleService) return;
    
    try {
      setLoading(true);
      const vehicles = await vehicleService.getVehicles();
      const vehiclesMap = new Map<string, ApiVehicle>();
      const transformedVehicles: Vehicle[] = [];
      
      (Array.isArray(vehicles) ? vehicles : []).forEach((v: ApiVehicle) => {
        const vehicleId = v.vehicle_id;
        
        // Skip excluded vehicles
        if (isExcludedVehicle(vehicleId)) {
          return;
        }
        
        // Store vehicle info for merging
        vehiclesMap.set(vehicleId, v);
        
        // Transform to Vehicle format
        const position = v.current_pose?.position || v.current_position || { x: 0, y: 0, z: 0 };
        const timestamp = v.current_pose?.timestamp || v.updated_at || v.created_at || new Date().toISOString();
        
        transformedVehicles.push({
          id: vehicleId,
          name: v.name || vehicleId,
          type: v.vehicle_type || v.type,
          status: v.status || 'offline',
          position,
          timestamp,
          source: 'api' as const
        });
      });
      
      setApiVehicles(vehiclesMap);
      setMapVehicles(prev => {
        // Merge with existing vehicles from MQTT
        const existingMap = new Map(prev.map(v => [v.id, v]));
        transformedVehicles.forEach(vehicle => {
          existingMap.set(vehicle.id, vehicle);
        });
        return Array.from(existingMap.values());
      });
      
      // Update vehicleLastSeen for vehicles from API
      const now = Date.now();
      setVehicleLastSeen(prev => {
        const updated = new Map(prev);
        transformedVehicles.forEach(vehicle => {
          const timestampMs = new Date(vehicle.timestamp).getTime();
          if (!isNaN(timestampMs)) {
            updated.set(vehicle.id, timestampMs);
          } else {
            updated.set(vehicle.id, now);
          }
        });
        return updated;
      });
      
      if (DEBUG) {
        console.log(`✅ [useVehicleMap2D] Fetched ${transformedVehicles.length} vehicles from API`);
      }
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to fetch vehicles';
      setError(errorMessage);
      console.error('❌ [useVehicleMap2D] Failed to fetch vehicles from API:', err);
    } finally {
      setLoading(false);
    }
  }, [vehicleService, isExcludedVehicle]);
  
  // Fetch vehicles from API when component mounts
  useEffect(() => {
    if (vehicleService) {
      fetchVehiclesFromAPI();
    }
  }, [vehicleService, fetchVehiclesFromAPI]);

  /**
   * Handle MQTT position updates for 2D map
   */
  useEffect(() => {
    if (!lastPositionUpdate?.vehicle_id) return;

    // Filter out excluded vehicles
    if (isExcludedVehicle(lastPositionUpdate.vehicle_id)) {
      if (DEBUG) {
        console.log(
          '❌ [2D Map] Ignoring excluded vehicle:',
          lastPositionUpdate.vehicle_id
        );
      }
      return;
    }

    const newPosition = extractPosition(lastPositionUpdate);
    if (!newPosition) return;

    // Update vehicleLastSeen timestamp - sử dụng timestamp từ MQTT
    const timestampMs = extractTimestampMsFromMQTT(lastPositionUpdate) || Date.now();
    setVehicleLastSeen(prev => {
      const updated = new Map(prev);
      updated.set(lastPositionUpdate.vehicle_id, timestampMs);
      return updated;
    });

    setMapVehicles(prevVehicles => {
      const existingIndex = prevVehicles.findIndex(
        v => v.id === lastPositionUpdate.vehicle_id
      );

      // Get vehicle info from API if available
      const apiVehicle = apiVehicles.get(lastPositionUpdate.vehicle_id);
      
      if (existingIndex >= 0) {
        // Update existing vehicle - merge API info with MQTT position
        const timestamp = extractTimestampStringFromMQTT(lastPositionUpdate, true);
        const updatedVehicles = [...prevVehicles];
        const existingVehicle = updatedVehicles[existingIndex];
        
        updatedVehicles[existingIndex] = {
          ...existingVehicle,
          position: newPosition,
          timestamp: timestamp as string,
          source: 'mqtt' as const,
          // Keep API info if available
          name: apiVehicle?.name || existingVehicle.name,
          type: apiVehicle?.vehicle_type || apiVehicle?.type || existingVehicle.type,
          status: apiVehicle?.status || existingVehicle.status || 'online'
        } as Vehicle & { source: 'mqtt' };

        if (DEBUG) {
          console.log(
            '✅ [2D Map] Updated existing vehicle:',
            lastPositionUpdate.vehicle_id
          );
        }
        return updatedVehicles;
      } else {
        // Create new vehicle - merge API info with MQTT data
        const timestamp = extractTimestampStringFromMQTT(lastPositionUpdate, true);
        const newVehicle = {
          id: lastPositionUpdate.vehicle_id,
          name: apiVehicle?.name || `Vehicle ${lastPositionUpdate.vehicle_id}`,
          type: apiVehicle?.vehicle_type || apiVehicle?.type,
          status: apiVehicle?.status || 'online' as const,
          position: newPosition,
          timestamp: timestamp as string,
          source: 'mqtt' as const
        } as Vehicle & { source: 'mqtt' };

        if (DEBUG) {
          console.log(
            '✅ [2D Map] Created new vehicle:',
            lastPositionUpdate.vehicle_id
          );
        }
        return [...prevVehicles, newVehicle];
      }
    });
  }, [lastPositionUpdate, extractPosition, isExcludedVehicle, apiVehicles]);

  /**
   * Handle vehicle status updates - remove from list and canvas when status = offline
   */
  useEffect(() => {
    if (!lastVehicleStatus) return;

    const vehicleId = lastVehicleStatus.vehicle_id;
    const newStatus = lastVehicleStatus.status;

    if (DEBUG) {
      console.log('🔍 [useVehicleMap2D] Vehicle status update:', vehicleId, '->', newStatus);
    }

    if (newStatus === 'offline') {
      // Remove vehicle from list when status = offline
      setMapVehicles((prev) => {
        const filtered = prev.filter(vehicle => vehicle.id !== vehicleId);
        if (filtered.length < prev.length) {
          console.log('🗑️ [useVehicleMap2D] Removed vehicle from list (offline):', vehicleId);
        }
        return filtered;
      });

      // Remove from lastSeen tracking
      setVehicleLastSeen(prev => {
        const updated = new Map(prev);
        updated.delete(vehicleId);
        return updated;
      });
    } else {
      // Update vehicle status for other status changes
      setMapVehicles((prev) => {
        return prev.map(vehicle => {
          if (vehicle.id === vehicleId) {
            if (DEBUG) {
              console.log('✅ Updated vehicle status:', vehicleId, vehicle.status, '->', newStatus);
            }
            // Only two states are supported: online/offline
            const validStatus: 'online' | 'offline' =
              newStatus === 'online' ? 'online'
              : newStatus === 'offline' ? 'offline'
              : vehicle.status;
            return {
              ...vehicle,
              status: validStatus
            };
          }
          return vehicle;
        });
      });
    }

    // Remove from lastSeen if vehicle is deleted
    if (newStatus === 'deleted' || (lastVehicleStatus.action && lastVehicleStatus.action === 'deleted')) {
      setVehicleLastSeen(prev => {
        const updated = new Map(prev);
        updated.delete(vehicleId);
        return updated;
      });
    }
  }, [lastVehicleStatus]);

  /**
   * Filter vehicles: excluding excluded vehicles
   * Include all types & statuses (online, offline) để hiển thị đầy đủ
   */
  const filteredVehicles = useMemo(() => {
    return mapVehicles.filter(vehicle => !isExcludedVehicle(vehicle.id));
  }, [mapVehicles, isExcludedVehicle]);

  return {
    mapData,
    mapVehicles,
    filteredVehicles,
    loading,
    error,
    loadMapData
  };
}
