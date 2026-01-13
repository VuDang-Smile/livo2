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

      if (existingIndex >= 0) {
        // Update existing vehicle - sử dụng timestamp từ MQTT
        const timestamp = extractTimestampStringFromMQTT(lastPositionUpdate, true);
        const updatedVehicles = [...prevVehicles];
        updatedVehicles[existingIndex] = {
          ...updatedVehicles[existingIndex],
          position: newPosition,
          timestamp: timestamp as string,
          source: 'mqtt' as const
        } as Vehicle & { source: 'mqtt' };

        if (DEBUG) {
          console.log(
            '✅ [2D Map] Updated existing vehicle:',
            lastPositionUpdate.vehicle_id
          );
        }
        return updatedVehicles;
      } else {
        // Create new vehicle - sử dụng timestamp từ MQTT
        const timestamp = extractTimestampStringFromMQTT(lastPositionUpdate, true);
        const newVehicle = {
          id: lastPositionUpdate.vehicle_id,
          name: `Vehicle ${lastPositionUpdate.vehicle_id}`,
          status: 'active' as const,
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
  }, [lastPositionUpdate, extractPosition, isExcludedVehicle]);

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
            // Cast status to valid Vehicle status type
            const validStatus: "active" | "inactive" | "offline" | "maintenance" = 
              (newStatus === 'active' || newStatus === 'inactive' || newStatus === 'offline' || newStatus === 'maintenance') 
                ? newStatus 
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
   * Timeout check - remove vehicles that haven't updated in timeout period
   */
  useVehicleTimeout(
    vehicleLastSeen,
    lastAnyPositionUpdate,
    useCallback((vehicleId: string) => {
      setMapVehicles(current => {
        const filtered = current.filter(vehicle => vehicle.id !== vehicleId);
        if (filtered.length !== current.length && DEBUG) {
          console.log('✅ Cleared timeout 2D map vehicles for:', vehicleId);
        }
        return filtered;
      });

      setVehicleLastSeen(prev => {
        const updated = new Map(prev);
        updated.delete(vehicleId);

        // Clear all if no active vehicles
        if (updated.size === 0 && prev.size > 0) {
          if (DEBUG) {
            console.log('🗑️ No active vehicles, clearing all map data');
          }
          setMapVehicles([]);
        }

        return updated;
      });
    }, []),
    useCallback(() => {
      if (DEBUG) {
        console.log('⏰ No position updates, clearing all map data');
      }
      setMapVehicles([]);
      setVehicleLastSeen(new Map());
    }, [])
  );

  /**
   * Filter vehicles: excluding excluded vehicles
   * Include all types & statuses (active, inactive, offline) để hiển thị đầy đủ
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
