import { useState, useEffect, useCallback, useMemo } from 'react';
import { useMQTT } from '../../contexts/MQTTContext';
import { Vehicle, VehicleMapResponse } from '../../types/vehicleMap2D';
import { UseVehicleMap2DResult } from '../../types/monitoring';
import { useMQTTPositionHandler } from '../shared/useMQTTPositionHandler';
import { useVehicleMapService } from '../api/useVehicleMapService';
import { useVehicleTimeout } from '../shared/useVehicleTimeout';
import {
  extractTimestampStringFromMQTT,
  extractTimestampMsFromMQTT
} from '../../utils/timestampHelpers';

const DEBUG = process.env.REACT_APP_DEBUG_LOGS === '1';

/**
 * Hook to manage vehicles and map data for 2D map mode
 * Handles MQTT position updates, API vehicle fetching, and timeout cleanup
 */
export function useVehicleMap2D(): UseVehicleMap2DResult {
  const vehicleMapService = useVehicleMapService();
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
   * Load map data and vehicles from API
   */
  const loadMapData = useCallback(async () => {
    if (!vehicleMapService) return; // Skip if service not ready
    try {
      setLoading(true);
      setError('');
      const [mapResponse, vehicleData] = await Promise.all([
        vehicleMapService.getLatest2DMap(),
        vehicleMapService.getVehiclesWithPositions()
      ]);
      setMapData(mapResponse);

      if (DEBUG) {
        console.log('🔍 [Monitoring] API vehicle data:', vehicleData);
      }

      // Mark API vehicles with source='api'
      const apiVehicles = vehicleData.map((v: Vehicle) => ({
        ...v,
        source: 'api' as const
      }));

      // Merge API vehicles with MQTT vehicles instead of overwriting
      setMapVehicles(prevVehicles => {
        // Get existing MQTT vehicles
        const mqttVehicles = prevVehicles.filter(
          v => (v as Vehicle & { source?: string }).source === 'mqtt'
        );
        const apiVehiclesOnly = apiVehicles.filter(
          (apiV: Vehicle & { source: 'api' }) =>
            !mqttVehicles.some(mqttV => mqttV.id === apiV.id)
        );

        const mergedVehicles = [...mqttVehicles, ...apiVehiclesOnly];

        if (DEBUG) {
          console.log('🔍 [Monitoring] Merging vehicles:', {
            mqttCount: mqttVehicles.length,
            apiCount: apiVehiclesOnly.length,
            totalCount: mergedVehicles.length
          });
        }

        return mergedVehicles;
      });

      if (DEBUG) {
        console.info('[Monitoring] loadMapData', {
          unit: mapResponse?.unit,
          modelUrl: mapResponse?.modelUrl,
          vehiclesCount: vehicleData.length
        });
      }
    } catch (e: any) {
      console.error('❌ [Monitoring] Error loading map data:', e);
      // Nếu không có PCD/floorplan, không coi là lỗi fatal cho toàn bộ tab
      const message = e?.message || 'Error loading map data';
      if (
        message.includes('No PCD files available') ||
        message.includes('No floorplan available')
      ) {
        // Trường hợp thiếu dữ liệu: không coi là lỗi, chỉ clear mapData
        setMapData(null);
        setError('');
      } else {
        // Các lỗi khác vẫn cần hiển thị
        setError(message);
      }
    } finally {
      setLoading(false);
    }
  }, [vehicleMapService]);

  /**
   * Auto-load map data when service becomes available
   */
  useEffect(() => {
    if (vehicleMapService) {
      loadMapData();
    }
  }, [vehicleMapService, loadMapData]);

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
   * Handle vehicle status updates - update status instead of removing vehicles
   */
  useEffect(() => {
    if (!lastVehicleStatus) return;

    const vehicleId = lastVehicleStatus.vehicle_id;
    const newStatus = lastVehicleStatus.status;

    if (DEBUG) {
      console.log('🔍 Vehicle status update:', vehicleId, '->', newStatus);
    }

    // Update vehicle status instead of removing
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

    // Only remove from lastSeen if vehicle is deleted (not just offline)
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
