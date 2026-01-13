import { useState, useEffect, useCallback } from 'react';
import { useMQTT } from '../../contexts/MQTTContext';
import { useVehicleService } from '../api/useVehicleService';
import { VehicleMarker3D } from '../../types/vehicle';
import { UseVehicleMarkers3DResult } from '../../types/monitoring';
import { useMQTTPositionHandler } from '../shared/useMQTTPositionHandler';

const DEBUG = process.env.REACT_APP_DEBUG_LOGS === '1';

/**
 * Hook to manage vehicle markers for 3D monitoring mode
 * Handles MQTT position updates, API vehicle fetching, and timeout cleanup
 */
export function useVehicleMarkers3D(): UseVehicleMarkers3DResult {
  const vehicleService = useVehicleService();
  const [vehicleMarkers, setVehicleMarkers] = useState<VehicleMarker3D[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  
  const { lastPositionUpdate, lastVehicleStatus } = useMQTT();
  const { extractPose, isExcludedVehicle } = useMQTTPositionHandler();

  /**
   * Fetch vehicles from API
   */
  const fetchVehicles = useCallback(async () => {
    if (!vehicleService) return; // Skip if service not ready
    setLoading(true);
    setError(null);
    try {
      const vehicles = await vehicleService.getVehicles();
      const markers = (Array.isArray(vehicles) ? vehicles : [])
        .filter((v: any) => v && (v.current_pose || v.current_position))
        .filter((v: any) => {
          const vehicleId = v.vehicle_id || String(v._id || v.id);
          return !isExcludedVehicle(vehicleId);
        })
        .map((v: any): VehicleMarker3D => {
          let position: [number, number, number];
          let orientation: [number, number, number, number] | undefined;
          let showOrientation = false;

          if (v.current_pose?.position) {
            const pos = v.current_pose.position;
            position = [pos.x, pos.y, pos.z];
            if (v.current_pose.orientation) {
              const orient = v.current_pose.orientation;
              orientation = [orient.w, orient.x, orient.y, orient.z];
              showOrientation = true;
            }
          } else if (v.current_position) {
            const pos = v.current_position;
            position = [pos.x, pos.y, pos.z];
          } else {
            position = [0, 0, 0];
          }

          const color = v.type === 'scanner' ? '#4f46e5' : '#ef4444';
          return {
            id: v.vehicle_id || String(v._id || v.id),
            position,
            orientation,
            color,
            showOrientation
          };
        });
      
      setVehicleMarkers(markers);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to fetch vehicles';
      setError(errorMessage);
      console.error('Failed to fetch vehicles for monitoring:', err);
    } finally {
      setLoading(false);
    }
  }, [isExcludedVehicle, vehicleService]);

  /**
   * Handle MQTT position updates
   */
  useEffect(() => {
    if (!lastPositionUpdate?.vehicle_id) return;

    // Filter out excluded vehicles
    if (isExcludedVehicle(lastPositionUpdate.vehicle_id)) {
      if (DEBUG) {
        console.log('❌ Ignoring excluded vehicle:', lastPositionUpdate.vehicle_id);
      }
      return;
    }

    if (DEBUG) {
      console.log('✅ Processing position update for vehicle:', lastPositionUpdate.vehicle_id);
    }

    const positionData = extractPose(lastPositionUpdate);
    if (!positionData) return;

    setVehicleMarkers((prev) => {
      const next = new Map(prev.map((m) => [m.id, m] as const));
      const newMarker: VehicleMarker3D = {
        id: lastPositionUpdate.vehicle_id,
        position: positionData.position,
        orientation: positionData.orientation,
        color: '#ef4444',
        showOrientation: positionData.showOrientation
      };
      
      next.set(lastPositionUpdate.vehicle_id, newMarker);
      const result = Array.from(next.values());
      
      if (DEBUG) {
        console.log('✅ Updated vehicle markers:', result.length);
      }
      return result;
    });
  }, [lastPositionUpdate, extractPose, isExcludedVehicle]);

  /**
   * Handle vehicle status updates - remove markers when status = offline
   */
  useEffect(() => {
    if (!lastVehicleStatus) return;

    const vehicleId = lastVehicleStatus.vehicle_id;
    const newStatus = lastVehicleStatus.status;

    if (DEBUG) {
      console.log('🔍 [useVehicleMarkers3D] Vehicle status update:', vehicleId, '->', newStatus);
    }

    if (newStatus === 'offline') {
      // Remove marker from canvas when vehicle goes offline
      setVehicleMarkers((prev) => {
        const filtered = prev.filter(marker => marker.id !== vehicleId);
        if (filtered.length < prev.length) {
          console.log('🗑️ [useVehicleMarkers3D] Removed marker for offline vehicle:', vehicleId);
        }
        return filtered;
      });
    } else {
      // Update marker status for other status changes
      setVehicleMarkers((prev) => {
        return prev.map(marker => {
          if (marker.id === vehicleId) {
            if (DEBUG) {
              console.log('✅ Updated marker status:', vehicleId, marker.status || 'unknown', '->', newStatus);
            }
            // Only two states are supported: online/offline
            const validStatus: 'online' | 'offline' | undefined =
              newStatus === 'online'
                ? 'online'
                : marker.status === 'online' || marker.status === 'offline'
                  ? marker.status
                  : undefined;
            return {
              ...marker,
              status: validStatus
            };
          }
          return marker;
        });
      });
    }
  }, [lastVehicleStatus]);

  return {
    vehicleMarkers,
    loading,
    error,
    fetchVehicles
  };
}
