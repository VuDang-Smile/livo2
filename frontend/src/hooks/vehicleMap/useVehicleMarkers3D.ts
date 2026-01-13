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
  // Store vehicles from API for merging with MQTT updates
  const [apiVehicles, setApiVehicles] = useState<Map<string, any>>(new Map());
  
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
      
      // Store all vehicles from API for merging
      const vehiclesMap = new Map<string, any>();
      const markers: VehicleMarker3D[] = [];
      
      (Array.isArray(vehicles) ? vehicles : []).forEach((v: any) => {
        const vehicleId = v.vehicle_id || String(v._id || v.id);
        
        // Skip excluded vehicles
        if (isExcludedVehicle(vehicleId)) {
          return;
        }
        
        // Store vehicle info for merging
        vehiclesMap.set(vehicleId, v);
        
        // Create marker for all vehicles (even without position) to show in sidebar
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
          // Vehicle exists in DB but has no position - use default position
          // This ensures all vehicles from API are visible in sidebar
          position = [0, 0, 0];
        }

        const color = v.type === 'scanner' || v.vehicle_type === 'scanner' ? '#4f46e5' : '#ef4444';
        markers.push({
          id: vehicleId,
          position,
          orientation,
          color,
          showOrientation,
          status: v.status || 'offline',
          name: v.name || vehicleId,
          type: v.vehicle_type || v.type || 'worker'
        });
      });
      
      setApiVehicles(vehiclesMap);
      setVehicleMarkers(markers);
      
      if (DEBUG) {
        console.log(`✅ [useVehicleMarkers3D] Fetched ${markers.length} vehicles from API`);
      }
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to fetch vehicles';
      setError(errorMessage);
      console.error('Failed to fetch vehicles for monitoring:', err);
    } finally {
      setLoading(false);
    }
  }, [isExcludedVehicle, vehicleService]);
  
  // Auto-fetch vehicles when component mounts
  useEffect(() => {
    if (vehicleService) {
      fetchVehicles();
    }
  }, [vehicleService, fetchVehicles]);

  /**
   * Handle MQTT position updates - merge with API vehicles
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
      const vehicleId = lastPositionUpdate.vehicle_id;
      
      // Get vehicle info from API if available
      const apiVehicle = apiVehicles.get(vehicleId);
      
      // Determine color based on vehicle type from API or default
      const color = apiVehicle?.type === 'scanner' || apiVehicle?.vehicle_type === 'scanner' 
        ? '#4f46e5' 
        : '#ef4444';
      
      // Merge: use API info (name, type, status) and update position from MQTT
      const existingMarker = next.get(vehicleId);
      const newMarker: VehicleMarker3D = {
        id: vehicleId,
        position: positionData.position,
        orientation: positionData.orientation,
        color: existingMarker?.color || color,
        showOrientation: positionData.showOrientation,
        status: apiVehicle?.status || existingMarker?.status || 'online',
        name: apiVehicle?.name || existingMarker?.name || vehicleId,
        type: apiVehicle?.vehicle_type || apiVehicle?.type || existingMarker?.type || 'worker'
      };
      
      next.set(vehicleId, newMarker);
      const result = Array.from(next.values());
      
      if (DEBUG) {
        console.log('✅ Updated vehicle markers:', result.length);
      }
      return result;
    });
  }, [lastPositionUpdate, extractPose, isExcludedVehicle, apiVehicles]);

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
            const validStatus: 'online' | 'offline' =
              newStatus === 'online' ? 'online' : 'offline';
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
