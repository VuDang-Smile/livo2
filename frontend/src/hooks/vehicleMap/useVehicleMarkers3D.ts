import { useState, useEffect, useCallback } from 'react';
import { useMQTT } from '../../contexts/MQTTContext';
import { useVehicleService } from '../api/useVehicleService';
import { VehicleMarker3D } from '../../types/vehicle';
import { UseVehicleMarkers3DResult } from '../../types/monitoring';
import { useMQTTPositionHandler } from '../shared/useMQTTPositionHandler';

const DEBUG = process.env.REACT_APP_DEBUG_LOGS === '1';

/**
 * Helper function to get marker color based on status
 * Green for online, red for offline
 */
function getMarkerColorByStatus(status: 'online' | 'offline' | undefined): string {
  return status === 'online' ? '#22c55e' : '#ef4444'; // Green for online, red for offline
}

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

        // Determine color based on status: green for online, red for offline
        const vehicleStatus = v.status || 'offline';
        const color = getMarkerColorByStatus(vehicleStatus);
        markers.push({
          id: vehicleId,
          position,
          orientation,
          color,
          showOrientation,
          status: vehicleStatus,
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
      
      // Determine color based on status: green for online, red for offline
      // Position update means vehicle is definitely online
      const color = getMarkerColorByStatus('online'); // Green for online
      
      // Merge: use API info (name, type) and update position from MQTT
      // If we receive position update from MQTT, vehicle is definitely online
      const existingMarker = next.get(vehicleId);
      const newMarker: VehicleMarker3D = {
        id: vehicleId,
        position: positionData.position,
        orientation: positionData.orientation,
        color, // Always green when receiving position updates (online)
        showOrientation: positionData.showOrientation,
        status: 'online', // Position update means vehicle is online
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
   * Handle vehicle status updates - update status instead of removing when offline
   */
  useEffect(() => {
    if (!lastVehicleStatus) return;

    const vehicleId = lastVehicleStatus.vehicle_id;
    const newStatus = lastVehicleStatus.status;

    if (DEBUG) {
      console.log('🔍 [useVehicleMarkers3D] Vehicle status update:', vehicleId, '->', newStatus);
    }

    setVehicleMarkers((prev) => {
      const existingMarker = prev.find(marker => marker.id === vehicleId);
      
      if (existingMarker) {
        // Marker exists - update status and color
        const validStatus: 'online' | 'offline' =
          newStatus === 'online' ? 'online' : 'offline';
        
        // Update color based on new status: green for online, red for offline
        const newColor = getMarkerColorByStatus(validStatus);
        
        if (DEBUG) {
          console.log('✅ Updated marker status:', vehicleId, existingMarker.status || 'unknown', '->', validStatus);
        }
        
        return prev.map(marker => 
          marker.id === vehicleId 
            ? { ...marker, status: validStatus, color: newColor }
            : marker
        );
      } else if (newStatus === 'online') {
        // Marker doesn't exist but coming online - create from API data
        const apiVehicle = apiVehicles.get(vehicleId);
        if (apiVehicle) {
          let position: [number, number, number];
          let orientation: [number, number, number, number] | undefined;
          let showOrientation = false;

          if (apiVehicle.current_pose?.position) {
            const pos = apiVehicle.current_pose.position;
            position = [pos.x, pos.y, pos.z];
            if (apiVehicle.current_pose.orientation) {
              const orient = apiVehicle.current_pose.orientation;
              orientation = [orient.w, orient.x, orient.y, orient.z];
              showOrientation = true;
            }
          } else if (apiVehicle.current_position) {
            const pos = apiVehicle.current_position;
            position = [pos.x, pos.y, pos.z];
          } else {
            // Use default position if no position data
            position = [0, 0, 0];
          }

          // Determine color based on status: green for online, red for offline
          const color = getMarkerColorByStatus('online'); // Green for online (we're in the online branch)
          
          const newMarker: VehicleMarker3D = {
            id: vehicleId,
            position,
            orientation,
            color,
            showOrientation,
            status: 'online',
            name: apiVehicle.name || vehicleId,
            type: apiVehicle.vehicle_type || apiVehicle.type || 'worker'
          };
          
          if (DEBUG) {
            console.log('✅ Created marker from API (coming online):', vehicleId);
          }
          return [...prev, newMarker];
        }
      }
      
      return prev;
    });
  }, [lastVehicleStatus, apiVehicles]);

  return {
    vehicleMarkers,
    loading,
    error,
    fetchVehicles
  };
}
