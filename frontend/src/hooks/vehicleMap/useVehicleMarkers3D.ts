import { useState, useEffect, useCallback } from 'react';
import { useMQTT } from '../../contexts/MQTTContext';
import { useVehicleService } from '../api/useVehicleService';
import { VehicleMarker3D, ApiVehicle } from '../../types/vehicle';
import { UseVehicleMarkers3DResult } from '../../types/monitoring';
import { useMQTTPositionHandler } from '../shared/useMQTTPositionHandler';
import { loadRotationMetadata, getRotationMatrix, applyRotationToPosition, applyRotationToOrientation } from '../../utils/rotationUtils';

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
  const [apiVehicles, setApiVehicles] = useState<Map<string, ApiVehicle>>(new Map());
  // Store rotation matrix for applying to poses
  const [rotationMatrix, setRotationMatrix] = useState<number[][] | null>(null);
  
  const { lastPositionUpdate, lastVehicleStatus } = useMQTT();
  const { extractPose, isExcludedVehicle } = useMQTTPositionHandler();

  // Load rotation metadata when hook initializes
  useEffect(() => {
    loadRotationMetadata().then(metadata => {
      if (metadata) {
        const matrix = getRotationMatrix(metadata);
        setRotationMatrix(matrix);
        if (DEBUG) {
          console.log('✅ [useVehicleMarkers3D] Rotation matrix loaded:', matrix ? 'available' : 'invalid');
        }
      } else {
        if (DEBUG) {
          console.log('ℹ️ [useVehicleMarkers3D] No rotation metadata available, poses will not be rotated');
        }
        setRotationMatrix(null);
      }
    });
  }, []);

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
      const vehiclesMap = new Map<string, ApiVehicle>();
      const markers: VehicleMarker3D[] = [];
      
      (Array.isArray(vehicles) ? vehicles : []).forEach((v: ApiVehicle) => {
        const vehicleId = v.vehicle_id;
        
        // Skip excluded vehicles
        if (isExcludedVehicle(vehicleId)) {
          return;
        }
        
        // Store vehicle info for merging
        vehiclesMap.set(vehicleId, v);
        
        // Create marker for all vehicles (even without position) to show in sidebar
        let position: [number, number, number] = [0, 0, 0];
        let orientation: [number, number, number, number] | undefined;
        let showOrientation = false;

        if (v.latest_pose?.position) {
          const pos = v.latest_pose.position;
          position = [pos.x, pos.y, pos.z];
          
          // Apply rotation to position if rotation matrix is available
          if (rotationMatrix) {
            position = applyRotationToPosition(position, rotationMatrix);
          }
          
          if (v.latest_pose.orientation) {
            const orient = v.latest_pose.orientation;
            let quat = { x: orient.x, y: orient.y, z: orient.z, w: orient.w };
            
            // Apply rotation to orientation if rotation matrix is available
            if (rotationMatrix) {
              quat = applyRotationToOrientation(quat, rotationMatrix);
            }
            
            orientation = [quat.w, quat.x, quat.y, quat.z];
            showOrientation = true;
          }
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
          vehicleType: v.vehicle_type,
          vehicleCategory: v.vehicle_category
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
  }, [isExcludedVehicle, vehicleService, rotationMatrix]);
  
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
      
      // Apply rotation to position and orientation if rotation matrix is available
      let rotatedPosition = positionData.position;
      let rotatedOrientation = positionData.orientation;
      
      if (rotationMatrix) {
        rotatedPosition = applyRotationToPosition(positionData.position, rotationMatrix);
        
        if (positionData.orientation) {
          // Convert from [w, x, y, z] to {x, y, z, w}
          const quat = {
            x: positionData.orientation[1],
            y: positionData.orientation[2],
            z: positionData.orientation[3],
            w: positionData.orientation[0]
          };
          const rotatedQuat = applyRotationToOrientation(quat, rotationMatrix);
          // Convert back to [w, x, y, z]
          rotatedOrientation = [rotatedQuat.w, rotatedQuat.x, rotatedQuat.y, rotatedQuat.z];
        }
      }
      
      // Determine color based on status: green for online, red for offline
      // Position update means vehicle is definitely online
      const color = getMarkerColorByStatus('online'); // Green for online
      
      // Merge: use API info (name, type) and update position from MQTT
      // If we receive position update from MQTT, vehicle is definitely online
      const existingMarker = next.get(vehicleId);
      const newMarker: VehicleMarker3D = {
        id: vehicleId,
        position: rotatedPosition,
        orientation: rotatedOrientation,
        color, // Always green when receiving position updates (online)
        showOrientation: positionData.showOrientation,
        status: 'online', // Position update means vehicle is online
        name: apiVehicle?.name || existingMarker?.name || vehicleId,
        vehicleType: apiVehicle?.vehicle_type || existingMarker?.vehicleType || 'worker',
        vehicleCategory: apiVehicle?.vehicle_category || existingMarker?.vehicleCategory
      };
      
      next.set(vehicleId, newMarker);
      const result = Array.from(next.values());
      
      if (DEBUG) {
        console.log('✅ Updated vehicle markers:', result.length);
      }
      return result;
    });
  }, [lastPositionUpdate, extractPose, isExcludedVehicle, apiVehicles, rotationMatrix]);

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
          let position: [number, number, number] = [0, 0, 0];
          let orientation: [number, number, number, number] | undefined;
          let showOrientation = false;

          if (apiVehicle.latest_pose?.position) {
            const pos = apiVehicle.latest_pose.position;
            position = [pos.x, pos.y, pos.z];
            
            // Apply rotation to position if rotation matrix is available
            if (rotationMatrix) {
              position = applyRotationToPosition(position, rotationMatrix);
            }
            
            if (apiVehicle.latest_pose.orientation) {
              const orient = apiVehicle.latest_pose.orientation;
              let quat = { x: orient.x, y: orient.y, z: orient.z, w: orient.w };
              
              // Apply rotation to orientation if rotation matrix is available
              if (rotationMatrix) {
                quat = applyRotationToOrientation(quat, rotationMatrix);
              }
              
              orientation = [quat.w, quat.x, quat.y, quat.z];
              showOrientation = true;
            }
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
            vehicleType: apiVehicle.vehicle_type,
            vehicleCategory: apiVehicle.vehicle_category
          };
          
          if (DEBUG) {
            console.log('✅ Created marker from API (coming online):', vehicleId);
          }
          return [...prev, newMarker];
        }
      }
      
      return prev;
    });
  }, [lastVehicleStatus, apiVehicles, rotationMatrix]);

  return {
    vehicleMarkers,
    loading,
    error,
    fetchVehicles
  };
}
