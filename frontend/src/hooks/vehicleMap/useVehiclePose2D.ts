import { useState, useEffect, useCallback, useRef } from 'react';
import { useMQTT, PositionUpdateNotification } from '../../contexts/MQTTContext';
import { transformPoseToPixel } from '../../utils/coordinateTransform';
import { MapMetadata, Pose3D } from '../../types/mapMetadata';
import { VehicleMarker2D, ApiVehicle } from '../../types/vehicle';
import { UseVehiclePose2DResult } from '../../types/monitoring';
import { MAP_FLOORPLAN_METADATA_URL } from '../../config/dataSources';
import { useVehicleService } from '../api/useVehicleService';
import { useMQTTPositionHandler } from '../shared/useMQTTPositionHandler';
import { loadRotationMetadata, getRotationMatrix, applyRotationToPose } from '../../utils/rotationUtils';

/**
 * Helper function to get marker color based on status
 * Green for online, red for offline
 * Special colors (yellow/orange) are preserved for out of bounds/clamped
 */
function getMarkerColorByStatus(status: 'online' | 'offline' | undefined, preserveSpecialColors: boolean = false, currentColor?: string): string {
  if (preserveSpecialColors && currentColor && (currentColor === '#fbbf24' || currentColor === '#f97316')) {
    return currentColor; // Keep special colors
  }
  return status === 'online' ? '#22c55e' : '#ef4444'; // Green for online, red for offline
}

/**
 * Hook to track vehicle positions on 2D floorplan
 * 
 * Subscribes to MQTT position updates, transforms 3D poses to 2D pixel coordinates,
 * and returns vehicle markers ready for rendering on 2D map.
 * 
 * @returns Vehicle markers, map metadata, loading state, and refresh function
 */
export function useVehiclePose2D(view: 'top' | 'side_x' = 'top'): UseVehiclePose2DResult {
  const [mapMetadata, setMapMetadata] = useState<MapMetadata | null>(null);
  const [vehicleMarkers, setVehicleMarkers] = useState<VehicleMarker2D[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const lastProcessedUpdateRef = useRef<string | null>(null);
  
  const { lastPositionUpdate, lastVehicleStatus, isConnected } = useMQTT();
  const vehicleService = useVehicleService();
  const { isExcludedVehicle } = useMQTTPositionHandler();
  // Store vehicles from API for merging with MQTT updates
  const [apiVehicles, setApiVehicles] = useState<Map<string, ApiVehicle>>(new Map());
  // Store rotation matrix for applying to poses
  const [rotationMatrix, setRotationMatrix] = useState<number[][] | null>(null);
  
  // Fetch map metadata từ storage thực tế
  const fetchMetadata = useCallback(async () => {
    try {
      setIsLoading(true);
      setError(null);
      
      console.log('🗺️ [useVehiclePose2D] Loading map metadata from storage:', MAP_FLOORPLAN_METADATA_URL);
      
      const response = await fetch(MAP_FLOORPLAN_METADATA_URL);
      
      if (!response.ok) {
        throw new Error(`Failed to load metadata: ${response.statusText}`);
      }
      
      const metadata: MapMetadata = await response.json();
      
      // Validate metadata structure
      if (!metadata.views || !metadata.views[view]) {
        throw new Error(`Invalid metadata structure: missing view '${view}'`);
      }
      
      console.log('✅ [useVehiclePose2D] Loaded map metadata:', {
        view,
        size: `${metadata.views[view].image.width}x${metadata.views[view].image.height}`,
        bounds: metadata.views[view].bounds,
        resolution: metadata.resolution_m_per_pixel
      });
      
      setMapMetadata(metadata);
      console.log('✅ [useVehiclePose2D] Map metadata loaded, ready to transform poses');
      
      // If we have a pending pose update, transform it now
      if (lastPositionUpdate) {
        console.log('🔄 [useVehiclePose2D] Transforming pending pose update after metadata load');
      }
    } catch (err: any) {
      console.error('❌ [useVehiclePose2D] Failed to load map metadata:', err);
      setError(err.message || 'Failed to load map metadata');
    } finally {
      setIsLoading(false);
    }
  }, [view, lastPositionUpdate]);
  
  useEffect(() => {
    fetchMetadata();
  }, [fetchMetadata]);

  // Load rotation metadata when hook initializes
  useEffect(() => {
    loadRotationMetadata().then(metadata => {
      if (metadata) {
        const matrix = getRotationMatrix(metadata);
        setRotationMatrix(matrix);
        console.log('✅ [useVehiclePose2D] Rotation matrix loaded:', matrix ? 'available' : 'invalid');
      } else {
        console.log('ℹ️ [useVehiclePose2D] No rotation metadata available, poses will not be rotated');
        setRotationMatrix(null);
      }
    });
  }, []);
  
  /**
   * Fetch vehicles from API and transform to 2D markers
   */
  const fetchVehiclesFromAPI = useCallback(async () => {
    if (!vehicleService || !mapMetadata) return; // Need metadata to transform poses
    
    try {
      const vehicles = await vehicleService.getVehicles();
      const vehiclesMap = new Map<string, ApiVehicle>();
      const markers: VehicleMarker2D[] = [];
      
      (Array.isArray(vehicles) ? vehicles : []).forEach((v: ApiVehicle) => {
        const vehicleId = v.vehicle_id;
        
        // Skip excluded vehicles
        if (isExcludedVehicle(vehicleId)) {
          return;
        }
        
        // Store vehicle info for merging
        vehiclesMap.set(vehicleId, v);
        
        // Create marker for all vehicles (even without position) to show in sidebar
        if (v.latest_pose?.position) {
          // Vehicle has position - transform to 2D pixel coordinates
          let pose: Pose3D = {
            position: v.latest_pose.position,
            orientation: v.latest_pose.orientation || { x: 0, y: 0, z: 0, w: 1 },
            frame_id: 'map',
            timestamp: v.latest_pose.timestamp || v.updated_at || new Date().toISOString()
          };
          
          // Apply rotation to pose if rotation matrix is available
          if (rotationMatrix) {
            pose = applyRotationToPose(pose, rotationMatrix);
          }
          
          // Transform 3D pose → 2D pixel coordinates
          const debugMode = process.env.NODE_ENV === 'development' || process.env.REACT_APP_DEBUG_LOGS === '1';
          const pixel = transformPoseToPixel(pose, mapMetadata, view, debugMode);
          
          if (pixel) {
            // Determine marker color based on status: green for online, red for offline
            // But keep special colors for out of bounds and clamped
            const vehicleStatus = v.status || 'offline';
            let markerColor = getMarkerColorByStatus(vehicleStatus);
            if (pixel.is_out_of_bounds) {
              markerColor = '#fbbf24'; // Yellow for out of bounds
            } else if (pixel.is_clamped) {
              markerColor = '#f97316'; // Orange for clamped
            }
            
            const yawHalf = pixel.yaw / 2;
            markers.push({
              id: vehicleId,
              position: [pixel.pixel_x, pixel.pixel_y, 0],
              orientation: [Math.cos(yawHalf), 0, 0, Math.sin(yawHalf)],
              color: markerColor,
              showOrientation: true,
              label: v.name || vehicleId,
              lastUpdate: new Date(pose.timestamp || Date.now()),
              name: v.name,
              vehicleType: v.vehicle_type,
              vehicleCategory: v.vehicle_category,
              status: vehicleStatus
            });
          } else {
            // Transform failed - create marker with default position for sidebar display
            const vehicleStatus = v.status || 'offline';
            const markerColor = getMarkerColorByStatus(vehicleStatus);
            markers.push({
              id: vehicleId,
              position: [0, 0, 0],
              orientation: [1, 0, 0, 0],
              color: markerColor,
              showOrientation: false,
              label: v.name || vehicleId,
              lastUpdate: new Date(),
              name: v.name,
              vehicleType: v.vehicle_type,
              vehicleCategory: v.vehicle_category,
              status: vehicleStatus
            });
          }
        } else {
          // Vehicle has no position - create marker with default position for sidebar display
          const vehicleStatus = v.status || 'offline';
          const markerColor = vehicleStatus === 'online' ? '#22c55e' : '#ef4444'; // Green for online, red for offline
          markers.push({
            id: vehicleId,
            position: [0, 0, 0],
            orientation: [1, 0, 0, 0],
            color: markerColor,
            showOrientation: false,
            label: v.name || vehicleId,
            lastUpdate: new Date(),
            name: v.name,
            vehicleType: v.vehicle_type,
            vehicleCategory: v.vehicle_category,
            status: vehicleStatus
          });
        }
      });
      
      setApiVehicles(vehiclesMap);
      setVehicleMarkers(prev => {
        // Merge with existing markers from MQTT
        const existingMap = new Map(prev.map(m => [m.id, m]));
        markers.forEach(marker => {
          existingMap.set(marker.id, marker);
        });
        return Array.from(existingMap.values());
      });
      
      console.log(`✅ [useVehiclePose2D] Fetched ${markers.length} vehicles from API`);
    } catch (err) {
      console.error('❌ [useVehiclePose2D] Failed to fetch vehicles from API:', err);
    }
  }, [vehicleService, mapMetadata, view, isExcludedVehicle, rotationMatrix]);
  
  // Fetch vehicles from API when metadata is loaded
  useEffect(() => {
    if (mapMetadata && vehicleService) {
      fetchVehiclesFromAPI();
    }
  }, [mapMetadata, vehicleService, fetchVehiclesFromAPI]);
  
  // Helper function to transform a pose update to marker
  const transformPoseToMarker = useCallback((update: PositionUpdateNotification, metadata: MapMetadata): VehicleMarker2D | null => {
    if (!update.vehicle_id) return null;
    
    // Extract pose from MQTT message
    let pose: Pose3D = {
      position: update.pose?.position || update.position,
      orientation: update.pose?.orientation || { x: 0, y: 0, z: 0, w: 1 },
      frame_id: update.pose?.frame_id || 'map',
      timestamp: update.pose?.timestamp || update.position.timestamp
    };
    
    // Validate pose data
    if (!pose.position || !pose.orientation) {
      console.warn('⚠️ [useVehiclePose2D] Pose missing position or orientation:', update);
      return null;
    }
    
    // Normalize orientation format
    const orient = pose.orientation;
    const normalizedOrientation = {
      x: orient.x ?? 0,
      y: orient.y ?? 0,
      z: orient.z ?? 0,
      w: orient.w ?? 1
    };
    pose.orientation = normalizedOrientation;
    
    // Apply rotation to pose if rotation matrix is available
    if (rotationMatrix) {
      pose = applyRotationToPose(pose, rotationMatrix);
    }
    
    // Transform 3D pose → 2D pixel coordinates
    const debugMode = process.env.NODE_ENV === 'development' || process.env.REACT_APP_DEBUG_LOGS === '1';
    const pixel = transformPoseToPixel(pose, metadata, view, debugMode);
    
    if (!pixel) {
      console.error('❌ [useVehiclePose2D] Transform failed for pose:', pose, 'view:', view);
      return null;
    }
    
    // Get vehicle info from API if available
    const apiVehicle = apiVehicles.get(update.vehicle_id);
    
    // Determine marker color based on status (position update means online)
    // But keep special colors for out of bounds and clamped
    let markerColor = '#22c55e'; // Default green for online (position update means online)
    if (pixel.is_out_of_bounds) {
      markerColor = '#fbbf24'; // Yellow for out of bounds
    } else if (pixel.is_clamped) {
      markerColor = '#f97316'; // Orange for clamped
    }
    
    // Create marker
    const yawHalf = pixel.yaw / 2;
    return {
      id: update.vehicle_id,
      position: [pixel.pixel_x, pixel.pixel_y, 0],
      orientation: [Math.cos(yawHalf), 0, 0, Math.sin(yawHalf)],
      color: markerColor,
      showOrientation: true,
      label: apiVehicle?.name || update.vehicle_id,
      lastUpdate: new Date(),
      name: apiVehicle?.name,
      vehicleType: apiVehicle?.vehicle_type,
      vehicleCategory: apiVehicle?.vehicle_category,
      status: 'online' // Position update means vehicle is online
    };
  }, [view, apiVehicles, rotationMatrix]);
  
  // Transform pose when MQTT update arrives
  useEffect(() => {
    if (!lastPositionUpdate) {
      return;
    }
    
    // Create a unique key for this update to avoid processing the same update twice
    const updateKey = `${lastPositionUpdate.vehicle_id}-${lastPositionUpdate.pose?.timestamp || lastPositionUpdate.position.timestamp}`;
    if (lastProcessedUpdateRef.current === updateKey) {
      return; // Already processed this update
    }
    
    if (!mapMetadata) {
      console.warn('⚠️ [useVehiclePose2D] Map metadata not loaded yet, skipping pose transform for vehicle:', lastPositionUpdate.vehicle_id);
      return;
    }
    
    console.log('🔍 [useVehiclePose2D] Processing pose update:', {
      vehicle_id: lastPositionUpdate.vehicle_id,
      hasPose: !!lastPositionUpdate.pose,
      hasPosition: !!lastPositionUpdate.position,
      mapMetadataLoaded: !!mapMetadata
    });
    
    const marker = transformPoseToMarker(lastPositionUpdate, mapMetadata);
    if (!marker) {
      return;
    }
    
    console.log('✅ [useVehiclePose2D] Transform successful:', {
      vehicle_id: lastPositionUpdate.vehicle_id,
      pixel_x: marker.position[0],
      pixel_y: marker.position[1]
    });
    
    // Update markers (merge: keep API info, update position from MQTT)
    setVehicleMarkers(prev => {
      const existingMarker = prev.find(m => m.id === marker.id);
      const apiVehicle = apiVehicles.get(marker.id);
      
      // Merge: use API info (name, type) and update position from MQTT
      // If we receive position update from MQTT, vehicle is definitely online
      // Ensure color is green for online, but keep special colors (yellow/orange) if they exist
      const finalColor = getMarkerColorByStatus('online', true, marker.color);
      
      const mergedMarker: VehicleMarker2D = {
        ...marker,
        color: finalColor, // Ensure green color for online
        label: apiVehicle?.name || existingMarker?.label || marker.id,
        name: apiVehicle?.name || existingMarker?.name,
        vehicleType: apiVehicle?.vehicle_type || existingMarker?.vehicleType,
        vehicleCategory: apiVehicle?.vehicle_category || existingMarker?.vehicleCategory,
        status: 'online' // Position update means vehicle is online
      };
      
      const filtered = prev.filter(m => m.id !== mergedMarker.id);
      const updated = [...filtered, mergedMarker];
      console.log('✅ [useVehiclePose2D] Updated markers, total:', updated.length);
      return updated;
    });
    
    lastProcessedUpdateRef.current = updateKey;
  }, [lastPositionUpdate, mapMetadata, transformPoseToMarker, apiVehicles]);
  
  // Handle vehicle status updates - update status instead of removing when offline
  useEffect(() => {
    if (!lastVehicleStatus) return;

    const vehicleId = lastVehicleStatus.vehicle_id;
    const newStatus = lastVehicleStatus.status;

    console.log('🔍 [useVehiclePose2D] Vehicle status update:', vehicleId, '->', newStatus);

    setVehicleMarkers(prev => {
      const existingMarker = prev.find(m => m.id === vehicleId);
      
      if (existingMarker) {
        // Marker exists - update status and color
        const validStatus: 'online' | 'offline' =
          newStatus === 'online' ? 'online'
          : newStatus === 'offline' ? 'offline'
          : existingMarker.status || 'offline';
        
        // Update color based on status: green for online, red for offline
        // But keep special colors (yellow/orange) if they exist
        const newColor = getMarkerColorByStatus(validStatus, true, existingMarker.color);
        
        return prev.map(marker => 
          marker.id === vehicleId 
            ? { ...marker, status: validStatus, color: newColor }
            : marker
        );
      } else if (newStatus === 'online' && mapMetadata) {
        // Marker doesn't exist but coming online - create from API data
        const apiVehicle = apiVehicles.get(vehicleId);
        if (apiVehicle && apiVehicle.latest_pose?.position) {
          let pose: Pose3D = {
            position: apiVehicle.latest_pose.position,
            orientation: apiVehicle.latest_pose.orientation || { x: 0, y: 0, z: 0, w: 1 },
            frame_id: 'map',
            timestamp: apiVehicle.latest_pose.timestamp || apiVehicle.updated_at || new Date().toISOString()
          };
          
          // Apply rotation to pose if rotation matrix is available
          if (rotationMatrix) {
            pose = applyRotationToPose(pose, rotationMatrix);
          }
          
          const debugMode = process.env.NODE_ENV === 'development' || process.env.REACT_APP_DEBUG_LOGS === '1';
          const pixel = transformPoseToPixel(pose, mapMetadata, view, debugMode);
          
          if (pixel) {
            // Determine marker color: green for online, but keep special colors for out of bounds/clamped
            let markerColor = getMarkerColorByStatus('online');
            if (pixel.is_out_of_bounds) {
              markerColor = '#fbbf24'; // Yellow for out of bounds
            } else if (pixel.is_clamped) {
              markerColor = '#f97316'; // Orange for clamped
            }
            
            const yawHalf = pixel.yaw / 2;
            const newMarker: VehicleMarker2D = {
              id: vehicleId,
              position: [pixel.pixel_x, pixel.pixel_y, 0],
              orientation: [Math.cos(yawHalf), 0, 0, Math.sin(yawHalf)],
              color: markerColor,
              showOrientation: true,
              label: apiVehicle.name || vehicleId,
              lastUpdate: new Date(pose.timestamp || Date.now()),
              name: apiVehicle.name,
              vehicleType: apiVehicle.vehicle_type,
              vehicleCategory: apiVehicle.vehicle_category,
              status: 'online'
            };
            
            console.log('✅ Created marker from API (coming online):', vehicleId);
            return [...prev, newMarker];
          }
        }
      }
      
      return prev;
    });
  }, [lastVehicleStatus, mapMetadata, view, apiVehicles, rotationMatrix]);

  // Log MQTT connection status changes
  useEffect(() => {
    if (isConnected) {
      console.log('✅ MQTT connected - ready to receive pose updates');
    } else {
      console.warn('⚠️ MQTT disconnected - pose updates paused');
    }
  }, [isConnected]);
  
  return {
    vehicleMarkers,
    mapMetadata,
    isLoading,
    error,
    refreshMetadata: fetchMetadata
  };
}
