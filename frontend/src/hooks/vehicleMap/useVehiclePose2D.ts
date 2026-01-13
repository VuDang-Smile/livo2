import { useState, useEffect, useCallback, useRef } from 'react';
import { useMQTT, PositionUpdateNotification } from '../../contexts/MQTTContext';
import { transformPoseToPixel } from '../../utils/coordinateTransform';
import { MapMetadata, Pose3D } from '../../types/mapMetadata';
import { VehicleMarker2D } from '../../types/vehicle';
import { UseVehicleMap2DResult } from '../../types/monitoring';
import { MAP_FLOORPLAN_METADATA_URL } from '../../config/dataSources';

/**
 * Hook to track vehicle positions on 2D floorplan
 * 
 * Subscribes to MQTT position updates, transforms 3D poses to 2D pixel coordinates,
 * and returns vehicle markers ready for rendering on 2D map.
 * 
 * @returns Vehicle markers, map metadata, loading state, and refresh function
 */
export function useVehiclePose2D(view: 'top' | 'side_x' | 'side_y' = 'top'): UseVehicleMap2DResult {
  const [mapMetadata, setMapMetadata] = useState<MapMetadata | null>(null);
  const [vehicleMarkers, setVehicleMarkers] = useState<VehicleMarker2D[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const lastProcessedUpdateRef = useRef<string | null>(null);
  
  const { lastPositionUpdate, lastVehicleStatus, isConnected } = useMQTT();
  
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
  
  // Helper function to transform a pose update to marker
  const transformPoseToMarker = useCallback((update: PositionUpdateNotification, metadata: MapMetadata): VehicleMarker2D | null => {
    if (!update.vehicle_id) return null;
    
    // Extract pose from MQTT message
    const pose: Pose3D = {
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
    
    // Transform 3D pose → 2D pixel coordinates
    const debugMode = process.env.NODE_ENV === 'development' || process.env.REACT_APP_DEBUG_LOGS === '1';
    const pixel = transformPoseToPixel(pose, metadata, view, debugMode);
    
    if (!pixel) {
      console.error('❌ [useVehiclePose2D] Transform failed for pose:', pose, 'view:', view);
      return null;
    }
    
    // Determine marker color
    let markerColor = '#ef4444'; // Default red
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
      label: update.vehicle_id,
      lastUpdate: new Date()
    };
  }, [view]);
  
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
    
    // Update markers (replace existing marker with same ID)
    setVehicleMarkers(prev => {
      const filtered = prev.filter(m => m.id !== marker.id);
      const updated = [...filtered, marker];
      console.log('✅ [useVehiclePose2D] Updated markers, total:', updated.length);
      return updated;
    });
    
    lastProcessedUpdateRef.current = updateKey;
  }, [lastPositionUpdate, mapMetadata, transformPoseToMarker]);
  
  // Clean up old markers (remove if not updated for 30 seconds)
  useEffect(() => {
    const interval = setInterval(() => {
      const now = Date.now();
      setVehicleMarkers(prev => {
        const filtered = prev.filter(marker => {
          const age = marker.lastUpdate ? now - marker.lastUpdate.getTime() : Infinity;
          return age < 30000; // 30 seconds
        });
        
        if (filtered.length < prev.length) {
          console.log(`🧹 Cleaned up ${prev.length - filtered.length} old markers`);
        }
        
        return filtered;
      });
    }, 5000); // Check every 5 seconds
    
    return () => clearInterval(interval);
  }, []);
  
  // Handle vehicle status updates - remove markers when status = offline
  useEffect(() => {
    if (!lastVehicleStatus) return;

    const vehicleId = lastVehicleStatus.vehicle_id;
    const newStatus = lastVehicleStatus.status;

    console.log('🔍 [useVehiclePose2D] Vehicle status update:', vehicleId, '->', newStatus);

    if (newStatus === 'offline') {
      // Remove marker from canvas when vehicle goes offline
      setVehicleMarkers(prev => {
        const filtered = prev.filter(marker => marker.id !== vehicleId);
        if (filtered.length < prev.length) {
          console.log('🗑️ [useVehiclePose2D] Removed marker for offline vehicle:', vehicleId);
        }
        return filtered;
      });
    }
  }, [lastVehicleStatus]);

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
