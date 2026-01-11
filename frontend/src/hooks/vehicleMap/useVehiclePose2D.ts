import { useState, useEffect, useCallback } from 'react';
import { useMQTT } from '../../contexts/MQTTContext';
import { transformPoseToPixel } from '../../utils/coordinateTransform';
import { MapMetadata, Pose3D } from '../../types/mapMetadata';
import { VehicleMarker2D } from '../../types/vehicle';
import { UseVehiclePose2DResult } from '../../types/monitoring';
import { useMapMetadataService } from '../api/useMapMetadataService';

/**
 * Hook to track vehicle positions on 2D floorplan
 * 
 * Subscribes to MQTT position updates, transforms 3D poses to 2D pixel coordinates,
 * and returns vehicle markers ready for rendering on 2D map.
 * 
 * @returns Vehicle markers, map metadata, loading state, and refresh function
 */
export function useVehiclePose2D(view: 'top' | 'side_x' | 'side_y' = 'top'): UseVehiclePose2DResult {
  const mapMetadataService = useMapMetadataService();
  const [mapMetadata, setMapMetadata] = useState<MapMetadata | null>(null);
  const [vehicleMarkers, setVehicleMarkers] = useState<VehicleMarker2D[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  
  const { lastPositionUpdate, isConnected } = useMQTT();
  
  // Fetch map metadata on mount
  const fetchMetadata = useCallback(async () => {
    if (!mapMetadataService) return; // Skip if service not ready
    try {
      setIsLoading(true);
      setError(null);
      
      console.log('🗺️ Fetching latest map metadata...');
      const metadata = await mapMetadataService.getLatestMapMetadata();
      
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
    } catch (err: any) {
      console.error('❌ Failed to load map metadata:', err);
      setError(err.message || 'Failed to load map metadata');
    } finally {
      setIsLoading(false);
    }
  }, [mapMetadataService, view]);
  
  useEffect(() => {
    fetchMetadata();
  }, [fetchMetadata]);
  
  // Transform pose when MQTT update arrives
  useEffect(() => {
    if (!lastPositionUpdate || !mapMetadata) {
      return;
    }
    
    // Extract pose from MQTT message (pose is nested in lastPositionUpdate)
    const pose: Pose3D = {
      position: lastPositionUpdate.pose?.position || lastPositionUpdate.position,
      orientation: lastPositionUpdate.pose?.orientation || { x: 0, y: 0, z: 0, w: 1 },
      frame_id: lastPositionUpdate.pose?.frame_id || 'map',
      timestamp: lastPositionUpdate.pose?.timestamp || lastPositionUpdate.position.timestamp
    };
    
    // Validate pose data
    if (!pose.position || !pose.orientation) {
      console.warn('MQTT message missing position or orientation:', lastPositionUpdate);
      return;
    }
    
    // Check if we need different map metadata (if map_id is provided)
    if (lastPositionUpdate.map_id && mapMetadataService) {
      // Try to load new metadata if map_id is different
      // Note: We don't have file_id in MapMetadata, so we'll skip this check for now
      // This can be enhanced later if needed
    }
    
    // Transform 3D pose → 2D pixel coordinates
    const debugMode = process.env.NODE_ENV === 'development' || process.env.REACT_APP_DEBUG_LOGS === '1';
    const pixel = transformPoseToPixel(pose, mapMetadata, view, debugMode);
    
    if (!pixel) {
      console.error('Transform failed for pose:', pose);
      return;
    }
    
    // Determine marker color based on status
    let markerColor = '#ef4444'; // Default red
    if (pixel.is_out_of_bounds) {
      markerColor = '#fbbf24'; // Yellow for out of bounds
    } else if (pixel.is_clamped) {
      markerColor = '#f97316'; // Orange for clamped
    }
    
    // Create or update marker
    // Create valid quaternion for yaw-only rotation around Z-axis: [w, x, y, z] = [cos(yaw/2), 0, 0, sin(yaw/2)]
    const yawHalf = pixel.yaw / 2;
    const marker: VehicleMarker2D = {
      id: lastPositionUpdate.vehicle_id,
      position: [pixel.pixel_x, pixel.pixel_y, 0],  // Z=0 for 2D
      orientation: [Math.cos(yawHalf), 0, 0, Math.sin(yawHalf)],  // Valid quaternion for Z-axis rotation
      color: markerColor,
      showOrientation: true,
      label: lastPositionUpdate.vehicle_id,
      lastUpdate: new Date()
    };
    
    // Update markers (replace existing marker with same ID)
    setVehicleMarkers(prev => {
      const filtered = prev.filter(m => m.id !== marker.id);
      return [...filtered, marker];
    });
    
  }, [lastPositionUpdate, mapMetadata, mapMetadataService, view]);
  
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
