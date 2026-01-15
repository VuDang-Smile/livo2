import { useMemo } from 'react';
import { PositionUpdateNotification } from '../../contexts/MQTTContext';

/**
 * Hook to extract position/pose data from MQTT position updates
 * Provides utilities to filter excluded vehicles and extract position data
 */
export function useMQTTPositionHandler() {
  // List of vehicle IDs to exclude from updates
  const excludedVehicles = useMemo(() => {
    return new Set<string>([
      // Add vehicle IDs to exclude here if needed
      // Example: 'test_vehicle_1', 'debug_vehicle'
    ]);
  }, []);

  /**
   * Check if a vehicle ID should be excluded
   */
  const isExcludedVehicle = useMemo(() => {
    return (vehicleId: string): boolean => {
      return excludedVehicles.has(vehicleId);
    };
  }, [excludedVehicles]);

  /**
   * Extract pose data (position + orientation) from MQTT update
   * Returns position as [x, y, z] tuple and optional orientation as quaternion
   */
  const extractPose = useMemo(() => {
    return (update: PositionUpdateNotification | null | undefined): {
      position: [number, number, number];
      orientation?: [number, number, number, number];
      showOrientation: boolean;
    } | null => {
      if (!update) return null;

      let position: [number, number, number];
      let orientation: [number, number, number, number] | undefined;
      let showOrientation = false;

      // Prefer pose data if available
      if (update.pose?.position) {
        const pos = update.pose.position;
        position = [pos.x, pos.y, pos.z];
        
        if (update.pose.orientation) {
          const orient = update.pose.orientation;
          orientation = [orient.w, orient.x, orient.y, orient.z];
          showOrientation = true;
        }
      } else if (update.position) {
        // Fallback to position only
        position = [update.position.x, update.position.y, update.position.z];
      } else {
        return null;
      }

      return {
        position,
        orientation,
        showOrientation
      };
    };
  }, []);

  /**
   * Extract position only (without orientation) from MQTT update
   * Returns position as {x, y, z} object
   */
  const extractPosition = useMemo(() => {
    return (update: PositionUpdateNotification | null | undefined): { x: number; y: number; z: number } | null => {
      if (!update) return null;

      // Prefer pose position
      if (update.pose?.position) {
        return update.pose.position;
      }

      // Fallback to position
      if (update.position) {
        return {
          x: update.position.x,
          y: update.position.y,
          z: update.position.z
        };
      }

      return null;
    };
  }, []);

  return {
    extractPose,
    extractPosition,
    isExcludedVehicle
  };
}
