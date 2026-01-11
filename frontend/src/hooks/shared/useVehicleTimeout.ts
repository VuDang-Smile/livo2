import { useEffect, useRef } from 'react';

const DEFAULT_TIMEOUT_MS = 30000; // 30 seconds

/**
 * Hook to handle vehicle timeout cleanup
 * Removes vehicles that haven't updated in the timeout period
 * 
 * @param vehicleLastSeen - Map of vehicle ID to last seen timestamp (ms)
 * @param lastAnyPositionUpdate - Timestamp of last position update from any vehicle (ms)
 * @param onVehicleTimeout - Callback when a vehicle times out
 * @param onAllTimeout - Callback when all vehicles timeout (no updates for timeout period)
 * @param timeoutMs - Timeout period in milliseconds (default: 30s)
 */
export function useVehicleTimeout(
  vehicleLastSeen: Map<string, number>,
  lastAnyPositionUpdate: number | null,
  onVehicleTimeout: (vehicleId: string) => void,
  onAllTimeout: () => void,
  timeoutMs: number = DEFAULT_TIMEOUT_MS
) {
  const timeoutRef = useRef<NodeJS.Timeout | null>(null);
  const lastUpdateRef = useRef<number | null>(lastAnyPositionUpdate);

  useEffect(() => {
    // Update last update ref
    if (lastAnyPositionUpdate !== null) {
      lastUpdateRef.current = lastAnyPositionUpdate;
    }

    // Clear existing timeout
    if (timeoutRef.current) {
      clearTimeout(timeoutRef.current);
    }

    // Check for timed out vehicles
    const checkTimeouts = () => {
      const now = Date.now();
      const cutoff = now - timeoutMs;

      // Check individual vehicles
      vehicleLastSeen.forEach((lastSeen, vehicleId) => {
        if (lastSeen < cutoff) {
          onVehicleTimeout(vehicleId);
        }
      });

      // Check if all vehicles timed out (no updates for timeout period)
      if (lastUpdateRef.current !== null && (now - lastUpdateRef.current) > timeoutMs) {
        onAllTimeout();
      }
    };

    // Check immediately
    checkTimeouts();

    // Set up interval to check periodically
    timeoutRef.current = setInterval(checkTimeouts, 5000); // Check every 5 seconds

    return () => {
      if (timeoutRef.current) {
        clearInterval(timeoutRef.current);
      }
    };
  }, [vehicleLastSeen, lastAnyPositionUpdate, onVehicleTimeout, onAllTimeout, timeoutMs]);
}
