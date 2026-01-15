import { PositionUpdateNotification } from '../contexts/MQTTContext';

/**
 * Normalize timestamp from various formats to ISO string
 * @param timestamp - Timestamp in various formats (number, string, Date)
 * @param asString - If true, return as ISO string; if false, return as number (ms)
 * @returns Normalized timestamp
 */
export function normalizeTimestamp(timestamp: number | string | Date | undefined | null, asString: boolean = true): string | number {
  if (!timestamp) {
    const now = Date.now();
    return asString ? new Date(now).toISOString() : now;
  }

  if (timestamp instanceof Date) {
    return asString ? timestamp.toISOString() : timestamp.getTime();
  }

  if (typeof timestamp === 'number') {
    // If timestamp is in seconds (less than year 2000), convert to milliseconds
    if (timestamp < 946684800000) {
      timestamp = timestamp * 1000;
    }
    return asString ? new Date(timestamp).toISOString() : timestamp;
  }

  if (typeof timestamp === 'string') {
    // Try to parse as ISO string or number
    const parsed = Date.parse(timestamp);
    if (!isNaN(parsed)) {
      return asString ? new Date(parsed).toISOString() : parsed;
    }
    // If it's a number string, parse it
    const num = parseFloat(timestamp);
    if (!isNaN(num)) {
      if (num < 946684800000) {
        return asString ? new Date(num * 1000).toISOString() : num * 1000;
      }
      return asString ? new Date(num).toISOString() : num;
    }
  }

  // Fallback to current time
  const now = Date.now();
  return asString ? new Date(now).toISOString() : now;
}

/**
 * Extract timestamp string from MQTT position update
 * @param update - Position update notification
 * @param asString - If true, return as ISO string; if false, return as number (ms)
 * @returns Timestamp string or number
 */
export function extractTimestampStringFromMQTT(
  update: PositionUpdateNotification | null | undefined,
  asString: boolean = true
): string | number {
  if (!update) {
    const now = Date.now();
    return asString ? new Date(now).toISOString() : now;
  }

  // Try pose timestamp first
  if (update.pose?.timestamp) {
    return normalizeTimestamp(update.pose.timestamp, asString);
  }

  // Fallback to position timestamp
  if (update.position?.timestamp) {
    return normalizeTimestamp(update.position.timestamp, asString);
  }

  // Fallback to current time
  const now = Date.now();
  return asString ? new Date(now).toISOString() : now;
}

/**
 * Extract timestamp in milliseconds from MQTT position update
 * @param update - Position update notification
 * @returns Timestamp in milliseconds, or null if not available
 */
export function extractTimestampMsFromMQTT(
  update: PositionUpdateNotification | null | undefined
): number | null {
  if (!update) return null;

  // Try pose timestamp first
  if (update.pose?.timestamp) {
    const normalized = normalizeTimestamp(update.pose.timestamp, false);
    return typeof normalized === 'number' ? normalized : null;
  }

  // Fallback to position timestamp
  if (update.position?.timestamp) {
    const normalized = normalizeTimestamp(update.position.timestamp, false);
    return typeof normalized === 'number' ? normalized : null;
  }

  return null;
}
