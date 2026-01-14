import { PositionUpdateNotification } from '../contexts/MQTTContext';

/**
 * Timestamp normalization helpers for MQTT and API data.
 *
 * Contract:
 * - Upstream services should send ISO8601 strings WITH timezone (UTC canonical, e.g. `...Z`).
 * - These helpers are defensive: chúng cố gắng normalize nhiều format, nhưng sẽ cảnh báo
 *   khi gặp timestamp thiếu timezone hoặc không parse được.
 */

type NormalizedResult = {
  isoString: string;
  epochMs: number;
  /** true nếu timestamp thiếu tz hoặc phải suy đoán/convert đặc biệt */
  inferred?: boolean;
};

/**
 * Internal: normalize any supported input to Date.
 */
function normalizeToDate(value: number | string | Date): NormalizedResult | null {
  let date: Date | null = null;
  let inferred = false;

  if (value instanceof Date) {
    date = value;
  } else if (typeof value === 'number') {
    // If timestamp is in seconds (likely < year 2000 ms), convert to milliseconds
    if (value < 946684800000) {
      inferred = true;
      date = new Date(value * 1000);
    } else {
      date = new Date(value);
    }
  } else if (typeof value === 'string') {
    // First, try as ISO string
    const parsed = Date.parse(value);
    if (!isNaN(parsed)) {
      date = new Date(parsed);
      // Nếu string không chứa 'Z' hoặc offset, rất có thể thiếu tz
      if (!/[zZ]|[+-]\d{2}:?\d{2}$/.test(value)) {
        console.warn('[timestampHelpers] Timestamp string missing timezone:', value);
        inferred = true;
      }
    } else {
      // Try as numeric string
      const num = parseFloat(value);
      if (!isNaN(num)) {
        if (num < 946684800000) {
          inferred = true;
          date = new Date(num * 1000);
        } else {
          date = new Date(num);
        }
      }
    }
  }

  if (!date || isNaN(date.getTime())) {
    return null;
  }

  const epochMs = date.getTime();
  const isoString = new Date(epochMs).toISOString(); // always UTC ISO string
  return { isoString, epochMs, inferred };
}

/**
 * Normalize timestamp from various formats to ISO string or epoch ms.
 * @param timestamp - Timestamp in various formats (number, string, Date)
 * @param asString - If true, return ISO string; if false, return number (ms)
 */
export function normalizeTimestamp(
  timestamp: number | string | Date | undefined | null,
  asString: boolean = true
): string | number {
  if (!timestamp) {
    const now = Date.now();
    return asString ? new Date(now).toISOString() : now;
  }

  const normalized = normalizeToDate(timestamp as any);
  if (!normalized) {
    console.warn('[timestampHelpers] Failed to normalize timestamp, falling back to now():', timestamp);
    const now = Date.now();
    return asString ? new Date(now).toISOString() : now;
  }

  return asString ? normalized.isoString : normalized.epochMs;
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
