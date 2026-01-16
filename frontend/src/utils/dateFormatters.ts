/**
 * Utility functions for date/time formatting.
 *
 * Contract:
 * - Input timestamps are ISO8601 strings with timezone (UTC canonical, e.g. `...Z`).
 * - These helpers are only responsible for presentation; they must not change the
 *   underlying instant in time.
 */

/**
 * Safely parse ISO8601 string into Date.
 * Returns null if invalid.
 */
export function parseISOToDate(isoString: string): Date | null {
  try {
    const date = new Date(isoString);
    if (isNaN(date.getTime())) {
      console.warn(`Invalid ISO date string: ${isoString}`);
      return null;
    }
    return date;
  } catch (error) {
    console.error(`Error parsing date: ${isoString}`, error);
    return null;
  }
}

/**
 * Format ISO 8601 datetime string to display format: "YYYY-MM-DD HH:mm:ss"
 * using the browser's local timezone.
 *
 * @param isoString - ISO 8601 datetime string with timezone
 * @returns Formatted datetime string (e.g., "2026-01-13 08:27:10")
 */
export function formatISOToLocalDisplay(isoString: string): string {
  const date = parseISOToDate(isoString);
  if (!date) return isoString;

  const year = date.getFullYear();
  const month = String(date.getMonth() + 1).padStart(2, '0');
  const day = String(date.getDate()).padStart(2, '0');
  const hours = String(date.getHours()).padStart(2, '0');
  const minutes = String(date.getMinutes()).padStart(2, '0');
  const seconds = String(date.getSeconds()).padStart(2, '0');

  return `${year}-${month}-${day} ${hours}:${minutes}:${seconds}`;
}

/**
 * Format ISO 8601 datetime string to display in UTC: "YYYY-MM-DD HH:mm:ss"
 * Useful for logs/debug where absolute UTC is preferred.
 */
export function formatISOToUTCDisplay(isoString: string): string {
  const date = parseISOToDate(isoString);
  if (!date) return isoString;

  const year = date.getUTCFullYear();
  const month = String(date.getUTCMonth() + 1).padStart(2, '0');
  const day = String(date.getUTCDate()).padStart(2, '0');
  const hours = String(date.getUTCHours()).padStart(2, '0');
  const minutes = String(date.getUTCMinutes()).padStart(2, '0');
  const seconds = String(date.getUTCSeconds()).padStart(2, '0');

  return `${year}-${month}-${day} ${hours}:${minutes}:${seconds} UTC`;
}

