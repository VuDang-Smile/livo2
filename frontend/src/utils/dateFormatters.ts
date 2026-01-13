/**
 * Utility functions for date/time formatting
 */

/**
 * Format ISO 8601 datetime string to display format: "YYYY-MM-DD HH:mm:ss"
 * @param isoString - ISO 8601 datetime string (e.g., "2026-01-13T08:27:10.079823+00:00")
 * @returns Formatted datetime string (e.g., "2026-01-13 08:27:10")
 */
export function formatISOToDisplay(isoString: string): string {
  try {
    const date = new Date(isoString);
    
    // Check if date is valid
    if (isNaN(date.getTime())) {
      console.warn(`Invalid ISO date string: ${isoString}`);
      return isoString; // Return original string if invalid
    }
    
    // Format: YYYY-MM-DD HH:mm:ss
    const year = date.getFullYear();
    const month = String(date.getMonth() + 1).padStart(2, '0');
    const day = String(date.getDate()).padStart(2, '0');
    const hours = String(date.getHours()).padStart(2, '0');
    const minutes = String(date.getMinutes()).padStart(2, '0');
    const seconds = String(date.getSeconds()).padStart(2, '0');
    
    return `${year}-${month}-${day} ${hours}:${minutes}:${seconds}`;
  } catch (error) {
    console.error(`Error formatting date: ${isoString}`, error);
    return isoString; // Return original string on error
  }
}
