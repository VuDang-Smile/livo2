/**
 * Map Metadata Service
 * 
 * Chức năng: Xử lý các API calls liên quan đến map metadata
 * - Lấy latest map metadata
 * - Lấy cached map metadata theo map_id
 * 
 * Sử dụng: Trong các hooks và components cần map metadata cho coordinate transformation
 * API Base: /maps
 */

import { MapMetadata } from '../../types/mapMetadata';
import { getMapMetadataUrl } from '../../constants/mapConfig';

export class MapMetadataService {
  private cache: Map<string, MapMetadata> = new Map();

  constructor(private apiBaseUrl: string) {
    // Nhận API base URL qua dependency injection
  }

  /**
   * Get latest map metadata from current map
   * @deprecated API calls removed - use local files from /floorplan_2d/ instead
   */
  async getLatestMapMetadata(): Promise<MapMetadata> {
    // API calls removed - using local files instead
    // This method is kept for backward compatibility but should not be used
    throw new Error('getLatestMapMetadata is deprecated - use local files from /floorplan_2d/ instead');
  }

  /**
   * Get map metadata by map_id (with caching)
   */
  async getCachedMapMetadata(mapId: string): Promise<MapMetadata> {
    // Check cache first
    if (this.cache.has(mapId)) {
      return this.cache.get(mapId)!;
    }

    try {
      // Fetch metadata from API
      const metadataUrl = getMapMetadataUrl(mapId);
      const response = await fetch(metadataUrl, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        throw new Error(`Failed to get map metadata: ${response.statusText}`);
      }

      const metadata: MapMetadata = await response.json();

      // Cache the metadata
      this.cache.set(mapId, metadata);

      return metadata;
    } catch (error) {
      console.error(`Failed to get map metadata for ${mapId}:`, error);
      throw error;
    }
  }

  /**
   * Clear cache
   */
  clearCache(): void {
    this.cache.clear();
  }

  /**
   * Clear specific map from cache
   */
  clearCacheForMap(mapId: string): void {
    this.cache.delete(mapId);
  }
}
