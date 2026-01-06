/**
 * Image Cache Manager - Singleton pattern
 * Quản lý cache hình ảnh để tránh load duplicate và xử lý race conditions
 * 
 * Features:
 * - Reference counting để cleanup khi không còn component nào sử dụng
 * - Promise-based loading để tránh race conditions
 * - AbortController để cancel loading khi cần
 * - Proper cleanup để tránh memory leaks
 */

interface ImageCacheEntry {
  image: HTMLImageElement;
  loadPromise: Promise<HTMLImageElement>;
  abortController: AbortController;
  refCount: number;
  error: Error | null;
}

class MapImageCache {
  private cache: Map<string, ImageCacheEntry> = new Map();

  /**
   * Load image từ URL với caching và reference counting
   * @param url URL của hình ảnh
   * @param signal AbortSignal để cancel loading nếu cần
   * @returns Promise<HTMLImageElement>
   */
  loadImage(url: string, signal?: AbortSignal): Promise<HTMLImageElement> {
    // Nếu đã có trong cache và đã load thành công
    const existingEntry = this.cache.get(url);
    if (existingEntry && existingEntry.image.complete && !existingEntry.error) {
      existingEntry.refCount++;
      return Promise.resolve(existingEntry.image);
    }

    // Nếu đang load, return existing Promise để tránh race condition
    if (existingEntry && !existingEntry.image.complete && !existingEntry.error) {
      existingEntry.refCount++;
      return existingEntry.loadPromise;
    }

    // Tạo entry mới
    const abortController = new AbortController();
    
    // Nếu có signal từ bên ngoài, kết nối với abortController
    if (signal) {
      signal.addEventListener('abort', () => {
        abortController.abort();
      });
    }

    const image = new Image();
    image.crossOrigin = 'anonymous'; // Cho phép CORS nếu cần

    const loadPromise = new Promise<HTMLImageElement>((resolve, reject) => {
      // Check nếu đã bị abort trước khi bắt đầu
      if (abortController.signal.aborted) {
        reject(new Error('Image loading aborted'));
        return;
      }

      const onLoad = () => {
        if (abortController.signal.aborted) {
          this.cleanupImage(image);
          reject(new Error('Image loading aborted'));
          return;
        }
        
        // Remove listeners để tránh memory leak
        image.removeEventListener('load', onLoad);
        image.removeEventListener('error', onError);
        
        // Update entry với image đã load
        const entry = this.cache.get(url);
        if (entry) {
          entry.error = null;
        }
        
        resolve(image);
      };

      const onError = (event: ErrorEvent) => {
        // Remove listeners
        image.removeEventListener('load', onLoad);
        image.removeEventListener('error', onError);
        
        const error = new Error(`Failed to load image: ${url}`);
        
        // Cache error để tránh retry vô hạn
        const entry = this.cache.get(url);
        if (entry) {
          entry.error = error;
        }
        
        this.cleanupImage(image);
        reject(error);
      };

      image.addEventListener('load', onLoad);
      image.addEventListener('error', onError);

      // Start loading
      image.src = url;

      // Check nếu image đã được cache bởi browser (complete ngay lập tức)
      if (image.complete) {
        onLoad();
      }
    });

    // Tạo cache entry
    const entry: ImageCacheEntry = {
      image,
      loadPromise,
      abortController,
      refCount: 1,
      error: null,
    };

    this.cache.set(url, entry);

    return loadPromise;
  }

  /**
   * Release image reference
   * Giảm refCount và cleanup nếu không còn reference nào
   * @param url URL của hình ảnh
   */
  releaseImage(url: string): void {
    const entry = this.cache.get(url);
    if (!entry) return;

    entry.refCount--;

    // Nếu không còn reference nào, cleanup
    if (entry.refCount <= 0) {
      this.cleanupImage(entry.image);
      this.cache.delete(url);
    }
  }

  /**
   * Cleanup image object để release memory
   * @param image Image object cần cleanup
   */
  private cleanupImage(image: HTMLImageElement): void {
    // Abort loading nếu đang load
    if (!image.complete) {
      // Set src to empty để cancel loading
      image.src = '';
    }

    // Remove tất cả event listeners (nếu có)
    // Note: removeEventListener chỉ hoạt động nếu dùng cùng function reference
    // Nhưng trong trường hợp này, chúng ta đã remove trong onLoad/onError
    
    // Clear image source để release memory
    image.src = '';
  }

  /**
   * Clear toàn bộ cache
   * Useful cho testing hoặc khi cần reset
   */
  clearCache(): void {
    this.cache.forEach((entry) => {
      this.cleanupImage(entry.image);
    });
    this.cache.clear();
  }

  /**
   * Get cache entry (for debugging)
   * @param url URL của hình ảnh
   * @returns ImageCacheEntry | undefined
   */
  getCacheEntry(url: string): ImageCacheEntry | undefined {
    return this.cache.get(url);
  }
}

// Export singleton instance
export const mapImageCache = new MapImageCache();

