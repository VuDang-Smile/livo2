import { useState, useEffect, useRef } from 'react';
import { mapImageCache } from '../utils/mapImageCache';

/**
 * Custom hook để load và quản lý map image với proper cleanup
 * 
 * Features:
 * - Load image từ cache manager
 * - Xử lý loading/error states
 * - Cleanup khi component unmount
 * - Abort loading nếu component unmount trước khi load xong
 * - Guard against state updates sau unmount
 */
export interface UseMapImageResult {
  image: HTMLImageElement | null;
  isLoading: boolean;
  error: Error | null;
}

/**
 * Hook để load map image với caching và proper cleanup
 * @param url URL của hình ảnh
 * @returns UseMapImageResult với image, isLoading, và error states
 */
export function useMapImage(url: string): UseMapImageResult {
  const [image, setImage] = useState<HTMLImageElement | null>(null);
  const [isLoading, setIsLoading] = useState<boolean>(true);
  const [error, setError] = useState<Error | null>(null);

  // Sử dụng ref để track mounted state và tránh race conditions
  const isMountedRef = useRef<boolean>(true);
  const abortControllerRef = useRef<AbortController | null>(null);

  useEffect(() => {
    // Reset mounted flag
    isMountedRef.current = true;
    setIsLoading(true);
    setError(null);
    setImage(null);

    // Tạo AbortController để có thể cancel loading
    const abortController = new AbortController();
    abortControllerRef.current = abortController;

    // Load image từ cache
    mapImageCache
      .loadImage(url, abortController.signal)
      .then((loadedImage) => {
        // Guard: chỉ update state nếu component vẫn còn mounted
        if (isMountedRef.current && !abortController.signal.aborted) {
          setImage(loadedImage);
          setIsLoading(false);
          setError(null);
        }
      })
      .catch((err: Error) => {
        // Guard: chỉ update state nếu component vẫn còn mounted
        // Và không phải do abort (abort không phải là error thực sự)
        if (
          isMountedRef.current &&
          !abortController.signal.aborted &&
          err.message !== 'Image loading aborted'
        ) {
          setError(err);
          setIsLoading(false);
          setImage(null);
        }
      });

    // Cleanup function: chạy khi component unmount hoặc url thay đổi
    return () => {
      // Đánh dấu component đã unmount
      isMountedRef.current = false;

      // Abort loading nếu đang load
      if (abortControllerRef.current && !abortControllerRef.current.signal.aborted) {
        abortControllerRef.current.abort();
      }

      // Release image từ cache (chỉ release một lần)
      mapImageCache.releaseImage(url);

      // Reset refs
      abortControllerRef.current = null;
    };
  }, [url]); // Chỉ re-run khi url thay đổi

  return {
    image,
    isLoading,
    error,
  };
}

