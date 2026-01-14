import { useState, useEffect, useCallback, useRef } from 'react';
import { QRCodeService } from '../../services/api/qrCodeService';
import { QRCodeInfo } from '../../types/qrCode';
import { QR_DETECT_URL } from '../../config/dataSources';

/**
 * Hook result interface
 */
export interface UseQRCodesResult {
  qrCodes: QRCodeInfo[];
  isLoading: boolean;
  error: string | null;
  refetch: () => Promise<void>;
}

/**
 * Hook to fetch and manage QR codes from storage
 * 
 * Features:
 * - Automatic fetch on mount
 * - Loading and error state management
 * - Proper cleanup on unmount
 * - Manual refetch capability
 * 
 * @returns UseQRCodesResult with qrCodes, isLoading, error, and refetch function
 */
export function useQRCodes(): UseQRCodesResult {
  const [qrCodes, setQrCodes] = useState<QRCodeInfo[]>([]);
  const [isLoading, setIsLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);
  
  // Use ref to track mounted state and prevent state updates after unmount
  const isMountedRef = useRef<boolean>(true);
  const abortControllerRef = useRef<AbortController | null>(null);
  
  // Create service instance
  const service = useRef<QRCodeService | null>(null);
  
  useEffect(() => {
    // Initialize service
    service.current = new QRCodeService(QR_DETECT_URL);
    
    // Reset mounted flag
    isMountedRef.current = true;
    
    return () => {
      // Cleanup: mark as unmounted
      isMountedRef.current = false;
      
      // Abort any pending fetch
      if (abortControllerRef.current) {
        abortControllerRef.current.abort();
      }
    };
  }, []);
  
  /**
   * Fetch QR codes from storage
   */
  const fetchQRCodes = useCallback(async () => {
    if (!service.current) {
      console.warn('[useQRCodes] Service not initialized');
      return;
    }
    
    // Abort previous request if exists
    if (abortControllerRef.current) {
      abortControllerRef.current.abort();
    }
    
    // Create new abort controller
    const abortController = new AbortController();
    abortControllerRef.current = abortController;
    
    // Only update loading state if component is still mounted
    if (isMountedRef.current) {
      setIsLoading(true);
      setError(null);
    }
    
    try {
      // Pass abort signal to service for proper request cancellation
      const codes = await service.current.getQRCodes(abortController.signal);
      
      // Guard: only update state if component is still mounted and not aborted
      if (isMountedRef.current && !abortController.signal.aborted) {
        setQrCodes(codes);
        setIsLoading(false);
        setError(null);
      }
    } catch (err: any) {
      // Don't update state if request was aborted (component unmounted or new request started)
      if (err?.name === 'AbortError') {
        return; // Silently ignore abort errors
      }
      
      // Guard: only update state if component is still mounted and not aborted
      if (isMountedRef.current && !abortController.signal.aborted) {
        const errorMessage = err?.message || 'Failed to fetch QR codes';
        setError(errorMessage);
        setIsLoading(false);
        setQrCodes([]);
        console.error('[useQRCodes] Failed to fetch QR codes:', err);
      }
    } finally {
      // Clear abort controller reference
      if (abortControllerRef.current === abortController) {
        abortControllerRef.current = null;
      }
    }
  }, []);
  
  // Fetch on mount
  useEffect(() => {
    fetchQRCodes();
  }, [fetchQRCodes]);
  
  return {
    qrCodes,
    isLoading,
    error,
    refetch: fetchQRCodes,
  };
}
