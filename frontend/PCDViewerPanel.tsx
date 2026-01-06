import React, {
  useState,
  useEffect,
  useCallback,
  forwardRef,
  useImperativeHandle,
  createContext,
  useContext,
  PropsWithChildren,
  useMemo,
} from 'react';
import { PCDLoader } from 'three/examples/jsm/loaders/PCDLoader';
import { PointCloudViewer, ViewerShell } from '../../components';
import { PointCloudData } from '../../types/viewer';
import { Range } from 'react-range';
import PCDQrPanel from './PCDQrPanel';
import { PCDViewerPanelHandle, PCDViewerPanelProps } from '../../types/map';
import { usePcdService } from '../../hooks/api/usePcdService';
import AnimatedInfoCard from '../../components/map-scanning/AnimatedInfoCard';
import { RecentPcdItem } from '../../types/pcd';

interface PCDViewerContextValue {
  t: (k: string) => string;
  pointCloudData: PointCloudData | null;
  loadingPCD: boolean;
  clipXRange: [number, number];
  clipYRange: [number, number];
  clipZRange: [number, number];
  setClipXRange: (range: [number, number]) => void;
  setClipYRange: (range: [number, number]) => void;
  setClipZRange: (range: [number, number]) => void;
  getRealXRange: () => [number, number];
  getRealYRange: () => [number, number];
  getRealZRange: () => [number, number];
  activePcdMeta?: RecentPcdItem | null;
  error: string | null;
  currentPcdFileId?: string;
  isViewerFullscreen?: boolean;
  onToggleFullscreen?: () => void;
}

const PCDViewerContext = createContext<PCDViewerContextValue | null>(null);

export const usePcdViewerContext = () => {
  const ctx = useContext(PCDViewerContext);
  if (!ctx) {
    throw new Error('PCDViewer components must be used within PCDViewerPanel');
  }
  return ctx;
};

const PCDViewerPanel = forwardRef<PCDViewerPanelHandle, PropsWithChildren<PCDViewerPanelProps>>(({
  t,
  onCurrentFileChange,
  activePcdMeta,
  isViewerFullscreen = false,
  onToggleFullscreen,
  children,
}, ref) => {
  const pcdService = usePcdService();
  const [pointCloudData, setPointCloudData] = useState<PointCloudData | null>(null);
  const [clipZRange, setClipZRange] = useState<[number, number]>([0, 100]);
  const [clipXRange, setClipXRange] = useState<[number, number]>([0, 100]);
  const [clipYRange, setClipYRange] = useState<[number, number]>([0, 100]);
  const [error, setError] = useState<string | null>(null);
  const [loadingPCD, setLoadingPCD] = useState(false);
  const [currentPcdFileId, setCurrentPcdFileId] = useState<string | undefined>(undefined);

  const parsePCDWithPCDLoader = useCallback(async (pcdBuffer: ArrayBuffer): Promise<PointCloudData> => {
    const loader = new PCDLoader();
    const pointsObj: any = loader.parse(pcdBuffer);
    const geometry: any = pointsObj.geometry;

    const positionsAttr: any = geometry.getAttribute('position');
    const colorsAttr: any = geometry.getAttribute('color');

    const positionsArray = positionsAttr ? Array.from(positionsAttr.array as Float32Array) : [];
    let colorsArray: number[] = [];
    if (colorsAttr && colorsAttr.array) {
      colorsArray = Array.from(colorsAttr.array as Float32Array);
    } else {
      // Fallback: color by Z
      colorsArray = new Array(positionsArray.length).fill(0);
      let minZ = Infinity;
      let maxZ = -Infinity;
      for (let i = 2; i < positionsArray.length; i += 3) {
        const z = positionsArray[i];
        if (z < minZ) minZ = z;
        if (z > maxZ) maxZ = z;
      }
      const range = maxZ - minZ || 1;
      for (let i = 0; i < positionsArray.length; i += 3) {
        const z = positionsArray[i + 2];
        const normalized = (z - minZ) / range;
        colorsArray[i] = Math.min(1, Math.max(0, normalized));
        colorsArray[i + 1] = Math.min(1, Math.max(0, 1 - normalized));
        colorsArray[i + 2] = 0.5;
      }
    }

    // Bounds
    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;
    for (let i = 0; i < positionsArray.length; i += 3) {
      const x = positionsArray[i];
      const y = positionsArray[i + 1];
      const z = positionsArray[i + 2];
      if (x < minX) minX = x; if (x > maxX) maxX = x;
      if (y < minY) minY = y; if (y > maxY) maxY = y;
      if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
    }

    return {
      positions: positionsArray,
      colors: colorsArray,
      bounds: { minX, maxX, minY, maxY, minZ, maxZ },
      numPoints: positionsArray.length / 3,
    };
  }, []);

  // Download PCD by file_id from backend
  const downloadPCDById = useCallback(async (fileId: string) => {
    if (!pcdService) return;
    setLoadingPCD(true);
    setError(null);
    try {
      const pcdBuffer = await pcdService.downloadPcdFile(fileId);
      const pcdData = await parsePCDWithPCDLoader(pcdBuffer);
      setPointCloudData(pcdData);
      setCurrentPcdFileId(fileId);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to download PCD');
    } finally {
      setLoadingPCD(false);
    }
  }, [pcdService, parsePCDWithPCDLoader]);

  const fetchLatestPCD = useCallback(async () => {
    if (!pcdService) return;
    setLoadingPCD(true);
    setError(null);
    
    try {
      const list = await pcdService.getPcdFiles({ limit: 1 });
      const first = Array.isArray(list) && list.length > 0 ? list[0] : null;
      if (first?.file_id) {
        await downloadPCDById(first.file_id);
      } else {
        throw new Error(t('pcd_no_pcd_available'));
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to load latest PCD');
    } finally {
      setLoadingPCD(false);
    }
  }, [pcdService, downloadPCDById]);

  const loadLatestPCD = () => {
    fetchLatestPCD();
  };

  // Loại bỏ logic tự động load khi có MAP_UPDATE
  // Chỉ load PCD khi mount lần đầu hoặc khi được gọi từ parent component

  // Set mặc định range slider = full range (0-100%) khi có dữ liệu mới
  useEffect(() => {
    if (pointCloudData && (clipZRange[0] === 0 && clipZRange[1] === 0)) {
      setClipZRange([0, 100]);
      setClipXRange([0, 100]);
      setClipYRange([0, 100]);
    }
  }, [pointCloudData]); // Chỉ cần phụ thuộc vào pointCloudData

  // Hàm chuyển đổi từ phần trăm sang giá trị thực
  const getRealValue = (percentage: number, min: number, max: number) => {
    return min + (percentage / 100) * (max - min);
  };

  // Lấy giá trị thực từ range slider
  const getRealXRange = () => {
    if (!pointCloudData) return [0, 0];
    return [
      getRealValue(clipXRange[0], pointCloudData.bounds.minX, pointCloudData.bounds.maxX),
      getRealValue(clipXRange[1], pointCloudData.bounds.minX, pointCloudData.bounds.maxX)
    ];
  };

  const getRealYRange = () => {
    if (!pointCloudData) return [0, 0];
    return [
      getRealValue(clipYRange[0], pointCloudData.bounds.minY, pointCloudData.bounds.maxY),
      getRealValue(clipYRange[1], pointCloudData.bounds.minY, pointCloudData.bounds.maxY)
    ];
  };

  const getRealZRange = () => {
    if (!pointCloudData) return [0, 0];
    return [
      getRealValue(clipZRange[0], pointCloudData.bounds.minZ, pointCloudData.bounds.maxZ),
      getRealValue(clipZRange[1], pointCloudData.bounds.minZ, pointCloudData.bounds.maxZ)
    ];
  };

  useImperativeHandle(ref, () => ({
    loadLatest: fetchLatestPCD,
    loadById: downloadPCDById
  }));

  useEffect(() => {
    onCurrentFileChange?.(currentPcdFileId);
  }, [currentPcdFileId, onCurrentFileChange]);

  const contextValue: PCDViewerContextValue = useMemo(
    () => ({
      t,
      pointCloudData,
      loadingPCD,
      clipXRange,
      clipYRange,
      clipZRange,
      setClipXRange: (range: [number, number]) => setClipXRange(range),
      setClipYRange: (range: [number, number]) => setClipYRange(range),
      setClipZRange: (range: [number, number]) => setClipZRange(range),
      getRealXRange: getRealXRange as () => [number, number],
      getRealYRange: getRealYRange as () => [number, number],
      getRealZRange: getRealZRange as () => [number, number],
      activePcdMeta,
      error,
      currentPcdFileId,
      isViewerFullscreen,
      onToggleFullscreen,
    }),
    [
      t,
      pointCloudData,
      loadingPCD,
      clipXRange,
      clipYRange,
      clipZRange,
      activePcdMeta,
      error,
      currentPcdFileId,
      isViewerFullscreen,
      onToggleFullscreen,
    ]
  );

  return (
    <PCDViewerContext.Provider value={contextValue}>
      {children}
    </PCDViewerContext.Provider>
  );
});

PCDViewerPanel.displayName = 'PCDViewerPanel';

export const PCDViewerControls: React.FC = () => {
  const {
    t,
    pointCloudData,
    clipXRange,
    clipYRange,
    clipZRange,
    setClipXRange,
    setClipYRange,
    setClipZRange,
    activePcdMeta,
    error,
    loadingPCD,
    getRealXRange,
    getRealYRange,
    getRealZRange,
    isViewerFullscreen,
  } = usePcdViewerContext();

  if (isViewerFullscreen) return null;

  return (
    <div className="space-y-4">
      {pointCloudData && (
        <div className="space-y-4">
          <div className="flex items-center gap-4">
            <label className="text-sm text-gray-700">{t('pcd_x_range')}</label>
            <div className="w-64">
              <Range
                step={1}
                min={0}
                max={100}
                values={clipXRange}
                onChange={(values) => setClipXRange(values as [number, number])}
                renderTrack={({ props, children }) => (
                  <div {...props} style={{ ...props.style, height: '6px', width: '100%', backgroundColor: '#ddd', borderRadius: '3px' }}>
                    <div
                      style={{
                        position: 'absolute',
                        height: '6px',
                        width: `${((clipXRange[1] - clipXRange[0]) / 100) * 100}%`,
                        left: `${(clipXRange[0] / 100) * 100}%`,
                        backgroundColor: '#548BF4',
                        borderRadius: '3px'
                      }}
                    />
                    {children}
                  </div>
                )}
                renderThumb={({ props }) => (
                  <div {...props} style={{ ...props.style, height: '20px', width: '20px', backgroundColor: '#548BF4', borderRadius: '50%', border: '2px solid white', boxShadow: '0 2px 4px rgba(0,0,0,0.2)' }} />
                )}
              />
            </div>
            <div className="text-sm text-gray-600">
              {clipXRange[0].toFixed(0)}% - {clipXRange[1].toFixed(0)}%
            </div>
          </div>
          
          <div className="flex items-center gap-4">
            <label className="text-sm text-gray-700">{t('pcd_y_range')}</label>
            <div className="w-64">
              <Range
                step={1}
                min={0}
                max={100}
                values={clipYRange}
                onChange={(values) => setClipYRange(values as [number, number])}
                renderTrack={({ props, children }) => (
                  <div {...props} style={{ ...props.style, height: '6px', width: '100%', backgroundColor: '#ddd', borderRadius: '3px' }}>
                    <div
                      style={{
                        position: 'absolute',
                        height: '6px',
                        width: `${((clipYRange[1] - clipYRange[0]) / 100) * 100}%`,
                        left: `${(clipYRange[0] / 100) * 100}%`,
                        backgroundColor: '#548BF4',
                        borderRadius: '3px'
                      }}
                    />
                    {children}
                  </div>
                )}
                renderThumb={({ props }) => (
                  <div {...props} style={{ ...props.style, height: '20px', width: '20px', backgroundColor: '#548BF4', borderRadius: '50%', border: '2px solid white', boxShadow: '0 2px 4px rgba(0,0,0,0.2)' }} />
                )}
              />
            </div>
            <div className="text-sm text-gray-600">
              {clipYRange[0].toFixed(0)}% - {clipYRange[1].toFixed(0)}%
            </div>
          </div>
          
          <div className="flex items-center gap-4">
            <label className="text-sm text-gray-700">{t('pcd_z_range')}</label>
            <div className="w-64">
              <Range
                step={1}
                min={0}
                max={100}
                values={clipZRange}
                onChange={(values) => setClipZRange(values as [number, number])}
                renderTrack={({ props, children }) => (
                  <div {...props} style={{ ...props.style, height: '6px', width: '100%', backgroundColor: '#ddd', borderRadius: '3px' }}>
                    <div
                      style={{
                        position: 'absolute',
                        height: '6px',
                        width: `${((clipZRange[1] - clipZRange[0]) / 100) * 100}%`,
                        left: `${(clipZRange[0] / 100) * 100}%`,
                        backgroundColor: '#548BF4',
                        borderRadius: '3px'
                      }}
                    />
                    {children}
                  </div>
                )}
                renderThumb={({ props }) => (
                  <div {...props} style={{ ...props.style, height: '20px', width: '20px', backgroundColor: '#548BF4', borderRadius: '50%', border: '2px solid white', boxShadow: '0 2px 4px rgba(0,0,0,0.2)' }} />
                )}
              />
            </div>
            <div className="text-sm text-gray-600">
              {clipZRange[0].toFixed(0)}% - {clipZRange[1].toFixed(0)}%
            </div>
          </div>
        </div>
      )}

      {activePcdMeta && (
        <AnimatedInfoCard
          hasNewNotification={false}
          color="blue"
          title={t('pcd_current_viewing_title')}
        >
          <div className="space-y-2">
            <div className="text-base">
              <span className="font-medium text-blue-900">{activePcdMeta.filename}</span>
              {activePcdMeta.uploaded_at_formatted && (
                <span className="text-blue-600 text-sm ml-2">({activePcdMeta.uploaded_at_formatted})</span>
              )}
            </div>
            <div className="grid grid-cols-2 gap-2 text-sm">
              <div>
                <span className="text-blue-700 font-medium">{t('pcd_uploaded_by')}:</span>
                <div className="text-blue-900">{activePcdMeta.uploaded_by}</div>
              </div>
              <div>
                <span className="text-blue-700 font-medium">{t('pcd_size')}:</span>
                <div className="text-blue-900">{activePcdMeta.file_size_formatted}</div>
              </div>
            </div>
            <div className="font-mono text-sm text-blue-600 truncate">
              <span className="font-semibold text-blue-800">{t('pcd_id')}:</span> {activePcdMeta.file_id}
            </div>
          </div>
        </AnimatedInfoCard>
      )}

      {error && (
        <div className="mt-4 p-3 bg-red-50 border border-red-200 rounded-lg">
          <p className="text-red-700">{error}</p>
        </div>
      )}

      {loadingPCD && (
        <div className="mt-4 p-3 bg-blue-50 border border-blue-200 rounded-lg">
          <p className="text-blue-700">{t('loading_pcd')}</p>
        </div>
      )}

      {pointCloudData && (
        <div className="mt-4 p-3 bg-green-50 border border-green-200 rounded-lg">
          <p className="text-green-700">
            {t('pcd_loaded')} - {pointCloudData.numPoints.toLocaleString()} {t('pcd_points_rendered')}
          </p>
          <p className="text-sm text-green-600 mt-1">
            {t('pcd_bounds')} X({pointCloudData.bounds.minX.toFixed(2)} {t('pcd_to')} {pointCloudData.bounds.maxX.toFixed(2)}), 
            Y({pointCloudData.bounds.minY.toFixed(2)} {t('pcd_to')} {pointCloudData.bounds.maxY.toFixed(2)}), 
            Z({pointCloudData.bounds.minZ.toFixed(2)} {t('pcd_to')} {pointCloudData.bounds.maxZ.toFixed(2)})
          </p>
          <p className="text-sm text-green-600 mt-1">
            {t('pcd_current_range')} X({getRealXRange()[0].toFixed(2)} {t('pcd_to')} {getRealXRange()[1].toFixed(2)}), 
            Y({getRealYRange()[0].toFixed(2)} {t('pcd_to')} {getRealYRange()[1].toFixed(2)}), 
            Z({getRealZRange()[0].toFixed(2)} {t('pcd_to')} {getRealZRange()[1].toFixed(2)})
          </p>
        </div>
      )}
    </div>
  );
};

export const PCDViewerViewer: React.FC<{ className?: string }> = ({ className }) => {
  const {
    t,
    pointCloudData,
    loadingPCD,
    getRealXRange,
    getRealYRange,
    getRealZRange,
    isViewerFullscreen,
    onToggleFullscreen,
    currentPcdFileId,
  } = usePcdViewerContext();

  return (
    <ViewerShell isFullscreen={isViewerFullscreen} fullscreenBg="black" className={className}>
      <div className="flex-1 min-h-0 flex overflow-hidden">
        <div className="flex-1 min-h-0 overflow-hidden bg-black relative">
          {onToggleFullscreen && (
            <button
              onClick={onToggleFullscreen}
              className="absolute top-4 right-4 p-2 bg-white/90 hover:bg-white border border-gray-300 rounded shadow-lg z-10"
              title={isViewerFullscreen ? t('exit_fullscreen') : t('fullscreen')}
            >
              <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                {isViewerFullscreen ? (
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
                ) : (
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 8V4m0 0h4M4 4l5 5m11-1V4m0 0h-4m4 0l-5 5M4 16v4m0 0h4m-4 0l5-5m11 5l-5-5m5 5v-4m0 4h-4" />
                )}
              </svg>
            </button>
          )}
          <PointCloudViewer
            points={pointCloudData}
            loading={loadingPCD}
            placeholderText={t('no_pcd_loaded')}
            clipTopEnabled={true}
            clipZMin={getRealZRange()[0]}
            clipZMax={getRealZRange()[1]}
            clipXMin={getRealXRange()[0]}
            clipXMax={getRealXRange()[1]}
            clipYMin={getRealYRange()[0]}
            clipYMax={getRealYRange()[1]}
          />
        </div>

        {!isViewerFullscreen && (
          <div className="w-80 border-l border-gray-200">
            <PCDQrPanel
              pcdFileId={currentPcdFileId}
            />
          </div>
        )}
      </div>
    </ViewerShell>
  );
};

export default PCDViewerPanel;

