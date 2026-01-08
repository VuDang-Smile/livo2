import React from 'react';
import { Range } from 'react-range';
import { PointCloudBounds } from './PCDMap';
import { useLanguage } from '../contexts/LanguageContext';

interface PCDClipControlsProps {
  bounds: PointCloudBounds | null;
  clipXRange: [number, number];
  clipYRange: [number, number];
  clipZRange: [number, number];
  onClipXChange: (range: [number, number]) => void;
  onClipYChange: (range: [number, number]) => void;
  onClipZChange: (range: [number, number]) => void;
}

/**
 * Component hiển thị slider controls cho clipping X, Y, Z
 * Sử dụng react-range cho slider với 2 thumbs (range selection)
 */
export const PCDClipControls: React.FC<PCDClipControlsProps> = ({
  bounds,
  clipXRange,
  clipYRange,
  clipZRange,
  onClipXChange,
  onClipYChange,
  onClipZChange,
}) => {
  const { t } = useLanguage();

  if (!bounds) {
    return (
      <div className="p-4 text-sm text-gray-500">
        {t('pcd_loading')}
      </div>
    );
  }

  // Chuyển đổi phần trăm sang giá trị thực
  const getRealValue = (percentage: number, min: number, max: number) => {
    return min + (percentage / 100) * (max - min);
  };

  const getRealXRange = () => [
    getRealValue(clipXRange[0], bounds.minX, bounds.maxX),
    getRealValue(clipXRange[1], bounds.minX, bounds.maxX),
  ];

  const getRealYRange = () => [
    getRealValue(clipYRange[0], bounds.minY, bounds.maxY),
    getRealValue(clipYRange[1], bounds.minY, bounds.maxY),
  ];

  const getRealZRange = () => [
    getRealValue(clipZRange[0], bounds.minZ, bounds.maxZ),
    getRealValue(clipZRange[1], bounds.minZ, bounds.maxZ),
  ];

  const realXRange = getRealXRange();
  const realYRange = getRealYRange();
  const realZRange = getRealZRange();

  return (
    <div className="space-y-4">
      <div className="mb-4">
        <h3 className="text-sm font-semibold text-gray-900 mb-2">{t('pcd_adjust_display_area')}</h3>
        <p className="text-xs text-gray-600">
          {t('pcd_slider_hint')}
        </p>
      </div>

      {/* X Range Slider */}
      <div className="space-y-2">
        <div className="flex items-center justify-between">
          <label className="text-sm font-medium text-gray-700">{t('pcd_axis_x')}</label>
          <div className="text-xs text-gray-600">
            {clipXRange[0].toFixed(0)}% - {clipXRange[1].toFixed(0)}%
          </div>
        </div>
        <div className="px-2">
          <Range
            step={0.5}
            min={0}
            max={100}
            values={clipXRange}
            onChange={(values: number[]) => onClipXChange(values as [number, number])}
            renderTrack={({ props, children }: { props: React.HTMLAttributes<HTMLDivElement>; children: React.ReactNode }) => (
              <div
                {...props}
                style={{
                  ...props.style,
                  height: '6px',
                  width: '100%',
                  backgroundColor: '#e5e7eb',
                  borderRadius: '3px',
                  position: 'relative',
                }}
              >
                <div
                  style={{
                    position: 'absolute',
                    height: '6px',
                    width: `${((clipXRange[1] - clipXRange[0]) / 100) * 100}%`,
                    left: `${(clipXRange[0] / 100) * 100}%`,
                    backgroundColor: '#3b82f6',
                    borderRadius: '3px',
                  }}
                />
                {children}
              </div>
            )}
            renderThumb={({ props }: { props: React.HTMLAttributes<HTMLDivElement> }) => {
              // Tách key ra khỏi props để tránh cảnh báo React
              // key là prop đặc biệt của React, không có trong HTMLAttributes type
              const propsWithKey = props as React.HTMLAttributes<HTMLDivElement> & { key?: React.Key };
              const { key, ...restProps } = propsWithKey;
              return (
                <div
                  key={key}
                  {...restProps}
                  style={{
                    ...restProps.style,
                    height: '20px',
                    width: '20px',
                    backgroundColor: '#3b82f6',
                    borderRadius: '50%',
                    border: '2px solid white',
                    boxShadow: '0 2px 4px rgba(0,0,0,0.2)',
                    outline: 'none',
                  }}
                />
              );
            }}
          />
        </div>
        <div className="text-xs text-gray-500">
          {t('pcd_value')} {realXRange[0].toFixed(2)} {t('pcd_to')} {realXRange[1].toFixed(2)}
        </div>
      </div>

      {/* Y Range Slider */}
      <div className="space-y-2">
        <div className="flex items-center justify-between">
          <label className="text-sm font-medium text-gray-700">{t('pcd_axis_y')}</label>
          <div className="text-xs text-gray-600">
            {clipYRange[0].toFixed(0)}% - {clipYRange[1].toFixed(0)}%
          </div>
        </div>
        <div className="px-2">
          <Range
            step={0.5}
            min={0}
            max={100}
            values={clipYRange}
            onChange={(values: number[]) => onClipYChange(values as [number, number])}
            renderTrack={({ props, children }: { props: React.HTMLAttributes<HTMLDivElement>; children: React.ReactNode }) => (
              <div
                {...props}
                style={{
                  ...props.style,
                  height: '6px',
                  width: '100%',
                  backgroundColor: '#e5e7eb',
                  borderRadius: '3px',
                  position: 'relative',
                }}
              >
                <div
                  style={{
                    position: 'absolute',
                    height: '6px',
                    width: `${((clipYRange[1] - clipYRange[0]) / 100) * 100}%`,
                    left: `${(clipYRange[0] / 100) * 100}%`,
                    backgroundColor: '#10b981',
                    borderRadius: '3px',
                  }}
                />
                {children}
              </div>
            )}
            renderThumb={({ props }: { props: React.HTMLAttributes<HTMLDivElement> }) => {
              // Tách key ra khỏi props để tránh cảnh báo React
              // key là prop đặc biệt của React, không có trong HTMLAttributes type
              const propsWithKey = props as React.HTMLAttributes<HTMLDivElement> & { key?: React.Key };
              const { key, ...restProps } = propsWithKey;
              return (
                <div
                  key={key}
                  {...restProps}
                  style={{
                    ...restProps.style,
                    height: '20px',
                    width: '20px',
                    backgroundColor: '#10b981',
                    borderRadius: '50%',
                    border: '2px solid white',
                    boxShadow: '0 2px 4px rgba(0,0,0,0.2)',
                    outline: 'none',
                  }}
                />
              );
            }}
          />
        </div>
        <div className="text-xs text-gray-500">
          {t('pcd_value')} {realYRange[0].toFixed(2)} {t('pcd_to')} {realYRange[1].toFixed(2)}
        </div>
      </div>

      {/* Z Range Slider */}
      <div className="space-y-2">
        <div className="flex items-center justify-between">
          <label className="text-sm font-medium text-gray-700">{t('pcd_axis_z')}</label>
          <div className="text-xs text-gray-600">
            {clipZRange[0].toFixed(0)}% - {clipZRange[1].toFixed(0)}%
          </div>
        </div>
        <div className="px-2">
          <Range
            step={0.5}
            min={0}
            max={100}
            values={clipZRange}
            onChange={(values: number[]) => onClipZChange(values as [number, number])}
            renderTrack={({ props, children }: { props: React.HTMLAttributes<HTMLDivElement>; children: React.ReactNode }) => (
              <div
                {...props}
                style={{
                  ...props.style,
                  height: '6px',
                  width: '100%',
                  backgroundColor: '#e5e7eb',
                  borderRadius: '3px',
                  position: 'relative',
                }}
              >
                <div
                  style={{
                    position: 'absolute',
                    height: '6px',
                    width: `${((clipZRange[1] - clipZRange[0]) / 100) * 100}%`,
                    left: `${(clipZRange[0] / 100) * 100}%`,
                    backgroundColor: '#f59e0b',
                    borderRadius: '3px',
                  }}
                />
                {children}
              </div>
            )}
            renderThumb={({ props }: { props: React.HTMLAttributes<HTMLDivElement> }) => {
              // Tách key ra khỏi props để tránh cảnh báo React
              // key là prop đặc biệt của React, không có trong HTMLAttributes type
              const propsWithKey = props as React.HTMLAttributes<HTMLDivElement> & { key?: React.Key };
              const { key, ...restProps } = propsWithKey;
              return (
                <div
                  key={key}
                  {...restProps}
                  style={{
                    ...restProps.style,
                    height: '20px',
                    width: '20px',
                    backgroundColor: '#f59e0b',
                    borderRadius: '50%',
                    border: '2px solid white',
                    boxShadow: '0 2px 4px rgba(0,0,0,0.2)',
                    outline: 'none',
                  }}
                />
              );
            }}
          />
        </div>
        <div className="text-xs text-gray-500">
          {t('pcd_value')} {realZRange[0].toFixed(2)} {t('pcd_to')} {realZRange[1].toFixed(2)}
        </div>
      </div>

      {/* Bounds Info */}
      <div className="mt-4 pt-4 border-t border-gray-200">
        <div className="text-xs text-gray-600 space-y-1">
          <div>
            <span className="font-medium">{t('pcd_bounds_x')}</span> {bounds.minX.toFixed(2)} {t('pcd_to')} {bounds.maxX.toFixed(2)}
          </div>
          <div>
            <span className="font-medium">{t('pcd_bounds_y')}</span> {bounds.minY.toFixed(2)} {t('pcd_to')} {bounds.maxY.toFixed(2)}
          </div>
          <div>
            <span className="font-medium">{t('pcd_bounds_z')}</span> {bounds.minZ.toFixed(2)} {t('pcd_to')} {bounds.maxZ.toFixed(2)}
          </div>
        </div>
      </div>
    </div>
  );
};

