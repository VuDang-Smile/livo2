import React, { useState, useEffect, useRef, useMemo, useCallback } from 'react';
import { useLanguage } from '../contexts/LanguageContext';
import { useVehicleMarkers3D } from '../hooks/vehicleMap/useVehicleMarkers3D';
import { useVehiclePose2D } from '../hooks/vehicleMap/useVehiclePose2D';
import { useVehicleMap2D } from '../hooks/vehicleMap/useVehicleMap2D';
import MapView3D from '../components/vehicleMap/MapView3D';
import MapView2D from '../components/vehicleMap/MapView2D';
import { PCDClipControls } from '../components/PCDClipControls';
import { PointCloudBounds } from '../components/PCDMap';
import { DEFAULT_PCD_URL } from '../constants/pcdConfig';
import { MapMetadata } from '../types/mapMetadata';
import { Vehicle } from '../types/vehicleMap2D';
import VehicleStatusCard from '../components/vehicleMap/VehicleStatusCard';

// Component chính cho trang
const VehicleMap: React.FC = () => {
  const { t } = useLanguage();
  const [selectedVehicleId, setSelectedVehicleId] = useState<string | null>(null);
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [showFilter, setShowFilter] = useState(false);
  const [viewMode, setViewMode] = useState<'2D' | '3D'>('2D');
  const [selectedView, setSelectedView] = useState<'top' | 'side_x' | 'side_y'>('top');
  const [mapMetadata] = useState<MapMetadata | null>(null);
  const [uploadId] = useState<string | null>(null);
  const [isLoadingMetadata, setIsLoadingMetadata] = useState(false);
  const [filters, setFilters] = useState({
    status: 'all',
    vehicleType: 'all',
  });
  const [searchQuery, setSearchQuery] = useState<string>('');
  const [searchInputValue, setSearchInputValue] = useState<string>('');
  const searchTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  
  // PCD Clipping state
  const [pcdBounds, setPcdBounds] = useState<PointCloudBounds | null>(null);
  const [clipXRange, setClipXRange] = useState<[number, number]>([0, 100]);
  const [clipYRange, setClipYRange] = useState<[number, number]>([0, 100]);
  const [clipZRange, setClipZRange] = useState<[number, number]>([0, 100]);
  const [showClipControls, setShowClipControls] = useState(false);

  // Hooks for data
  const { vehicleMarkers: markers3D } = useVehicleMarkers3D();
  const { vehicleMarkers: markers2D, mapMetadata: poseMetadata } = useVehiclePose2D(selectedView);
  useVehicleMap2D();

  // Map metadata is now loaded by useVehiclePose2D hook from local files
  // No need to load from API anymore
  useEffect(() => {
    // Metadata is loaded by useVehiclePose2D hook, so we don't need to load it here
    setIsLoadingMetadata(false);
  }, []);

  const toggleFullscreen = () => {
    setIsFullscreen(!isFullscreen);
  };

  const handleVehicleSelect = (id: string) => {
    setSelectedVehicleId(id);
  };

  // PCD Bounds callback handler
  const handleBoundsCalculated = useCallback((bounds: PointCloudBounds) => {
    setPcdBounds(bounds);
    setClipXRange([0, 100]);
    setClipYRange([0, 100]);
    setClipZRange([0, 100]);
  }, []);

  // Helper functions: Chuyển đổi phần trăm sang giá trị thực
  const getRealValue = (percentage: number, min: number, max: number) => {
    return min + (percentage / 100) * (max - min);
  };

  const getRealXRange = (): [number, number] => {
    if (!pcdBounds) return [0, 0];
    return [
      getRealValue(clipXRange[0], pcdBounds.minX, pcdBounds.maxX),
      getRealValue(clipXRange[1], pcdBounds.minX, pcdBounds.maxX),
    ];
  };

  const getRealYRange = (): [number, number] => {
    if (!pcdBounds) return [0, 0];
    return [
      getRealValue(clipYRange[0], pcdBounds.minY, pcdBounds.maxY),
      getRealValue(clipYRange[1], pcdBounds.minY, pcdBounds.maxY),
    ];
  };

  const getRealZRange = (): [number, number] => {
    if (!pcdBounds) return [0, 0];
    return [
      getRealValue(clipZRange[0], pcdBounds.minZ, pcdBounds.maxZ),
      getRealValue(clipZRange[1], pcdBounds.minZ, pcdBounds.maxZ),
    ];
  };

  const realXRange = pcdBounds ? getRealXRange() : null;
  const realYRange = pcdBounds ? getRealYRange() : null;
  const realZRange = pcdBounds ? getRealZRange() : null;

  // Cleanup timeout when component unmounts
  useEffect(() => {
    return () => {
      if (searchTimeoutRef.current) {
        clearTimeout(searchTimeoutRef.current);
      }
    };
  }, []);

  // Get vehicles for display based on view mode
  const displayVehicles: Vehicle[] = useMemo(() => {
    if (viewMode === '2D') {
      // Convert 2D markers from useVehiclePose2D to Vehicle format for list display
      return markers2D.map(marker => ({
        id: marker.id,
        name: marker.label || marker.id,
        status: 'online' as const, // Default status for MQTT vehicles
        position: { 
          x: marker.position[0], 
          y: marker.position[1], 
          z: marker.position[2] 
        },
        timestamp: marker.lastUpdate.toISOString(),
      }));
    } else {
      // Convert 3D markers to display format
      return markers3D.map(marker => ({
        id: marker.id,
        name: marker.id,
        status: marker.status || 'online' as const,
        position: { x: marker.position[0], y: marker.position[1], z: marker.position[2] },
        timestamp: new Date().toISOString(),
      }));
    }
  }, [viewMode, markers2D, markers3D]);

  const selectedVehicle = displayVehicles.find(v => v.id === selectedVehicleId);

  // Filter vehicles
  const filteredDisplayVehicles = useMemo(() => {
    let result = displayVehicles;
    
    // Apply search filter
    if (searchQuery.trim()) {
      const lowerQuery = searchQuery.toLowerCase().trim();
      result = result.filter(vehicle => 
        vehicle.id.toLowerCase().includes(lowerQuery) ||
        vehicle.name.toLowerCase().includes(lowerQuery)
      );
    }
    
    // Apply status filter
    if (filters.status !== 'all') {
      result = result.filter(vehicle => vehicle.status === filters.status);
    }
    
    // Apply vehicle type filter
    if (filters.vehicleType !== 'all') {
      result = result.filter(vehicle => (vehicle.type || 'unknown') === filters.vehicleType);
    }
    
    return result;
  }, [displayVehicles, searchQuery, filters.status, filters.vehicleType]);

  const handleFilterChange = (key: string, value: string) => {
    setFilters(prev => ({
      ...prev,
      [key]: value
    }));
  };

  const clearFilters = () => {
    setFilters({
      status: 'all',
      vehicleType: 'all',
    });
  };

  const handleSearchChange = (value: string) => {
    setSearchInputValue(value);
    
    if (searchTimeoutRef.current) {
      clearTimeout(searchTimeoutRef.current);
    }
    
    searchTimeoutRef.current = setTimeout(() => {
      setSearchQuery(value);
      searchTimeoutRef.current = null;
    }, 300);
  };

  const getUniqueValues = (key: 'status' | 'vehicleType') => {
    const values = displayVehicles.map(v =>
      key === 'status' ? v.status : (v.type || 'unknown')
    );
    return ['all', ...Array.from(new Set(values))];
  };

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex justify-between items-center">
        <div>
          <h1 className="text-3xl font-bold text-gray-900">{t('vehicle_map_3d')}</h1>
          <p className="text-gray-600 mt-2">{t('track_real_time')}</p>
        </div>
        <div className="flex space-x-3">
          {/* View Mode Toggle */}
          <div className="flex bg-gray-100 rounded-lg p-1">
            <button
              onClick={() => setViewMode('2D')}
              className={`px-4 py-2 rounded-md font-medium transition-colors duration-200 flex items-center space-x-2 ${
                viewMode === '2D'
                  ? 'bg-white text-blue-600 shadow-sm'
                  : 'text-gray-600 hover:text-gray-900'
              }`}
            >
              <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 20l-5.447-2.724A1 1 0 013 16.382V5.618a1 1 0 011.447-.894L9 7m0 13l6-3m-6 3V7m6 10l4.553 2.276A1 1 0 0021 18.382V7.618a1 1 0 00-1.447-.894L15 4m0 13V4m-6 3l6-3" />
              </svg>
              <span>2D</span>
            </button>
            <button
              onClick={() => setViewMode('3D')}
              className={`px-4 py-2 rounded-md font-medium transition-colors duration-200 flex items-center space-x-2 ${
                viewMode === '3D'
                  ? 'bg-white text-blue-600 shadow-sm'
                  : 'text-gray-600 hover:text-gray-900'
              }`}
            >
              <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M20 7l-8-4-8 4m16 0l-8 4m8-4v10l-8 4m0-10L4 7m8 4v10M4 7v10l8 4" />
              </svg>
              <span>3D</span>
            </button>
          </div>

          {/* View Selection (only for 2D) */}
          {viewMode === '2D' && (
            <div className="flex bg-gray-100 rounded-lg p-1">
              <button
                onClick={() => setSelectedView('top')}
                className={`px-3 py-2 rounded-md text-sm font-medium transition-colors ${
                  selectedView === 'top'
                    ? 'bg-white text-blue-600 shadow-sm'
                    : 'text-gray-600 hover:text-gray-900'
                }`}
              >
                Top
              </button>
              <button
                onClick={() => setSelectedView('side_x')}
                className={`px-3 py-2 rounded-md text-sm font-medium transition-colors ${
                  selectedView === 'side_x'
                    ? 'bg-white text-blue-600 shadow-sm'
                    : 'text-gray-600 hover:text-gray-900'
                }`}
              >
                Side X
              </button>
              <button
                onClick={() => setSelectedView('side_y')}
                className={`px-3 py-2 rounded-md text-sm font-medium transition-colors ${
                  selectedView === 'side_y'
                    ? 'bg-white text-blue-600 shadow-sm'
                    : 'text-gray-600 hover:text-gray-900'
                }`}
              >
                Side Y
              </button>
            </div>
          )}

          {/* Fullscreen Toggle */}
          <button
            onClick={toggleFullscreen}
            className="px-4 py-2 bg-gray-100 hover:bg-gray-200 text-gray-700 rounded-lg transition-colors duration-200"
          >
            <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 8V4m0 0h4M4 4l5 5m11-1V4m0 0h-4m4 0l-5 5M4 16v4m0 0h4m-4 0l5-5m11 5l-5-5m5 5v-4m0 4h-4" />
            </svg>
          </button>
        </div>
      </div>

      {/* Stats Cards */}
      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        <VehicleStatusCard
          statusKey="total"
          value={displayVehicles.length}
        />
        <VehicleStatusCard
          statusKey="online"
          value={displayVehicles.filter(v => v.status === 'online').length}
        />
        <VehicleStatusCard
          statusKey="offline"
          value={displayVehicles.filter(v => v.status === 'offline').length}
        />
      </div>

      {/* Map Container with Vehicle List */}
      <div className={`bg-white rounded-lg shadow-sm border border-gray-200 overflow-hidden ${
        isFullscreen ? 'fixed inset-0 z-50' : 'h-[70vh] min-h-[400px]'
      }`}>
        {isFullscreen && (
          <div className="absolute top-4 right-4 z-10">
            <button
              onClick={toggleFullscreen}
              className="bg-black bg-opacity-50 text-white p-2 rounded-lg hover:bg-opacity-70"
            >
              <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
              </svg>
            </button>
          </div>
        )}
        
        <div className="flex h-full">
          {/* Map Container */}
          <div className="flex-1">
            {viewMode === '3D' ? (
              <div className="relative w-full h-full">
                <MapView3D
                  vehicleMarkers={markers3D}
                  pcdUrl={DEFAULT_PCD_URL}
                  clipXMin={realXRange ? realXRange[0] : undefined}
                  clipXMax={realXRange ? realXRange[1] : undefined}
                  clipYMin={realYRange ? realYRange[0] : undefined}
                  clipYMax={realYRange ? realYRange[1] : undefined}
                  clipZMin={realZRange ? realZRange[0] : undefined}
                  clipZMax={realZRange ? realZRange[1] : undefined}
                  selectedVehicleId={selectedVehicleId}
                  onVehicleSelect={handleVehicleSelect}
                  onBoundsCalculated={handleBoundsCalculated}
                />
                
                {/* PCD Clipping Controls Overlay */}
                {showClipControls && (
                  <div className="absolute top-4 left-4 bg-white rounded-lg shadow-lg border border-gray-200 p-4 z-10 max-w-sm">
                    <div className="flex items-center justify-between mb-3">
                      <h3 className="text-sm font-semibold text-gray-900">{t('pcd_adjust_point_cloud')}</h3>
                      <button
                        onClick={() => setShowClipControls(false)}
                        className="text-gray-400 hover:text-gray-600 p-1 rounded hover:bg-gray-100"
                      >
                        <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
                        </svg>
                      </button>
                    </div>
                    <PCDClipControls
                      bounds={pcdBounds}
                      clipXRange={clipXRange}
                      clipYRange={clipYRange}
                      clipZRange={clipZRange}
                      onClipXChange={setClipXRange}
                      onClipYChange={setClipYRange}
                      onClipZChange={setClipZRange}
                    />
                  </div>
                )}
                
                {/* Toggle button để show/hide controls */}
                {!showClipControls && pcdBounds && (
                  <button
                    onClick={() => setShowClipControls(true)}
                    className="absolute top-4 left-4 bg-white hover:bg-gray-50 text-gray-700 px-3 py-2 rounded-lg shadow-md border border-gray-200 z-10 flex items-center space-x-2"
                  >
                    <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 6V4m0 2a2 2 0 100 4m0-4a2 2 0 110 4m-6 8a2 2 0 100-4m0 4a2 2 0 110-4m0 4v2m0-6V4m6 6v10m6-2a2 2 0 100-4m0 4a2 2 0 110-4m0 4v2m0-6V4" />
                    </svg>
                    <span className="text-sm font-medium">{t('pcd_adjust')}</span>
                  </button>
                )}
              </div>
            ) : (
              <div style={{ height: isFullscreen ? '100vh' : '100%' }}>
                <MapView2D
                  vehicleMarkers={markers2D}
                  mapMetadata={poseMetadata || mapMetadata}
                  uploadId={uploadId}
                  view={selectedView}
                  selectedVehicleId={selectedVehicleId}
                  onVehicleSelect={handleVehicleSelect}
                />
                {isLoadingMetadata && (
                  <div className="absolute top-4 right-4 bg-white bg-opacity-90 p-2 rounded-lg shadow-sm">
                    <span className="text-sm text-gray-600">Loading map metadata...</span>
                  </div>
                )}
              </div>
            )}
          </div>
          
          {/* Vehicle List Sidebar */}
          <div className="w-80 bg-gray-50 border-l border-gray-200 overflow-y-auto">
            <div className="p-4">
              <div className="flex items-center justify-between mb-4">
                <h3 className="text-lg font-medium text-gray-900">{t('vehicle_list_title')}</h3>
                <button
                  onClick={() => setShowFilter(true)}
                  className="p-2 text-gray-600 hover:text-gray-900 hover:bg-gray-200 rounded-lg transition-colors"
                >
                  <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M3 4a1 1 0 011-1h16a1 1 0 011 1v2.586a1 1 0 01-.293.707l-6.414 6.414a1 1 0 00-.293.707V17l-4 4v-6.586a1 1 0 00-.293-.707L3.293 7.207A1 1 0 013 6.5V4z" />
                  </svg>
                </button>
              </div>
              
              {/* Search Input */}
              <div className="mb-4">
                <div className="relative">
                  <div className="absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none">
                    <svg className="h-5 w-5 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 21l-6-6m2-5a7 7 0 11-14 0 7 7 0 0114 0z" />
                    </svg>
                  </div>
                  <input
                    type="text"
                    value={searchInputValue}
                    onChange={(e) => handleSearchChange(e.target.value)}
                    placeholder={t('search_vehicle')}
                    className="block w-full pl-10 pr-10 py-2 border border-gray-300 rounded-lg bg-white text-gray-900 placeholder-gray-500 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
                  />
                  {searchInputValue && (
                    <button
                      onClick={() => {
                        setSearchInputValue('');
                        setSearchQuery('');
                        if (searchTimeoutRef.current) {
                          clearTimeout(searchTimeoutRef.current);
                          searchTimeoutRef.current = null;
                        }
                      }}
                      className="absolute inset-y-0 right-0 pr-3 flex items-center text-gray-400 hover:text-gray-600"
                    >
                      <svg className="h-5 w-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
                      </svg>
                    </button>
                  )}
                </div>
              </div>
              
              {/* Filter Summary */}
              {(filters.status !== 'all' || filters.vehicleType !== 'all') && (
                <div className="mb-4 p-3 bg-blue-50 border border-blue-200 rounded-lg">
                  <div className="flex items-center justify-between mb-2">
                    <span className="text-sm font-medium text-blue-900">{t('filter_applied')}</span>
                    <button
                      onClick={clearFilters}
                      className="text-blue-600 hover:text-blue-800 text-sm"
                    >
                      {t('clear_all')}
                    </button>
                  </div>
                </div>
              )}
              
              <div className="space-y-3">
                {filteredDisplayVehicles.map((vehicle) => (
                  <div
                    key={vehicle.id}
                    onClick={() => handleVehicleSelect(vehicle.id)}
                    className={`p-4 rounded-lg border cursor-pointer transition-all duration-200 ${
                      selectedVehicleId === vehicle.id
                        ? 'border-blue-500 bg-blue-50 shadow-md'
                        : 'border-gray-200 bg-white hover:border-gray-300 hover:shadow-sm'
                    }`}
                  >
                    <div className="flex items-start justify-between">
                      <div className="flex-1">
                        <div className="flex items-center space-x-2 mb-2">
                          <div 
                            className={`w-3 h-3 rounded-full ${
                              vehicle.status === 'online' ? 'bg-green-500' : 'bg-red-500'
                            }`}
                          />
                          <h4 className="font-medium text-gray-900">{vehicle.name}</h4>
                        </div>
                        
                        <div className="text-sm text-gray-600 space-y-1">
                          <p>
                            <span className="font-medium">{t('type_label')}</span>{' '}
                            {vehicle.type || t('vehicle_type_unknown')}
                          </p>
                          <p><span className="font-medium">{t('status_label_short')}</span> 
                            <span className={`ml-1 ${
                              vehicle.status === 'online' ? 'text-green-600' : 'text-red-600'
                            }`}>
                              {vehicle.status === 'online' ? (t('online') || 'Online') :
                               (t('offline') || 'Offline')}
                            </span>
                          </p>
                          <p className="text-xs text-gray-500">
                            {t('update_label')} {new Date(vehicle.timestamp).toLocaleTimeString()}
                          </p>
                        </div>
                      </div>
                      
                      {selectedVehicleId === vehicle.id && (
                        <div className="text-blue-500">
                          <svg className="w-5 h-5" fill="currentColor" viewBox="0 0 20 20">
                            <path fillRule="evenodd" d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" clipRule="evenodd" />
                          </svg>
                        </div>
                      )}
                    </div>
                  </div>
                ))}
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Vehicle Details */}
      {selectedVehicle && (
        <div className="bg-white p-6 rounded-lg shadow-sm border border-gray-200">
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-medium text-gray-900">{t('vehicle_info')}</h3>
            <button
              onClick={() => setSelectedVehicleId(null)}
              className="text-gray-500 hover:text-gray-700 p-1 rounded hover:bg-gray-100"
            >
              <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
              </svg>
            </button>
          </div>
          
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
            <div className="space-y-2">
              <h4 className="font-medium text-gray-700 text-sm uppercase tracking-wide">{t('vehicle_details')}</h4>
              <div className="space-y-1">
                <p className="text-sm"><span className="font-medium">{t('id_label')}</span> {selectedVehicle.id}</p>
                <p className="text-sm">
                  <span className="font-medium">{t('vehicle_type_label')}</span>{' '}
                  {selectedVehicle.type || t('vehicle_type_unknown')}
                </p>
                <p className="text-sm"><span className="font-medium">{t('status_label')}</span> 
                  <span className={`ml-1 ${
                    selectedVehicle.status === 'online' ? 'text-green-600' : 'text-red-600'
                  }`}>
                    {selectedVehicle.status === 'online' ? (t('online') || 'Online') :
                     (t('offline') || 'Offline')}
                  </span>
                </p>
              </div>
            </div>
            
            <div className="space-y-2">
              <h4 className="font-medium text-gray-700 text-sm uppercase tracking-wide">{t('position_time')}</h4>
              <div className="space-y-1">
                <p className="text-sm"><span className="font-medium">{t('coordinates')}</span></p>
                <p className="text-xs text-gray-600">X: {selectedVehicle.position.x.toFixed(2)}</p>
                <p className="text-xs text-gray-600">Y: {selectedVehicle.position.y.toFixed(2)}</p>
                <p className="text-xs text-gray-600">Z: {selectedVehicle.position.z.toFixed(2)}</p>
                <p className="text-sm mt-2"><span className="font-medium">{t('update_time')}</span> {new Date(selectedVehicle.timestamp).toLocaleString()}</p>
              </div>
            </div>
          </div>
        </div>
      )}

      {/* Filter Popup */}
      {showFilter && (
        <div className="fixed inset-0 bg-black bg-opacity-50 z-50 flex items-center justify-center">
          <div className="bg-white rounded-lg shadow-xl max-w-md w-full mx-4 max-h-[90vh] overflow-y-auto">
            <div className="p-6">
              <div className="flex items-center justify-between mb-6">
                <h3 className="text-lg font-medium text-gray-900">{t('filter_popup_title')}</h3>
                <button
                  onClick={() => setShowFilter(false)}
                  className="text-gray-400 hover:text-gray-600 p-1 rounded hover:bg-gray-100"
                >
                  <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
                  </svg>
                </button>
              </div>

              <div className="space-y-6">
                {/* Status Filter */}
                <div>
                  <label className="block text-sm font-medium text-gray-700 mb-3">
                    {t('status')}
                  </label>
                  <div className="space-y-2">
                    {getUniqueValues('status').map((status) => (
                      <label key={status} className="flex items-center">
                        <input
                          type="radio"
                          name="status"
                          value={status}
                          checked={filters.status === status}
                          onChange={(e) => handleFilterChange('status', e.target.value)}
                          className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300"
                        />
                        <span className="ml-3 text-sm text-gray-700">
                          {status === 'all' ? t('all') :
                           status === 'online' ? (t('online') || 'Online') :
                           (t('offline') || 'Offline')}
                        </span>
                      </label>
                    ))}
                  </div>
                </div>

                {/* Vehicle Type Filter */}
                <div>
                  <label className="block text-sm font-medium text-gray-700 mb-3">
                    {t('vehicle_type')}
                  </label>
                  <div className="space-y-2">
                    {getUniqueValues('vehicleType').map((type) => (
                      <label key={type} className="flex items-center">
                        <input
                          type="radio"
                          name="vehicleType"
                          value={type}
                          checked={filters.vehicleType === type}
                          onChange={(e) => handleFilterChange('vehicleType', e.target.value)}
                          className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300"
                        />
                        <span className="ml-3 text-sm text-gray-700">
                          {type === 'all' ? t('all') : type}
                        </span>
                      </label>
                    ))}
                  </div>
                </div>
              </div>

              <div className="flex space-x-3 mt-8 pt-6 border-t border-gray-200">
                <button
                  onClick={clearFilters}
                  className="flex-1 px-4 py-2 border border-gray-300 text-gray-700 rounded-lg hover:bg-gray-50 transition-colors"
                >
                  {t('clear_filter')}
                </button>
                <button
                  onClick={() => setShowFilter(false)}
                  className="flex-1 px-4 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700 transition-colors"
                >
                  {t('apply_filter')}
                </button>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default VehicleMap;
