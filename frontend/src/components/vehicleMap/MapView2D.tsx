import React, { useRef, useEffect, useMemo, useState } from 'react';
import { useLanguage } from '../../contexts/LanguageContext';
import { VehicleMarker2D } from '../../types/vehicle';
import { MapMetadata } from '../../types/mapMetadata';
import { getMapImageUrl } from '../../constants/mapConfig';
import { useMapImage } from '../../hooks/useMapImage';

interface MapView2DProps {
  vehicleMarkers: VehicleMarker2D[];
  mapMetadata: MapMetadata | null;
  uploadId?: string | null;
  view?: 'top' | 'side_x';
  selectedVehicleId?: string | null;
  onVehicleSelect?: (id: string) => void;
}

const MapView2D: React.FC<MapView2DProps> = ({
  vehicleMarkers,
  mapMetadata,
  uploadId,
  view = 'top',
  selectedVehicleId,
  onVehicleSelect,
}) => {
  const { t } = useLanguage();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [hoveredVehicle, setHoveredVehicle] = useState<string | null>(null);
  
  // Determine image URL - dùng floorplan từ storage khi đã có metadata
  const imageUrl = useMemo(() => {
    if (mapMetadata) {
      // Sử dụng URL thực tế, uploadId hiện tại chưa dùng
      return getMapImageUrl(uploadId || undefined, view);
    }
    // Fallback: dùng ảnh 2D mặc định (view top) khi chưa có metadata
    return getMapImageUrl(undefined, 'top');
  }, [view, mapMetadata, uploadId]);
  
  // Load map image
  const { image: mapImage, error: imageError } = useMapImage(imageUrl);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Set canvas size
    canvas.width = canvas.offsetWidth;
    canvas.height = canvas.offsetHeight;

    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw background image if loaded
    if (mapImage && mapImage.complete && !imageError) {
      const scale = Math.min(
        canvas.width / mapImage.naturalWidth,
        canvas.height / mapImage.naturalHeight
      );
      const scaledWidth = mapImage.naturalWidth * scale;
      const scaledHeight = mapImage.naturalHeight * scale;
      const x = (canvas.width - scaledWidth) / 2;
      const y = (canvas.height - scaledHeight) / 2;
      
      ctx.drawImage(mapImage, x, y, scaledWidth, scaledHeight);
    } else {
      // Fallback: Draw grid
      ctx.strokeStyle = '#ecf0f1';
      ctx.lineWidth = 1;
      const gridSize = 20;
      for (let x = 0; x < canvas.width; x += gridSize) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, canvas.height);
        ctx.stroke();
      }
      for (let y = 0; y < canvas.height; y += gridSize) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(canvas.width, y);
        ctx.stroke();
      }
    }

    // Draw vehicle markers
    if (vehicleMarkers.length > 0) {
      console.log(`🎨 [MapView2D] Rendering ${vehicleMarkers.length} markers on ${view} view`);
    }
    
    vehicleMarkers.forEach(marker => {
      const isSelected = marker.id === selectedVehicleId;
      const isHovered = marker.id === hoveredVehicle;
      
      // Get pixel position from marker
      let x: number, y: number;
      if (mapImage && mapImage.complete && mapImage.naturalWidth > 0 && mapImage.naturalHeight > 0) {
        // Scale pixel coordinates to canvas size
        const scale = Math.min(
          canvas.width / mapImage.naturalWidth,
          canvas.height / mapImage.naturalHeight
        );
        const scaledWidth = mapImage.naturalWidth * scale;
        const scaledHeight = mapImage.naturalHeight * scale;
        const offsetX = (canvas.width - scaledWidth) / 2;
        const offsetY = (canvas.height - scaledHeight) / 2;
        
        x = offsetX + (marker.position[0] / mapImage.naturalWidth) * scaledWidth;
        y = offsetY + (marker.position[1] / mapImage.naturalHeight) * scaledHeight;
        
        console.log(`🎨 [MapView2D] Marker ${marker.id}: pixel=[${marker.position[0]}, ${marker.position[1]}], canvas=[${x.toFixed(1)}, ${y.toFixed(1)}]`);
      } else {
        // Fallback: use relative position
        x = (marker.position[0] / 1000) * canvas.width;
        y = (marker.position[1] / 1000) * canvas.height;
        console.log(`🎨 [MapView2D] Marker ${marker.id}: fallback position=[${x.toFixed(1)}, ${y.toFixed(1)}]`);
      }
      
      // Vehicle color
      const color = marker.color || '#ef4444';

      // Draw vehicle circle
      ctx.beginPath();
      ctx.arc(x, y, isSelected ? 12 : isHovered ? 10 : 8, 0, 2 * Math.PI);
      ctx.fillStyle = color;
      ctx.fill();
      
      if (isSelected || isHovered) {
        ctx.strokeStyle = '#2c3e50';
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      // Draw orientation arrow if available
      if (marker.showOrientation && marker.orientation) {
        const yaw = Math.atan2(
          2 * (marker.orientation[0] * marker.orientation[3] + marker.orientation[1] * marker.orientation[2]),
          1 - 2 * (marker.orientation[2] * marker.orientation[2] + marker.orientation[3] * marker.orientation[3])
        );
        const arrowLength = 15;
        const arrowX = x + Math.cos(yaw) * arrowLength;
        const arrowY = y + Math.sin(yaw) * arrowLength;
        
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(arrowX, arrowY);
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      // Draw vehicle label
      if (isSelected || isHovered) {
        ctx.fillStyle = '#2c3e50';
        ctx.font = '12px Arial';
        ctx.textAlign = 'center';
        ctx.fillText(marker.label || marker.id, x, y - 15);
      }
    });
  }, [vehicleMarkers, selectedVehicleId, hoveredVehicle, mapImage, imageError, mapMetadata, view]);

  const handleCanvasClick = (event: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;

    // Check if click is on a vehicle
    vehicleMarkers.forEach(marker => {
      let markerX: number, markerY: number;
      if (mapImage && mapImage.complete && mapImage.naturalWidth > 0 && mapImage.naturalHeight > 0) {
        const scale = Math.min(
          canvas.width / mapImage.naturalWidth,
          canvas.height / mapImage.naturalHeight
        );
        const scaledWidth = mapImage.naturalWidth * scale;
        const scaledHeight = mapImage.naturalHeight * scale;
        const offsetX = (canvas.width - scaledWidth) / 2;
        const offsetY = (canvas.height - scaledHeight) / 2;
        
        markerX = offsetX + (marker.position[0] / mapImage.naturalWidth) * scaledWidth;
        markerY = offsetY + (marker.position[1] / mapImage.naturalHeight) * scaledHeight;
      } else {
        markerX = (marker.position[0] / 1000) * canvas.width;
        markerY = (marker.position[1] / 1000) * canvas.height;
      }
      
      const distance = Math.sqrt((x - markerX) ** 2 + (y - markerY) ** 2);
      if (distance <= 12) {
        onVehicleSelect?.(marker.id);
      }
    });
  };

  const handleMouseMove = (event: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;

    let foundVehicle = null;
    vehicleMarkers.forEach(marker => {
      let markerX: number, markerY: number;
      if (mapImage && mapImage.complete && mapImage.naturalWidth > 0 && mapImage.naturalHeight > 0) {
        const scale = Math.min(
          canvas.width / mapImage.naturalWidth,
          canvas.height / mapImage.naturalHeight
        );
        const scaledWidth = mapImage.naturalWidth * scale;
        const scaledHeight = mapImage.naturalHeight * scale;
        const offsetX = (canvas.width - scaledWidth) / 2;
        const offsetY = (canvas.height - scaledHeight) / 2;
        
        markerX = offsetX + (marker.position[0] / mapImage.naturalWidth) * scaledWidth;
        markerY = offsetY + (marker.position[1] / mapImage.naturalHeight) * scaledHeight;
      } else {
        markerX = (marker.position[0] / 1000) * canvas.width;
        markerY = (marker.position[1] / 1000) * canvas.height;
      }
      
      const distance = Math.sqrt((x - markerX) ** 2 + (y - markerY) ** 2);
      if (distance <= 12) {
        foundVehicle = marker.id;
      }
    });

    setHoveredVehicle(foundVehicle);
    canvas.style.cursor = foundVehicle ? 'pointer' : 'default';
  };

  return (
    <div className="relative w-full h-full">
      <canvas
        ref={canvasRef}
        className="w-full h-full cursor-default"
        onClick={handleCanvasClick}
        onMouseMove={handleMouseMove}
      />
      
      {/* Legend for 2D map */}
      <div className="absolute bottom-4 left-4 bg-white bg-opacity-90 p-3 rounded-lg shadow-sm border">
        <div className="text-sm font-medium text-gray-700 mb-2">{t('legend')}</div>
        <div className="space-y-1">
          <div className="flex items-center space-x-2">
            <div className="w-3 h-3 bg-green-500 rounded-full"></div>
            <span className="text-xs text-gray-600">{t('online')}</span>
          </div>
          <div className="flex items-center space-x-2">
            <div className="w-3 h-3 bg-red-500 rounded-full"></div>
            <span className="text-xs text-gray-600">{t('offline')}</span>
          </div>
        </div>
      </div>
    </div>
  );
};

export default MapView2D;
