import React, { useRef, useEffect, useMemo, useState } from 'react';
import { useLanguage } from '../../contexts/LanguageContext';
import { VehicleMarker2D } from '../../types/vehicle';
import { MapMetadata } from '../../types/mapMetadata';
import { getMapImageUrl } from '../../constants/mapConfig';
import { useMapImage } from '../../hooks/useMapImage';
import { isValidArray } from '../../utils/validationUtils';

interface MapView2DProps {
  vehicleMarkers: VehicleMarker2D[];
  mapMetadata: MapMetadata | null;
  uploadId?: string | null;
  view?: 'top' | 'side_x';
  rotation?: number; // Rotation angle in degrees (0 or 90)
  selectedVehicleId?: string | null;
  onVehicleSelect?: (id: string) => void;
}

const MapView2D: React.FC<MapView2DProps> = ({
  vehicleMarkers,
  mapMetadata,
  uploadId,
  view = 'top',
  rotation = 0,
  selectedVehicleId,
  onVehicleSelect,
}) => {
  const { t } = useLanguage();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
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

  // Calculate canvas size based on image and rotation
  // Canvas will be at least as large as the image (after rotation)
  const canvasSize = useMemo(() => {
    if (mapImage && mapImage.complete && mapImage.naturalWidth > 0 && mapImage.naturalHeight > 0) {
      if (rotation === 90) {
        // When rotated 90°, swap width and height
        return {
          width: mapImage.naturalHeight,
          height: mapImage.naturalWidth,
        };
      } else {
        return {
          width: mapImage.naturalWidth,
          height: mapImage.naturalHeight,
        };
      }
    }
    // Fallback size
    return { width: 1000, height: 1000 };
  }, [mapImage, rotation]);

  useEffect(() => {
    const canvas = canvasRef.current;
    const container = containerRef.current;
    if (!canvas || !container) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Set canvas size (will be at least container size, but larger if image is bigger)
    canvas.width = canvasSize.width;
    canvas.height = canvasSize.height;

    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Calculate rotation in radians
    const rotationRad = (rotation * Math.PI) / 180;
    const centerX = canvas.width / 2;
    const centerY = canvas.height / 2;

    // Save context state
    ctx.save();

    // Apply rotation transformation
    if (rotation !== 0) {
      ctx.translate(centerX, centerY);
      ctx.rotate(rotationRad);
      ctx.translate(-centerX, -centerY);
    }

    // Draw background image if loaded
    if (mapImage && mapImage.complete && !imageError) {
      // Draw image at full size, centered
      const x = (canvas.width - mapImage.naturalWidth) / 2;
      const y = (canvas.height - mapImage.naturalHeight) / 2;
      ctx.drawImage(mapImage, x, y, mapImage.naturalWidth, mapImage.naturalHeight);
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
    if (!isValidArray(vehicleMarkers)) {
      console.warn('[MapView2D] Invalid vehicleMarkers, expected array');
      return;
    }
    
    if (vehicleMarkers.length > 0) {
      console.log(`🎨 [MapView2D] Rendering ${vehicleMarkers.length} markers on ${view} view`);
    }
    
    vehicleMarkers.forEach(marker => {
      // Validate marker structure
      if (!marker || !marker.id || !isValidArray(marker.position) || marker.position.length < 2) {
        console.warn('[MapView2D] Skipping invalid marker:', marker);
        return;
      }
      const isSelected = marker.id === selectedVehicleId;
      const isHovered = marker.id === hoveredVehicle;
      
      // Get pixel position from marker (in original image coordinates)
      let x: number, y: number;
      if (mapImage && mapImage.complete && mapImage.naturalWidth > 0 && mapImage.naturalHeight > 0) {
        // Use original image size for positioning (centered on canvas)
        x = (canvas.width - mapImage.naturalWidth) / 2 + (marker.position[0] / mapImage.naturalWidth) * mapImage.naturalWidth;
        y = (canvas.height - mapImage.naturalHeight) / 2 + (marker.position[1] / mapImage.naturalHeight) * mapImage.naturalHeight;
        
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
        // Adjust yaw based on rotation
        const adjustedYaw = yaw + rotationRad;
        const arrowLength = 15;
        const arrowX = x + Math.cos(adjustedYaw) * arrowLength;
        const arrowY = y + Math.sin(adjustedYaw) * arrowLength;
        
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

    // Restore context state
    ctx.restore();
  }, [vehicleMarkers, selectedVehicleId, hoveredVehicle, mapImage, imageError, mapMetadata, view, rotation, canvasSize]);

  const handleCanvasClick = (event: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    const container = containerRef.current;
    if (!canvas || !container) return;

    const rect = canvas.getBoundingClientRect();
    let x = event.clientX - rect.left;
    let y = event.clientY - rect.top;

    // Add scroll offset
    x += container.scrollLeft;
    y += container.scrollTop;

    // Transform mouse coordinates with inverse rotation
    if (rotation !== 0) {
      const rotationRad = (rotation * Math.PI) / 180;
      const centerX = canvas.width / 2;
      const centerY = canvas.height / 2;
      
      // Translate to origin, rotate inverse, translate back
      const dx = x - centerX;
      const dy = y - centerY;
      const cos = Math.cos(-rotationRad);
      const sin = Math.sin(-rotationRad);
      x = centerX + dx * cos - dy * sin;
      y = centerY + dx * sin + dy * cos;
    }

    // Check if click is on a vehicle
    if (!isValidArray(vehicleMarkers)) {
      return;
    }
    
    vehicleMarkers.forEach(marker => {
      // Validate marker structure
      if (!marker || !marker.id || !isValidArray(marker.position) || marker.position.length < 2) {
        return;
      }
      let markerX: number, markerY: number;
      if (mapImage && mapImage.complete && mapImage.naturalWidth > 0 && mapImage.naturalHeight > 0) {
        // Use original image size for positioning (centered on canvas)
        markerX = (canvas.width - mapImage.naturalWidth) / 2 + (marker.position[0] / mapImage.naturalWidth) * mapImage.naturalWidth;
        markerY = (canvas.height - mapImage.naturalHeight) / 2 + (marker.position[1] / mapImage.naturalHeight) * mapImage.naturalHeight;
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
    const container = containerRef.current;
    if (!canvas || !container) return;

    const rect = canvas.getBoundingClientRect();
    let x = event.clientX - rect.left;
    let y = event.clientY - rect.top;

    // Add scroll offset
    x += container.scrollLeft;
    y += container.scrollTop;

    // Transform mouse coordinates with inverse rotation
    if (rotation !== 0) {
      const rotationRad = (rotation * Math.PI) / 180;
      const centerX = canvas.width / 2;
      const centerY = canvas.height / 2;
      
      // Translate to origin, rotate inverse, translate back
      const dx = x - centerX;
      const dy = y - centerY;
      const cos = Math.cos(-rotationRad);
      const sin = Math.sin(-rotationRad);
      x = centerX + dx * cos - dy * sin;
      y = centerY + dx * sin + dy * cos;
    }

    if (!isValidArray(vehicleMarkers)) {
      return;
    }
    
    let foundVehicle = null;
    vehicleMarkers.forEach(marker => {
      // Validate marker structure
      if (!marker || !marker.id || !isValidArray(marker.position) || marker.position.length < 2) {
        return;
      }
      let markerX: number, markerY: number;
      if (mapImage && mapImage.complete && mapImage.naturalWidth > 0 && mapImage.naturalHeight > 0) {
        // Use original image size for positioning (centered on canvas)
        markerX = (canvas.width - mapImage.naturalWidth) / 2 + (marker.position[0] / mapImage.naturalWidth) * mapImage.naturalWidth;
        markerY = (canvas.height - mapImage.naturalHeight) / 2 + (marker.position[1] / mapImage.naturalHeight) * mapImage.naturalHeight;
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

  // Determine scroll direction based on rotation
  const scrollClass = rotation === 90 
    ? 'overflow-y-auto overflow-x-hidden' // Vertical scroll when rotated 90°
    : 'overflow-x-auto overflow-y-hidden'; // Horizontal scroll when 0°

  return (
    <div className="relative w-full h-full">
      <div
        ref={containerRef}
        className={`w-full h-full ${scrollClass} flex items-center justify-center`}
        style={{
          // Center canvas when it's smaller than container
          justifyContent: 'center',
          alignItems: 'center',
        }}
      >
        <canvas
          ref={canvasRef}
          style={{
            display: 'block',
            width: `${canvasSize.width}px`,
            height: `${canvasSize.height}px`,
            // Center canvas when smaller than container
            margin: 'auto',
          }}
          className="cursor-default"
          onClick={handleCanvasClick}
          onMouseMove={handleMouseMove}
        />
      </div>
      
      {/* Legend for 2D map */}
      <div className="absolute bottom-4 left-4 bg-white bg-opacity-90 p-3 rounded-lg shadow-sm border z-10">
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
