import React, { useRef, useEffect, Suspense, useState } from 'react';
import type { ErrorInfo } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import { ArrowHelper } from 'three';
import * as THREE from 'three';
import { VehicleMarker3D } from '../../types/vehicle';
import PCDMap, { PointCloudBounds } from '../PCDMap';
import { MapMetadata, CoordinateSystemConfig } from '../../types/mapMetadata';
import { getPCDRotation, transformWorldToScene } from '../../utils/coordinateTransformer';
import { useLanguage } from '../../contexts/LanguageContext';
import {
  DEFAULT_PCD_URL,
  PCD_CAMERA_FOV,
  PCD_CAMERA_POSITION,
  PCD_ORBIT_MAX_DISTANCE,
  PCD_ORBIT_MIN_DISTANCE,
} from '../../constants/pcdConfig';

interface MapView3DProps {
  vehicleMarkers: VehicleMarker3D[];
  pcdUrl?: string;
  /** Metadata của bản đồ/PCD để chuẩn hóa hệ trục */
  mapMetadata?: MapMetadata | null;
  clipXMin?: number;
  clipXMax?: number;
  clipYMin?: number;
  clipYMax?: number;
  clipZMin?: number;
  clipZMax?: number;
  selectedVehicleId?: string | null;
  onVehicleSelect?: (id: string) => void;
  onBoundsCalculated?: (bounds: PointCloudBounds) => void;
}

// Component cho phương tiện 3D
const VehicleMarker: React.FC<{ 
  marker: VehicleMarker3D; 
  isSelected?: boolean; 
  onSelect?: () => void;
  coordinateConfig?: CoordinateSystemConfig;
}> = ({ marker, isSelected = false, onSelect, coordinateConfig }) => {
  const groupRef = useRef<THREE.Group>(null);
  const [hovered, setHovered] = React.useState(false);
  
  // Chuẩn hóa toạ độ world [x, y, z] sang scene Three.js dựa trên config hệ trục
  const basePosition: [number, number, number] = transformWorldToScene(
    marker.position,
    coordinateConfig
  );

  const color = marker.color || '#ef4444';
  const isGreenMarker = color === '#22c55e';
  const size = isSelected ? 1.5 : hovered ? 1.2 : 1.0;

  // Chỉ animate cho marker màu xanh (online), marker màu đỏ đứng im
  useFrame((state) => {
    if (!isGreenMarker || !groupRef.current) return;
    groupRef.current.position.x = basePosition[0];
    groupRef.current.position.y = basePosition[1];
    groupRef.current.position.z = basePosition[2] + Math.sin(state.clock.elapsedTime * 2) * 0.1;
  });

  return (
    <group
      ref={groupRef}
      position={basePosition}
      onPointerOver={() => setHovered(true)}
      onPointerOut={() => setHovered(false)}
      onClick={onSelect}
    >
      <mesh>
        <boxGeometry args={[size, size, size]} />
        <meshStandardMaterial color={color} />
      </mesh>
      
      {marker.showOrientation && marker.orientation && (
        <primitive
          object={new ArrowHelper(
            new THREE.Vector3(0, 0, 1),
            new THREE.Vector3(0, 0, 0),
            2,
            color,
            0.5,
            0.3
          )}
        />
      )}
      
      {(isSelected || hovered) && (
        <mesh>
          <ringGeometry args={[1.5, 2, 32]} />
          <meshBasicMaterial color={color} transparent opacity={0.3} />
        </mesh>
      )}
    </group>
  );
};

// Error Boundary Component cho PCD loading
class PCDErrorBoundary extends React.Component<
  { children: React.ReactNode; onError: (error: Error) => void },
  { hasError: boolean; error: Error | null }
> {
  constructor(props: { children: React.ReactNode; onError: (error: Error) => void }) {
    super(props);
    this.state = { hasError: false, error: null };
  }

  static getDerivedStateFromError(error: Error) {
    return { hasError: true, error };
  }

  componentDidCatch(error: Error, errorInfo: ErrorInfo) {
    console.error('[MapView3D] PCD loading error:', error, errorInfo);
    this.props.onError(error);
  }

  render() {
    if (this.state.hasError) {
      return null; // Return null để parent component hiển thị fallback UI
    }
    return this.props.children;
  }
}

// Component chính cho 3D map
const MapView3D: React.FC<MapView3DProps> = ({
  vehicleMarkers,
  pcdUrl = DEFAULT_PCD_URL,
  mapMetadata,
  clipXMin,
  clipXMax,
  clipYMin,
  clipYMax,
  clipZMin,
  clipZMax,
  selectedVehicleId,
  onVehicleSelect,
  onBoundsCalculated,
}) => {
  const { t } = useLanguage();
  const controlsRef = useRef<any>(null);
  const coordinateConfig = mapMetadata?.coordinate_system;
  const [pcdError, setPcdError] = useState<Error | null>(null);

  // Focus camera vào phương tiện được chọn
  useEffect(() => {
    if (!selectedVehicleId || !controlsRef.current) return;

    const controls = controlsRef.current;
    const selectedMarker = vehicleMarkers.find(m => m.id === selectedVehicleId);
    if (!selectedMarker) return;

    // Chuẩn hóa toạ độ world [x, y, z] sang scene Three.js dựa trên config hệ trục
    const [sx, sy, sz] = transformWorldToScene(selectedMarker.position, coordinateConfig);
    const targetPosition = new THREE.Vector3(sx, sy, sz);

    const currentTarget = controls.target.clone();
    const camera = controls.object;
    const viewDir = camera.position.clone().sub(currentTarget);
    const currentDistance = viewDir.length();
    const normalizedDir = viewDir.normalize();

    const minDistance = controls.minDistance ?? 0;
    const safetyMargin = 1;
    const desiredDistance = Math.max(minDistance + safetyMargin, currentDistance);
    const newCameraPosition = targetPosition.clone().add(normalizedDir.multiplyScalar(desiredDistance));

    const startPosition = camera.position.clone();
    const startTarget = currentTarget.clone();
    const duration = 1000;
    const startTime = Date.now();

    const animate = () => {
      const elapsed = Date.now() - startTime;
      const progress = Math.min(elapsed / duration, 1);
      const easeProgress = 1 - Math.pow(1 - progress, 3);

      camera.position.lerpVectors(startPosition, newCameraPosition, easeProgress);
      controls.target.lerpVectors(startTarget, targetPosition, easeProgress);
      controls.update();

      if (progress < 1) {
        requestAnimationFrame(animate);
      }
    };

    animate();
  }, [selectedVehicleId, vehicleMarkers]);

  // Hiển thị UI "chưa có" nếu PCD file không tồn tại
  if (pcdError) {
    const is404 = pcdError.message?.includes('404') || pcdError.message?.includes('Not Found');
    
    return (
      <div className="w-full h-full flex items-center justify-center bg-gray-900">
        <div className="text-center p-8 max-w-md">
          <div className="mb-4">
            <svg className="w-16 h-16 mx-auto text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M20 7l-8-4-8 4m16 0l-8 4m8-4v10l-8 4m0-10L4 7m8 4v10M4 7v10l8 4" />
            </svg>
          </div>
          <h3 className="text-lg font-medium text-white mb-2">{t('no_pcd_data') || 'Chưa có dữ liệu PCD'}</h3>
          <p className="text-sm text-gray-400 mb-4">
            {is404 
              ? (t('pcd_file_not_found') || 'File PCD không tồn tại. Vui lòng tải lên file PCD trước.')
              : (t('pcd_load_error') || 'Không thể tải file PCD. Vui lòng thử lại sau.')
            }
          </p>
          <p className="text-xs text-gray-500">{t('pcd_upload_hint') || 'Bạn có thể tải lên file PCD từ trang Upload.'}</p>
        </div>
      </div>
    );
  }

  return (
    <div className="w-full h-full relative">
      <Canvas camera={{ position: PCD_CAMERA_POSITION, fov: PCD_CAMERA_FOV }}>
        <OrbitControls 
          ref={controlsRef}
          enablePan={true}
          enableZoom={true}
          enableRotate={true}
          maxPolarAngle={Math.PI / 2}
          minDistance={PCD_ORBIT_MIN_DISTANCE}
          maxDistance={PCD_ORBIT_MAX_DISTANCE}
        />
        
        <ambientLight intensity={0.4} />
        <directionalLight position={[10, 10, 5]} intensity={0.8} />
        
        <color attach="background" args={['#000000']} />
        
        {pcdUrl && (
          <PCDErrorBoundary onError={setPcdError}>
            <Suspense fallback={null}>
              <PCDMap 
                url={pcdUrl} 
                scale={1}
                rotation={getPCDRotation(coordinateConfig)}
                position={[0, 0, 0]}
                clipXMin={clipXMin}
                clipXMax={clipXMax}
                clipYMin={clipYMin}
                clipYMax={clipYMax}
                clipZMin={clipZMin}
                clipZMax={clipZMax}
                onBoundsCalculated={onBoundsCalculated}
              />
            </Suspense>
          </PCDErrorBoundary>
        )}
        
        {vehicleMarkers.map((marker) => (
          <VehicleMarker 
            key={marker.id} 
            marker={marker} 
            isSelected={marker.id === selectedVehicleId}
            onSelect={() => onVehicleSelect?.(marker.id)}
            coordinateConfig={coordinateConfig}
          />
        ))}
        
        <gridHelper args={[200, 20, '#34495e', '#2c3e50']} />
      </Canvas>
    </div>
  );
};

export default MapView3D;
