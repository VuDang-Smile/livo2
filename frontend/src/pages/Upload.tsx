import React, { useState, useRef, useEffect, useMemo, useCallback, Suspense } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { OrbitControls, TransformControls } from '@react-three/drei';
import * as THREE from 'three';
import { Edit, Trash2, Grid3x3, ImageIcon, Maximize2, X } from 'lucide-react';
import { useLanguage } from '../contexts/LanguageContext';
import type { VehicleCanvasPosition } from '../types/vehicle';
import { MapInfo } from '../types/mapInfo';
import { QRCodeInfo } from '../types/qrCode';
import { EditingRow, ManualPin } from '../types/upload';
import PCDMap from '../components/PCDMap';
import { DEFAULT_PCD_URL } from '../constants/pcdConfig';
import { useMapImage } from '../hooks/useMapImage';
import { MAP_2D_IMAGE_URL } from '../constants/mapConfig';
import { getVehicle2DPosition } from '../utils/vehicle2DHelper';
import { useMapInfo } from '../hooks/api/useMapInfo';
import { useQRCodes } from '../hooks/api/useQRCodes';
import { MapMetadata, CoordinateSystemConfig } from '../types/mapMetadata';
import { transformPoseToPixel, pixelToWorld } from '../utils/coordinateTransform';
import { getPCDRotation, transformWorldToScene, transformSceneToWorld } from '../utils/coordinateTransformer';
import { MAP_FLOORPLAN_METADATA_URL } from '../config/dataSources';
import { extract2DFrom3DScene, convert2DTo3DScene, convert2DTo3DObject, extract2DFrom3DWorld } from '../utils/qrCoordinateHelper';

// Component cho đường hầm (copy từ VehicleMap)
const Tunnel: React.FC = () => {
  return (
    <group>
      {/* Sàn đường hầm */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -2, 0]}>
        <planeGeometry args={[200, 100]} />
        <meshStandardMaterial color="#2c3e50" />
      </mesh>

      {/* Trần đường hầm */}
      <mesh rotation={[Math.PI / 2, 0, 0]} position={[0, 8, 0]}>
        <planeGeometry args={[200, 100]} />
        <meshStandardMaterial color="#34495e" />
      </mesh>

      {/* Tường trái */}
      <mesh position={[-50, 3, 0]}>
        <boxGeometry args={[2, 10, 100]} />
        <meshStandardMaterial color="#7f8c8d" />
      </mesh>

      {/* Tường phải */}
      <mesh position={[50, 3, 0]}>
        <boxGeometry args={[2, 10, 100]} />
        <meshStandardMaterial color="#7f8c8d" />
      </mesh>

      {/* Đèn chiếu sáng */}
      {Array.from({ length: 10 }, (_, i) => (
        <group key={i} position={[0, 6, -45 + i * 10]}>
          <mesh>
            <sphereGeometry args={[0.5, 16, 16]} />
            <meshStandardMaterial color="#f1c40f" emissive="#f39c12" emissiveIntensity={0.5} />
          </mesh>
          <pointLight position={[0, 0, 0]} intensity={0.5} distance={15} color="#f1c40f" />
        </group>
      ))}
    </group>
  );
};

// Component cho phương tiện (copy từ VehicleMap) - đã chuẩn hóa theo hệ trục
const Vehicle: React.FC<{
  vehicle: VehicleCanvasPosition;
  coordinateConfig?: CoordinateSystemConfig;
}> = ({ vehicle, coordinateConfig }) => {
  const meshRef = useRef<THREE.Mesh>(null);
  // Chuẩn hóa vị trí world [x, y, z] sang scene Three.js
  const basePosition: [number, number, number] = transformWorldToScene(
    vehicle.position,
    coordinateConfig
  );

  // Animation cho phương tiện
  useFrame((state) => {
    if (meshRef.current) {
      // Nhún nhẹ theo trục Y của scene (đã chuẩn hoá)
      meshRef.current.position.y = basePosition[1] + Math.sin(state.clock.elapsedTime * 2) * 0.1;
    }
  });

  const getVehicleColor = (status?: string) => {
    switch (status) {
      case 'online': return '#27ae60';
      case 'offline': return '#e74c3c';
      default: return '#95a5a6';
    }
  };

  const getVehicleGeometry = (vehicleType?: string) => {
    const type = vehicleType || 'default';
    switch (type) {
      case 'Xe đào hầm TBM':
        // Cylinder: bán kính 0.7m, chiều cao 1.4m (giảm 30% so với kích thước thực tế)
        return <cylinderGeometry args={[0.7, 0.7, 1.4, 8]} />;
      case 'Xe vận chuyển':
        // Box: rộng 1.4m, cao 1.4m, dài 2.1m (giảm 30% so với kích thước thực tế)
        return <boxGeometry args={[1.4, 1.4, 2.1]} />;
      case 'Xe bơm bê tông':
        // Cylinder: bán kính 0.56m, chiều cao 1.4m (giảm 30% so với kích thước thực tế)
        return <cylinderGeometry args={[0.56, 0.56, 1.4, 8]} />;
      case 'Xe cẩu':
        // Box: rộng 1.4m, cao 1.4m, dài 2.1m (giảm 30% so với kích thước thực tế)
        return <boxGeometry args={[1.4, 1.4, 2.1]} />;
      default:
        // Box: rộng 1.4m, cao 1.4m, dài 2.1m (giảm 30% so với kích thước thực tế)
        return <boxGeometry args={[1.4, 1.4, 2.1]} />;
    }
  };

  return (
    <group position={basePosition}>
      {/* Phương tiện */}
      <mesh ref={meshRef}>
        {getVehicleGeometry(vehicle.vehicleType)}
        <meshStandardMaterial
          color={getVehicleColor(vehicle.status)}
          emissive={getVehicleColor(vehicle.status)}
          emissiveIntensity={0.2}
        />
      </mesh>

      {/* Đèn phát sáng cho phương tiện đang hoạt động */}
      {vehicle.status === 'online' && (
        <pointLight
          position={[0, 2, 0]}
          intensity={0.3}
          distance={10}
          color={getVehicleColor(vehicle.status)}
        />
      )}
    </group>
  );
};

// Detect which tunnel surface was hit
const detectTunnelSurface = (point: THREE.Vector3, normal: THREE.Vector3): 'floor' | 'ceiling' | 'left' | 'right' => {
  const absNormalY = Math.abs(normal.y);
  const absNormalX = Math.abs(normal.x);
  const absNormalZ = Math.abs(normal.z);
  if (absNormalY > absNormalX && absNormalY > absNormalZ) {
    return point.y > 3 ? 'ceiling' : 'floor';
  }
  if (absNormalX > absNormalY && absNormalX > absNormalZ) {
    return point.x < 0 ? 'left' : 'right';
  }
  if (point.y > 3) return 'ceiling';
  if (point.y < 1) return 'floor';
  return point.x < 0 ? 'left' : 'right';
};

// Component để hiển thị preview marker khi picking
const PreviewMarker3D: React.FC<{
  position: [number, number, number] | null;
  surface?: 'floor' | 'ceiling' | 'left' | 'right';
}> = ({ position, surface = 'floor' }) => {
  if (!position) return null;

  return (
    <group position={position}>
      {/* Preview sphere - semi-transparent yellow */}
      <mesh>
        <sphereGeometry args={[0.3, 16, 16]} />
        <meshStandardMaterial
          color="#fbbf24"
          emissive="#f59e0b"
          emissiveIntensity={0.6}
          transparent={true}
          opacity={0.7}
        />
      </mesh>

      {/* Preview ring */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.01, 0]}>
        <torusGeometry args={[0.4, 0.05, 8, 32]} />
        <meshStandardMaterial
          color="#fbbf24"
          emissive="#f59e0b"
          emissiveIntensity={0.8}
        />
      </mesh>

      {/* Preview light */}
      <pointLight
        position={[0, 0, 0]}
        intensity={1}
        distance={5}
        color="#fbbf24"
      />
    </group>
  );
};

// Component to display 3D QR code marker
const QRMarker3D: React.FC<{
  position: [number, number, number];
  id: string;
  isActive?: boolean;
  surface?: 'floor' | 'ceiling' | 'left' | 'right';
  onSelect?: (id: string) => void;
  onMount?: (id: string, mesh: THREE.Mesh | null) => void;
}> = ({ position, id, isActive = false, surface = 'floor', onSelect, onMount }) => {
  const meshRef = useRef<THREE.Mesh>(null);

  // Calculate rotation based on surface
  const getRotation = (): [number, number, number] => {
    switch (surface) {
      case 'floor':
        return [0, 0, 0];
      case 'ceiling':
        return [Math.PI, 0, 0];
      case 'left':
        return [0, 0, Math.PI / 2];
      case 'right':
        return [0, 0, -Math.PI / 2];
    }
  };

  useFrame((state) => {
    if (meshRef.current) {
      // No continuous rotation: keep marker stable
      if (isActive) {
        // Pulse animation when active
        meshRef.current.scale.setScalar(1 + Math.sin(state.clock.elapsedTime * 3) * 0.1);
      } else {
        // Ensure default scale when not active
        meshRef.current.scale.setScalar(1);
      }
    }
  });

  const rotation = getRotation();

  useEffect(() => {
    if (onMount) {
      onMount(id, meshRef.current as THREE.Mesh);
    }
    return () => {
      if (onMount) onMount(id, null);
    };
  }, [id, onMount]);

  return (
    <group position={position} rotation={rotation}>
      {/* Cube marker for QR code */}
      <mesh
        ref={meshRef}
        onPointerDown={(e) => { e.stopPropagation(); onSelect?.(id); }}
      >
        <boxGeometry args={[1.2, 1.2, 1.2]} />
        <meshStandardMaterial
          color={isActive ? '#3b82f6' : '#93c5fd'}
          emissive={isActive ? '#1e40af' : '#60a5fa'}
          emissiveIntensity={isActive ? 0.6 : 0.3}
          wireframe={false}
        />
      </mesh>

      {/* Glow effect for active marker */}
      {isActive && (
        <mesh>
          <boxGeometry args={[1.4, 1.4, 1.4]} />
          <meshStandardMaterial
            color="#3b82f6"
            emissive="#1e40af"
            emissiveIntensity={0.4}
            transparent={true}
            opacity={0.18}
            wireframe={true}
          />
        </mesh>
      )}

      {/* Point light for visual feedback */}
      <pointLight
        position={[0, 0, 0.5]}
        intensity={isActive ? 0.8 : 0.4}
        distance={3}
        color={isActive ? '#3b82f6' : '#93c5fd'}
      />
    </group>
  );
};

// Component để xử lý clicking trên 3D scene
const ClickHandler3D: React.FC<{
  onPositionClick?: (position: [number, number, number], surface: 'floor' | 'ceiling' | 'left' | 'right') => void;
  onPreviewPositionUpdate?: (position: [number, number, number] | null) => void;
  isPickingMode?: boolean;
}> = ({ onPositionClick, onPreviewPositionUpdate, isPickingMode = false }) => {
  const { camera, gl, scene } = useThree();
  const isPickingModeRef = useRef(isPickingMode);
  const onPositionClickRef = useRef(onPositionClick);
  const onPreviewPositionUpdateRef = useRef(onPreviewPositionUpdate);

  // Keep refs up to date
  useEffect(() => {
    isPickingModeRef.current = isPickingMode;
    onPositionClickRef.current = onPositionClick;
    onPreviewPositionUpdateRef.current = onPreviewPositionUpdate;
  }, [isPickingMode, onPositionClick, onPreviewPositionUpdate]);

  // Get raycasted position
  const getRaycastPosition = useCallback((event: MouseEvent): { position: [number, number, number]; surface: 'floor' | 'ceiling' | 'left' | 'right' } | null => {
    const raycaster = new THREE.Raycaster();
    const mouse = new THREE.Vector2();

    const rect = gl.domElement.getBoundingClientRect();
    mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    raycaster.setFromCamera(mouse, camera);
    const intersects = raycaster.intersectObjects(scene.children, true);

    for (const intersection of intersects) {
      const obj = intersection.object;

      // Ignore helpers/lines but accept Mesh and Points
      if (obj instanceof THREE.GridHelper || obj instanceof THREE.LineSegments) {
        continue;
      }

      if (!(obj instanceof THREE.Mesh) && !(obj instanceof THREE.Points)) {
        continue;
      }

      const point = intersection.point;

      // Compute normal: prefer face normal transformed to world-space when available,
      // otherwise fall back to the inverse ray direction (approximate surface normal for Points)
      let normal: THREE.Vector3 | null = null;
      if (intersection.face) {
        normal = intersection.face.normal.clone();
        const normalMatrix = new THREE.Matrix3().getNormalMatrix(obj.matrixWorld);
        normal.applyMatrix3(normalMatrix).normalize();
      } else {
        normal = raycaster.ray.direction.clone().negate().normalize();
      }

      const surface = detectTunnelSurface(point, normal);

      // Use the exact intersection point
      const finalPosition: [number, number, number] = [point.x, point.y, point.z];

      return { position: finalPosition, surface };
    }

    return null;
  }, [camera, gl, scene]);

  useEffect(() => {
    const canvas = gl.domElement;

    const handleMouseMove = (event: MouseEvent) => {
      if (!isPickingModeRef.current || !onPreviewPositionUpdateRef.current) return;
      const result = getRaycastPosition(event);
      onPreviewPositionUpdateRef.current(result?.position || null);
    };

    const handleDoubleClick = (event: MouseEvent) => {
      console.log('🖱️ dblclick event fired', {
        isPickingMode: isPickingModeRef.current,
        hasCallback: !!onPositionClickRef.current
      });

      if (!isPickingModeRef.current || !onPositionClickRef.current) {
        return;
      }

      const result = getRaycastPosition(event);
      if (result) {
        const { position, surface } = result;
        console.log(`✅ Position picked on ${surface}: [${position[0].toFixed(2)}, ${position[1].toFixed(2)}, ${position[2].toFixed(2)}]`);
        onPositionClickRef.current(position, surface);
        onPreviewPositionUpdateRef.current?.(null);
      } else {
        console.log('⚠️ No valid mesh intersection found');
      }
    };

    canvas.addEventListener('mousemove', handleMouseMove, false);
    canvas.addEventListener('dblclick', handleDoubleClick, false);
    console.log('✅ ClickHandler3D: Listeners attached');

    return () => {
      canvas.removeEventListener('mousemove', handleMouseMove);
      canvas.removeEventListener('dblclick', handleDoubleClick);
      console.log('🧹 ClickHandler3D: Listeners removed');
    };
  }, [getRaycastPosition, gl]);

  return null;
};

// Component cho bản đồ 3D Preview
const PCDPreview3D: React.FC<{
  vehicles: VehicleCanvasPosition[];
  pcdUrl?: string | null;
  mapMetadata?: MapMetadata | null;
  isPickingMode?: boolean;
  qrMarkers?: Array<{ position: [number, number, number]; id: string; isActive?: boolean; surface?: 'floor' | 'ceiling' | 'left' | 'right' }>;
  onPositionPick?: (position: [number, number, number], surface: 'floor' | 'ceiling' | 'left' | 'right') => void;
  previewPosition?: [number, number, number] | null;
  onPreviewPositionUpdate?: (position: [number, number, number] | null) => void;
}> = ({ vehicles, pcdUrl, isPickingMode = false, qrMarkers = [], onPositionPick, previewPosition, onPreviewPositionUpdate, mapMetadata }) => {
  const coordinateConfig = mapMetadata?.coordinate_system;
  return (
    <>
      {/* Camera controls */}
      <OrbitControls
        enablePan={true}
        enableZoom={true}
        enableRotate={true}
        maxPolarAngle={Math.PI / 2}
        minDistance={10}
        maxDistance={200}
      />

      {/* Background color */}
      <color attach="background" args={['#000000']} />

      {/* Lighting */}
      <ambientLight intensity={0.4} />
      <directionalLight position={[10, 10, 5]} intensity={0.8} />

      {/* Tunnel / PCD map */}
      {pcdUrl ? (
        <Suspense fallback={null}>
          <PCDMap
            url={pcdUrl}
            // Đơn vị: mét (scale = 1)
            scale={1}
            rotation={getPCDRotation(coordinateConfig)}
            position={[0, 0, 0]}
          />
        </Suspense>
      ) : (
        <Tunnel />
      )}

      {/* Vehicles */}
      {vehicles.map((vehicle) => (
        <Vehicle
          key={vehicle.id}
          vehicle={vehicle}
          coordinateConfig={coordinateConfig}
        />
      ))}

      {/* QR Code Markers */}
      {qrMarkers.map((marker) => (
        <QRMarker3D
          key={marker.id}
          position={marker.position}
          id={marker.id}
          isActive={marker.isActive}
          surface={marker.surface || 'floor'}
        />
      ))}

      {/* Preview Marker - shows where QR code will be placed */}
      {isPickingMode && previewPosition && <PreviewMarker3D position={previewPosition} />}

      {/* Grid helper */}
      <gridHelper args={[200, 20, '#34495e', '#2c3e50']} />

      {/* Click handler for picking positions - always render but only listens when isPickingMode is true */}
      <ClickHandler3D
        onPositionClick={onPositionPick}
        onPreviewPositionUpdate={onPreviewPositionUpdate}
        isPickingMode={isPickingMode}
      />
    </>
  );
};

// Component cho bản đồ 2D Preview
const Image2DPreview: React.FC<{
  vehicles: VehicleCanvasPosition[];
  qrPins?: ManualPin[];
  picking?: boolean;
  onPick?: (pos: [number, number]) => void;
  mapMetadata?: MapMetadata | null;
  view?: 'top' | 'side_x' | 'side_y';
}> = ({ vehicles, qrPins = [], picking = false, onPick, mapMetadata, view = 'top' }) => {
  const { t } = useLanguage();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [canvasSize, setCanvasSize] = useState<{ width: number; height: number }>({ width: 0, height: 0 });

  // Load map image với proper cleanup
  const { image: mapImage, error: imageError } = useMapImage(MAP_2D_IMAGE_URL);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const updateSize = () => {
      const w = canvas.clientWidth;
      const h = canvas.clientHeight;
      if (w !== canvasSize.width || h !== canvasSize.height) {
        setCanvasSize({ width: w, height: h });
      }
    };

    updateSize();
    const observer = new ResizeObserver(updateSize);
    observer.observe(canvas);

    const onWindowResize = () => updateSize();
    window.addEventListener('resize', onWindowResize);

    return () => {
      observer.disconnect();
      window.removeEventListener('resize', onWindowResize);
    };
  }, [canvasSize.width, canvasSize.height]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    if (!canvasSize.width || !canvasSize.height) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Set canvas size
    canvas.width = canvasSize.width;
    canvas.height = canvasSize.height;

    // Helper: compute scale & offsets for image fit (contain)
    const computeImageTransform = () => {
      if (mapImage && mapImage.complete && !imageError && mapImage.naturalWidth > 0 && mapImage.naturalHeight > 0) {
        const scale = Math.min(
          canvas.width / mapImage.naturalWidth,
          canvas.height / mapImage.naturalHeight
        );
        const scaledWidth = mapImage.naturalWidth * scale;
        const scaledHeight = mapImage.naturalHeight * scale;
        const offsetX = (canvas.width - scaledWidth) / 2;
        const offsetY = (canvas.height - scaledHeight) / 2;
        return { scale, offsetX, offsetY, scaledWidth, scaledHeight, imageWidth: mapImage.naturalWidth, imageHeight: mapImage.naturalHeight };
      }
      return null;
    };

    const imageTransform = computeImageTransform();

    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw background image nếu đã load thành công
    if (imageTransform && mapImage) {
      const { offsetX, offsetY, scaledWidth, scaledHeight } = imageTransform;
      ctx.drawImage(mapImage, offsetX, offsetY, scaledWidth, scaledHeight);
    } else {
      // Fallback: Draw tunnel outline và grid nếu image chưa load hoặc load fail
      ctx.strokeStyle = '#34495e';
      ctx.lineWidth = 2;
      ctx.strokeRect(50, 50, canvas.width - 100, canvas.height - 100);

      // Draw grid
      ctx.strokeStyle = '#ecf0f1';
      ctx.lineWidth = 1;
      const gridSize = 20;
      for (let x = 50; x < canvas.width - 50; x += gridSize) {
        ctx.beginPath();
        ctx.moveTo(x, 50);
        ctx.lineTo(x, canvas.height - 50);
        ctx.stroke();
      }
      for (let y = 50; y < canvas.height - 50; y += gridSize) {
        ctx.beginPath();
        ctx.moveTo(50, y);
        ctx.lineTo(canvas.width - 50, y);
        ctx.stroke();
      }
    }

    const worldToCanvasFallback = (worldX: number, worldZ: number) => {
      const x = ((worldX + 100) / 200) * (canvas.width - 100) + 50;
      const y = ((worldZ + 50) / 100) * (canvas.height - 100) + 50;
      return { x, y };
    };

    const pixelToCanvas = (pixelX: number, pixelY: number) => {
      if (!imageTransform) return null;
      const { offsetX, offsetY, scaledWidth, scaledHeight, imageWidth, imageHeight } = imageTransform;
      return {
        x: offsetX + (pixelX / imageWidth) * scaledWidth,
        y: offsetY + (pixelY / imageHeight) * scaledHeight,
      };
    };

    // Draw vehicles
    vehicles.forEach(vehicle => {
      const { x, y } = getVehicle2DPosition(vehicle, canvas.width, canvas.height);

      let color = '#95a5a6';
      switch (vehicle.status) {
        case 'online': color = '#27ae60'; break;
        case 'offline': color = '#e74c3c'; break;
      }

      ctx.beginPath();
      ctx.arc(x, y, 8, 0, 2 * Math.PI);
      ctx.fillStyle = color;
      ctx.fill();

      ctx.strokeStyle = '#2c3e50';
      ctx.lineWidth = 2;
      ctx.stroke();

      ctx.fillStyle = '#2c3e50';
      ctx.font = '12px Arial';
      ctx.textAlign = 'center';
      const label = vehicle.licensePlate || vehicle.id;
      ctx.fillText(label, x, y - 15);
    });

    // Draw QR pins (existing + drafting)
    qrPins.forEach(pin => {
      const [px, py] = pin.position;
      if (!isFinite(px) || !isFinite(py)) return;

      let canvasPos: { x: number; y: number } | null = null;

      if (pin.pixelPosition) {
        canvasPos = pixelToCanvas(pin.pixelPosition[0], pin.pixelPosition[1]);
      }

      if (!canvasPos) {
        // Fallback using world-to-canvas legacy mapping
        const fallback = worldToCanvasFallback(px, py);
        canvasPos = fallback;
      }

      const { x, y } = canvasPos;

      // Draw square marker for QR codes
      const size = pin.isActive ? 14 : 12;
      const halfSize = size / 2;
      ctx.beginPath();
      ctx.rect(x - halfSize, y - halfSize, size, size);
      ctx.fillStyle = pin.isDraft ? '#93c5fd' : '#93c5fd'; // Light blue for all QR markers
      ctx.globalAlpha = pin.isActive ? 0.9 : 0.75;
      ctx.fill();
      ctx.globalAlpha = 1;
      ctx.strokeStyle = pin.isActive ? '#3b82f6' : '#2563eb'; // Blue border
      ctx.lineWidth = pin.isActive ? 2 : 1.5;
      ctx.stroke();

      if (pin.label) {
        ctx.fillStyle = '#111827';
        ctx.font = '11px Arial';
        ctx.textAlign = 'center';
        ctx.fillText(pin.label, x, y - 12);
      }
    });
  }, [vehicles, qrPins, canvasSize, mapImage, imageError]);

  const handleCanvasClick = (event: React.MouseEvent<HTMLCanvasElement>) => {
    if (!picking || !onPick) return;
    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const clickX = event.clientX - rect.left;
    const clickY = event.clientY - rect.top;

    const computeImageTransform = () => {
      if (mapImage && mapImage.complete && !imageError && mapImage.naturalWidth > 0 && mapImage.naturalHeight > 0) {
        const scale = Math.min(
          canvas.width / mapImage.naturalWidth,
          canvas.height / mapImage.naturalHeight
        );
        const scaledWidth = mapImage.naturalWidth * scale;
        const scaledHeight = mapImage.naturalHeight * scale;
        const offsetX = (canvas.width - scaledWidth) / 2;
        const offsetY = (canvas.height - scaledHeight) / 2;
        return { scale, offsetX, offsetY, scaledWidth, scaledHeight, imageWidth: mapImage.naturalWidth, imageHeight: mapImage.naturalHeight };
      }
      return null;
    };

    const imageTransform = computeImageTransform();

    if (imageTransform && mapMetadata?.views?.[view]) {
      const { offsetX, offsetY, scale, imageWidth, imageHeight } = imageTransform;
      const imgX = (clickX - offsetX) / scale;
      const imgY = (clickY - offsetY) / scale;

      // Validate inside image bounds
      // Bounds check: valid pixel indices are [0, width-1] and [0, height-1]
      if (imgX < 0 || imgY < 0 || imgX >= imageWidth || imgY >= imageHeight) {
        return;
      }

      const world = pixelToWorld(imgX, imgY, mapMetadata, view);
      const projection = mapMetadata.views[view].projection.world_axes;
      const worldU = world[projection.horizontal];
      const worldV = world[projection.vertical];

      if (isFinite(worldU) && isFinite(worldV)) {
        const roundedU = parseFloat(worldU.toFixed(2));
        const roundedV = parseFloat(worldV.toFixed(2));
        onPick([roundedU, roundedV]);
        return;
      }
    }

    // Fallback when thiếu metadata/ảnh
    const worldX = ((clickX - 50) / (canvas.width - 100)) * 200 - 100;
    const worldZ = ((clickY - 50) / (canvas.height - 100)) * 100 - 50;
    const roundedX = parseFloat(worldX.toFixed(2));
    const roundedY = parseFloat(worldZ.toFixed(2));
    onPick([roundedX, roundedY]);
  };

  return (
    <div className="relative w-full h-full">
      <canvas
        ref={canvasRef}
        className={`w-full h-full ${picking ? 'cursor-crosshair' : 'cursor-default'}`}
        onClick={handleCanvasClick}
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

// Utility function: Format file size
const formatFileSize = (bytes: number): string => {
  if (bytes === 0) return '0 B';
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(2)} KB`;
  return `${(bytes / (1024 * 1024)).toFixed(2)} MB`;
};

// Utility function: Format datetime from YYYY-MM-DD HH:mm:ss to dd/MM/yyyy HH:mm:ss
const formatDateTime = (dateTimeStr: string): string => {
  try {
    const [datePart, timePart] = dateTimeStr.split(' ');
    const [year, month, day] = datePart.split('-');
    return `${day}/${month}/${year} ${timePart || ''}`.trim();
  } catch (error) {
    return dateTimeStr; // Return original if parsing fails
  }
};

// Component hiển thị thông tin map (info card style)
const MapInfoCard: React.FC<{
  mapInfo: MapInfo | null;
  isLoading?: boolean;
  error?: string | null;
}> = ({ mapInfo, isLoading = false, error = null }) => {
  const { t } = useLanguage();

  // Loading state
  if (isLoading) {
    return (
      <div className="bg-blue-50 border-l-4 border-blue-500 rounded-md p-4">
        <div className="flex items-start">
          <div className="flex-shrink-0">
            <svg className="h-5 w-5 text-blue-500 mt-0.5 animate-spin" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15" />
            </svg>
          </div>
          <div className="ml-3 flex-1">
            <h3 className="text-sm font-medium text-blue-800 mb-3">
              {t('map_info_title')}
            </h3>
            <p className="text-xs text-blue-600">{t('loading') || 'Loading...'}</p>
          </div>
        </div>
      </div>
    );
  }

  // Error state
  if (error) {
    return (
      <div className="bg-red-50 border-l-4 border-red-500 rounded-md p-4">
        <div className="flex items-start">
          <div className="flex-shrink-0">
            <svg className="h-5 w-5 text-red-500 mt-0.5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4m0 4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
            </svg>
          </div>
          <div className="ml-3 flex-1">
            <h3 className="text-sm font-medium text-red-800 mb-2">
              {t('map_info_title')}
            </h3>
            <p className="text-xs text-red-700">
              {t('error_loading_map_info') || 'Error loading map information'}: {error}
            </p>
          </div>
        </div>
      </div>
    );
  }

  // No data state
  if (!mapInfo) {
    return null;
  }

  // Success state - display map info
  return (
    <div className="bg-blue-50 border-l-4 border-blue-500 rounded-md p-4">
      <div className="flex items-start">
        <div className="flex-shrink-0">
          <svg className="h-5 w-5 text-blue-500 mt-0.5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
          </svg>
        </div>
        <div className="ml-3 flex-1">
          <h3 className="text-sm font-medium text-blue-800 mb-3">
            {t('map_info_title')}
          </h3>
          <div className="grid grid-cols-1 md:grid-cols-2 gap-x-6 gap-y-2 text-xs text-blue-700">
            <div>
              <span className="font-medium">{t('map_name')}:</span>
              <span className="ml-2">{mapInfo.name}</span>
            </div>
            <div>
              <span className="font-medium">{t('map_created_at')}:</span>
              <span className="ml-2">{formatDateTime(mapInfo.createdAt)}</span>
            </div>
            <div>
              <span className="font-medium">{t('map_uploaded_by')}:</span>
              <span className="ml-2">{mapInfo.uploadedBy}</span>
            </div>
            <div>
              <span className="font-medium">{t('map_uploaded_at')}:</span>
              <span className="ml-2">{formatDateTime(mapInfo.uploadedAt)}</span>
            </div>
            <div>
              <span className="font-medium">{t('map_file_size')}:</span>
              <span className="ml-2">{formatFileSize(mapInfo.fileSize)}</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

const Upload: React.FC = () => {
  const { t } = useLanguage();
  const [selectedZipName, setSelectedZipName] = useState<string | null>(null);
  const fileInputRef = useRef<HTMLInputElement>(null);
  const [previewVehicles, setPreviewVehicles] = useState<VehicleCanvasPosition[]>([]);
  // Danh sách QRCode ban đầu rỗng; chỉ hiển thị sau khi người dùng click "Load ZIP gần nhất"
  const [previewQRCodes, setPreviewQRCodes] = useState<QRCodeInfo[]>([]);
  // State quản lý các dòng đang chỉnh sửa
  const [editingRows, setEditingRows] = useState<EditingRow[]>([]);
  const [nextTempId, setNextTempId] = useState<number>(1);
  // State để chuyển đổi giữa 2D và 3D preview
  const [previewMode, setPreviewMode] = useState<'2d' | '3d'>('2d');
  // State để fullscreen preview
  const [isFullscreenPreview, setIsFullscreenPreview] = useState(false);
  // State để track preview position saat picking
  const [previewPickPosition, setPreviewPickPosition] = useState<[number, number, number] | null>(null);
  const didAutoLoadRef = useRef(false);
  const [isLoadingLastZip, setIsLoadingLastZip] = useState(false);
  const [isPickingPosition, setIsPickingPosition] = useState(false);
  const [isPickingPosition3D, setIsPickingPosition3D] = useState(false);
  const [pickingRowId, setPickingRowId] = useState<string | null>(null);
  const mapCardRef = useRef<HTMLDivElement>(null);
  const [pcdUrl, setPcdUrl] = useState<string | null>(null);
  const pcdObjectUrlRef = useRef<string | null>(null);
  // Map metadata (floorplan)
  const [mapMetadata, setMapMetadata] = useState<MapMetadata | null>(null);
  const [isLoadingMetadata, setIsLoadingMetadata] = useState<boolean>(false);
  const [mapMetadataError, setMapMetadataError] = useState<string | null>(null);
  // Fetch map info from storage using hook
  const { mapInfo, isLoading: isLoadingMapInfo, error: mapInfoError } = useMapInfo();
  // Fetch QR codes from storage using hook
  const { qrCodes: apiQRCodes, isLoading: isLoadingQRCodes, error: qrCodesError, refetch: refetchQRCodes } = useQRCodes();
  // State quản lý các QR code đang được chỉnh sửa (từ previewQRCodes)
  const [editingExistingQRCodes, setEditingExistingQRCodes] = useState<Map<string, {
    id: string;
    code: string;
    position: [number, number];
    position3D?: [number, number, number];
    surface?: 'floor' | 'ceiling' | 'left' | 'right';
    originalPosition: [number, number];
    errors?: {
      position?: string;
    };
  }>>(new Map());
  // State quản lý các QR code đã bị xóa (tạm thời)
  const [deletedQRCodeIds, setDeletedQRCodeIds] = useState<Set<string>>(new Set());
  const fetchMapMetadata = useCallback(async () => {
    try {
      setIsLoadingMetadata(true);
      setMapMetadataError(null);
      const response = await fetch(MAP_FLOORPLAN_METADATA_URL);
      if (!response.ok) {
        throw new Error(`Failed to load map metadata: ${response.statusText}`);
      }
      const metadata: MapMetadata = await response.json();
      if (!metadata?.views?.top) {
        throw new Error('Invalid map metadata: missing top view');
      }
      setMapMetadata(metadata);
      console.log('✅ [Upload] Loaded map metadata', {
        imageSize: metadata.views.top.image,
        resolution: metadata.resolution_m_per_pixel,
      });
    } catch (error: any) {
      console.error('❌ [Upload] Failed to load map metadata', error);
      setMapMetadataError(error?.message || 'Failed to load map metadata');
      setMapMetadata(null);
    } finally {
      setIsLoadingMetadata(false);
    }
  }, []);

  const handleZipChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (file) {
      setSelectedZipName(file.name);
      // TODO: xử lý nội dung ZIP thật để tìm file PCD nếu cần
    }
  };

  // Validation function: kiểm tra index có trùng không
  const validateQRCode = (codeIndex: string, excludeTempId?: string): { isValid: boolean; error?: string; normalizedIndex?: string } => {
    // Kiểm tra format: phải là số nguyên hợp lệ từ 0-999 (không có ký tự non-numeric)
    // Kiểm tra xem string có chứa chỉ số không (bao gồm empty string)
    if (!codeIndex || codeIndex.trim() === '') {
      return { isValid: false, error: t('upload_qr_index_invalid') };
    }

    // Kiểm tra xem tất cả ký tự có phải là số không
    if (!/^\d+$/.test(codeIndex)) {
      return { isValid: false, error: t('upload_qr_index_invalid') };
    }

    const indexNum = parseInt(codeIndex, 10);
    if (isNaN(indexNum) || indexNum < 0 || indexNum > 999) {
      return { isValid: false, error: t('upload_qr_index_invalid') };
    }

    // Normalize index thành string với format 3 chữ số
    const normalizedIndex = String(indexNum).padStart(3, '0');
    const code = `TM:${normalizedIndex}`;

    // Kiểm tra trùng với các QR code đã lưu
    const existsInSaved = previewQRCodes.some(qr => qr.code === code);
    if (existsInSaved) {
      return { isValid: false, error: t('upload_qr_index_duplicate') };
    }

    // Kiểm tra trùng với các dòng đang chỉnh sửa (trừ chính dòng đang edit)
    const existsInEditing = editingRows.some(row => {
      if (excludeTempId && row.tempId === excludeTempId) return false;
      // Normalize row.codeIndex trước khi so sánh
      if (!row.codeIndex || !/^\d+$/.test(row.codeIndex)) return false;
      const rowIndexNum = parseInt(row.codeIndex, 10);
      if (isNaN(rowIndexNum)) return false;
      const rowNormalizedIndex = String(rowIndexNum).padStart(3, '0');
      const rowCode = `TM:${rowNormalizedIndex}`;
      return rowCode === code;
    });
    if (existsInEditing) {
      return { isValid: false, error: t('upload_qr_index_duplicate') };
    }

    return { isValid: true, normalizedIndex };
  };

  // Handler: Thêm dòng mới vào editingRows
  const handleAddNewRow = () => {
    const newRow: EditingRow = {
      tempId: `temp-${nextTempId}`,
      codeIndex: '',
      position: [NaN, NaN], // Giá trị rỗng thay vì 0
      errors: {},
    };
    setEditingRows(prev => [...prev, newRow]);
    setNextTempId(prev => prev + 1);
  };

  const stopPicking = () => {
    console.log('🛑 stopPicking called');
    setIsPickingPosition(false);
    setIsPickingPosition3D(false);
    setPickingRowId(null);
  };

  const startPicking = (id: string, mode: '2d' | '3d' = '2d') => {
    console.log(`🎯 startPicking called: id=${id}, mode=${mode}`);
    setPickingRowId(id);
    if (mode === '2d') {
      setIsPickingPosition(true);
      setIsPickingPosition3D(false);
      console.log('✅ Enabled 2D picking mode');
    } else {
      setIsPickingPosition(false);
      setIsPickingPosition3D(true);
      console.log('✅ Enabled 3D picking mode');
    }
  };

  const handleMapPositionPick = (pos: [number, number]) => {
    if (!pickingRowId) return;

    // pos is in world coordinates (from pixelToWorld conversion)
    // Get surface from existing data if available
    let surface: 'floor' | 'ceiling' | 'left' | 'right' = 'floor';
    
    // Check if we're editing an existing row
    const editingRow = editingRows.find(r => r.tempId === pickingRowId);
    if (editingRow && editingRow.surface) {
      surface = editingRow.surface;
    }
    
    // Check if we're editing an existing QR code
    const editingQR = editingExistingQRCodes.get(pickingRowId);
    if (editingQR && editingQR.surface) {
      surface = editingQR.surface;
    }
    
    // Convert 2D world position to 3D scene coordinates (our standard)
    const position3D = convert2DTo3DScene(pos, surface, mapMetadata, 'top');

    // Kiểm tra xem pickingRowId có trong editingRows không
    const isEditingRow = editingRows.some(r => r.tempId === pickingRowId);
    if (isEditingRow) {
      handleEditRowChange(pickingRowId, 'position', pos);
      // Update position3D and surface information
      setEditingRows(rows => rows.map(row => row.tempId === pickingRowId ? {
        ...row,
        surface,
        position3D
      } : row));
      return;
    }

    // Kiểm tra xem pickingRowId có trong editingExistingQRCodes không
    if (editingExistingQRCodes.has(pickingRowId)) {
      handleUpdateQRPosition(pickingRowId, pos);
      // Update position3D and surface information
      setEditingExistingQRCodes(prev => {
        const next = new Map(prev);
        const current = next.get(pickingRowId);
        if (current) {
          next.set(pickingRowId, {
            ...current,
            surface,
            position3D
          });
        }
        return next;
      });
      // Also update previewQRCodes to show it immediately
      setPreviewQRCodes(qrs => qrs.map(qr => qr.id === pickingRowId ? {
        ...qr,
        surface,
        position3D
      } : qr));
      return;
    }
  };

  const handleMap3DPositionPick = (pos: [number, number, number], surface: 'floor' | 'ceiling' | 'left' | 'right') => {
    console.log(`🎯 handleMap3DPositionPick called with (scene coords):`, pos, `surface: ${surface}`, `pickingRowId: ${pickingRowId}`);

    if (!pickingRowId) {
      console.log('⚠️ pickingRowId is null');
      return;
    }

    // pos is already in scene coordinates (Three.js) - this is our standard
    // Extract 2D position from 3D scene coordinates (convert scene -> world -> 2D)
    const pos2D: [number, number] = extract2DFrom3DScene(pos, mapMetadata, 'top');
    console.log(`🎯 Converted to 2D: [${pos2D[0].toFixed(2)}, ${pos2D[1].toFixed(2)}] on surface: ${surface}`);

    // Kiểm tra xem pickingRowId có trong editingRows không
    const isEditingRow = editingRows.some(r => r.tempId === pickingRowId);
    console.log(`🎯 isEditingRow: ${isEditingRow}`);

    if (isEditingRow) {
      console.log(`✅ Updating editing row ${pickingRowId}`);
      handleEditRowChange(pickingRowId, 'position', pos2D);
      // Update position3D and surface information (use scene coordinates as standard)
      setEditingRows(rows => rows.map(row => row.tempId === pickingRowId ? {
        ...row,
        surface,
        position3D: pos // Store scene coordinates directly
      } : row));
      return;
    }

    // Kiểm tra xem pickingRowId có trong editingExistingQRCodes không
    const isExistingQR = editingExistingQRCodes.has(pickingRowId);
    console.log(`🎯 isExistingQR: ${isExistingQR}`);

    if (isExistingQR) {
      console.log(`✅ Updating existing QR ${pickingRowId}`);
      handleUpdateQRPosition(pickingRowId, pos2D);
      // Update position3D and surface information (use scene coordinates as standard)
      setEditingExistingQRCodes(prev => {
        const next = new Map(prev);
        const current = next.get(pickingRowId);
        if (current) {
          next.set(pickingRowId, {
            ...current,
            surface,
            position3D: pos // Store scene coordinates directly
          });
        }
        return next;
      });
      // Also update previewQRCodes to show it immediately (optimistic update logic might handle this but ensuring consistent view)
      setPreviewQRCodes(qrs => qrs.map(qr => qr.id === pickingRowId ? {
        ...qr,
        surface,
        position3D: pos // Store scene coordinates directly
      } : qr));
      return;
    }

    console.log('⚠️ pickingRowId not found in either editingRows or editingExistingQRCodes');
  };

  // Handler: Cập nhật giá trị của một dòng đang edit
  const handleEditRowChange = (tempId: string, field: 'codeIndex' | 'position', value: string | [number, number]) => {
    setEditingRows(prev => prev.map(row => {
      if (row.tempId !== tempId) return row;

      const updatedRow = { ...row };
      if (field === 'codeIndex') {
        updatedRow.codeIndex = value as string;
        // Validate khi thay đổi
        const validation = validateQRCode(value as string, tempId);
        updatedRow.errors = {
          ...updatedRow.errors,
          codeIndex: validation.isValid ? undefined : validation.error,
        };
      } else if (field === 'position') {
        updatedRow.position = value as [number, number];
        // Validate position
        const [x, y] = value as [number, number];
        if (isNaN(x) || isNaN(y) || !isFinite(x) || !isFinite(y)) {
          updatedRow.errors = {
            ...updatedRow.errors,
            position: t('upload_qr_position_invalid'),
          };
        } else {
          updatedRow.errors = {
            ...updatedRow.errors,
            position: undefined,
          };
        }
      }

      return updatedRow;
    }));
  };

  // Handler: Lưu tất cả các dòng đang thêm mới
  const handleSaveAllRows = () => {
    if (editingRows.length === 0) return;

    // Trích xuất và chuẩn hóa tất cả các index trong batch để kiểm tra trùng lặp nội bộ
    const batchNormalizedIndices = editingRows.map(r => {
      if (!r.codeIndex || !/^\d+$/.test(r.codeIndex)) return null;
      const indexNum = parseInt(r.codeIndex, 10);
      return (isNaN(indexNum) || indexNum < 0 || indexNum > 999) ? null : String(indexNum).padStart(3, '0');
    });

    // Validate tất cả các dòng
    const rowsWithValidation = editingRows.map((row, idx) => {
      const codeValidation = validateQRCode(row.codeIndex, row.tempId);
      const [x, y] = row.position;
      const positionValid = !isNaN(x) && !isNaN(y) && isFinite(x) && isFinite(y);

      // Kiểm tra trùng lặp ngay trong batch đang save
      const currentNormalized = batchNormalizedIndices[idx];
      const isDuplicateInBatch = currentNormalized !== null &&
        batchNormalizedIndices.some((otherIndex, otherIdx) => otherIdx !== idx && otherIndex === currentNormalized);

      let codeError = codeValidation.isValid ? undefined : codeValidation.error;
      if (!codeError && isDuplicateInBatch) {
        codeError = t('upload_qr_index_duplicate');
      }

      return {
        ...row,
        errors: {
          codeIndex: codeError,
          position: positionValid ? undefined : t('upload_qr_position_invalid'),
        },
        normalizedIndex: codeValidation.normalizedIndex
      };
    });

    const anyInvalid = rowsWithValidation.some(r => r.errors.codeIndex || r.errors.position);

    if (anyInvalid) {
      // Cập nhật errors cho tất cả các dòng để hiển thị cho người dùng
      setEditingRows(rowsWithValidation.map(({ normalizedIndex, ...rest }) => rest));
      return;
    }

    // Nếu tất cả hợp lệ, tạo danh sách QRCode mới
    const newQRCodes: QRCodeInfo[] = rowsWithValidation.map(row => ({
      id: `qr-${Date.now()}-${Math.random()}`,
      code: `TM:${row.normalizedIndex || String(parseInt(row.codeIndex, 10)).padStart(3, '0')}`,
      position: row.position,
      isManual: true,
    }));

    setPreviewQRCodes(prev => [...prev, ...newQRCodes]);
    setEditingRows([]);
    stopPicking();
  };

  // Handler: Bắt đầu chỉnh sửa QR code
  const handleStartEditQR = (qrId: string) => {
    const qr = previewQRCodes.find(q => q.id === qrId);
    if (!qr) return;

    setEditingExistingQRCodes(prev => {
      const newMap = new Map(prev);
      newMap.set(qrId, {
        id: qr.id,
        code: qr.code,
        position: [...qr.position] as [number, number],
        position3D: qr.position3D ? [...qr.position3D] as [number, number, number] : undefined,
        surface: qr.surface,
        originalPosition: [...qr.position] as [number, number],
        errors: {}
      });
      return newMap;
    });
  };

  // Handler: Cập nhật vị trí khi chỉnh sửa
  const handleUpdateQRPosition = (qrId: string, position: [number, number]) => {
    setEditingExistingQRCodes(prev => {
      const newMap = new Map(prev);
      const existing = newMap.get(qrId);
      if (!existing) return prev;

      const [x, y] = position;
      const errors = {
        ...existing.errors,
        position: (isNaN(x) || isNaN(y) || !isFinite(x) || !isFinite(y))
          ? t('upload_qr_position_invalid')
          : undefined
      };

      newMap.set(qrId, { ...existing, position, errors });
      return newMap;
    });
  };

  // Handler: Lưu thay đổi
  const handleSaveQRChanges = (qrId: string) => {
    const editedQR = editingExistingQRCodes.get(qrId);
    if (!editedQR) return;

    // Validate trước khi lưu
    if (editedQR.errors?.position) return;

    setPreviewQRCodes(prev => prev.map(qr =>
      qr.id === qrId
        ? {
          ...qr,
          position: editedQR.position,
          position3D: editedQR.position3D,
          surface: editedQR.surface
        }
        : qr
    ));

    setEditingExistingQRCodes(prev => {
      const newMap = new Map(prev);
      newMap.delete(qrId);
      return newMap;
    });

    if (pickingRowId === qrId) {
      stopPicking();
    }
  };

  // Handler: Hủy chỉnh sửa
  const handleCancelEditQR = (qrId: string) => {
    setEditingExistingQRCodes(prev => {
      const newMap = new Map(prev);
      newMap.delete(qrId);
      return newMap;
    });

    if (pickingRowId === qrId) {
      stopPicking();
    }
  };

  // Handler: Xóa QR code
  const handleDeleteQR = (qrId: string) => {
    if (!window.confirm(t('upload_qr_delete_confirm'))) return;

    // Nếu đang chỉnh sửa, hủy chỉnh sửa trước
    if (editingExistingQRCodes.has(qrId)) {
      handleCancelEditQR(qrId);
    }

    setDeletedQRCodeIds(prev => new Set(prev).add(qrId));
  };

  const handleLastZipLoad = useCallback(() => {
    // Loại bỏ mock vehicles - preview vehicles sẽ được load từ API nếu cần
    // Hiện tại để empty array vì vehicles sẽ được hiển thị từ real-time MQTT data
    setPreviewVehicles([]);

    // Tạm thời dùng file PCD tĩnh trong public cho demo
    // Sau này có thể thay bằng PCD lấy từ ZIP hoặc backend
    setPcdUrl(DEFAULT_PCD_URL);

    // Load QR codes from API (already fetched by useQRCodes hook)
    // Refresh QR codes when loading ZIP to ensure latest data
    if (apiQRCodes.length > 0) {
      // Convert QR codes from API (world coordinates) to scene coordinates (our standard)
      const coordinateConfig = mapMetadata?.coordinate_system;
      const convertedQRCodes = apiQRCodes.map(qr => {
        if (qr.position3D) {
          // Convert world coordinates to scene coordinates
          const scenePos3D = transformWorldToScene(qr.position3D, coordinateConfig);
          return {
            ...qr,
            position3D: scenePos3D
          };
        }
        return qr;
      });
      setPreviewQRCodes(convertedQRCodes);
    } else if (!isLoadingQRCodes && !qrCodesError) {
      // If no QR codes and not loading, try to refetch
      refetchQRCodes();
    }

    // Map info is now loaded automatically via useMapInfo hook
    // No need to manually set it here
  }, [apiQRCodes, isLoadingQRCodes, qrCodesError, refetchQRCodes, mapMetadata]);

  const loadLastZip = useCallback(() => {
    if (isLoadingLastZip) return; // Tránh race/đúp click
    setIsLoadingLastZip(true);
    try {
      handleLastZipLoad();
    } finally {
      setIsLoadingLastZip(false);
    }
  }, [isLoadingLastZip, handleLastZipLoad]);

  useEffect(() => {
    if (didAutoLoadRef.current) return;
    didAutoLoadRef.current = true;

    let canceled = false;

    const run = () => {
      if (canceled) return;
      loadLastZip();
    };

    run();

    return () => {
      canceled = true;
    };
  }, [loadLastZip]);

  // Cleanup ObjectURL nếu sau này chúng ta tạo từ File trong ZIP
  useEffect(() => {
    const previousUrl = pcdObjectUrlRef.current;
    return () => {
      if (previousUrl) {
        URL.revokeObjectURL(previousUrl);
      }
    };
  }, []);

  useEffect(() => {
    fetchMapMetadata();
  }, [fetchMapMetadata]);

  useEffect(() => {
    document.body.style.overflow = isPickingPosition ? 'hidden' : '';
    return () => {
      document.body.style.overflow = '';
    };
  }, [isPickingPosition]);

  // Auto-sync QR codes from API when available (only if previewQRCodes is empty)
  useEffect(() => {
    if (apiQRCodes.length > 0 && previewQRCodes.length === 0 && !isLoadingQRCodes) {
      // Convert QR codes from API (world coordinates) to scene coordinates (our standard)
      const coordinateConfig = mapMetadata?.coordinate_system;
      const convertedQRCodes = apiQRCodes.map(qr => {
        if (qr.position3D) {
          // Convert world coordinates to scene coordinates
          const scenePos3D = transformWorldToScene(qr.position3D, coordinateConfig);
          return {
            ...qr,
            position3D: scenePos3D
          };
        }
        return qr;
      });
      setPreviewQRCodes(convertedQRCodes);
    }
  }, [apiQRCodes, previewQRCodes.length, isLoadingQRCodes, mapMetadata]);

  useEffect(() => {
    if (pickingRowId) {
      const isEditingRow = editingRows.some(r => r.tempId === pickingRowId);
      const isEditingQR = editingExistingQRCodes.has(pickingRowId);
      if (!isEditingRow && !isEditingQR) {
        stopPicking();
      }
    }
  }, [editingRows, editingExistingQRCodes, pickingRowId]);

  const qrPins = useMemo<ManualPin[]>(() => {
    const pins: ManualPin[] = [];

    const viewMeta = mapMetadata?.views?.top;

    const buildPin = (id: string, label: string | undefined, worldPair: [number, number], position3D?: { x: number; y: number; z: number }, isDraft = false, isActive = false): ManualPin => {
      let pixelPosition: [number, number] | undefined;
      let isOutOfBounds: boolean | undefined;
      let isClamped: boolean | undefined;

      if (viewMeta) {
        const pose = position3D
          ? {
            position: position3D,
            orientation: { x: 0, y: 0, z: 0, w: 1 },
          }
          : {
            position: convert2DTo3DObject(worldPair, 'floor', mapMetadata, 'top'),
            orientation: { x: 0, y: 0, z: 0, w: 1 },
          };
        const pixel = transformPoseToPixel(pose, mapMetadata!, 'top', process.env.NODE_ENV === 'development' || process.env.REACT_APP_DEBUG_LOGS === '1');
        if (pixel) {
          pixelPosition = [pixel.pixel_x, pixel.pixel_y];
          isOutOfBounds = pixel.is_out_of_bounds;
          isClamped = pixel.is_clamped;
        }
      }

      return {
        id,
        position: worldPair,
        pixelPosition,
        label,
        isDraft,
        isActive,
        isOutOfBounds,
        isClamped,
      };
    };

    // Thêm các QR code đã lưu (loại trừ các QR đã xóa)
    previewQRCodes
      .filter(qr => !deletedQRCodeIds.has(qr.id))
      .forEach(qr => {
        // position3D is now in scene coordinates (our standard)
        // If not available, convert from 2D world position to scene coordinates
        const scenePos3d = qr.position3D
          ? { x: qr.position3D[0], y: qr.position3D[1], z: qr.position3D[2] }
          : convert2DTo3DObject(qr.position, qr.surface || 'floor', mapMetadata, 'top');

        // Kiểm tra xem QR này có đang được chỉnh sửa không
        const editedQR = editingExistingQRCodes.get(qr.id);
        
        // Extract 2D world position from scene coordinates
        const worldPair = editedQR 
          ? editedQR.position 
          : extract2DFrom3DScene([scenePos3d.x, scenePos3d.y, scenePos3d.z], mapMetadata, 'top');
        const isBeingEdited = !!editedQR;

        // Use edited 3D position if available (in scene coordinates), otherwise original
        const effectivePos3D = editedQR && editedQR.position3D ?
          { x: editedQR.position3D[0], y: editedQR.position3D[1], z: editedQR.position3D[2] } :
          (editedQR ? convert2DTo3DObject(editedQR.position, editedQR.surface || 'floor', mapMetadata, 'top') : scenePos3d);
        
        // Convert scene coordinates to world coordinates for transformPoseToPixel
        const coordinateConfig = mapMetadata?.coordinate_system;
        const worldPos3D = transformSceneToWorld(
          [effectivePos3D.x, effectivePos3D.y, effectivePos3D.z],
          coordinateConfig
        );
        const worldPos3DObject = { x: worldPos3D[0], y: worldPos3D[1], z: worldPos3D[2] };

        pins.push(
          buildPin(
            qr.id,
            qr.code,
            worldPair,
            worldPos3DObject, // Use world coordinates for transformPoseToPixel
            false,
            isBeingEdited || pickingRowId === qr.id
          )
        );
      });

    // Thêm các dòng đang chỉnh sửa (editingRows)
    editingRows.forEach(row => {
      const [x, y] = row.position;
      if (isNaN(x) || isNaN(y)) return;
      const normalizedIndex = row.codeIndex && /^\d+$/.test(row.codeIndex)
        ? `TM:${String(parseInt(row.codeIndex, 10)).padStart(3, '0')}`
        : undefined;

      // row.position3D is in scene coordinates (our standard)
      const scenePos3D = row.position3D
        ? { x: row.position3D[0], y: row.position3D[1], z: row.position3D[2] }
        : convert2DTo3DObject(row.position, row.surface || 'floor', mapMetadata, 'top');
      
      // Convert scene coordinates to world coordinates for transformPoseToPixel
      const coordinateConfig = mapMetadata?.coordinate_system;
      const worldPos3D = transformSceneToWorld(
        [scenePos3D.x, scenePos3D.y, scenePos3D.z],
        coordinateConfig
      );
      const worldPos3DObject = { x: worldPos3D[0], y: worldPos3D[1], z: worldPos3D[2] };

      pins.push(
        buildPin(
          row.tempId,
          normalizedIndex,
          row.position,
          worldPos3DObject, // Use world coordinates for transformPoseToPixel
          true,
          row.tempId === pickingRowId
        )
      );
    });

    return pins;
  }, [previewQRCodes, editingRows, editingExistingQRCodes, deletedQRCodeIds, pickingRowId, mapMetadata]);

  // Convert 2D QR pins to 3D markers for 3D view
  const qrMarkers3D = useMemo<Array<{ position: [number, number, number]; id: string; isActive?: boolean; surface?: 'floor' | 'ceiling' | 'left' | 'right' }>>(() => {
    const markers: Array<{ position: [number, number, number]; id: string; isActive?: boolean; surface?: 'floor' | 'ceiling' | 'left' | 'right' }> = [];
    const coordinateConfig = mapMetadata?.coordinate_system;

    // Add saved QR codes
    previewQRCodes
      .filter(qr => !deletedQRCodeIds.has(qr.id))
      .forEach(qr => {
        const surface = qr.surface || 'floor';
        // position3D is now in scene coordinates (our standard)
        // If not available, convert from 2D world position to scene coordinates
        const scenePos3d: [number, number, number] = qr.position3D 
          ? qr.position3D
          : convert2DTo3DScene(qr.position, surface, mapMetadata, 'top');
        
        markers.push({
          position: scenePos3d, // Use scene coordinates directly
          id: qr.id,
          isActive: pickingRowId === qr.id,
          surface
        });
      });

    // Add editing rows (draft positions)
    editingRows.forEach(row => {
      const [x, z] = row.position;
      if (isNaN(x) || isNaN(z)) return;

      const surface = row.surface || 'floor';
      // row.position3D is in scene coordinates (our standard)
      const scenePos3d: [number, number, number] = row.position3D
        ? row.position3D
        : convert2DTo3DScene(row.position, surface, mapMetadata, 'top');

      markers.push({
        position: scenePos3d, // Use scene coordinates directly
        id: row.tempId,
        isActive: row.tempId === pickingRowId,
        surface: row.surface || 'floor'
      });
    });

    console.log(`📍 qrMarkers3D updated: ${markers.length} markers`, markers);
    return markers;
  }, [previewQRCodes, editingRows, deletedQRCodeIds, pickingRowId, mapMetadata]);

  return (
    <div className="space-y-6">
      {isPickingPosition && (
        <>
          <div className="fixed inset-0 bg-black/40 backdrop-blur-[1px] z-40 pointer-events-auto" />
          <div className="fixed top-4 left-1/2 -translate-x-1/2 z-50 pointer-events-none">
            <div className="px-4 py-2 bg-white/95 text-gray-800 rounded shadow pointer-events-auto">
              <p className="text-sm font-semibold">{t('upload_qr_pick_overlay')}</p>
            </div>
          </div>
        </>
      )}
      {/* Page header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-3xl font-bold text-gray-900">{t('upload_page_title')}</h1>
          <p className="text-gray-600 mt-2">{t('upload_page_desc')}</p>
        </div>
        <div className="flex items-center gap-3">
          <input
            ref={fileInputRef}
            type="file"
            accept=".zip"
            className="hidden"
            onChange={handleZipChange}
          />
          {/* <button
            type="button"
            onClick={() => fileInputRef.current?.click()}
            className="inline-flex items-center px-3 py-2 text-white text-sm font-medium rounded-md shadow-sm transition-colors bg-blue-600 hover:bg-blue-700"
          >
            <svg className="w-5 h-5 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 12l8-8 8 8M12 4v16" />
            </svg>
            {t('upload_zip_title')}
          </button> */}
          {selectedZipName && (
            <span className="text-xs text-gray-600">{selectedZipName}</span>
          )}
        </div>
      </div>
      {/* Map Info Card */}
      <MapInfoCard
        mapInfo={mapInfo}
        isLoading={isLoadingMapInfo}
        error={mapInfoError}
      />

      {/* Section 2 & 3: Preview PCD/Image + QR codes */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* PCD & Image Preview */}
        <div className="lg:col-span-2 space-y-6">
          {/* Toggle Buttons for 2D/3D Preview */}
          <div className="bg-white rounded-lg shadow-sm border border-gray-200 p-6">
            <div className="flex items-center justify-between mb-4">
              <h2 className="text-lg font-semibold text-gray-900">
                {previewMode === '2d' ? t('upload_image_preview_title') : t('upload_pcd_preview_title')}
              </h2>

              {/* Toggle Switch for 2D/3D and Fullscreen */}
              <div className="flex gap-2 items-center">
                <button
                  onClick={() => setPreviewMode('2d')}
                  className={`inline-flex items-center gap-2 px-4 py-2 rounded-lg font-medium text-sm transition-all ${previewMode === '2d'
                    ? 'bg-blue-600 text-white shadow-md'
                    : 'bg-gray-100 text-gray-700 hover:bg-gray-200'
                    }`}
                >
                  <ImageIcon size={18} />
                  2D View
                </button>
                <button
                  onClick={() => setPreviewMode('3d')}
                  className={`inline-flex items-center gap-2 px-4 py-2 rounded-lg font-medium text-sm transition-all ${previewMode === '3d'
                    ? 'bg-blue-600 text-white shadow-md'
                    : 'bg-gray-100 text-gray-700 hover:bg-gray-200'
                    }`}
                >
                  <Grid3x3 size={18} />
                  3D View
                </button>

                {/* Fullscreen Button */}
                <button
                  onClick={() => setIsFullscreenPreview(true)}
                  className="inline-flex items-center gap-2 px-4 py-2 rounded-lg font-medium text-sm transition-all bg-gray-100 text-gray-700 hover:bg-gray-200"
                  title="Fullscreen Preview"
                >
                  <Maximize2 size={18} />
                </button>
              </div>
            </div>

            {isLoadingMetadata && (
              <div className="mb-3 text-xs text-blue-700 bg-blue-50 border border-blue-200 rounded px-3 py-2">
                {t('loading') || 'Đang tải metadata bản đồ...'}
              </div>
            )}
            {mapMetadataError && (
              <div className="mb-3 text-xs text-red-700 bg-red-50 border border-red-200 rounded px-3 py-2">
                {t('error_loading_map_info') || 'Lỗi tải metadata bản đồ'}: {mapMetadataError}
              </div>
            )}

            {/* 2D Preview */}
            {previewMode === '2d' && (
              <div
                ref={mapCardRef}
                className={`h-[70vh] min-h-[400px] border border-gray-300 rounded-lg overflow-hidden bg-gray-50 relative ${isPickingPosition ? 'ring-2 ring-blue-500 z-50' : ''}`}
              >
                <Image2DPreview
                  vehicles={previewVehicles}
                  qrPins={qrPins}
                  picking={isPickingPosition}
                  onPick={handleMapPositionPick}
                  mapMetadata={mapMetadata}
                  view="top"
                />
                {isPickingPosition && (
                  <div className="absolute inset-0 pointer-events-none">
                    <div className="absolute top-3 left-3 bg-white/95 border border-blue-200 shadow-sm rounded px-3 py-2 max-w-xs pointer-events-auto">
                      <p className="text-xs text-gray-700">{t('upload_qr_pick_hint')}</p>
                      <div className="mt-2 flex gap-2">
                        <button
                          onClick={stopPicking}
                          className="px-2 py-1 text-xs bg-blue-600 text-white rounded hover:bg-blue-700 transition-colors"
                        >
                          {t('upload_qr_pick_done')}
                        </button>
                      </div>
                    </div>
                  </div>
                )}
              </div>
            )}

            {/* 3D Preview */}
            {previewMode === '3d' && (
              <div className={`h-[70vh] min-h-[400px] border border-gray-300 rounded-lg overflow-hidden bg-gray-50 relative ${isPickingPosition3D ? 'ring-2 ring-blue-500 z-50' : ''}`}>
                <Canvas
                  camera={{ position: [0, 10, 20], fov: 60 }}
                  style={{ height: '100%' }}
                >
                  <PCDPreview3D
                    vehicles={previewVehicles}
                    pcdUrl={pcdUrl}
                    isPickingMode={isPickingPosition3D}
                    qrMarkers={qrMarkers3D}
                    onPositionPick={handleMap3DPositionPick}
                    previewPosition={previewPickPosition}
                    onPreviewPositionUpdate={setPreviewPickPosition}
                    mapMetadata={mapMetadata}
                  />
                </Canvas>
                {isPickingPosition3D && (
                  <div className="absolute inset-0 pointer-events-none">
                    <div className="absolute top-3 left-3 bg-white/95 border border-blue-200 shadow-sm rounded px-3 py-2 max-w-xs pointer-events-auto">
                      <p className="text-xs text-gray-700">Double click trên một vị trí để đặt QR code</p>
                      <div className="mt-2 flex gap-2">
                        <button
                          onClick={stopPicking}
                          className="px-2 py-1 text-xs bg-blue-600 text-white rounded hover:bg-blue-700 transition-colors"
                        >
                          {t('upload_qr_pick_done')}
                        </button>
                      </div>
                    </div>
                  </div>
                )}
              </div>
            )}
          </div>
        </div>

        {/* QRCode list */}
        <div className="bg-white rounded-lg shadow-sm border border-gray-200 p-6">
          <div className="flex items-center justify-between mb-3">
            <h2 className="text-lg font-semibold text-gray-900">
              {t('upload_qr_list_title')}
            </h2>
          </div>
          <p className="text-xs text-gray-500 mb-3">
            {t('upload_qr_list_desc')}
          </p>

          {/* Loading and Error States for QR Codes */}
          {isLoadingQRCodes && (
            <div className="mb-4 p-3 bg-blue-50 border border-blue-200 rounded-lg">
              <div className="flex items-center space-x-2">
                <div className="animate-spin rounded-full h-4 w-4 border-b-2 border-blue-600"></div>
                <span className="text-sm text-blue-700">{t('loading_qr_codes') || 'Loading QR codes...'}</span>
              </div>
            </div>
          )}
          {qrCodesError && (
            <div className="mb-4 p-3 bg-red-50 border border-red-200 rounded-lg">
              <div className="flex items-center justify-between">
                <div className="flex items-center space-x-2">
                  <svg className="w-4 h-4 text-red-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4m0 4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
                  </svg>
                  <span className="text-sm text-red-700">
                    {t('error_loading_qr_codes') || 'Error loading QR codes'}: {qrCodesError}
                  </span>
                </div>
                <button
                  onClick={() => refetchQRCodes()}
                  className="text-sm text-red-600 hover:text-red-800 underline"
                >
                  {t('retry') || 'Retry'}
                </button>
              </div>
            </div>
          )}
          <div className="border border-gray-200 rounded-lg overflow-hidden">
            <table className="min-w-full divide-y divide-gray-200">
              <thead className="bg-gray-50">
                <tr>
                  <th className="px-3 py-2 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    {t('upload_qr_code')}
                  </th>
                  <th className="px-3 py-2 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    {t('upload_qr_position')}
                  </th>
                  <th className="px-3 py-2 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    {t('upload_qr_actions')}
                  </th>
                </tr>
              </thead>
              <tbody className="bg-white divide-y divide-gray-200">
                {/* Hiển thị các QR code đã lưu */}
                {previewQRCodes
                  .filter(qr => !deletedQRCodeIds.has(qr.id))
                  .map((qr) => {
                    const isEditing = editingExistingQRCodes.has(qr.id);
                    const editedQR = editingExistingQRCodes.get(qr.id);
                    const displayPosition = editedQR ? editedQR.position : qr.position;

                    return (
                      <tr
                        key={qr.id}
                        className={isEditing ? 'bg-yellow-50' : (qr.isManual ? 'bg-blue-50' : '')}
                      >
                        <td className="px-3 py-2 text-xs font-mono text-gray-900">
                          {qr.code}
                        </td>
                        <td className="px-3 py-2 text-xs text-gray-500">
                          {isEditing ? (
                            <div className="space-y-2">
                              <div className="text-xs text-gray-700">
                                {t('upload_qr_position_x')}: {isNaN(displayPosition[0]) ? '--' : displayPosition[0]}, {t('upload_qr_position_y')}: {isNaN(displayPosition[1]) ? '--' : displayPosition[1]}
                              </div>
                              <div className="flex flex-wrap gap-2">
                                <button
                                  onClick={() => startPicking(qr.id, previewMode)}
                                  className={`px-3 py-1 text-xs rounded transition-colors ${(isPickingPosition || isPickingPosition3D) && pickingRowId === qr.id ? 'bg-blue-600 text-white hover:bg-blue-700' : 'bg-amber-500 text-white hover:bg-amber-600'}`}
                                >
                                  {(isPickingPosition || isPickingPosition3D) && pickingRowId === qr.id ? t('upload_qr_pick_active') : t('upload_qr_pick_position')}
                                </button>
                                {(isPickingPosition || isPickingPosition3D) && pickingRowId === qr.id && (
                                  <button
                                    onClick={stopPicking}
                                    className="px-3 py-1 text-xs rounded bg-gray-600 text-white hover:bg-gray-700 transition-colors"
                                  >
                                    {t('upload_qr_pick_done')}
                                  </button>
                                )}
                              </div>
                              {(isPickingPosition || isPickingPosition3D) && pickingRowId === qr.id && (
                                <p className="text-[11px] text-blue-600">{previewMode === '2d' ? t('upload_qr_pick_hint') : 'Double click để đặt QR code'}</p>
                              )}
                              {editedQR?.errors?.position && (
                                <p className="text-xs text-red-500 mt-1">{editedQR.errors.position}</p>
                              )}
                            </div>
                          ) : (
                            <span>X: {isNaN(qr.position[0]) ? '--' : qr.position[0]}, Y: {isNaN(qr.position[1]) ? '--' : qr.position[1]}</span>
                          )}
                        </td>
                        <td className="px-3 py-2">
                          {isEditing ? (
                            <div className="flex flex-wrap gap-2">
                              <button
                                onClick={() => handleSaveQRChanges(qr.id)}
                                className="px-3 py-1 text-xs bg-green-600 text-white rounded hover:bg-green-700 transition-colors"
                              >
                                {t('upload_qr_save_changes')}
                              </button>
                              <button
                                onClick={() => handleCancelEditQR(qr.id)}
                                className="px-3 py-1 text-xs bg-gray-500 text-white rounded hover:bg-gray-600 transition-colors"
                              >
                                {t('upload_qr_cancel_edit')}
                              </button>
                            </div>
                          ) : (
                            <div className="flex flex-wrap gap-2">
                              <button
                                onClick={() => handleStartEditQR(qr.id)}
                                className="p-1.5 text-blue-600 hover:bg-blue-50 rounded transition-colors"
                                title={t('upload_qr_edit')}
                              >
                                <Edit size={16} />
                              </button>
                              <button
                                onClick={() => handleDeleteQR(qr.id)}
                                className="p-1.5 text-red-600 hover:bg-red-50 rounded transition-colors"
                                title={t('upload_qr_delete')}
                              >
                                <Trash2 size={16} />
                              </button>
                            </div>
                          )}
                        </td>
                      </tr>
                    );
                  })}
                {/* Hiển thị các dòng đang chỉnh sửa */}
                {editingRows.map((row) => (
                  <tr key={row.tempId} className="bg-yellow-50">
                    <td className="px-3 py-2">
                      <div>
                        <input
                          type="text"
                          value={row.codeIndex}
                          onChange={(e) => handleEditRowChange(row.tempId, 'codeIndex', e.target.value)}
                          placeholder={t('upload_qr_index_placeholder')}
                          className="w-full px-2 py-1 text-xs border border-gray-300 rounded focus:outline-none focus:ring-2 focus:ring-blue-500"
                        />
                        {row.errors.codeIndex && (
                          <p className="text-xs text-red-500 mt-1">{row.errors.codeIndex}</p>
                        )}
                      </div>
                    </td>
                    <td className="px-3 py-2">
                      <div className="space-y-2">
                        <div className="text-xs text-gray-700">
                          X: {isNaN(row.position[0]) ? '--' : row.position[0]}, Y: {isNaN(row.position[1]) ? '--' : row.position[1]}
                        </div>
                        <div className="flex flex-wrap gap-2">
                          <button
                            onClick={() => startPicking(row.tempId, previewMode)}
                            className={`px-3 py-1 text-xs rounded transition-colors ${(isPickingPosition || isPickingPosition3D) && pickingRowId === row.tempId ? 'bg-blue-600 text-white hover:bg-blue-700' : 'bg-amber-500 text-white hover:bg-amber-600'}`}
                          >
                            {(isPickingPosition || isPickingPosition3D) && pickingRowId === row.tempId ? t('upload_qr_pick_active') : t('upload_qr_pick_position')}
                          </button>
                          {(isPickingPosition || isPickingPosition3D) && pickingRowId === row.tempId && (
                            <button
                              onClick={stopPicking}
                              className="px-3 py-1 text-xs rounded bg-gray-600 text-white hover:bg-gray-700 transition-colors"
                            >
                              {t('upload_qr_pick_done')}
                            </button>
                          )}
                        </div>
                        {(isPickingPosition || isPickingPosition3D) && pickingRowId === row.tempId && (
                          <p className="text-[11px] text-blue-600">{previewMode === '2d' ? t('upload_qr_pick_hint') : 'Double click để đặt QR code'}</p>
                        )}
                        {row.errors.position && (
                          <p className="text-xs text-red-500 mt-1">{row.errors.position}</p>
                        )}
                      </div>
                    </td>
                    <td className="px-3 py-2">
                      {/* Empty cell for editing rows */}
                    </td>
                  </tr>
                ))}
                {/* Empty state */}
                {previewQRCodes.filter(qr => !deletedQRCodeIds.has(qr.id)).length === 0 && editingRows.length === 0 && (
                  <tr>
                    <td
                      className="px-3 py-4 text-xs text-gray-500 text-center"
                      colSpan={3}
                    >
                      {t('upload_qr_empty')}
                    </td>
                  </tr>
                )}
                {/* Dòng cuối cùng chứa các buttons - luôn ở dưới cùng của table */}
                {(editingRows.length > 0 || previewQRCodes.filter(qr => !deletedQRCodeIds.has(qr.id)).length > 0) && (
                  <tr className="bg-gray-50">
                    <td colSpan={3} className="px-3 py-4">
                      <div className="flex gap-2 justify-center">
                        {/* Hiển thị button "Lưu" và "Hủy" cho tất cả các dòng đang edit */}
                        {editingRows.length > 0 && (
                          <>
                            <button
                              onClick={handleSaveAllRows}
                              className="px-4 py-2 text-sm bg-green-600 text-white rounded hover:bg-green-700 transition-colors"
                            >
                              {t('upload_qr_save')}
                            </button>
                            <button
                              onClick={() => {
                                stopPicking();
                                setEditingRows([]);
                              }}
                              className="px-4 py-2 text-sm bg-gray-500 text-white rounded hover:bg-gray-600 transition-colors"
                            >
                              {t('upload_qr_cancel')}
                            </button>
                          </>
                        )}
                        {/* Chỉ hiển thị button "thêm mới dữ liệu" khi đã load dữ liệu */}
                        {previewQRCodes.filter(qr => !deletedQRCodeIds.has(qr.id)).length > 0 && (
                          <button
                            onClick={handleAddNewRow}
                            className="px-4 py-2 text-sm bg-blue-600 text-white rounded hover:bg-blue-700 transition-colors"
                          >
                            {t('upload_qr_add_new')}
                          </button>
                        )}
                      </div>
                    </td>
                  </tr>
                )}
              </tbody>
            </table>
          </div>
        </div>
      </div>

      {/* Fullscreen Preview Modal */}
      {isFullscreenPreview && (
        <div className="fixed inset-0 z-[100] bg-black">
          {/* Header Bar */}
          <div className="absolute top-0 left-0 right-0 bg-gray-900 border-b border-gray-700 p-4 flex items-center justify-between">
            <h2 className="text-lg font-semibold text-white">
              {previewMode === '2d' ? t('upload_image_preview_title') : t('upload_pcd_preview_title')}
            </h2>

            {/* Controls */}
            <div className="flex gap-3 items-center">
              {/* View Toggle */}
              <div className="flex gap-2">
                <button
                  onClick={() => setPreviewMode('2d')}
                  className={`inline-flex items-center gap-2 px-3 py-1.5 rounded font-medium text-sm transition-all ${previewMode === '2d'
                    ? 'bg-blue-600 text-white'
                    : 'bg-gray-700 text-gray-300 hover:bg-gray-600'
                    }`}
                >
                  <ImageIcon size={16} />
                  2D
                </button>
                <button
                  onClick={() => setPreviewMode('3d')}
                  className={`inline-flex items-center gap-2 px-3 py-1.5 rounded font-medium text-sm transition-all ${previewMode === '3d'
                    ? 'bg-blue-600 text-white'
                    : 'bg-gray-700 text-gray-300 hover:bg-gray-600'
                    }`}
                >
                  <Grid3x3 size={16} />
                  3D
                </button>
              </div>

              {/* Close Button */}
              <button
                onClick={() => setIsFullscreenPreview(false)}
                className="inline-flex items-center justify-center w-9 h-9 rounded bg-gray-700 text-gray-300 hover:bg-gray-600 transition-colors"
                title="Close Fullscreen"
              >
                <X size={20} />
              </button>
            </div>
          </div>

          {/* Preview Content */}
          <div className="absolute inset-0 top-16 overflow-hidden">
            {/* 2D Preview */}
            {previewMode === '2d' && (
              <div className="w-full h-full bg-gray-50">
                <Image2DPreview
                  vehicles={previewVehicles}
                  qrPins={qrPins}
                  picking={isPickingPosition}
                  onPick={handleMapPositionPick}
                  mapMetadata={mapMetadata}
                  view="top"
                />
                {isPickingPosition && (
                  <div className="absolute inset-0 pointer-events-none">
                    <div className="absolute top-3 left-3 bg-white/95 border border-blue-200 shadow-sm rounded px-3 py-2 max-w-xs pointer-events-auto">
                      <p className="text-xs text-gray-700">{t('upload_qr_pick_hint')}</p>
                      <div className="mt-2 flex gap-2">
                        <button
                          onClick={stopPicking}
                          className="px-2 py-1 text-xs bg-blue-600 text-white rounded hover:bg-blue-700 transition-colors"
                        >
                          {t('upload_qr_pick_done')}
                        </button>
                      </div>
                    </div>
                  </div>
                )}
              </div>
            )}

            {/* 3D Preview */}
            {previewMode === '3d' && (
              <div className="w-full h-full">
                <Canvas
                  camera={{ position: [0, 10, 20], fov: 60 }}
                  style={{ height: '100%' }}
                >
                  <PCDPreview3D
                    vehicles={previewVehicles}
                    pcdUrl={pcdUrl}
                    isPickingMode={isPickingPosition3D}
                    qrMarkers={qrMarkers3D}
                    onPositionPick={handleMap3DPositionPick}
                    previewPosition={previewPickPosition}
                    onPreviewPositionUpdate={setPreviewPickPosition}
                  />
                </Canvas>
                {isPickingPosition3D && (
                  <div className="absolute inset-0 pointer-events-none">
                    <div className="absolute top-3 left-3 bg-white/95 border border-blue-200 shadow-sm rounded px-3 py-2 max-w-xs pointer-events-auto">
                      <p className="text-xs text-gray-700">Double click để đặt QR code</p>
                      <div className="mt-2 flex gap-2">
                        <button
                          onClick={stopPicking}
                          className="px-2 py-1 text-xs bg-blue-600 text-white rounded hover:bg-blue-700 transition-colors"
                        >
                          {t('upload_qr_pick_done')}
                        </button>
                      </div>
                    </div>
                  </div>
                )}
              </div>
            )}
          </div>
        </div>
      )}
    </div>
  );
};

export default Upload;
