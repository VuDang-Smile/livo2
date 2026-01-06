/**
 * PointCloudViewer Component
 * 
 * Component hiển thị Point Cloud 3D với các tính năng:
 * - Render point cloud với Three.js và React Three Fiber
 * - Hỗ trợ vehicle markers và localization zones
 * - Clipping controls (X, Y, Z ranges)
 * - Auto-fit camera to point cloud bounds
 * - Loading và empty states
 * 
 * @component
 * @usedIn MapView3D, DevicesMonitor, PCDViewerPanel
 */
import React, { useRef, useEffect, useState } from 'react';
import * as THREE from 'three';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { OrbitControls, Html } from '@react-three/drei';
import { VehicleMarker3D } from '../../types/vehicleMarker';
import { LocalizationZone } from '../../types/localization';
import { PointCloudData } from '../../types/viewer';
import { useLanguage } from '../contexts/LanguageContext';

export type { PointCloudData } from '../../types/viewer';

interface PointCloudViewerProps {
  points: PointCloudData | null;
  loading?: boolean;
  placeholderText?: string;
  markers?: VehicleMarker3D[];
  localizationZones?: LocalizationZone[];
  clipTopEnabled?: boolean;
  clipZMin?: number;
  clipZMax?: number;
  clipXMin?: number;
  clipXMax?: number;
  clipYMin?: number;
  clipYMax?: number;
  focusPosition?: { x: number; y: number; z: number } | null;
  highlightedVehicleId?: string | null;
}

function PointCloudMesh({ 
  points, 
  clipTopEnabled, 
  clipZMin, 
  clipZMax, 
  clipXMin, 
  clipXMax, 
  clipYMin, 
  clipYMax 
}: {
  points: PointCloudData;
  clipTopEnabled?: boolean;
  clipZMin?: number;
  clipZMax?: number;
  clipXMin?: number;
  clipXMax?: number;
  clipYMin?: number;
  clipYMax?: number;
}) {
  const meshRef = useRef<THREE.Points>(null);
  const [geometry, setGeometry] = useState<THREE.BufferGeometry | null>(null);

  useEffect(() => {
    if (!points) return;

    const { positions, colors } = points;
    
    // Create geometry
    const newGeometry = new THREE.BufferGeometry();
    newGeometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    newGeometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
    
    // Apply clipping
    if (clipTopEnabled && clipZMin !== undefined && clipZMax !== undefined) {
      const positionArray = newGeometry.attributes.position.array as Float32Array;
      const colorArray = newGeometry.attributes.color.array as Float32Array;
      
      const filteredPositions: number[] = [];
      const filteredColors: number[] = [];
      
      for (let i = 0; i < positionArray.length; i += 3) {
        const x = positionArray[i];
        const y = positionArray[i + 1];
        const z = positionArray[i + 2];
        
        // Apply clipping filters
        let include = true;
        
        if (clipZMin !== undefined && z < clipZMin) include = false;
        if (clipZMax !== undefined && z > clipZMax) include = false;
        if (clipXMin !== undefined && x < clipXMin) include = false;
        if (clipXMax !== undefined && x > clipXMax) include = false;
        if (clipYMin !== undefined && y < clipYMin) include = false;
        if (clipYMax !== undefined && y > clipYMax) include = false;
        
        if (include) {
          filteredPositions.push(x, y, z);
          filteredColors.push(colorArray[i], colorArray[i + 1], colorArray[i + 2]);
        }
      }
      
      newGeometry.setAttribute('position', new THREE.Float32BufferAttribute(filteredPositions, 3));
      newGeometry.setAttribute('color', new THREE.Float32BufferAttribute(filteredColors, 3));
    }
    
    setGeometry(newGeometry);
  }, [points, clipTopEnabled, clipZMin, clipZMax, clipXMin, clipXMax, clipYMin, clipYMax]);

  if (!geometry) return null;

  return (
    <points ref={meshRef} geometry={geometry}>
      <pointsMaterial 
        size={0.05} 
        vertexColors 
        sizeAttenuation={true}
        transparent={false}
        alphaTest={0.1}
      />
    </points>
  );
}

function VehicleMarker({ marker, isHighlighted }: { marker: VehicleMarker3D; isHighlighted?: boolean }) {
  const groupRef = useRef<THREE.Group>(null);
  const sphereRef = useRef<THREE.Mesh>(null);
  const [pulseScale, setPulseScale] = useState(1);
  
  useEffect(() => {
    if (groupRef.current) {
      groupRef.current.position.set(...marker.position);
      
      // Set orientation if available
      if (marker.orientation && marker.showOrientation) {
        const [w, x, y, z] = marker.orientation;
        groupRef.current.quaternion.set(x, y, z, w);
      }
    }
  }, [marker.position, marker.orientation, marker.showOrientation]);

  // Pulse animation for highlighted marker
  useFrame((state) => {
    if (isHighlighted && sphereRef.current) {
      const time = state.clock.getElapsedTime();
      const scale = 1 + Math.sin(time * 3) * 0.1; // Pulse between 1.0 and 1.1
      setPulseScale(scale);
      sphereRef.current.scale.setScalar(scale * (isHighlighted ? 1.5 : 1));
    }
  });

  const baseRadius = 0.2;
  const highlightRadius = baseRadius * 1.5;

  return (
    <group ref={groupRef}>
      {/* Highlight glow ring */}
      {isHighlighted && (
        <mesh position={[0, 0, 0]}>
          <ringGeometry args={[highlightRadius * 1.2, highlightRadius * 1.4, 32]} />
          <meshBasicMaterial color={marker.color || '#ff0000'} transparent opacity={0.5} />
        </mesh>
      )}
      
      {/* Main vehicle sphere */}
      <mesh ref={sphereRef}>
        <sphereGeometry args={[isHighlighted ? highlightRadius : baseRadius, 16, 16]} />
        <meshStandardMaterial 
          color={marker.color || '#ff0000'} 
          emissive={isHighlighted ? (marker.color || '#ff0000') : '#000000'}
          emissiveIntensity={isHighlighted ? 0.5 : 0}
        />
      </mesh>
      
      {/* Orientation arrow if available */}
      {marker.showOrientation && marker.orientation && (
        <mesh position={[0, 0, (isHighlighted ? highlightRadius : baseRadius) + 0.1]}>
          <coneGeometry args={[0.1, 0.3, 8]} />
          <meshBasicMaterial color={marker.color || '#ff0000'} />
        </mesh>
      )}
      
      {/* Vehicle ID label */}
      <Html position={[0, (isHighlighted ? highlightRadius : baseRadius) + 0.3, 0]} center>
        <div className={`bg-black bg-opacity-80 text-white px-2 py-1 rounded text-xs whitespace-nowrap ${isHighlighted ? 'ring-2 ring-yellow-400' : ''}`}>
          {marker.id}
        </div>
      </Html>
    </group>
  );
}

function LocalizationZoneMarker({ zone }: { zone: LocalizationZone }) {
  const meshRef = useRef<THREE.Mesh>(null);
  const [time, setTime] = useState(0);
  
  useFrame((state) => {
    const elapsedTime = state.clock.getElapsedTime();
    setTime(elapsedTime);
    
    if (meshRef.current) {
      meshRef.current.rotation.z += 0.005;
    }
  });

  // Tạo hiệu ứng radar với nhiều vòng tròn
  const radarRings = [];
  const numRings = 3;
  
  for (let i = 0; i < numRings; i++) {
    const progress = (time + i * 0.5) % 2; // Chu kỳ 2 giây
    const normalizedProgress = progress / 2;
    
    // Vòng tròn mở rộng từ tâm ra ngoài
    const currentRadius = normalizedProgress * zone.radius;
    const opacity = 1 - normalizedProgress; // Mờ dần khi mở rộng
    
    if (currentRadius > 0.1 && currentRadius < zone.radius) {
      radarRings.push(
        <mesh 
          key={`radar-ring-${i}`}
          position={[0, 0, 0.01 + i * 0.001]}
        >
          <ringGeometry args={[currentRadius * 0.8, currentRadius, 32]} />
          <meshBasicMaterial 
            color={zone.color || '#ffff00'} 
            transparent 
            opacity={opacity * 0.6}
            side={2}
          />
        </mesh>
      );
    }
  }

  return (
    <group position={zone.center}>
      {/* Vòng tròn cố định nằm ngang trên mặt phẳng XY - KHÔNG XOAY */}
      <mesh ref={meshRef} position={[0, 0, 0.1]}>
        <ringGeometry args={[zone.radius * 0.95, zone.radius, 32]} />
        <meshBasicMaterial 
          color={zone.color || '#ffff00'} 
          transparent 
          opacity={0.9}
          side={2}
        />
      </mesh>
      {radarRings}
    </group>
  );
}

function ResizeHandler() {
  const { setSize, gl } = useThree();

  useEffect(() => {
    // Tìm container div bên ngoài Canvas (div với className "h-full w-full bg-black")
    const canvasElement = gl.domElement;
    const container = canvasElement.parentElement?.parentElement;
    
    if (!container) return;

    const resizeObserver = new ResizeObserver((entries) => {
      for (const entry of entries) {
        const { width, height } = entry.contentRect;
        if (width > 0 && height > 0) {
          setSize(width, height);
        }
      }
    });

    resizeObserver.observe(container);

    // Trigger initial resize
    const rect = container.getBoundingClientRect();
    if (rect.width > 0 && rect.height > 0) {
      setSize(rect.width, rect.height);
    }

    return () => {
      resizeObserver.disconnect();
    };
  }, [setSize, gl]);

  return null;
}

function CameraFocusController({ 
  focusPosition, 
  focusedVehicleId 
}: { 
  focusPosition: { x: number; y: number; z: number } | null;
  focusedVehicleId: string | null;
}) {
  const { camera } = useThree();
  const controlsRef = useRef<any>(null);
  const prevFocusPositionRef = useRef<{ x: number; y: number; z: number } | null>(null);
  const animationFrameRef = useRef<number | null>(null);

  useEffect(() => {
    if (!focusPosition || !controlsRef.current) {
      return;
    }

    // Chỉ animate nếu position thay đổi
    const prev = prevFocusPositionRef.current;
    if (prev && prev.x === focusPosition.x && prev.y === focusPosition.y && prev.z === focusPosition.z) {
      return;
    }
    prevFocusPositionRef.current = focusPosition;

    const targetPosition = new THREE.Vector3(focusPosition.x, focusPosition.y, focusPosition.z);
    const startPosition = controlsRef.current.object.position.clone();
    const startTarget = controlsRef.current.target.clone();
    
    // Camera position: offset từ target (ví dụ: 4 units Y, 12 units Z)
    const cameraOffset = new THREE.Vector3(0, 4, 12);
    const finalCameraPosition = targetPosition.clone().add(cameraOffset);
    
    const duration = 800; // 800ms animation
    const startTime = Date.now();

    const animate = () => {
      const elapsed = Date.now() - startTime;
      const progress = Math.min(elapsed / duration, 1);
      
      // Easing function: ease-out cubic
      const easeProgress = 1 - Math.pow(1 - progress, 3);
      
      // Interpolate camera position
      const currentCameraPos = startPosition.clone().lerp(finalCameraPosition, easeProgress);
      controlsRef.current.object.position.copy(currentCameraPos);
      
      // Interpolate target
      const currentTarget = startTarget.clone().lerp(targetPosition, easeProgress);
      controlsRef.current.target.copy(currentTarget);
      controlsRef.current.update();
      
      if (progress < 1) {
        animationFrameRef.current = requestAnimationFrame(animate);
      } else {
        animationFrameRef.current = null;
      }
    };

    animationFrameRef.current = requestAnimationFrame(animate);

    return () => {
      if (animationFrameRef.current !== null) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, [focusPosition, focusedVehicleId]);

  return (
    <OrbitControls 
      ref={controlsRef}
      enableDamping 
      dampingFactor={0.1}
      enablePan={true}
      minDistance={0.5}
      maxDistance={1000}
      mouseButtons={{
        LEFT: 0,    // Rotate
        MIDDLE: 1,  // Zoom 
        RIGHT: 2    // Pan
      }}
      touches={{
        ONE: 1,     // Rotate
        TWO: 2      // Zoom và Pan
      }}
      panSpeed={1.2}
      rotateSpeed={1.0}
      zoomSpeed={1.5}
      enableZoom={true}
      enableRotate={true}
      autoRotate={false}
    />
  );
}

function Scene({ 
  points, 
  markers = [],
  localizationZones = [],
  clipTopEnabled, 
  clipZMin, 
  clipZMax, 
  clipXMin, 
  clipXMax, 
  clipYMin, 
  clipYMax,
  highlightedVehicleId
}: {
  points: PointCloudData | null;
  markers?: VehicleMarker3D[];
  localizationZones?: LocalizationZone[];
  clipTopEnabled?: boolean;
  clipZMin?: number;
  clipZMax?: number;
  clipXMin?: number;
  clipXMax?: number;
  clipYMin?: number;
  clipYMax?: number;
  highlightedVehicleId?: string | null;
}) {
  const { camera } = useThree();

  useEffect(() => {
    if (points) {
      // Auto-fit camera to point cloud bounds
      const { bounds } = points;
      const centerX = (bounds.minX + bounds.maxX) / 2;
      const centerY = (bounds.minY + bounds.maxY) / 2;
      const centerZ = (bounds.minZ + bounds.maxZ) / 2;
      
      const sizeX = bounds.maxX - bounds.minX;
      const sizeY = bounds.maxY - bounds.minY;
      const sizeZ = bounds.maxZ - bounds.minZ;
      const maxSize = Math.max(sizeX, sizeY, sizeZ);
      
      camera.position.set(centerX, centerY, centerZ + maxSize * 0.2);
      camera.lookAt(centerX, centerY, centerZ);
    }
  }, [points, camera]);

  if (!points) return null;

  return (
    <>
      <ambientLight intensity={0.8} />
      <directionalLight position={[10, 10, 5]} intensity={1.2} />
      <directionalLight position={[-10, -10, -5]} intensity={0.6} />
      <PointCloudMesh 
        points={points}
        clipTopEnabled={clipTopEnabled}
        clipZMin={clipZMin}
        clipZMax={clipZMax}
        clipXMin={clipXMin}
        clipXMax={clipXMax}
        clipYMin={clipYMin}
        clipYMax={clipYMax}
      />
      {markers.map((marker) => (
        <VehicleMarker key={marker.id} marker={marker} isHighlighted={highlightedVehicleId === marker.id} />
      ))}
      {localizationZones.map((zone) => (
        <LocalizationZoneMarker key={zone.id} zone={zone} />
      ))}
    </>
  );
}

const PointCloudViewer: React.FC<PointCloudViewerProps> = ({
  points,
  loading = false,
  placeholderText,
  markers = [],
  localizationZones = [],
  clipTopEnabled = false,
  clipZMin,
  clipZMax,
  clipXMin,
  clipXMax,
  clipYMin,
  clipYMax,
  focusPosition,
  highlightedVehicleId,
}) => {
  const { t } = useLanguage();
  const defaultPlaceholderText = placeholderText || t('pcd_no_data');

  if (loading) {
    return (
      <div className="h-full flex items-center justify-center">
        <div className="text-center">
          <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-purple-600 mx-auto"></div>
          <p className="mt-2 text-gray-600">{t('pcd_loading_point_cloud')}</p>
        </div>
      </div>
    );
  }

  if (!points) {
    return (
      <div className="h-full flex items-center justify-center text-gray-500">
        {defaultPlaceholderText}
      </div>
    );
  }

  return (
    <div className="h-full w-full bg-black">
      <Canvas 
        camera={{ position: [0, 0, 5], fov: 75 }}
        gl={{ 
          antialias: true, 
          alpha: false,
          powerPreference: "high-performance"
        }}
        dpr={[1, 2]}
      >
        <color attach="background" args={["#000000"]} />
        <ResizeHandler />
        <Scene 
          points={points}
          markers={markers}
          localizationZones={localizationZones}
          clipTopEnabled={clipTopEnabled}
          clipZMin={clipZMin}
          clipZMax={clipZMax}
          clipXMin={clipXMin}
          clipXMax={clipXMax}
          clipYMin={clipYMin}
          clipYMax={clipYMax}
          highlightedVehicleId={highlightedVehicleId}
        />
        <CameraFocusController 
          focusPosition={focusPosition || null}
          focusedVehicleId={highlightedVehicleId || null}
        />
      </Canvas>
    </div>
  );
};

export default PointCloudViewer;

