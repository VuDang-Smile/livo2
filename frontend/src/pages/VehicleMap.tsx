import React, { useState, useEffect, useRef, useMemo, useCallback, Suspense } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Html } from '@react-three/drei';
import * as THREE from 'three';
import { useLanguage } from '../contexts/LanguageContext';
import { VehiclePosition, getMockVehiclePositions } from '../mock/vehicleMockData';
import PCDMap, { PointCloudBounds } from '../components/PCDMap';
import { PCDClipControls } from '../components/PCDClipControls';
import { DEFAULT_PCD_URL } from '../constants/pcdConfig';
import { useMapImage } from '../hooks/useMapImage';
import { MAP_2D_IMAGE_URL, getCurrentMap, getMapImageUrl, getMapMetadataUrl } from '../constants/mapConfig';
import { getVehicle2DPosition } from '../utils/vehicle2DHelper';
import { MapMetadata, Pose3D } from '../types/mapMetadata';
import { transformPoseToPixel } from '../utils/coordinateTransform';

// Assign mock function to window so Upload page can use it
declare global {
  interface Window {
    __getMockVehiclePositions?: typeof getMockVehiclePositions;
  }
}

// Component cho đường hầm
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

// Component cho phương tiện
const Vehicle: React.FC<{ 
  vehicle: VehiclePosition; 
  isSelected?: boolean; 
  onSelect?: () => void;
}> = ({ vehicle, isSelected = false, onSelect }) => {
  const { t } = useLanguage();
  const meshRef = useRef<THREE.Mesh>(null);
  const [hovered, setHovered] = useState(false);
  
  // Animation cho phương tiện
  useFrame((state) => {
    if (meshRef.current) {
      meshRef.current.position.y = vehicle.position[1] + Math.sin(state.clock.elapsedTime * 2) * 0.1;
    }
  });

  const getVehicleColor = (status: string) => {
    switch (status) {
      case 'active': return '#27ae60';
      case 'maintenance': return '#f39c12';
      case 'inactive': return '#e74c3c';
      default: return '#95a5a6';
    }
  };

  const getVehicleGeometry = (vehicleType: string) => {
    switch (vehicleType) {
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
    <group position={vehicle.position}>
      {/* Phương tiện */}
      <mesh
        ref={meshRef}
        onPointerOver={() => setHovered(true)}
        onPointerOut={() => setHovered(false)}
        onClick={onSelect}
        scale={hovered || isSelected ? 1.2 : 1}
      >
        {getVehicleGeometry(vehicle.vehicleType)}
        <meshStandardMaterial 
          color={getVehicleColor(vehicle.status)}
          emissive={hovered || isSelected ? getVehicleColor(vehicle.status) : '#000000'}
          emissiveIntensity={hovered || isSelected ? 0.3 : 0}
        />
      </mesh>
      
      {/* Label hiển thị thông tin */}
      {hovered && (
        <Html position={[0, 5, 0]} center>
          <div className="bg-black bg-opacity-80 text-white p-3 rounded-lg text-sm whitespace-nowrap">
            <div className="font-bold">{vehicle.licensePlate}</div>
            <div>{t('driver_label')} {vehicle.driver}</div>
            <div>{t('type_label')} {vehicle.vehicleType}</div>
            <div>{t('mission_label_short')} {vehicle.mission}</div>
            <div>{t('status_label_short')} {
              vehicle.status === 'active' ? t('active_status') :
              vehicle.status === 'maintenance' ? t('maintenance_status') :
              t('inactive_status')
            }</div>
            <div>{t('update_label')} {vehicle.lastUpdate}</div>
          </div>
        </Html>
      )}
      
      {/* Đèn phát sáng cho phương tiện đang hoạt động */}
      {vehicle.status === 'active' && (
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

// Component chính cho bản đồ
const VehicleMapScene: React.FC<{ 
  selectedVehicleId: string | null; 
  onVehicleSelect: (id: string) => void;
  vehicles: VehiclePosition[];
  pcdUrl?: string;
  clipXMin?: number;
  clipXMax?: number;
  clipYMin?: number;
  clipYMax?: number;
  clipZMin?: number;
  clipZMax?: number;
  onBoundsCalculated?: (bounds: PointCloudBounds) => void;
}> = ({ 
  selectedVehicleId, 
  onVehicleSelect,
  vehicles,
  pcdUrl,
  clipXMin,
  clipXMax,
  clipYMin,
  clipYMax,
  clipZMin,
  clipZMax,
  onBoundsCalculated,
}) => {
  const controlsRef = useRef<any>(null);

  // Focus camera vào phương tiện được chọn, giữ nguyên hướng nhìn hiện tại
  useEffect(() => {
    if (!selectedVehicleId || !controlsRef.current) return;

    const controls = controlsRef.current;
    const selectedVehicle = vehicles.find(v => v.id === selectedVehicleId);
    if (!selectedVehicle) return;

    const targetPosition = new THREE.Vector3(...selectedVehicle.position);

    // Bảo toàn hướng nhìn hiện tại (azimuth/polar) và tôn trọng minDistance
    const currentTarget = controls.target.clone();
    const camera = controls.object;
    const viewDir = camera.position.clone().sub(currentTarget); // từ target -> camera
    const currentDistance = viewDir.length();
    const normalizedDir = viewDir.normalize();

    const minDistance = controls.minDistance ?? 0;
    const safetyMargin = 1; // tránh chạm ngưỡng minDistance
    const desiredDistance = Math.max(minDistance + safetyMargin, currentDistance);
    const newCameraPosition = targetPosition.clone().add(normalizedDir.multiplyScalar(desiredDistance));

    // Animate camera & target tới vị trí mới, luôn update controls để không bị giật
    const startPosition = camera.position.clone();
    const startTarget = currentTarget.clone();
    const duration = 1000; // 1 giây
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
  }, [selectedVehicleId, vehicles]);

  return (
    <>
      {/* Camera controls */}
      <OrbitControls 
        ref={controlsRef}
        enablePan={true}
        enableZoom={true}
        enableRotate={true}
        maxPolarAngle={Math.PI / 2}
        minDistance={10}
        maxDistance={200}
      />
      
      {/* Lighting */}
      <ambientLight intensity={0.4} />
      <directionalLight position={[10, 10, 5]} intensity={0.8} />
      
      {/* Background color */}
      <color attach="background" args={['#000000']} />
      
      {/* Tunnel / PCD map */}
      {pcdUrl ? (
        <Suspense fallback={null}>
          <PCDMap 
            url={pcdUrl} 
            // Các giá trị transform tạm thời, sẽ tinh chỉnh sau khi có PCD thật
            scale={1}
            rotation={[-Math.PI / 2, 0, 0]}
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
      ) : (
        <Tunnel />
      )}
      
      {/* Vehicles */}
      {vehicles.map((vehicle) => (
        <Vehicle 
          key={vehicle.id} 
          vehicle={vehicle} 
          isSelected={vehicle.id === selectedVehicleId}
          onSelect={() => onVehicleSelect(vehicle.id)}
        />
      ))}
      
      {/* Grid helper */}
      <gridHelper args={[200, 20, '#34495e', '#2c3e50']} />
    </>
  );
};

// Component cho bản đồ 2D
const Map2D: React.FC<{ 
  vehicles: VehiclePosition[];
  selectedVehicleId: string | null;
  onVehicleSelect: (id: string) => void;
  view?: 'top' | 'side_x' | 'side_y';
  mapMetadata?: MapMetadata | null;
  uploadId?: string | null;
}> = ({ vehicles, selectedVehicleId, onVehicleSelect, view = 'top', mapMetadata, uploadId }) => {
  const { t } = useLanguage();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [hoveredVehicle, setHoveredVehicle] = useState<string | null>(null);
  
  // Determine image URL
  const imageUrl = useMemo(() => {
    if (uploadId && mapMetadata) {
      return getMapImageUrl(uploadId, view);
    }
    return MAP_2D_IMAGE_URL; // Fallback to default
  }, [uploadId, view, mapMetadata]);
  
  // Load map image với proper cleanup
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

    // Draw background image nếu đã load thành công
    if (mapImage && mapImage.complete && !imageError) {
      // Calculate scale để fit toàn bộ image trong canvas (contain mode - không crop)
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
      // Fallback: Draw tunnel outline và grid nếu image chưa load hoặc load fail
      // Draw tunnel outline
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

    // Draw vehicles
    vehicles.forEach(vehicle => {
      const isSelected = vehicle.id === selectedVehicleId;
      const isHovered = vehicle.id === hoveredVehicle;
      
      // Get 2D position - use transform if metadata available, otherwise fallback
      let x: number, y: number;
      if (mapMetadata && vehicle.position) {
        // Convert 3D pose to 2D pixel using coordinateTransform
        const pose3D: Pose3D = {
          position: {
            x: vehicle.position[0],
            y: vehicle.position[1],
            z: vehicle.position[2]
          },
          orientation: {
            x: 0, y: 0, z: 0, w: 1 // Default orientation (can be enhanced later)
          }
        };
        
        const pixel = transformPoseToPixel(pose3D, mapMetadata, view, false);
        if (pixel && mapImage && mapImage.complete && mapImage.naturalWidth > 0 && mapImage.naturalHeight > 0) {
          // Scale pixel coordinates to canvas size
          const scale = Math.min(
            canvas.width / mapImage.naturalWidth,
            canvas.height / mapImage.naturalHeight
          );
          const scaledWidth = mapImage.naturalWidth * scale;
          const scaledHeight = mapImage.naturalHeight * scale;
          const offsetX = (canvas.width - scaledWidth) / 2;
          const offsetY = (canvas.height - scaledHeight) / 2;
          
          x = offsetX + (pixel.pixel_x / mapImage.naturalWidth) * scaledWidth;
          y = offsetY + (pixel.pixel_y / mapImage.naturalHeight) * scaledHeight;
        } else {
          // Fallback to helper function
          const pos = getVehicle2DPosition(vehicle, canvas.width, canvas.height);
          x = pos.x;
          y = pos.y;
        }
      } else {
        // Fallback to helper function
        const pos = getVehicle2DPosition(vehicle, canvas.width, canvas.height);
        x = pos.x;
        y = pos.y;
      }
      
      // Vehicle color based on status
      let color = '#95a5a6';
      switch (vehicle.status) {
        case 'active': color = '#27ae60'; break;
        case 'maintenance': color = '#f39c12'; break;
        case 'inactive': color = '#e74c3c'; break;
      }

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

      // Draw vehicle label
      if (isSelected || isHovered) {
        ctx.fillStyle = '#2c3e50';
        ctx.font = '12px Arial';
        ctx.textAlign = 'center';
        ctx.fillText(vehicle.licensePlate, x, y - 15);
      }
    });
  }, [vehicles, selectedVehicleId, hoveredVehicle, mapImage, imageError, mapMetadata, view]);

  const handleCanvasClick = (event: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;

    // Check if click is on a vehicle
    vehicles.forEach(vehicle => {
      const { x: vehicleX, y: vehicleY } = getVehicle2DPosition(vehicle, canvas.width, canvas.height);
      
      const distance = Math.sqrt((x - vehicleX) ** 2 + (y - vehicleY) ** 2);
      if (distance <= 12) {
        onVehicleSelect(vehicle.id);
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
    vehicles.forEach(vehicle => {
      const { x: vehicleX, y: vehicleY } = getVehicle2DPosition(vehicle, canvas.width, canvas.height);
      
      const distance = Math.sqrt((x - vehicleX) ** 2 + (y - vehicleY) ** 2);
      if (distance <= 12) {
        foundVehicle = vehicle.id;
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
            <span className="text-xs text-gray-600">{t('active_status')}</span>
          </div>
          <div className="flex items-center space-x-2">
            <div className="w-3 h-3 bg-yellow-500 rounded-full"></div>
            <span className="text-xs text-gray-600">{t('maintenance_status')}</span>
          </div>
          <div className="flex items-center space-x-2">
            <div className="w-3 h-3 bg-red-500 rounded-full"></div>
            <span className="text-xs text-gray-600">{t('inactive_status')}</span>
          </div>
        </div>
      </div>
    </div>
  );
};

// Hàm filter tìm kiếm theo biển số hoặc tên tài xế
const filterBySearch = (vehicles: VehiclePosition[], query: string): VehiclePosition[] => {
  if (!query.trim()) return vehicles;
  
  const lowerQuery = query.toLowerCase().trim();
  
  return vehicles.filter(vehicle => {
    const matchesLicensePlate = vehicle.licensePlate
      .toLowerCase()
      .includes(lowerQuery);
    
    const matchesDriver = vehicle.driver
      .toLowerCase()
      .includes(lowerQuery);
    
    return matchesLicensePlate || matchesDriver;
  });
};

// Component chính cho trang
const VehicleMap: React.FC = () => {
  const { t } = useLanguage();
  const [selectedVehicleId, setSelectedVehicleId] = useState<string | null>(null);
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [showFilter, setShowFilter] = useState(false);
  const [viewMode, setViewMode] = useState<'2D' | '3D'>('3D');
  const [selectedView, setSelectedView] = useState<'top' | 'side_x' | 'side_y'>('top');
  const [mapMetadata, setMapMetadata] = useState<MapMetadata | null>(null);
  const [uploadId, setUploadId] = useState<string | null>(null);
  const [isLoadingMetadata, setIsLoadingMetadata] = useState(false);
  const [filters, setFilters] = useState({
    status: 'all',
    vehicleType: 'all',
    mission: 'all'
  });
  const [searchQuery, setSearchQuery] = useState<string>('');
  const [searchInputValue, setSearchInputValue] = useState<string>('');
  const searchTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  
  // Load map metadata when component mounts
  useEffect(() => {
    const loadMapMetadata = async () => {
      setIsLoadingMetadata(true);
      try {
        const currentMap = await getCurrentMap();
        if (currentMap && currentMap.upload_id) {
          setUploadId(currentMap.upload_id);
          
          // Load metadata JSON
          const metadataUrl = getMapMetadataUrl(currentMap.upload_id);
          const response = await fetch(metadataUrl);
          if (response.ok) {
            const metadata = await response.json();
            setMapMetadata(metadata);
          } else {
            console.warn('Failed to load map metadata');
          }
        }
      } catch (error) {
        console.error('Error loading map metadata:', error);
      } finally {
        setIsLoadingMetadata(false);
      }
    };
    
    loadMapMetadata();
  }, []);
  
  // PCD Clipping state
  const [pcdBounds, setPcdBounds] = useState<PointCloudBounds | null>(null);
  const [clipXRange, setClipXRange] = useState<[number, number]>([0, 100]);
  const [clipYRange, setClipYRange] = useState<[number, number]>([0, 100]);
  const [clipZRange, setClipZRange] = useState<[number, number]>([0, 100]);
  const [showClipControls, setShowClipControls] = useState(false);

  const toggleFullscreen = () => {
    setIsFullscreen(!isFullscreen);
  };

  const handleVehicleSelect = (id: string) => {
    setSelectedVehicleId(id);
  };

  // PCD Bounds callback handler - memoized để tránh re-run useEffect trong PCDMap
  const handleBoundsCalculated = useCallback((bounds: PointCloudBounds) => {
    setPcdBounds(bounds);
    // Reset clip ranges về full range khi có bounds mới
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

  // Tính toán clipping values từ ranges - chỉ tính khi có bounds
  // Không truyền undefined để tránh override default parameters trong PCDClippingMaterial
  const realXRange = pcdBounds ? getRealXRange() : null;
  const realYRange = pcdBounds ? getRealYRange() : null;
  const realZRange = pcdBounds ? getRealZRange() : null;

  // Đăng ký hàm mock lên window để các màn hình khác (vd: Upload) có thể dùng chung dữ liệu
  // Sử dụng useEffect để tránh side effect trong render phase và đảm bảo chạy sau khi component mount
  useEffect(() => {
    if (typeof window !== 'undefined') {
      window.__getMockVehiclePositions = getMockVehiclePositions;
    }
    // Cleanup function để xóa khi component unmount (optional, nhưng tốt cho cleanup)
    return () => {
      if (typeof window !== 'undefined' && window.__getMockVehiclePositions) {
        delete window.__getMockVehiclePositions;
      }
    };
  }, [t]); // Phụ thuộc vào t để đảm bảo dữ liệu mock cập nhật theo ngôn ngữ

  // Cleanup timeout khi component unmount để tránh memory leak
  useEffect(() => {
    return () => {
      if (searchTimeoutRef.current) {
        clearTimeout(searchTimeoutRef.current);
      }
    };
  }, []);

  const mockVehiclePositions = getMockVehiclePositions(t);
  const selectedVehicle = mockVehiclePositions.find(v => v.id === selectedVehicleId);

  // Filter vehicles: áp dụng search filter trước, sau đó áp dụng các filter khác
  const filteredVehicles = useMemo(() => {
    const searchFiltered = filterBySearch(mockVehiclePositions, searchQuery);
    return searchFiltered.filter(vehicle => {
      if (filters.status !== 'all' && vehicle.status !== filters.status) return false;
      if (filters.vehicleType !== 'all' && vehicle.vehicleType !== filters.vehicleType) return false;
      if (filters.mission !== 'all' && vehicle.mission !== filters.mission) return false;
      return true;
    });
  }, [mockVehiclePositions, searchQuery, filters.status, filters.vehicleType, filters.mission]);

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
      mission: 'all'
    });
  };

  const handleSearchChange = (value: string) => {
    // Cập nhật input value ngay lập tức để hiển thị
    setSearchInputValue(value);
    
    // Clear timeout cũ nếu có
    if (searchTimeoutRef.current) {
      clearTimeout(searchTimeoutRef.current);
    }
    
    // Tạo timeout mới (300ms) để set searchQuery (dùng cho filter)
    searchTimeoutRef.current = setTimeout(() => {
      setSearchQuery(value);
      searchTimeoutRef.current = null; // Reset sau khi chạy
    }, 300);
  };

  const getUniqueValues = (key: 'status' | 'vehicleType' | 'mission') => {
    const values = mockVehiclePositions.map(v => v[key]);
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
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 6a2 2 0 012-2h2a2 2 0 012 2v2a2 2 0 01-2 2H6a2 2 0 01-2-2V6zM14 6a2 2 0 012-2h2a2 2 0 012 2v2a2 2 0 01-2 2h-2a2 2 0 01-2-2V6zM4 16a2 2 0 012-2h2a2 2 0 012 2v2a2 2 0 01-2 2H6a2 2 0 01-2-2v-2zM14 16a2 2 0 012-2h2a2 2 0 012 2v2a2 2 0 01-2 2h-2a2 2 0 01-2-2v-2z" />
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
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M14 10l-2 1m0 0l-2-1m2 1v2.5M20 7l-2 1m2-1l-2-1m2 1v2.5M14 4l-2-1-2 1M4 7l2-1M4 7l2 1M4 7v2.5M12 21l-2-1m2 1l2-1m-2 1v-2.5M6 18l-2-1v-2.5M18 18l2-1v-2.5" />
              </svg>
              <span>3D</span>
            </button>
          </div>
          
          <button
            onClick={toggleFullscreen}
            className="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded-lg font-medium transition-colors duration-200 flex items-center space-x-2"
          >
            <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 8V4m0 0h4M4 4l5 5m11-1V4m0 0h-4m4 0l-5 5M4 16v4m0 0h4m-4 0l5-5m11 5l-5-5m5 5v-4m0 4h-4" />
            </svg>
            <span>{isFullscreen ? t('exit_fullscreen') : t('fullscreen')}</span>
          </button>
        </div>
      </div>

      {/* Stats Cards */}
      <div className="grid grid-cols-1 md:grid-cols-4 gap-6">
        <div className="bg-white p-6 rounded-lg shadow-sm border border-gray-200">
          <div className="flex items-center">
            <div className="p-2 bg-green-100 rounded-lg">
              <div className="w-4 h-4 bg-green-500 rounded-full"></div>
            </div>
            <div className="ml-4">
              <p className="text-sm font-medium text-gray-600">{t('active_vehicles_count')}</p>
              <p className="text-2xl font-bold text-gray-900">
                {mockVehiclePositions.filter(v => v.status === 'active').length}
              </p>
            </div>
          </div>
        </div>

        <div className="bg-white p-6 rounded-lg shadow-sm border border-gray-200">
          <div className="flex items-center">
            <div className="p-2 bg-yellow-100 rounded-lg">
              <div className="w-4 h-4 bg-yellow-500 rounded-full"></div>
            </div>
            <div className="ml-4">
              <p className="text-sm font-medium text-gray-600">{t('maintenance_vehicles_count')}</p>
              <p className="text-2xl font-bold text-gray-900">
                {mockVehiclePositions.filter(v => v.status === 'maintenance').length}
              </p>
            </div>
          </div>
        </div>

        <div className="bg-white p-6 rounded-lg shadow-sm border border-gray-200">
          <div className="flex items-center">
            <div className="p-2 bg-red-100 rounded-lg">
              <div className="w-4 h-4 bg-red-500 rounded-full"></div>
            </div>
            <div className="ml-4">
              <p className="text-sm font-medium text-gray-600">{t('inactive_vehicles_count')}</p>
              <p className="text-2xl font-bold text-gray-900">
                {mockVehiclePositions.filter(v => v.status === 'inactive').length}
              </p>
            </div>
          </div>
        </div>

        <div className="bg-white p-6 rounded-lg shadow-sm border border-gray-200">
          <div className="flex items-center">
            <div className="p-2 bg-blue-100 rounded-lg">
              <svg className="w-6 h-6 text-blue-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 20l-5.447-2.724A1 1 0 013 16.382V5.618a1 1 0 011.447-.894L9 7m0 13l6-3m-6 3V7m6 10l4.553 2.276A1 1 0 0021 18.382V7.618a1 1 0 00-1.447-.894L15 4m0 13V4m-6 3l6-3" />
              </svg>
            </div>
            <div className="ml-4">
              <p className="text-sm font-medium text-gray-600">{t('total_vehicles_count')}</p>
              <p className="text-2xl font-bold text-gray-900">{mockVehiclePositions.length}</p>
            </div>
          </div>
        </div>
      </div>

      {/* 3D Map Container with Vehicle List */}
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
                <Canvas
                  camera={{ position: [0, 10, 20], fov: 60 }}
                  style={{ height: isFullscreen ? '100vh' : '100%' }}
                >
                  <VehicleMapScene 
                    selectedVehicleId={selectedVehicleId}
                    onVehicleSelect={handleVehicleSelect}
                    vehicles={filteredVehicles}
                    pcdUrl={DEFAULT_PCD_URL}
                    clipXMin={realXRange ? realXRange[0] : undefined}
                    clipXMax={realXRange ? realXRange[1] : undefined}
                    clipYMin={realYRange ? realYRange[0] : undefined}
                    clipYMax={realYRange ? realYRange[1] : undefined}
                    clipZMin={realZRange ? realZRange[0] : undefined}
                    clipZMax={realZRange ? realZRange[1] : undefined}
                    onBoundsCalculated={handleBoundsCalculated}
                  />
                </Canvas>
                
                {/* PCD Clipping Controls Overlay */}
                {showClipControls && (
                  <div className="absolute top-4 left-4 bg-white rounded-lg shadow-lg border border-gray-200 p-4 z-10 max-w-sm">
                    <div className="flex items-center justify-between mb-3">
                      <h3 className="text-sm font-semibold text-gray-900">{t('pcd_adjust_point_cloud')}</h3>
                      <button
                        onClick={() => setShowClipControls(false)}
                        className="text-gray-400 hover:text-gray-600 p-1 rounded hover:bg-gray-100"
                        title={t('pcd_close')}
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
                    className="absolute top-4 left-4 bg-white hover:bg-gray-50 text-gray-700 px-3 py-2 rounded-lg shadow-md border border-gray-200 z-10 flex items-center space-x-2 transition-colors"
                    title={t('pcd_show_adjust')}
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
                <Map2D
                  vehicles={filteredVehicles}
                  selectedVehicleId={selectedVehicleId}
                  onVehicleSelect={handleVehicleSelect}
                  view={selectedView}
                  mapMetadata={mapMetadata}
                  uploadId={uploadId}
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
                  className="p-2 text-gray-600 hover:text-gray-900 hover:bg-gray-200 rounded-lg transition-colors duration-200"
                  title={t('filter_vehicles')}
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
              {(filters.status !== 'all' || filters.vehicleType !== 'all' || filters.mission !== 'all') && (
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
                  <div className="flex flex-wrap gap-2">
                    {filters.status !== 'all' && (
                      <span className="px-2 py-1 bg-blue-100 text-blue-800 text-xs rounded-full">
                        {t('status_filter')} {filters.status === 'active' ? t('active_status') : filters.status === 'maintenance' ? t('maintenance_status') : t('inactive_status')}
                      </span>
                    )}
                    {filters.vehicleType !== 'all' && (
                      <span className="px-2 py-1 bg-blue-100 text-blue-800 text-xs rounded-full">
                        {t('vehicle_type_filter')} {filters.vehicleType}
                      </span>
                    )}
                    {filters.mission !== 'all' && (
                      <span className="px-2 py-1 bg-blue-100 text-blue-800 text-xs rounded-full">
                        {t('mission_filter')} {filters.mission}
                      </span>
                    )}
                  </div>
                </div>
              )}
              
              <div className="space-y-3">
                {filteredVehicles.map((vehicle) => (
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
                              vehicle.status === 'active' ? 'bg-green-500' :
                              vehicle.status === 'maintenance' ? 'bg-yellow-500' :
                              'bg-red-500'
                            }`}
                          />
                          <h4 className="font-medium text-gray-900">{vehicle.licensePlate}</h4>
                        </div>
                        
                        <div className="text-sm text-gray-600 space-y-1">
                          <p><span className="font-medium">{t('driver_label')}</span> {vehicle.driver}</p>
                          <p><span className="font-medium">{t('type_label')}</span> {vehicle.vehicleType}</p>
                          <p><span className="font-medium">{t('mission_label_short')}</span> {vehicle.mission}</p>
                          <p><span className="font-medium">{t('status_label_short')}</span> 
                            <span className={`ml-1 ${
                              vehicle.status === 'active' ? 'text-green-600' :
                              vehicle.status === 'maintenance' ? 'text-yellow-600' :
                              'text-red-600'
                            }`}>
                              {vehicle.status === 'active' ? t('active_status') :
                               vehicle.status === 'maintenance' ? t('maintenance_status') :
                               t('inactive_status')}
                            </span>
                          </p>
                          <p className="text-xs text-gray-500">
                            {t('update_label')} {vehicle.lastUpdate}
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

      {/* Legend and Vehicle Details */}
      <div className="bg-white p-6 rounded-lg shadow-sm border border-gray-200">
        {selectedVehicle ? (
          /* Vehicle Details */
          <div>
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
                  <p className="text-sm"><span className="font-medium">{t('license_plate_label')}</span> {selectedVehicle.licensePlate}</p>
                  <p className="text-sm"><span className="font-medium">{t('vehicle_type_label')}</span> {selectedVehicle.vehicleType}</p>
                  <p className="text-sm"><span className="font-medium">{t('status_label')}</span> 
                    <span className={`ml-1 ${
                      selectedVehicle.status === 'active' ? 'text-green-600' :
                      selectedVehicle.status === 'maintenance' ? 'text-yellow-600' :
                      'text-red-600'
                    }`}>
                      {selectedVehicle.status === 'active' ? t('active_status') :
                       selectedVehicle.status === 'maintenance' ? t('maintenance_status') :
                       t('inactive_status')}
                    </span>
                  </p>
                </div>
              </div>
              
              <div className="space-y-2">
                <h4 className="font-medium text-gray-700 text-sm uppercase tracking-wide">{t('driver_info')}</h4>
                <div className="space-y-1">
                  <p className="text-sm"><span className="font-medium">{t('driver_name')}</span> {selectedVehicle.driver}</p>
                  <p className="text-sm"><span className="font-medium">{t('id_label')}</span> {selectedVehicle.id}</p>
                </div>
              </div>
              
              <div className="space-y-2">
                <h4 className="font-medium text-gray-700 text-sm uppercase tracking-wide">{t('mission_type')}</h4>
                <div className="space-y-1">
                  <p className="text-sm"><span className="font-medium">{t('mission_label')}</span> {selectedVehicle.mission}</p>
                </div>
              </div>
              
              <div className="space-y-2">
                <h4 className="font-medium text-gray-700 text-sm uppercase tracking-wide">{t('position_time')}</h4>
                <div className="space-y-1">
                  <p className="text-sm"><span className="font-medium">{t('coordinates')}</span></p>
                  <p className="text-xs text-gray-600">X: {selectedVehicle.position[0]}</p>
                  <p className="text-xs text-gray-600">Y: {selectedVehicle.position[1]}</p>
                  <p className="text-xs text-gray-600">Z: {selectedVehicle.position[2]}</p>
                  <p className="text-sm mt-2"><span className="font-medium">{t('update_time')}</span> {selectedVehicle.lastUpdate}</p>
                </div>
              </div>
            </div>
            
            <div className="mt-6 flex space-x-3">
              <button
                onClick={() => handleVehicleSelect(selectedVehicle.id)}
                className="px-4 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700 transition-colors duration-200 flex items-center space-x-2"
              >
                <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 12a3 3 0 11-6 0 3 3 0 016 0z" />
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M2.458 12C3.732 7.943 7.523 5 12 5c4.478 0 8.268 2.943 9.542 7-1.274 4.057-5.064 7-9.542 7-4.477 0-8.268-2.943-9.542-7z" />
                </svg>
                <span>{t('focus_camera')}</span>
              </button>
              
              <button
                onClick={() => setSelectedVehicleId(null)}
                className="px-4 py-2 border border-gray-300 text-gray-700 rounded-lg hover:bg-gray-50 transition-colors duration-200"
              >
                {t('deselect')}
              </button>
            </div>
          </div>
        ) : (
          /* Legend */
          <div>
            <h3 className="text-lg font-medium text-gray-900 mb-4">{t('legend')}</h3>
            <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
              <div className="flex items-center space-x-3">
                <div className="w-4 h-4 bg-green-500 rounded"></div>
                <span className="text-sm text-gray-700">{t('active_status')}</span>
              </div>
              <div className="flex items-center space-x-3">
                <div className="w-4 h-4 bg-yellow-500 rounded"></div>
                <span className="text-sm text-gray-700">{t('maintenance_status')}</span>
              </div>
              <div className="flex items-center space-x-3">
                <div className="w-4 h-4 bg-red-500 rounded"></div>
                <span className="text-sm text-gray-700">{t('inactive_status')}</span>
              </div>
            </div>
            <div className="mt-4 text-sm text-gray-600">
              <p>💡 <strong>{t('guide_text')}</strong> {viewMode === '2D' ? t('guide_text_2d') : t('guide_text_3d')} {t('guide_text_end')}</p>
            </div>
          </div>
        )}
      </div>

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
                           status === 'active' ? t('active_status') :
                           status === 'maintenance' ? t('maintenance_status') :
                           t('inactive_status')}
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

                {/* Mission Filter */}
                <div>
                  <label className="block text-sm font-medium text-gray-700 mb-3">
                    {t('mission')}
                  </label>
                  <div className="space-y-2">
                    {getUniqueValues('mission').map((mission) => (
                      <label key={mission} className="flex items-center">
                        <input
                          type="radio"
                          name="mission"
                          value={mission}
                          checked={filters.mission === mission}
                          onChange={(e) => handleFilterChange('mission', e.target.value)}
                          className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300"
                        />
                        <span className="ml-3 text-sm text-gray-700">
                          {mission === 'all' ? t('all') : mission}
                        </span>
                      </label>
                    ))}
                  </div>
                </div>
              </div>

              <div className="flex space-x-3 mt-8 pt-6 border-t border-gray-200">
                <button
                  onClick={clearFilters}
                  className="flex-1 px-4 py-2 border border-gray-300 text-gray-700 rounded-lg hover:bg-gray-50 transition-colors duration-200"
                >
                  {t('clear_filter')}
                </button>
                <button
                  onClick={() => setShowFilter(false)}
                  className="flex-1 px-4 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700 transition-colors duration-200"
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