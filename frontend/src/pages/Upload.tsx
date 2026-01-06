import React, { useState, useRef, useEffect, useMemo, useCallback, Suspense } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import * as THREE from 'three';
import { Edit, Trash2 } from 'lucide-react';
import { useLanguage } from '../contexts/LanguageContext';
import { VehiclePosition, getMockVehiclePositions } from '../mock/vehicleMockData';
import { getMockQRCodes } from '../mock/qrMockData';
import { MapInfo, getMockMapInfo } from '../mock/mapInfoMockData';
import PCDMap from '../components/PCDMap';
import { DEFAULT_PCD_URL } from '../constants/pcdConfig';
import { useMapImage } from '../hooks/useMapImage';
import { MAP_2D_IMAGE_URL } from '../constants/mapConfig';
import { getVehicle2DPosition } from '../utils/vehicle2DHelper';

interface QRCodeInfo {
  id: string;
  code: string;
  position: [number, number];
  isManual?: boolean; // Đánh dấu dòng được thêm thủ công
}

interface EditingRow {
  tempId: string;
  codeIndex: string;
  position: [number, number];
  errors: {
    codeIndex?: string;
    position?: string;
  };
}

interface ManualPin {
  id: string;
  position: [number, number];
  isActive?: boolean;
  isDraft?: boolean;
  label?: string;
}

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

// Component cho phương tiện (copy từ VehicleMap)
const Vehicle: React.FC<{ 
  vehicle: VehiclePosition; 
}> = ({ vehicle }) => {
  const meshRef = useRef<THREE.Mesh>(null);
  
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
      <mesh ref={meshRef}>
        {getVehicleGeometry(vehicle.vehicleType)}
        <meshStandardMaterial 
          color={getVehicleColor(vehicle.status)}
          emissive={getVehicleColor(vehicle.status)}
          emissiveIntensity={0.2}
        />
      </mesh>
      
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

// Component cho bản đồ 3D Preview
const PCDPreview3D: React.FC<{ 
  vehicles: VehiclePosition[];
  pcdUrl?: string | null;
}> = ({ vehicles, pcdUrl }) => {
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
            rotation={[-Math.PI / 2, 0, 0]}
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
        />
      ))}
      
      {/* Grid helper */}
      <gridHelper args={[200, 20, '#34495e', '#2c3e50']} />
    </>
  );
};

// Component cho bản đồ 2D Preview
const Image2DPreview: React.FC<{ 
  vehicles: VehiclePosition[];
  qrPins?: ManualPin[];
  picking?: boolean;
  onPick?: (pos: [number, number]) => void;
}> = ({ vehicles, qrPins = [], picking = false, onPick }) => {
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

    const worldToCanvas = (worldX: number, worldZ: number) => {
      const x = ((worldX + 100) / 200) * (canvas.width - 100) + 50;
      const y = ((worldZ + 50) / 100) * (canvas.height - 100) + 50;
      return { x, y };
    };

    // Draw vehicles
    vehicles.forEach(vehicle => {
      // Get 2D position using helper function (uses position2D if available, otherwise falls back to center)
      const { x, y } = getVehicle2DPosition(vehicle, canvas.width, canvas.height);
      
      // Vehicle color based on status
      let color = '#95a5a6';
      switch (vehicle.status) {
        case 'active': color = '#27ae60'; break;
        case 'maintenance': color = '#f39c12'; break;
        case 'inactive': color = '#e74c3c'; break;
      }

      // Draw vehicle circle
      ctx.beginPath();
      ctx.arc(x, y, 8, 0, 2 * Math.PI);
      ctx.fillStyle = color;
      ctx.fill();
      
      ctx.strokeStyle = '#2c3e50';
      ctx.lineWidth = 2;
      ctx.stroke();

      // Draw vehicle label
      ctx.fillStyle = '#2c3e50';
      ctx.font = '12px Arial';
      ctx.textAlign = 'center';
      ctx.fillText(vehicle.licensePlate, x, y - 15);
    });
    // Draw QR pins (existing + drafting)
    qrPins.forEach(pin => {
      const [px, py] = pin.position;
      if (!isFinite(px) || !isFinite(py)) return;
      const { x, y } = worldToCanvas(px, py);

      ctx.beginPath();
      ctx.arc(x, y, pin.isActive ? 9 : 7, 0, 2 * Math.PI);
      ctx.fillStyle = pin.isDraft ? '#0ea5e9' : '#6b7280';
      ctx.globalAlpha = pin.isActive ? 0.9 : 0.75;
      ctx.fill();
      ctx.globalAlpha = 1;
      ctx.strokeStyle = pin.isActive ? '#2563eb' : '#111827';
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
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;

    const worldX = ((x - 50) / (canvas.width - 100)) * 200 - 100;
    const worldZ = ((y - 50) / (canvas.height - 100)) * 100 - 50;
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
const MapInfoCard: React.FC<{ mapInfo: MapInfo }> = ({ mapInfo }) => {
  const { t } = useLanguage();

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
  const [previewVehicles, setPreviewVehicles] = useState<VehiclePosition[]>([]);
  // Danh sách QRCode ban đầu rỗng; chỉ hiển thị sau khi người dùng click "Load ZIP gần nhất"
  const [previewQRCodes, setPreviewQRCodes] = useState<QRCodeInfo[]>([]);
  // State quản lý các dòng đang chỉnh sửa
  const [editingRows, setEditingRows] = useState<EditingRow[]>([]);
  const [nextTempId, setNextTempId] = useState<number>(1);
  const didAutoLoadRef = useRef(false);
  const [isLoadingLastZip, setIsLoadingLastZip] = useState(false);
  const [isPickingPosition, setIsPickingPosition] = useState(false);
  const [pickingRowId, setPickingRowId] = useState<string | null>(null);
  const mapCardRef = useRef<HTMLDivElement>(null);
  const [pcdUrl, setPcdUrl] = useState<string | null>(null);
  const pcdObjectUrlRef = useRef<string | null>(null);
  const [mapInfo, setMapInfo] = useState<MapInfo | null>(null);
  // State quản lý các QR code đang được chỉnh sửa (từ previewQRCodes)
  const [editingExistingQRCodes, setEditingExistingQRCodes] = useState<Map<string, {
    id: string;
    code: string;
    position: [number, number];
    originalPosition: [number, number];
    errors?: {
      position?: string;
    };
  }>>(new Map());
  // State quản lý các QR code đã bị xóa (tạm thời)
  const [deletedQRCodeIds, setDeletedQRCodeIds] = useState<Set<string>>(new Set());

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
    setIsPickingPosition(false);
    setPickingRowId(null);
  };

  const startPicking = (id: string) => {
    setPickingRowId(id);
    setIsPickingPosition(true);
  };

  const handleMapPositionPick = (pos: [number, number]) => {
    if (!pickingRowId) return;
    
    // Kiểm tra xem pickingRowId có trong editingRows không
    const isEditingRow = editingRows.some(r => r.tempId === pickingRowId);
    if (isEditingRow) {
      handleEditRowChange(pickingRowId, 'position', pos);
      return;
    }
    
    // Kiểm tra xem pickingRowId có trong editingExistingQRCodes không
    if (editingExistingQRCodes.has(pickingRowId)) {
      handleUpdateQRPosition(pickingRowId, pos);
      return;
    }
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
        ? { ...qr, position: editedQR.position }
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
    // Sử dụng getMockVehiclePositions trực tiếp
    const vehicles: VehiclePosition[] = getMockVehiclePositions(t);
    setPreviewVehicles(vehicles);

    // Tạm thời dùng file PCD tĩnh trong public cho demo
    // Sau này có thể thay bằng PCD lấy từ ZIP hoặc backend
    setPcdUrl(DEFAULT_PCD_URL);

    // Lấy mock QR code riêng (không phụ thuộc vị trí phương tiện)
    const generatedQRCodes: QRCodeInfo[] = getMockQRCodes().map(qr => ({
      id: qr.id,
      code: qr.code,
      position: qr.position,
      isManual: qr.isManual ?? false,
    }));
    setPreviewQRCodes(generatedQRCodes);

    // Load map info
    const mapInfo = getMockMapInfo();
    setMapInfo(mapInfo);
  }, [t]);

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
    document.body.style.overflow = isPickingPosition ? 'hidden' : '';
    return () => {
      document.body.style.overflow = '';
    };
  }, [isPickingPosition]);

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
    
    // Thêm các QR code đã lưu (loại trừ các QR đã xóa)
    previewQRCodes
      .filter(qr => !deletedQRCodeIds.has(qr.id))
      .forEach(qr => {
        const [x, y] = qr.position;
        if (isNaN(x) || isNaN(y)) return;
        
        // Kiểm tra xem QR này có đang được chỉnh sửa không
        const editedQR = editingExistingQRCodes.get(qr.id);
        const finalPosition = editedQR ? editedQR.position : qr.position;
        const isBeingEdited = !!editedQR;
        
        pins.push({
          id: qr.id,
          position: finalPosition,
          isDraft: false,
          label: qr.code,
          isActive: isBeingEdited || pickingRowId === qr.id,
        });
      });

    // Thêm các dòng đang chỉnh sửa (editingRows) - giữ nguyên logic cũ
    editingRows.forEach(row => {
      const [x, y] = row.position;
      if (isNaN(x) || isNaN(y)) return;
      const normalizedIndex = row.codeIndex && /^\d+$/.test(row.codeIndex)
        ? `TM:${String(parseInt(row.codeIndex, 10)).padStart(3, '0')}`
        : undefined;
      pins.push({
        id: row.tempId,
        position: row.position,
        isDraft: true,
        isActive: row.tempId === pickingRowId,
        label: normalizedIndex,
      });
    });
    
    return pins;
  }, [previewQRCodes, editingRows, editingExistingQRCodes, deletedQRCodeIds, pickingRowId]);

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
          <button
            type="button"
            onClick={() => fileInputRef.current?.click()}
            className="inline-flex items-center px-3 py-2 text-white text-sm font-medium rounded-md shadow-sm transition-colors bg-blue-600 hover:bg-blue-700"
          >
            <svg className="w-5 h-5 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 12l8-8 8 8M12 4v16" />
            </svg>
            {t('upload_zip_title')}
          </button>
          {selectedZipName && (
            <span className="text-xs text-gray-600">{selectedZipName}</span>
          )}
        </div>
      </div>

      {/* Map Info Card */}
      {mapInfo && <MapInfoCard mapInfo={mapInfo} />}

      {/* Section 2 & 3: Preview PCD/Image + QR codes */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* PCD & Image Preview */}
        <div className="lg:col-span-2 space-y-6">
          {/* Image 2D Preview */}
          <div className="bg-white rounded-lg shadow-sm border border-gray-200 p-6">
            <h2 className="text-lg font-semibold text-gray-900 mb-3">
              {t('upload_image_preview_title')}
            </h2>
            {previewVehicles.length > 0 ? (
              <div
                ref={mapCardRef}
                className={`h-[70vh] min-h-[400px] border border-gray-300 rounded-lg overflow-hidden bg-gray-50 relative ${isPickingPosition ? 'ring-2 ring-blue-500 z-50' : ''}`}
              >
                <Image2DPreview 
                  vehicles={previewVehicles} 
                  qrPins={qrPins}
                  picking={isPickingPosition}
                  onPick={handleMapPositionPick}
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
            ) : (
              <div className="h-96 border border-dashed border-gray-300 rounded-lg flex items-center justify-center bg-gray-50">
                <span className="text-gray-500 text-sm">
                  {t('upload_image_preview_mock')}
                </span>
              </div>
            )}
          </div>

          {/* PCD Preview 3D */}
          <div className="bg-white rounded-lg shadow-sm border border-gray-200 p-6">
            <h2 className="text-lg font-semibold text-gray-900 mb-3">
              {t('upload_pcd_preview_title')}
            </h2>
            {previewVehicles.length > 0 ? (
              <div className="h-[70vh] min-h-[400px] border border-gray-300 rounded-lg overflow-hidden bg-gray-50">
                <Canvas
                  camera={{ position: [0, 10, 20], fov: 60 }}
                  style={{ height: '100%' }}
                >
                  <PCDPreview3D vehicles={previewVehicles} pcdUrl={pcdUrl} />
                </Canvas>
              </div>
            ) : (
              <div className="h-96 border border-dashed border-gray-300 rounded-lg flex items-center justify-center bg-gray-50">
                <span className="text-gray-500 text-sm">
                  {t('upload_pcd_preview_mock')}
                </span>
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
                                  onClick={() => startPicking(qr.id)}
                                  className={`px-3 py-1 text-xs rounded transition-colors ${isPickingPosition && pickingRowId === qr.id ? 'bg-blue-600 text-white hover:bg-blue-700' : 'bg-amber-500 text-white hover:bg-amber-600'}`}
                                >
                                  {isPickingPosition && pickingRowId === qr.id ? t('upload_qr_pick_active') : t('upload_qr_pick_position')}
                                </button>
                                {isPickingPosition && pickingRowId === qr.id && (
                                  <button
                                    onClick={stopPicking}
                                    className="px-3 py-1 text-xs rounded bg-gray-600 text-white hover:bg-gray-700 transition-colors"
                                  >
                                    {t('upload_qr_pick_done')}
                                  </button>
                                )}
                              </div>
                              {isPickingPosition && pickingRowId === qr.id && (
                                <p className="text-[11px] text-blue-600">{t('upload_qr_pick_hint')}</p>
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
                            onClick={() => startPicking(row.tempId)}
                            className={`px-3 py-1 text-xs rounded transition-colors ${isPickingPosition && pickingRowId === row.tempId ? 'bg-blue-600 text-white hover:bg-blue-700' : 'bg-amber-500 text-white hover:bg-amber-600'}`}
                          >
                            {isPickingPosition && pickingRowId === row.tempId ? t('upload_qr_pick_active') : t('upload_qr_pick_position')}
                          </button>
                          {isPickingPosition && pickingRowId === row.tempId && (
                            <button
                              onClick={stopPicking}
                              className="px-3 py-1 text-xs rounded bg-gray-600 text-white hover:bg-gray-700 transition-colors"
                            >
                              {t('upload_qr_pick_done')}
                            </button>
                          )}
                        </div>
                        {isPickingPosition && pickingRowId === row.tempId && (
                          <p className="text-[11px] text-blue-600">{t('upload_qr_pick_hint')}</p>
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
    </div>
  );
};

export default Upload;
