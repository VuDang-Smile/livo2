import React, { useState, useEffect } from 'react';
import { useLoader } from '@react-three/fiber';
import * as THREE from 'three';
import { PCDLoader } from 'three/examples/jsm/loaders/PCDLoader';
import { PCDClippingMaterial } from './PCDClippingMaterial';

type Vec3 = [number, number, number];

export interface PointCloudBounds {
  minX: number;
  maxX: number;
  minY: number;
  maxY: number;
  minZ: number;
  maxZ: number;
}

export interface PCDMapProps {
  url: string;
  /** Scale của point cloud, mặc định 1 */
  scale?: number | Vec3;
  /** Rotation Euler (rad) XYZ, mặc định [0, 0, 0] */
  rotation?: Vec3;
  /** Vị trí origin của point cloud trong scene */
  position?: Vec3;
  /** Clipping bounds - giá trị thực (không phải phần trăm) */
  clipXMin?: number;
  clipXMax?: number;
  clipYMin?: number;
  clipYMax?: number;
  clipZMin?: number;
  clipZMax?: number;
  /** Callback khi bounds được tính toán */
  onBoundsCalculated?: (bounds: PointCloudBounds) => void;
}

/**
 * Component hiển thị point cloud PCD dưới dạng primitive THREE.Points.
 * Hỗ trợ shader-based clipping cho real-time filtering.
 * Dùng chung cho VehicleMap và Upload Preview.
 */
export const PCDMap: React.FC<PCDMapProps> = ({
  url,
  scale = 1,
  rotation = [0, 0, 0],
  position = [0, 0, 0],
  clipXMin,
  clipXMax,
  clipYMin,
  clipYMax,
  clipZMin,
  clipZMax,
  onBoundsCalculated,
}) => {
  const [geometry, setGeometry] = useState<THREE.BufferGeometry | null>(null);

  // Load PCD file
  const pcdData = useLoader(PCDLoader as unknown as THREE.Loader, url) as unknown as THREE.Points;

  // Parse PCD và tính bounds
  useEffect(() => {
    if (!pcdData || !pcdData.geometry) return;

    const geom = pcdData.geometry as THREE.BufferGeometry;
    const positionsAttr = geom.getAttribute('position') as THREE.BufferAttribute;
    const colorsAttr = geom.getAttribute('color') as THREE.BufferAttribute;

    if (!positionsAttr) return;

    const positionsArray = positionsAttr.array as Float32Array;

    // Tính bounds
    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;

    for (let i = 0; i < positionsArray.length; i += 3) {
      const x = positionsArray[i];
      const y = positionsArray[i + 1];
      const z = positionsArray[i + 2];

      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
      if (z < minZ) minZ = z;
      if (z > maxZ) maxZ = z;
    }

    const calculatedBounds: PointCloudBounds = {
      minX,
      maxX,
      minY,
      maxY,
      minZ,
      maxZ,
    };

    // Callback về parent
    if (onBoundsCalculated) {
      onBoundsCalculated(calculatedBounds);
    }

    // Tạo geometry mới với positions và colors
    const newGeometry = new THREE.BufferGeometry();
    newGeometry.setAttribute('position', positionsAttr);

    // Xử lý colors
    if (colorsAttr && colorsAttr.array) {
      newGeometry.setAttribute('color', colorsAttr);
    } else {
      // Fallback: tạo màu từ Z (height-based coloring)
      const colorArray = new Float32Array(positionsArray.length);
      const zRange = maxZ - minZ || 1;

      for (let i = 0; i < positionsArray.length; i += 3) {
        const z = positionsArray[i + 2];
        const normalized = (z - minZ) / zRange;
        colorArray[i] = Math.min(1, Math.max(0, normalized)); // R
        colorArray[i + 1] = Math.min(1, Math.max(0, 1 - normalized)); // G
        colorArray[i + 2] = 0.5; // B
      }

      newGeometry.setAttribute('color', new THREE.Float32BufferAttribute(colorArray, 3));
    }

    setGeometry(newGeometry);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [pcdData]); // Chỉ phụ thuộc vào pcdData, không phụ thuộc vào onBoundsCalculated để tránh re-run không cần thiết

  // Clipping props nhận giá trị thực (đã được chuyển đổi từ phần trăm ở parent)
  // Nếu undefined, không áp dụng clipping cho trục đó

  if (!geometry) {
    return null;
  }

  return (
    <group
      position={position}
      rotation={new THREE.Euler(rotation[0], rotation[1], rotation[2])}
      scale={scale as any}
    >
      <points geometry={geometry}>
        <PCDClippingMaterial
          clipXMin={clipXMin}
          clipXMax={clipXMax}
          clipYMin={clipYMin}
          clipYMax={clipYMax}
          clipZMin={clipZMin}
          clipZMax={clipZMax}
          pointSize={0.05}
          sizeAttenuation={true}
        />
      </points>
    </group>
  );
};

export default PCDMap;


