


import React, { useRef } from 'react';
import { TransformControls } from '@react-three/drei';
import * as THREE from 'three';
import { EditableMeshProps } from '../types/Mesh';

export const EditableMesh: React.FC<EditableMeshProps> = ({ 
  meshData, 
  isSelected, 
  onSelect, 
  onChange 
}) => {
  // Ref này giúp TransformControls biết chính xác đối tượng nào cần điều khiển
  const meshRef = useRef<THREE.Mesh>(null!);

  return (
    <>
      <mesh 
        ref={meshRef}
        position={meshData.position} 
        onClick={(e) => {
          e.stopPropagation(); // Ngăn sự kiện click xuyên qua các vật thể phía sau
          onSelect(meshData.id);
        }}
      >
        {meshData.type === 'box' && <boxGeometry args={[2, 2, 2]} />}
        {meshData.type === 'sphere' && <sphereGeometry args={[1.2, 32, 32]} />}
        {meshData.type === 'cylinder' && <cylinderGeometry args={[1, 1, 2, 32]} />}
        
        <meshStandardMaterial 
          color={meshData.color} 
          emissive={isSelected ? meshData.color : "black"} 
          emissiveIntensity={0.2}
        />
      </mesh>

      {isSelected && (
        <TransformControls 
          object={meshRef.current} // Liên kết trực tiếp với mesh qua ref
          mode="translate"
          onMouseUp={() => {
            if (meshRef.current) {
              const { x, y, z } = meshRef.current.position;
              onChange(meshData.id, [x, y, z]);
            }
          }}
        />
      )}
    </>
  );
};