import React, { useRef, useEffect } from 'react';
import * as THREE from 'three';
import { extend } from '@react-three/fiber';

// Extend Three.js với ShaderMaterial để có thể dùng như JSX element
extend({ ShaderMaterial: THREE.ShaderMaterial });

declare global {
  namespace JSX {
    interface IntrinsicElements {
      shaderMaterial: React.DetailedHTMLProps<React.HTMLAttributes<THREE.ShaderMaterial>, THREE.ShaderMaterial> & {
        vertexShader: string;
        fragmentShader: string;
        uniforms: { [key: string]: { value: any } };
        vertexColors?: boolean;
        transparent?: boolean;
      };
    }
  }
}

interface PCDClippingMaterialProps {
  clipXMin?: number;
  clipXMax?: number;
  clipYMin?: number;
  clipYMax?: number;
  clipZMin?: number;
  clipZMax?: number;
  pointSize?: number;
  sizeAttenuation?: boolean;
}

// Giá trị float lớn để thay thế Infinity (an toàn hơn cho GLSL)
const LARGE_FLOAT = 1e30;
const NEGATIVE_LARGE_FLOAT = -1e30;

/**
 * Custom shader material cho point cloud với clipping support
 * Sử dụng GPU để filter points real-time
 * 
 * Lưu ý: Three.js tự động inject attribute 'color' khi vertexColors=true
 * nên không cần khai báo attribute color trong shader
 */
export const PCDClippingMaterial: React.FC<PCDClippingMaterialProps> = ({
  clipXMin,
  clipXMax,
  clipYMin,
  clipYMax,
  clipZMin,
  clipZMax,
  pointSize = 0.05,
  sizeAttenuation = true,
}) => {
  const materialRef = useRef<THREE.ShaderMaterial>(null);

  // Vertex shader: Kiểm tra bounds và discard points ngoài bounds
  // Three.js tự động inject: position, color (khi vertexColors=true), modelViewMatrix, projectionMatrix
  const vertexShader = `
    // Three.js tự động inject color attribute khi vertexColors=true
    // Không cần khai báo attribute color ở đây
    
    uniform float pointSize;
    uniform float sizeAttenuation; // 0.0 = false, 1.0 = true (tránh dùng bool trong if)
    uniform float clipXMin;
    uniform float clipXMax;
    uniform float clipYMin;
    uniform float clipYMax;
    uniform float clipZMin;
    uniform float clipZMax;
    
    varying vec3 vColor;
    varying float vDiscard;
    
    void main() {
      // position vẫn ở local space (trước khi áp dụng rotation của group)
      vec3 posForClipping = position;
      
      // Tính toán model-view position trước (cần cho cả discard và size calculation)
      vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
      
      // Kiểm tra clipping bounds trên hệ tọa độ gốc
      // Sử dụng logic: nếu clipMin > clipMax (giá trị mặc định rất lớn) thì không clip
      bool shouldDiscard = false;
      
      // Chỉ kiểm tra clip nếu giá trị hợp lệ (không phải giá trị mặc định rất lớn)
      if (clipXMin < clipXMax) {
        if (posForClipping.x < clipXMin || posForClipping.x > clipXMax) {
          shouldDiscard = true;
        }
      }
      if (clipYMin < clipYMax) {
        if (posForClipping.y < clipYMin || posForClipping.y > clipYMax) {
          shouldDiscard = true;
        }
      }
      if (clipZMin < clipZMax) {
        if (posForClipping.z < clipZMin || posForClipping.z > clipZMax) {
          shouldDiscard = true;
        }
      }
      
      // Nếu ngoài bounds, di chuyển point ra ngoài view frustum
      if (shouldDiscard) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0); // Đẩy ra ngoài view
        gl_PointSize = 0.0;
        vDiscard = 1.0;
      } else {
        gl_Position = projectionMatrix * mvPosition;
        vDiscard = 0.0;
        
        // Point size calculation
        if (sizeAttenuation > 0.5) {
          // Size attenuation: point size giảm theo khoảng cách
          gl_PointSize = pointSize * (300.0 / max(1.0, -mvPosition.z));
        } else {
          // Fixed size
          gl_PointSize = pointSize;
        }
      }
      
      // Pass color to fragment shader
      // Three.js tự động inject color attribute, chỉ cần assign
      vColor = color;
    }
  `;

  // Fragment shader: Render point với màu sắc
  const fragmentShader = `
    varying vec3 vColor;
    varying float vDiscard;
    
    void main() {
      // Discard nếu point ngoài bounds
      if (vDiscard > 0.5) {
        discard;
      }
      
      // Tính toán distance từ center của point để tạo hình tròn
      vec2 coord = gl_PointCoord - vec2(0.5);
      float dist = length(coord);
      
      // Tạo hình tròn mềm mại
      if (dist > 0.5) {
        discard;
      }
      
      // Áp dụng màu sắc
      gl_FragColor = vec4(vColor, 1.0);
    }
  `;

  // Giữ object uniforms ổn định để r3f/three không tạo material mới mỗi render
  const uniformsRef = useRef({
    pointSize: { value: pointSize },
    sizeAttenuation: { value: sizeAttenuation ? 1.0 : 0.0 },
    clipXMin: { value: clipXMin ?? NEGATIVE_LARGE_FLOAT },
    clipXMax: { value: clipXMax ?? LARGE_FLOAT },
    clipYMin: { value: clipYMin ?? NEGATIVE_LARGE_FLOAT },
    clipYMax: { value: clipYMax ?? LARGE_FLOAT },
    clipZMin: { value: clipZMin ?? NEGATIVE_LARGE_FLOAT },
    clipZMax: { value: clipZMax ?? LARGE_FLOAT },
  });

  // Update uniforms khi props thay đổi
  useEffect(() => {
    if (materialRef.current) {
      uniformsRef.current.pointSize.value = pointSize;
      uniformsRef.current.sizeAttenuation.value = sizeAttenuation ? 1.0 : 0.0;
      uniformsRef.current.clipXMin.value = clipXMin ?? NEGATIVE_LARGE_FLOAT;
      uniformsRef.current.clipXMax.value = clipXMax ?? LARGE_FLOAT;
      uniformsRef.current.clipYMin.value = clipYMin ?? NEGATIVE_LARGE_FLOAT;
      uniformsRef.current.clipYMax.value = clipYMax ?? LARGE_FLOAT;
      uniformsRef.current.clipZMin.value = clipZMin ?? NEGATIVE_LARGE_FLOAT;
      uniformsRef.current.clipZMax.value = clipZMax ?? LARGE_FLOAT;

      // Báo cho three.js cập nhật uniforms ngay frame tiếp theo
      materialRef.current.needsUpdate = true;
    }
  }, [pointSize, sizeAttenuation, clipXMin, clipXMax, clipYMin, clipYMax, clipZMin, clipZMax]);

  return (
    <shaderMaterial
      ref={materialRef}
      vertexShader={vertexShader}
      fragmentShader={fragmentShader}
      uniforms={uniformsRef.current}
      vertexColors={true}
      transparent={false}
    />
  );
};

