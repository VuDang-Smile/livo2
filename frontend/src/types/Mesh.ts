export interface CustomMesh {
  id: string;
  type: 'box' | 'sphere' | 'cylinder';
  position: [number, number, number];
  color: string;
}

export interface EditableMeshProps {
  meshData: CustomMesh;
  isSelected: boolean;
  onSelect: (id: string | null) => void;
  onChange: (id: string, newPos: [number, number, number]) => void;
}