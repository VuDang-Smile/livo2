export interface MockQRCode {
  id: string;
  code: string;
  position: [number, number]; // [x, y] on 2D map (x, z from 3D)
  isManual?: boolean;
}

// Mock QR codes with positions that do NOT overlap existing mock vehicles
// Vehicle positions (x, z): [0,0], [50,20], [-30,-15], [80,-40]
// QR positions chosen to avoid the above coordinates.
export const getMockQRCodes = (): MockQRCode[] => [
  {
    id: 'qr-m1',
    code: 'TM:100',
    position: [-90, -10],
    isManual: false,
  },
  {
    id: 'qr-m2',
    code: 'TM:101',
    position: [25, 45],
    isManual: false,
  },
  {
    id: 'qr-m3',
    code: 'TM:102',
    position: [70, -5],
    isManual: false,
  },
];

