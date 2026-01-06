export interface MapInfo {
  id: string;
  name: string;           // Tên map
  createdAt: string;      // Ngày tạo (format: YYYY-MM-DD HH:mm:ss)
  uploadedBy: string;     // Người tải lên
  uploadedAt: string;     // Ngày tải lên (format: YYYY-MM-DD HH:mm:ss)
  fileSize: number;       // Dung lượng (bytes)
}

export const getMockMapInfo = (): MapInfo => ({
  id: 'map-001',
  name: 'Tunnel Mining Project A',
  createdAt: '2024-01-10 08:30:00',
  uploadedBy: 'Scanner 004',
  uploadedAt: '2024-01-15 14:25:00',
  fileSize: 15728640,
});

