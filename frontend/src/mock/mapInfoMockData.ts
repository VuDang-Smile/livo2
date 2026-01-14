/**
 * @deprecated Use types from '../../types/mapInfo' instead
 * This file is kept for backward compatibility only
 */
export { MapInfo } from '../types/mapInfo';

/**
 * Mock data generator for MapInfo
 * @deprecated This is only used for testing/mocking purposes
 */
export const getMockMapInfo = (): MapInfo => ({
  id: 'map-001',
  name: 'Tunnel Mining Project A',
  createdAt: '2024-01-10 08:30:00',
  uploadedBy: 'Scanner 004',
  uploadedAt: '2024-01-15 14:25:00',
  fileSize: 15728640,
});

