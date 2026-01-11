/**
 * Vehicle Service
 * 
 * Chức năng: Quản lý các API calls liên quan đến vehicles
 * - Lấy danh sách vehicles (có thể filter theo type và status)
 * - Xóa vehicle
 * - Lấy thông tin vehicle theo ID
 * 
 * Sử dụng: Trong các hook và component cần tương tác với vehicles
 * API Base: /api/v1/vehicles
 */

import { ApiVehicle } from '../../types/vehicle';

export interface FetchVehiclesParams {
  type?: 'scanner' | 'worker';
  status?: 'active' | 'offline' | 'inactive';
}

export class VehicleService {
  constructor(private apiBaseUrl: string) {
    // Nhận API base URL qua dependency injection
    console.log(`[VehicleService] Initialized with API base URL: ${this.apiBaseUrl}`);
  }

  private async makeRequest<T>(endpoint: string, options: RequestInit = {}): Promise<T> {
    const url = `${this.apiBaseUrl}${endpoint}`;
    console.log(`[VehicleService] Making request to: ${url}`);
    const response = await fetch(url, {
      ...options,
      headers: {
        'Content-Type': 'application/json',
        ...(options.headers || {}),
      },
    });

    if (!response.ok) {
      const text = await response.text();
      throw new Error(text || `HTTP error! status: ${response.status}`);
    }

    return response.json();
  }

  /**
   * Lấy danh sách vehicles với optional filters
   */
  async getVehicles(params?: FetchVehiclesParams): Promise<ApiVehicle[]> {
    const queryParams = new URLSearchParams();
    if (params?.type) queryParams.append('type_category', params.type);
    if (params?.status) queryParams.append('vehicle_status', params.status);
    
    const query = queryParams.toString();
    const endpoint = `/api/v1/vehicles${query ? `/?${query}` : ''}`;
    
    const vehicles = await this.makeRequest<any[]>(endpoint);
    
    // Transform backend response to ApiVehicle format
    return vehicles.map((vehicle: any) => ({
      ...vehicle,
      type: vehicle.type_category || vehicle.type, // Map type_category to type
      updated_at: vehicle.updated_at || vehicle.current_pose?.timestamp || vehicle.current_position?.timestamp
    }));
  }

  /**
   * Lấy thông tin vehicle theo ID
   */
  async getVehicleById(vehicleId: string): Promise<ApiVehicle> {
    return this.makeRequest<ApiVehicle>(`/api/v1/vehicles/${vehicleId}`);
  }

  /**
   * Xóa vehicle
   */
  async deleteVehicle(vehicleId: string): Promise<void> {
    await this.makeRequest<void>(`/api/v1/vehicles/${vehicleId}`, {
      method: 'DELETE',
    });
  }

  /**
   * Lấy danh sách active scanners
   */
  async getActiveScanners(): Promise<ApiVehicle[]> {
    return this.makeRequest<ApiVehicle[]>('/api/v1/vehicles/?vehicle_type=scanner&vehicle_status=active');
  }
}
