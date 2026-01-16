/**
 * Vehicle Service
 *
 * Chức năng: Quản lý các API calls liên quan đến vehicles
 * - Lấy danh sách vehicles
 * - Lấy thông tin vehicle theo ID
 * - Cập nhật vehicle status / info
 *
 * Sử dụng: Trong các hook và component cần tương tác với vehicles
 * API Base: /api/v1/vehicles
 */

import { ApiVehicle, VehicleApi, VehiclesListApiResponse } from '../../types/vehicle';

export interface FetchVehiclesParams {
  type?: 'scanner' | 'worker';
  status?: 'online' | 'offline';
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
   * Lấy danh sách vehicles với optional filters.
   * Backend trả về VehiclesListResponse: { vehicles: VehicleListItem[], total: number }.
   *
   * Lưu ý: Backend hiện tại không hỗ trợ query parameters để filter.
   * Các query params sẽ bị ignore và API trả về tất cả vehicles.
   * Filter sẽ được thực hiện ở client side sau khi nhận dữ liệu.
   */
  async getVehicles(params?: FetchVehiclesParams): Promise<ApiVehicle[]> {
    // Backend không hỗ trợ query params, nhưng giữ lại để tương lai có thể thêm
    // Sử dụng đúng tên parameter theo backend model: vehicle_type thay vì type_category
    const queryParams = new URLSearchParams();
    if (params?.type) queryParams.append('vehicle_type', params.type);
    if (params?.status) queryParams.append('status', params.status);
    
    const query = queryParams.toString();
    const endpoint = `/api/v1/vehicles${query ? `/?${query}` : ''}`;

    // Backend trả về VehiclesListResponse format
    const response = await this.makeRequest<VehiclesListApiResponse>(endpoint);

    // Transform backend response to ApiVehicle (VehicleApi) format
    let vehicles: ApiVehicle[] = response.vehicles.map((vehicle: VehicleApi) => {
      const status: 'online' | 'offline' =
        vehicle.status?.toLowerCase() === 'online' ? 'online' : 'offline';

      const latest_pose = vehicle.latest_pose
        ? {
            position: vehicle.latest_pose.position,
            orientation: vehicle.latest_pose.orientation,
            timestamp: vehicle.latest_pose.timestamp,
          }
        : undefined;

      return {
        vehicle_id: vehicle.vehicle_id,
        name: vehicle.name,
        description: vehicle.description,
        vehicle_type: vehicle.vehicle_type,
        vehicle_category: vehicle.vehicle_category,
        status,
        metadata: vehicle.metadata,
        latest_pose,
        created_at: vehicle.created_at,
        updated_at: vehicle.updated_at ?? vehicle.latest_pose?.timestamp,
      };
    });

    // Client-side filtering vì backend không hỗ trợ query params
    if (params?.type) {
      vehicles = vehicles.filter((v: ApiVehicle) => 
        (v.vehicle_type || '').toLowerCase() === params.type?.toLowerCase()
      );
    }
    if (params?.status) {
      vehicles = vehicles.filter((v: ApiVehicle) => 
        (v.status || '').toLowerCase() === params.status?.toLowerCase()
      );
    }

    return vehicles;
  }

  /**
   * Lấy thông tin vehicle theo ID
   * Backend trả về VehicleResponse format
   */
  async getVehicleById(vehicleId: string): Promise<ApiVehicle> {
    const vehicle = await this.makeRequest<VehicleApi>(`/api/v1/vehicles/${vehicleId}`);

    const status: 'online' | 'offline' =
      vehicle.status?.toLowerCase() === 'online' ? 'online' : 'offline';

    const latest_pose = vehicle.latest_pose
      ? {
          position: vehicle.latest_pose.position,
          orientation: vehicle.latest_pose.orientation,
          timestamp: vehicle.latest_pose.timestamp,
        }
      : undefined;

    return {
      vehicle_id: vehicle.vehicle_id,
      name: vehicle.name,
      description: vehicle.description,
      vehicle_type: vehicle.vehicle_type,
      vehicle_category: vehicle.vehicle_category,
      status,
      metadata: vehicle.metadata ?? {},
      latest_pose,
      created_at: vehicle.created_at,
      updated_at: vehicle.updated_at ?? vehicle.latest_pose?.timestamp,
    };
  }

  /**
   * Cập nhật vehicle status
   */
  async updateVehicleStatus(vehicleId: string, status: 'online' | 'offline'): Promise<void> {
    await this.makeRequest<void>(`/api/v1/vehicles/${vehicleId}/status`, {
      method: 'PATCH',
      body: JSON.stringify({ status }),
    });
  }

  /**
   * Lấy danh sách online scanners
   */
  async getOnlineScanners(): Promise<ApiVehicle[]> {
    return this.getVehicles({ type: 'scanner', status: 'online' });
  }
}
