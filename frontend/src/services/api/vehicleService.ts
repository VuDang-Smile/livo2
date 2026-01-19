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
import {
  validateVehiclesListResponse,
  validateVehicleApi,
  validateAndNormalizeVehicles,
  getSafeVehicleStatus,
  getSafePosition,
  getSafeOrientation,
  createDefaultPosition,
  createDefaultOrientation,
} from '../../utils/validationUtils';

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
    
    try {
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

      const data = await response.json();
      
      // Validate response is not null/undefined
      if (data === null || data === undefined) {
        throw new Error('API returned null or undefined response');
      }

      return data as T;
    } catch (error) {
      console.error(`[VehicleService] Request failed for ${url}:`, error);
      throw error;
    }
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
    const response = await this.makeRequest<unknown>(endpoint);

    // Validate response structure
    if (!validateVehiclesListResponse(response)) {
      console.warn('[VehicleService] Invalid response structure, returning empty array');
      return [];
    }

    const validatedResponse = response as VehiclesListApiResponse;

    // Validate and normalize vehicles array
    const rawVehicles = validatedResponse.vehicles ?? [];
    let vehicles: ApiVehicle[] = validateAndNormalizeVehicles(rawVehicles);

    // Transform to ApiVehicle format with safe property access
    vehicles = vehicles.map((vehicle: ApiVehicle) => {
      const status = getSafeVehicleStatus(vehicle.status);

      // Safely extract latest_pose with validation
      let latest_pose: VehicleApi['latest_pose'] = undefined;
      if (vehicle.latest_pose) {
        const position = getSafePosition(vehicle.latest_pose.position);
        const orientation = getSafeOrientation(vehicle.latest_pose.orientation);
        const timestamp = vehicle.latest_pose.timestamp;

        latest_pose = {
          position,
          orientation,
          timestamp: timestamp ?? new Date().toISOString(),
        };
      }

      return {
        vehicle_id: vehicle.vehicle_id ?? '',
        name: vehicle.name ?? undefined,
        description: vehicle.description ?? undefined,
        vehicle_type: vehicle.vehicle_type ?? undefined,
        vehicle_category: vehicle.vehicle_category ?? undefined,
        status,
        metadata: vehicle.metadata ?? undefined,
        latest_pose,
        created_at: vehicle.created_at ?? undefined,
        updated_at: vehicle.updated_at ?? vehicle.latest_pose?.timestamp ?? undefined,
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
    if (!vehicleId || typeof vehicleId !== 'string') {
      throw new Error('Invalid vehicle ID provided');
    }

    const vehicle = await this.makeRequest<unknown>(`/api/v1/vehicles/${vehicleId}`);

    // Validate vehicle structure
    if (!validateVehicleApi(vehicle)) {
      throw new Error(`Invalid vehicle data received for ID: ${vehicleId}`);
    }

    const validatedVehicle = vehicle as VehicleApi;
    const status = getSafeVehicleStatus(validatedVehicle.status);

    // Safely extract latest_pose with validation
    let latest_pose: VehicleApi['latest_pose'] = undefined;
    if (validatedVehicle.latest_pose) {
      const position = getSafePosition(validatedVehicle.latest_pose.position);
      const orientation = getSafeOrientation(validatedVehicle.latest_pose.orientation);
      const timestamp = validatedVehicle.latest_pose.timestamp;

      latest_pose = {
        position,
        orientation,
        timestamp: timestamp ?? new Date().toISOString(),
      };
    }

    return {
      vehicle_id: validatedVehicle.vehicle_id ?? vehicleId,
      name: validatedVehicle.name ?? undefined,
      description: validatedVehicle.description ?? undefined,
      vehicle_type: validatedVehicle.vehicle_type ?? undefined,
      vehicle_category: validatedVehicle.vehicle_category ?? undefined,
      status,
      metadata: validatedVehicle.metadata ?? undefined,
      latest_pose,
      created_at: validatedVehicle.created_at ?? undefined,
      updated_at: validatedVehicle.updated_at ?? validatedVehicle.latest_pose?.timestamp ?? undefined,
    };
  }

  /**
   * Cập nhật vehicle status
   */
  async updateVehicleStatus(vehicleId: string, status: 'online' | 'offline'): Promise<void> {
    if (!vehicleId || typeof vehicleId !== 'string') {
      throw new Error('Invalid vehicle ID provided');
    }

    if (status !== 'online' && status !== 'offline') {
      throw new Error('Invalid status: must be "online" or "offline"');
    }

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
