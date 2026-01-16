import React, { useState, useEffect } from 'react';
import { useNavigate, useParams } from 'react-router-dom';
import { useLanguage } from '../contexts/LanguageContext';
import { useVehicleService } from '../hooks/api/useVehicleService';
import type { VehicleFormData, ApiVehicle } from '../types/vehicle';

const EditVehicle: React.FC = () => {
  const navigate = useNavigate();
  const { id } = useParams<{ id: string }>();
  const { t } = useLanguage();
  const vehicleService = useVehicleService();
  
  const [formData, setFormData] = useState<VehicleFormData>({
    id: '',
    licensePlate: '',
    driver: '',
    vehicleType: '',
    mission: '',
    status: 'online'
  });

  const [errors, setErrors] = useState<{[key: string]: string}>({});
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [submitting, setSubmitting] = useState(false);

  const vehicleTypes = [
    t('tbm_tunnel'),
    t('transport_vehicle'),
    t('concrete_pump'),
    t('crane'),
    t('loader'),
    t('truck'),
    t('other_specialized')
  ];

  const missionTypes = [
    t('tunnel_line_1'),
    t('tunnel_line_2'),
    t('material_transport'),
    t('concrete_wall_pump'),
    t('soil_transport'),
    t('equipment_installation'),
    t('tunnel_maintenance'),
    t('geological_survey'),
    t('other_mission')
  ];

  // Fetch vehicle from API
  useEffect(() => {
    const fetchVehicle = async () => {
      if (!id) {
        setError('Vehicle ID is required');
        setLoading(false);
        return;
      }

      if (!vehicleService) {
        setError('Vehicle service not available');
        setLoading(false);
        return;
      }

      try {
        setLoading(true);
        setError(null);
        const apiVehicle: ApiVehicle = await vehicleService.getVehicleById(id);
        
        // Map ApiVehicle to Vehicle form format
        const metadata = (apiVehicle.metadata as any) || {};
        setFormData({
          id: apiVehicle.vehicle_id,
          licensePlate: metadata.licensePlate || apiVehicle.vehicle_id,
          driver: metadata.driver || apiVehicle.name || '',
          vehicleType: apiVehicle.vehicle_type || '',
          mission: metadata.mission || apiVehicle.description || '',
          status: apiVehicle.status || 'online'
        });
      } catch (err) {
        const errorMessage = err instanceof Error ? err.message : 'Failed to fetch vehicle';
        setError(errorMessage);
        console.error('Error fetching vehicle:', err);
        // Navigate back if vehicle not found
        if (err instanceof Error && err.message.includes('404')) {
          setTimeout(() => {
            alert(t('vehicle_not_found') || 'Vehicle not found');
            navigate('/vehicles');
          }, 1000);
        }
      } finally {
        setLoading(false);
      }
    };

    fetchVehicle();
  }, [id, navigate, t, vehicleService]);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
    
    // Clear error when user starts typing
    if (errors[name]) {
      setErrors(prev => ({
        ...prev,
        [name]: ''
      }));
    }
  };

  const validateForm = () => {
    const newErrors: {[key: string]: string} = {};

    if (!formData.licensePlate.trim()) {
      newErrors.licensePlate = t('license_plate_required');
    }

    if (!formData.driver.trim()) {
      newErrors.driver = t('driver_required');
    }

    if (!formData.vehicleType) {
      newErrors.vehicleType = t('vehicle_type_required');
    }

    if (!formData.mission.trim()) {
      newErrors.mission = t('mission_required');
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    if (!validateForm()) return;
    
    if (!vehicleService) {
      alert('Vehicle service not available');
      return;
    }

    setSubmitting(true);
    try {
      // Backend chưa có UPDATE endpoint cho vehicle info
      // Hiện tại chỉ có thể update status qua PATCH /vehicles/{id}/status
      // Các field khác (licensePlate, driver, mission) sẽ được lưu trong metadata
      // Tạm thời log và hiển thị thông báo
      
      // Update status - frontend và backend đều dùng online/offline
      await vehicleService.updateVehicleStatus(formData.id, formData.status as 'online' | 'offline');
      
      // TODO: Khi backend có endpoint UPDATE vehicle info, gọi API ở đây
      // await vehicleService.updateVehicle(formData.id, {
      //   name: formData.driver,
      //   description: formData.mission,
      //   vehicle_type: formData.vehicleType,
      //   metadata: {
      //     licensePlate: formData.licensePlate,
      //     driver: formData.driver,
      //     mission: formData.mission
      //   }
      // });
      
      console.log('Updated vehicle data:', formData);
      
      // Show success message and redirect
      alert(t('update_success') || 'Vehicle updated successfully');
      navigate('/vehicles');
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to update vehicle';
      alert(errorMessage);
      console.error('Error updating vehicle:', err);
    } finally {
      setSubmitting(false);
    }
  };

  const handleCancel = () => {
    navigate('/vehicles');
  };

  if (loading) {
    return (
      <div className="flex items-center justify-center h-64">
        <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-blue-600"></div>
      </div>
    );
  }

  if (error && !formData.id) {
    return (
      <div className="max-w-2xl mx-auto">
        <div className="bg-red-50 border border-red-200 text-red-700 px-4 py-3 rounded-lg mt-6">
          <p className="font-medium">Error loading vehicle</p>
          <p className="text-sm">{error}</p>
          <button
            onClick={() => navigate('/vehicles')}
            className="mt-4 px-4 py-2 bg-red-600 text-white rounded-lg hover:bg-red-700"
          >
            Back to Vehicles
          </button>
        </div>
      </div>
    );
  }

  return (
    <div className="max-w-2xl mx-auto">
      {/* Header */}
      <div className="mb-8">
        <div className="flex items-center space-x-4 mb-4">
          <button
            onClick={handleCancel}
            className="text-gray-600 hover:text-gray-900 p-2 rounded-lg hover:bg-gray-100"
          >
            <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M10 19l-7-7m0 0l7-7m-7 7h18" />
            </svg>
          </button>
          <h1 className="text-3xl font-bold text-gray-900">{t('edit_vehicle_title')}</h1>
        </div>
        <p className="text-gray-600">{t('update_vehicle_info')}</p>
        {error && (
          <div className="mt-4 bg-yellow-50 border border-yellow-200 text-yellow-700 px-4 py-3 rounded-lg">
            <p className="text-sm">{error}</p>
            <p className="text-xs mt-1">Note: Backend currently only supports status updates. Other fields will be saved when backend API is available.</p>
          </div>
        )}
      </div>

      {/* Form */}
      <div className="bg-white rounded-lg shadow-sm border border-gray-200 p-6">
        <form onSubmit={handleSubmit} className="space-y-6">
          {/* Biển số máy */}
          <div>
            <label htmlFor="licensePlate" className="block text-sm font-medium text-gray-700 mb-2">
              {t('license_plate')} <span className="text-red-500">*</span>
            </label>
            <input
              type="text"
              id="licensePlate"
              name="licensePlate"
              value={formData.licensePlate}
              onChange={handleChange}
              className={`w-full px-3 py-2 border rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500 ${
                errors.licensePlate ? 'border-red-500' : 'border-gray-300'
              }`}
              placeholder="VD: 30A-12345"
            />
            {errors.licensePlate && (
              <p className="mt-1 text-sm text-red-600">{errors.licensePlate}</p>
            )}
          </div>

          {/* Tài xế */}
          <div>
            <label htmlFor="driver" className="block text-sm font-medium text-gray-700 mb-2">
              {t('driver')} <span className="text-red-500">*</span>
            </label>
            <input
              type="text"
              id="driver"
              name="driver"
              value={formData.driver}
              onChange={handleChange}
              className={`w-full px-3 py-2 border rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500 ${
                errors.driver ? 'border-red-500' : 'border-gray-300'
              }`}
              placeholder={t('driver_placeholder')}
            />
            {errors.driver && (
              <p className="mt-1 text-sm text-red-600">{errors.driver}</p>
            )}
          </div>

          {/* Loại xe */}
          <div>
            <label htmlFor="vehicleType" className="block text-sm font-medium text-gray-700 mb-2">
              {t('vehicle_type')} <span className="text-red-500">*</span>
            </label>
            <select
              id="vehicleType"
              name="vehicleType"
              value={formData.vehicleType}
              onChange={handleChange}
              className={`w-full px-3 py-2 border rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500 ${
                errors.vehicleType ? 'border-red-500' : 'border-gray-300'
              }`}
            >
              <option value="">{t('select_vehicle_type')}</option>
              {vehicleTypes.map((type) => (
                <option key={type} value={type}>
                  {type}
                </option>
              ))}
            </select>
            {errors.vehicleType && (
              <p className="mt-1 text-sm text-red-600">{errors.vehicleType}</p>
            )}
          </div>

          {/* Nhiệm vụ */}
          <div>
            <label htmlFor="mission" className="block text-sm font-medium text-gray-700 mb-2">
              {t('mission')} <span className="text-red-500">*</span>
            </label>
            <select
              id="mission"
              name="mission"
              value={formData.mission}
              onChange={handleChange}
              className={`w-full px-3 py-2 border rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500 ${
                errors.mission ? 'border-red-500' : 'border-gray-300'
              }`}
            >
              <option value="">{t('select_mission')}</option>
              {missionTypes.map((mission) => (
                <option key={mission} value={mission}>
                  {mission}
                </option>
              ))}
            </select>
            {errors.mission && (
              <p className="mt-1 text-sm text-red-600">{errors.mission}</p>
            )}
          </div>

          {/* Trạng thái */}
          <div>
            <label htmlFor="status" className="block text-sm font-medium text-gray-700 mb-2">
              {t('status')}
            </label>
            <select
              id="status"
              name="status"
              value={formData.status}
              onChange={handleChange}
              className="w-full px-3 py-2 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500"
            >
              <option value="online">{t('online') || t('active') || 'Online'}</option>
              <option value="offline">{t('offline') || 'Offline'}</option>
            </select>
          </div>

          {/* Form Actions */}
          <div className="flex items-center justify-end space-x-4 pt-6 border-t border-gray-200">
            <button
              type="button"
              onClick={handleCancel}
              className="px-6 py-2 border border-gray-300 rounded-lg text-gray-700 hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-gray-500"
            >
              {t('cancel')}
            </button>
            <button
              type="submit"
              disabled={submitting}
              className={`px-6 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500 ${
                submitting ? 'opacity-50 cursor-not-allowed' : ''
              }`}
            >
              {submitting ? t('updating') || 'Updating...' : t('update_button')}
            </button>
          </div>
        </form>
      </div>
    </div>
  );
};

export default EditVehicle; 