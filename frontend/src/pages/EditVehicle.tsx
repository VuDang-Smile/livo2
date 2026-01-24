import React, { useState, useEffect } from 'react';
import { useNavigate, useParams } from 'react-router-dom';
import { useLanguage } from '../contexts/LanguageContext';
import { useVehicleService } from '../hooks/api/useVehicleService';
import { getBackendUrl } from '../constants/mapConfig';
import type { VehicleFormData, VehicleApi, VehicleType, MissionType, VehicleCategory } from '../types/vehicle';

const EditVehicle: React.FC = () => {
  const navigate = useNavigate();
  const { id } = useParams<{ id: string }>();
  const { t } = useLanguage();
  const vehicleService = useVehicleService();
  
  const [formData, setFormData] = useState<VehicleFormData>({
    id: '',
    driver: '',
    vehicleType: '',
    vehicleCategory: undefined,
    mission: '',
    status: 'online'
  });

  const [errors, setErrors] = useState<{[key: string]: string}>({});
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [submitting, setSubmitting] = useState(false);
  const [availableCategories, setAvailableCategories] = useState<VehicleCategory[]>([]);
  const [categoryValueMap, setCategoryValueMap] = useState<Record<string, VehicleCategory>>({});

  // Vehicle Type (Role) - chỉ có 2 loại: scanner và worker
  const vehicleTypes: { value: VehicleType; label: string }[] = [
    { value: 'scanner', label: t('type.scanner') },
    { value: 'worker', label: t('type.worker') }
  ];

  const missionTypes: { value: MissionType; label: string }[] = [
    { value: 'tunnel_line_1', label: t('mission.tunnel_line_1') },
    { value: 'tunnel_line_2', label: t('mission.tunnel_line_2') },
    { value: 'material_transport', label: t('mission.material_transport') },
    { value: 'concrete_wall_pump', label: t('mission.concrete_wall_pump') },
    { value: 'soil_transport', label: t('mission.soil_transport') },
    { value: 'equipment_installation', label: t('mission.equipment_installation') },
    { value: 'tunnel_maintenance', label: t('mission.tunnel_maintenance') },
    { value: 'geological_survey', label: t('mission.geological_survey') },
    { value: 'other_mission', label: t('mission.other_mission') }
  ];

  // Fetch vehicle categories from backend API
  useEffect(() => {
    const fetchCategories = async () => {
      try {
        const backendUrl = getBackendUrl();
        const response = await fetch(`${backendUrl}/api/v1/vehicles/categories`, {
          headers: {
            'Content-Type': 'application/json',
          },
        });
        
        if (response.ok) {
          const data = await response.json();
          const categories = data.categories || [];
          setAvailableCategories(categories);
          
          // Build mapping from translated label to category value
          const map: Record<string, VehicleCategory> = {};
          categories.forEach((cat: VehicleCategory) => {
            const translatedLabel = t(`category.${cat}`);
            map[translatedLabel] = cat;
          });
          setCategoryValueMap(map);
        }
      } catch (err) {
        console.error('Error fetching vehicle categories:', err);
      }
    };

    fetchCategories();
  }, [t]);

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
        const apiVehicle: VehicleApi = await vehicleService.getVehicleById(id);
        
        // Map ApiVehicle to Vehicle form format
        const metadata = (apiVehicle.metadata as any) || {};
        setFormData({
          id: apiVehicle.vehicle_id,
          driver: metadata.driver || apiVehicle.name || '',
          vehicleType: apiVehicle.vehicle_type || '',
          vehicleCategory: apiVehicle.vehicle_category,
          mission: apiVehicle.mission || metadata.mission || '',
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
    
    // Special handling for vehicle category dropdown (translated label -> enum value)
    if (name === 'vehicleCategory') {
      const categoryValue = categoryValueMap[value] || value;
      setFormData(prev => ({
        ...prev,
        [name]: categoryValue || undefined
      }));
    } else {
      setFormData(prev => ({
        ...prev,
        [name]: value
      }));
    }
    
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

    if (!formData.driver.trim()) {
      newErrors.driver = t('driver_required');
    }

    if (!formData.vehicleType) {
      newErrors.vehicleType = t('vehicle_type_required');
    }

    // Mission is optional, no validation needed

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
      // Update vehicle info using new structure
      await vehicleService.updateVehicle(formData.id, {
        name: formData.driver,
        vehicle_type: formData.vehicleType as VehicleType,
        vehicle_category: formData.vehicleCategory,
        mission: formData.mission as MissionType,
        metadata: {
          driver: formData.driver,
        }
      });
      
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

  // Get current category label for display
  const currentCategoryLabel = formData.vehicleCategory 
    ? t(`category.${formData.vehicleCategory}`)
    : '';

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
      </div>

      {/* Form */}
      <div className="bg-white rounded-lg shadow-sm border border-gray-200 p-6">
        <form onSubmit={handleSubmit} className="space-y-6">
          {/* ID phương tiện */}
          <div>
            <label htmlFor="id" className="block text-sm font-medium text-gray-700 mb-2">
              {t('vehicle_id')}
            </label>
            <input
              type="text"
              id="id"
              name="id"
              value={formData.id}
              readOnly
              className="w-full px-3 py-2 border border-gray-200 rounded-lg bg-gray-50 text-gray-500 cursor-not-allowed outline-none"
            />
            <p className="mt-1 text-xs text-gray-400">{t('id_not_editable') || 'Vehicle ID cannot be changed'}</p>
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

          {/* Vehicle Type (Role) - chỉ có scanner và worker */}
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
                <option key={type.value} value={type.value}>
                  {type.label}
                </option>
              ))}
            </select>
            {errors.vehicleType && (
              <p className="mt-1 text-sm text-red-600">{errors.vehicleType}</p>
            )}
          </div>

          {/* Vehicle Category - fetch từ API */}
          <div>
            <label htmlFor="vehicleCategory" className="block text-sm font-medium text-gray-700 mb-2">
              {t('vehicle_category')}
            </label>
            <select
              id="vehicleCategory"
              name="vehicleCategory"
              value={currentCategoryLabel}
              onChange={handleChange}
              className="w-full px-3 py-2 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500"
            >
              <option value="">{t('select_vehicle_category') || 'Select category'}</option>
              {availableCategories.map((cat) => {
                const translatedLabel = t(`category.${cat}`);
                return (
                  <option key={cat} value={translatedLabel}>
                    {translatedLabel}
                  </option>
                );
              })}
            </select>
          </div>

          {/* Nhiệm vụ */}
          <div>
            <label htmlFor="mission" className="block text-sm font-medium text-gray-700 mb-2">
              {t('mission')}
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
                <option key={mission.value} value={mission.value}>
                  {mission.label}
                </option>
              ))}
            </select>
            {errors.mission && (
              <p className="mt-1 text-sm text-red-600">{errors.mission}</p>
            )}
          </div>

          {/* Trạng thái (Read-only) */}
          <div>
            <label htmlFor="status" className="block text-sm font-medium text-gray-700 mb-2">
              {t('status')}
            </label>
            <div className="flex items-center space-x-2 px-3 py-2 border border-gray-200 rounded-lg bg-gray-50 text-gray-500">
              <div className={`w-2 h-2 rounded-full ${formData.status === 'online' ? 'bg-green-500' : 'bg-gray-400'}`}></div>
              <span>{formData.status === 'online' ? (t('online') || 'Online') : (t('offline') || 'Offline')}</span>
            </div>
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
