import React, { useState, useEffect } from 'react';
import { useLanguage } from '../contexts/LanguageContext';
import { Activity, CheckCircle, XCircle, RefreshCw, Clock, Server, Globe } from 'lucide-react';

interface BackendHealth {
  status: 'healthy' | 'degraded' | 'unhealthy' | 'checking';
  service?: string;
  version?: string;
  timestamp?: number;
  uptime_seconds?: number;
  services?: {
    mongodb?: {
      status: string;
      latency_ms?: number;
    };
    mqtt?: {
      status: string;
      broker?: string;
    };
  };
  error?: string;
}

interface FrontendHealth {
  status: 'healthy';
  version: string;
  timestamp: number;
}

const Health: React.FC = () => {
  const { t } = useLanguage();
  const [backendHealth, setBackendHealth] = useState<BackendHealth>({ status: 'checking' });
  const [frontendHealth] = useState<FrontendHealth>({
    status: 'healthy',
    version: '1.0.0',
    timestamp: Date.now(),
  });
  const [isRefreshing, setIsRefreshing] = useState(false);

  // Get backend URL - try to detect from current host or use default
  const getBackendUrl = (): string => {
    const hostname = window.location.hostname;
    // If running on frontend.lidar.tm or lidar.tm, use backend.lidar.tm
    if (hostname === 'frontend.lidar.tm' || hostname === 'lidar.tm' || hostname.includes('lidar.tm')) {
      return 'http://backend.lidar.tm';
    }
    // For localhost or IP addresses, use same hostname with port 8000
    return `http://${hostname}:8000`;
  };

  const checkBackendHealth = async () => {
    setBackendHealth({ status: 'checking' });
    const backendUrl = getBackendUrl();

    try {
      // Create abort controller for timeout
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 5000);

      const response = await fetch(`${backendUrl}/health`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();
      setBackendHealth({
        status: data.status === 'healthy' ? 'healthy' : 'degraded',
        service: data.service,
        version: data.version,
        timestamp: data.timestamp,
        uptime_seconds: data.uptime_seconds,
        services: data.services,
      });
    } catch (error: any) {
      setBackendHealth({
        status: 'unhealthy',
        error: error.message || 'Failed to connect to backend',
      });
    }
  };

  useEffect(() => {
    checkBackendHealth();
    // Auto-refresh every 30 seconds
    const interval = setInterval(checkBackendHealth, 30000);
    return () => clearInterval(interval);
  }, []);

  const handleRefresh = async () => {
    setIsRefreshing(true);
    await checkBackendHealth();
    setIsRefreshing(false);
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case 'healthy':
        return <CheckCircle className="w-6 h-6 text-green-500" />;
      case 'degraded':
        return <Activity className="w-6 h-6 text-yellow-500" />;
      case 'unhealthy':
      case 'checking':
        return <XCircle className="w-6 h-6 text-red-500" />;
      default:
        return <Clock className="w-6 h-6 text-gray-500" />;
    }
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'healthy':
        return 'bg-green-100 text-green-800 border-green-200';
      case 'degraded':
        return 'bg-yellow-100 text-yellow-800 border-yellow-200';
      case 'unhealthy':
        return 'bg-red-100 text-red-800 border-red-200';
      case 'checking':
        return 'bg-gray-100 text-gray-800 border-gray-200';
      default:
        return 'bg-gray-100 text-gray-800 border-gray-200';
    }
  };

  const formatUptime = (seconds?: number) => {
    if (!seconds) return '-';
    const days = Math.floor(seconds / 86400);
    const hours = Math.floor((seconds % 86400) / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const secs = seconds % 60;

    if (days > 0) {
      return `${days}d ${hours}h ${minutes}m`;
    } else if (hours > 0) {
      return `${hours}h ${minutes}m`;
    } else if (minutes > 0) {
      return `${minutes}m ${secs}s`;
    }
    return `${secs}s`;
  };

  const formatTimestamp = (timestamp?: number) => {
    if (!timestamp) return '-';
    return new Date(timestamp * 1000).toLocaleString();
  };

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex justify-between items-center">
        <div>
          <h1 className="text-3xl font-bold text-gray-900">{t('health_title')}</h1>
          <p className="text-gray-600 mt-2">{t('health_description')}</p>
        </div>
        <button
          onClick={handleRefresh}
          disabled={isRefreshing}
          className="bg-blue-600 hover:bg-blue-700 disabled:bg-gray-400 text-white px-4 py-2 rounded-lg font-medium transition-colors duration-200 flex items-center space-x-2"
        >
          <RefreshCw className={`w-5 h-5 ${isRefreshing ? 'animate-spin' : ''}`} />
          <span>{t('refresh')}</span>
        </button>
      </div>

      {/* Health Status Cards */}
      <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
        {/* Frontend Health */}
        <div className="bg-white p-6 rounded-lg shadow-sm border border-gray-200">
          <div className="flex items-center justify-between mb-4">
            <div className="flex items-center space-x-3">
              <Globe className="w-6 h-6 text-blue-600" />
              <h2 className="text-xl font-semibold text-gray-900">{t('frontend_health')}</h2>
            </div>
            {getStatusIcon(frontendHealth.status)}
          </div>
          <div className="space-y-3">
            <div className="flex justify-between">
              <span className="text-gray-600">{t('status')}:</span>
              <span className={`px-3 py-1 rounded-full text-sm font-medium ${getStatusColor(frontendHealth.status)}`}>
                {t('healthy')}
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-600">{t('version')}:</span>
              <span className="text-gray-900 font-medium">{frontendHealth.version}</span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-600">{t('url')}:</span>
              <span className="text-gray-900 font-medium text-sm">{window.location.origin}</span>
            </div>
          </div>
        </div>

        {/* Backend Health */}
        <div className="bg-white p-6 rounded-lg shadow-sm border border-gray-200">
          <div className="flex items-center justify-between mb-4">
            <div className="flex items-center space-x-3">
              <Server className="w-6 h-6 text-purple-600" />
              <h2 className="text-xl font-semibold text-gray-900">{t('backend_health')}</h2>
            </div>
            {getStatusIcon(backendHealth.status)}
          </div>
          <div className="space-y-3">
            <div className="flex justify-between">
              <span className="text-gray-600">{t('status')}:</span>
              <span className={`px-3 py-1 rounded-full text-sm font-medium ${getStatusColor(backendHealth.status)}`}>
                {backendHealth.status === 'checking' ? t('checking') : 
                 backendHealth.status === 'healthy' ? t('healthy') :
                 backendHealth.status === 'degraded' ? t('degraded') : t('unhealthy')}
              </span>
            </div>
            {backendHealth.service && (
              <div className="flex justify-between">
                <span className="text-gray-600">{t('service')}:</span>
                <span className="text-gray-900 font-medium">{backendHealth.service}</span>
              </div>
            )}
            {backendHealth.version && (
              <div className="flex justify-between">
                <span className="text-gray-600">{t('version')}:</span>
                <span className="text-gray-900 font-medium">{backendHealth.version}</span>
              </div>
            )}
            {backendHealth.uptime_seconds !== undefined && (
              <div className="flex justify-between">
                <span className="text-gray-600">{t('uptime')}:</span>
                <span className="text-gray-900 font-medium">{formatUptime(backendHealth.uptime_seconds)}</span>
              </div>
            )}
            <div className="flex justify-between">
              <span className="text-gray-600">{t('url')}:</span>
              <span className="text-gray-900 font-medium text-sm">{getBackendUrl()}</span>
            </div>
            {backendHealth.error && (
              <div className="mt-3 p-3 bg-red-50 border border-red-200 rounded-lg">
                <p className="text-sm text-red-800">{backendHealth.error}</p>
              </div>
            )}
          </div>
        </div>
      </div>

      {/* Backend Services Details */}
      {backendHealth.services && backendHealth.status !== 'unhealthy' && (
        <div className="bg-white p-6 rounded-lg shadow-sm border border-gray-200">
          <h2 className="text-xl font-semibold text-gray-900 mb-4">{t('backend_services')}</h2>
          <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
            {/* MongoDB Status */}
            {backendHealth.services.mongodb && (
              <div className="border border-gray-200 rounded-lg p-4">
                <div className="flex items-center justify-between mb-2">
                  <h3 className="font-medium text-gray-900">MongoDB</h3>
                  {getStatusIcon(backendHealth.services.mongodb.status === 'connected' ? 'healthy' : 'unhealthy')}
                </div>
                <div className="space-y-2">
                  <div className="flex justify-between text-sm">
                    <span className="text-gray-600">{t('status')}:</span>
                    <span className={`font-medium ${
                      backendHealth.services.mongodb.status === 'connected' ? 'text-green-600' : 'text-red-600'
                    }`}>
                      {backendHealth.services.mongodb.status}
                    </span>
                  </div>
                  {backendHealth.services.mongodb.latency_ms !== undefined && (
                    <div className="flex justify-between text-sm">
                      <span className="text-gray-600">{t('latency')}:</span>
                      <span className="text-gray-900 font-medium">{backendHealth.services.mongodb.latency_ms}ms</span>
                    </div>
                  )}
                </div>
              </div>
            )}

            {/* MQTT Status */}
            {backendHealth.services.mqtt && (
              <div className="border border-gray-200 rounded-lg p-4">
                <div className="flex items-center justify-between mb-2">
                  <h3 className="font-medium text-gray-900">MQTT</h3>
                  {getStatusIcon(backendHealth.services.mqtt.status === 'connected' ? 'healthy' : 'unhealthy')}
                </div>
                <div className="space-y-2">
                  <div className="flex justify-between text-sm">
                    <span className="text-gray-600">{t('status')}:</span>
                    <span className={`font-medium ${
                      backendHealth.services.mqtt.status === 'connected' ? 'text-green-600' : 'text-red-600'
                    }`}>
                      {backendHealth.services.mqtt.status}
                    </span>
                  </div>
                  {backendHealth.services.mqtt.broker && (
                    <div className="flex justify-between text-sm">
                      <span className="text-gray-600">{t('broker')}:</span>
                      <span className="text-gray-900 font-medium text-xs">{backendHealth.services.mqtt.broker}</span>
                    </div>
                  )}
                </div>
              </div>
            )}
          </div>
        </div>
      )}

      {/* Last Update Time */}
      <div className="text-center text-sm text-gray-500">
        {t('last_updated')}: {formatTimestamp(backendHealth.timestamp)}
      </div>
    </div>
  );
};

export default Health;

