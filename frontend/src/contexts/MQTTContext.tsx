import React, { createContext, useContext, useEffect, useState, useCallback, useRef, ReactNode } from 'react';
import mqtt, { MqttClient } from 'mqtt';
import { normalizeTimestamp } from '../utils/timestampHelpers';

// Types
interface PCDNotification {
  type?: string;
  mac_address?: string;
  filename?: string;
  file_path?: string;
  file_size?: number;
  timestamp?: number | string;
  file_id?: string;
  uploaded_by?: string;
  metadata?: any;
}

interface MapUpdateNotification {
  file_id?: string;
  filename?: string;
  uploaded_by?: string;
  timestamp?: string;
}

export interface PositionUpdateNotification {
  vehicle_id: string;
  map_id?: string;  // ID of the map this position is relative to
  position: { x: number; y: number; z: number; timestamp?: string };
  pose?: {
    frame_id: string;
    position: { x: number; y: number; z: number };
    orientation: { w: number; x: number; y: number; z: number };
    timestamp: string;
  };
  confidence?: number;
  icp_fitness?: number;
  icp_rmse?: number;
  // QR data fields
  qr_code?: string;
  qr_position?: { x: number; y: number; z: number };
  distance_to_qr?: number;
  distance_to_origin?: number;
  qr_scan_timestamp?: string;
}

interface LastVehicleStatus {
  vehicle_id: string;
  status: string;
  action?: string;
  timestamp?: string;
  data?: any;
}

interface LidarStatusUpdate {
  status?: string;
  [key: string]: any;
}

interface MQTTContextType {
  client: MqttClient | null;
  isConnected: boolean;
  lastPCDNotification: PCDNotification | null;
  lastMapUpdate: MapUpdateNotification | null;
  lastPositionUpdate: PositionUpdateNotification | null;
  lastVehicleStatus: LastVehicleStatus | null;
  lastLidarStatus: LidarStatusUpdate | null;
  error: string | null;
  connect: () => void;
  disconnect: () => void;
  connectionStatus: string;
}

const MQTTContext = createContext<MQTTContextType | undefined>(undefined);

interface MQTTProviderProps {
  children: ReactNode;
  brokerUrl?: string;
}

export const MQTTProvider: React.FC<MQTTProviderProps> = ({
  children,
  brokerUrl = 'ws://localhost:9001'
}) => {
  const [client, setClient] = useState<MqttClient | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [lastPCDNotification, setLastPCDNotification] = useState<PCDNotification | null>(null);
  const [lastMapUpdate, setLastMapUpdate] = useState<MapUpdateNotification | null>(null);
  const [lastPositionUpdate, setLastPositionUpdate] = useState<PositionUpdateNotification | null>(null);
  const [lastVehicleStatus, setLastVehicleStatus] = useState<LastVehicleStatus | null>(null);
  const [lastLidarStatus, setLastLidarStatus] = useState<LidarStatusUpdate | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [connectionStatus, setConnectionStatus] = useState<string>('Disconnected');
  
  // Track current MQTT URL to avoid unnecessary reconnects
  const currentUrlRef = useRef<string | null>(null);
  const isConnectingRef = useRef<boolean>(false);

  const connect = useCallback(() => {
    // Prevent multiple simultaneous connection attempts
    if (isConnectingRef.current) {
      console.log('[MQTT] Connection attempt already in progress');
      return;
    }
    
    if (client && client.connected) {
      console.log('[MQTT] Client already connected');
      return;
    }
    
    // Disconnect existing client if URL changed
    if (client) {
      const newUrl = process.env.REACT_APP_MQTT_WS_URL || brokerUrl || 'ws://localhost:9001';
      
      if (currentUrlRef.current !== newUrl) {
        console.log('[MQTT] URL changed, disconnecting old client');
        client.end(true); // Force disconnect
        setClient(null);
      } else {
        console.log('[MQTT] Client exists with same URL, skipping reconnect');
        return;
      }
    }

    try {
      // Priority: Environment variable > brokerUrl prop > localhost fallback
      const envUrl = (process.env.REACT_APP_MQTT_WS_URL as string) || '';
      let finalUrl: string;
      
      if (envUrl && /^wss?:\/\//i.test(envUrl)) {
        finalUrl = envUrl;
        console.log('[MQTT] Using environment MQTT URL:', finalUrl);
      } else if (brokerUrl && /^wss?:\/\//i.test(brokerUrl)) {
        finalUrl = brokerUrl;
        console.log('[MQTT] Using prop MQTT URL:', finalUrl);
      } else {
        console.warn('[MQTT] No valid MQTT URL found. Using localhost fallback');
        finalUrl = 'ws://localhost:9001';
      }
      
      console.log('[MQTT] Connecting to MQTT broker:', finalUrl);
      setConnectionStatus('Connecting...');
      setError(null);
      currentUrlRef.current = finalUrl;
      isConnectingRef.current = true;
      
      const mqttClient = mqtt.connect(finalUrl, {
        reconnectPeriod: 5000,
        connectTimeout: 10000,
        clientId: `frontend_${Math.random().toString(16).substr(2, 8)}`,
        clean: true,
        keepalive: 60,
        username: process.env.REACT_APP_MQTT_WS_USERNAME,
        password: process.env.REACT_APP_MQTT_WS_PASSWORD
      });

      mqttClient.on('connect', () => {
        console.log('[MQTT] Connected successfully');
        isConnectingRef.current = false;
        setIsConnected(true);
        setConnectionStatus('Connected');
        setError(null);
        
        // Subscribe topics
        const topics = ['lidar/pcd/upload', 'lidar/pcd/processed', 'lidar/map/update', 'lidar/position/update', 'lidar/pose/update', 'lidar/vehicle/status', 'lidar/status'];
        topics.forEach((topic) =>
          mqttClient.subscribe(topic, (err) => {
            if (err) {
              console.error('[MQTT] Failed to subscribe to topic:', topic, err);
              setError(`Failed to subscribe: ${err.message}`);
            } else {
              console.log('[MQTT] Successfully subscribed to topic:', topic);
            }
          })
        );
      });

      mqttClient.on('message', (topic, payload) => {
        try {
          console.log('MQTT message received:', topic, payload.toString());
          if (topic === 'lidar/pcd/upload') {
            const notification: PCDNotification = JSON.parse(payload.toString());
            setLastPCDNotification(notification);
          } else if (topic === 'lidar/map/update') {
            const mapUpdate: MapUpdateNotification = JSON.parse(payload.toString());
            setLastMapUpdate(mapUpdate);
          } else if (topic === 'lidar/position/update') {
            const posUpdate: PositionUpdateNotification = JSON.parse(payload.toString());
            setLastPositionUpdate(posUpdate);
          } else if (topic === 'lidar/pose/update') {
            const poseMsg = JSON.parse(payload.toString());
            console.log('🔍 Raw pose message:', poseMsg);
            
            const posUpdate: PositionUpdateNotification = {
              vehicle_id: poseMsg.vehicle_id,
              position: { 
                x: poseMsg.position?.x ?? 0, 
                y: poseMsg.position?.y ?? 0, 
                z: poseMsg.position?.z ?? 0, 
                timestamp: normalizeTimestamp(poseMsg.timestamp, false)
              },
              pose: {
                frame_id: poseMsg.frame_id || 'map',
                position: { 
                  x: poseMsg.position?.x ?? 0, 
                  y: poseMsg.position?.y ?? 0, 
                  z: poseMsg.position?.z ?? 0 
                },
                orientation: { 
                  w: poseMsg.orientation?.w ?? 1, 
                  x: poseMsg.orientation?.x ?? 0, 
                  y: poseMsg.orientation?.y ?? 0, 
                  z: poseMsg.orientation?.z ?? 0 
                },
                timestamp: normalizeTimestamp(poseMsg.timestamp, false)
              },
              // Thêm QR data từ metadata nếu có
              ...(poseMsg.metadata?.qr_code && {
                qr_code: poseMsg.metadata.qr_code,
                qr_position: poseMsg.metadata.qr_position,
                distance_to_qr: poseMsg.metadata.distance_to_qr,
                distance_to_origin: poseMsg.metadata.distance_to_origin,
                qr_scan_timestamp: poseMsg.metadata.qr_scan_timestamp
              })
            };
            
            console.log('🔍 Processed position update:', posUpdate);
            setLastPositionUpdate(posUpdate);
          } else if (topic === 'lidar/vehicle/status') {
            const statusMsg = JSON.parse(payload.toString());
            console.log('🔍 [MQTT] Vehicle status message received:', statusMsg);
            setLastVehicleStatus(statusMsg);
          } else if (topic === 'lidar/status') {
            const lidarStatus: LidarStatusUpdate = JSON.parse(payload.toString());
            setLastLidarStatus(lidarStatus);
          }
        } catch (err) {
          console.error('Error parsing MQTT message:', err);
          setError('Error parsing MQTT message');
        }
      });

      mqttClient.on('error', (err) => {
        console.error('[MQTT] Connection error:', err);
        isConnectingRef.current = false;
        
        const url = currentUrlRef.current || 'unknown';
        let errorMessage = `MQTT error: ${err.message}`;
        
        // Provide helpful error messages
        if (err.message.includes('ECONNREFUSED') || err.message.includes('WebSocket')) {
          errorMessage = `Cannot connect to MQTT broker at ${url}. Please ensure:
1. Backend services are running (docker-compose up)
2. MQTT broker (Mosquitto) is running on port 9001
3. WebSocket is enabled in Mosquitto config`;
          console.warn('[MQTT] Connection refused:', errorMessage);
        } else if (err.message.includes('timeout')) {
          errorMessage = `MQTT connection timeout. Check if broker is accessible at ${url}`;
        }
        
        setError(errorMessage);
        setIsConnected(false);
        setConnectionStatus('Error');
      });

      mqttClient.on('close', () => {
        console.log('[MQTT] Connection closed');
        isConnectingRef.current = false;
        setIsConnected(false);
        setConnectionStatus('Disconnected');
      });

      mqttClient.on('offline', () => {
        console.log('MQTT client offline');
        setIsConnected(false);
        setConnectionStatus('Offline');
      });

      mqttClient.on('reconnect', () => {
        console.log('MQTT reconnecting...');
        setConnectionStatus('Reconnecting...');
        setError(null);
      });

      setClient(mqttClient);
      
    } catch (err) {
      console.error('[MQTT] Failed to create MQTT client:', err);
      isConnectingRef.current = false;
      setError(`Connection failed: ${err instanceof Error ? err.message : 'Unknown error'}`);
      setConnectionStatus('Failed');
    }
  }, [brokerUrl, client]);

  const disconnect = useCallback(() => {
    if (client) {
      console.log('[MQTT] Disconnecting MQTT client');
      isConnectingRef.current = false;
      currentUrlRef.current = null;
      client.end(true); // Force disconnect
      setClient(null);
      setIsConnected(false);
      setConnectionStatus('Disconnected');
      setError(null);
    }
  }, [client]);

  // Auto-connect on mount
  useEffect(() => {
    const newUrl = process.env.REACT_APP_MQTT_WS_URL || brokerUrl || 'ws://localhost:9001';
    
    // Only connect if URL changed or no client exists
    if (!client || currentUrlRef.current !== newUrl) {
      console.log('[MQTT] Auto-connecting...');
      connect();
    }
    
    // Cleanup on unmount
    return () => {
      disconnect();
    };
  }, [connect, disconnect, brokerUrl, client]);

  const value: MQTTContextType = {
    client,
    isConnected,
    lastPCDNotification,
    lastMapUpdate,
    lastPositionUpdate,
    lastVehicleStatus,
    lastLidarStatus,
    error,
    connect,
    disconnect,
    connectionStatus,
  };

  return (
    <MQTTContext.Provider value={value}>
      {children}
    </MQTTContext.Provider>
  );
};

export const useMQTT = (): MQTTContextType => {
  const context = useContext(MQTTContext);
  if (context === undefined) {
    throw new Error('useMQTT must be used within a MQTTProvider');
  }
  return context;
};

export default MQTTContext;
