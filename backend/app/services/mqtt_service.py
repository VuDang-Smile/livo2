"""MQTT client service."""
import json
import logging
from typing import Optional, Dict, Any

import paho.mqtt.client as mqtt

from app.config import settings
from app.utils.time import isoformat_utc

logger = logging.getLogger(__name__)


class MQTTService:
    """MQTT service for publishing and subscribing to messages."""
    
    def __init__(self):
        self.client: Optional[mqtt.Client] = None
        self.connected = False
        self._setup_client()
    
    def _setup_client(self):
        """Setup MQTT client."""
        self.client = mqtt.Client(client_id=f"{settings.SERVICE_NAME}_client")
        
        # Set callbacks
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_publish = self._on_publish
        
        # Set credentials if provided
        if settings.MQTT_USERNAME and settings.MQTT_PASSWORD:
            self.client.username_pw_set(settings.MQTT_USERNAME, settings.MQTT_PASSWORD)
    
    def _on_connect(self, client, userdata, flags, rc):
        """Callback when connected to broker."""
        if rc == 0:
            self.connected = True
            logger.info(f"Connected to MQTT broker at {settings.MQTT_BROKER_HOST}:{settings.MQTT_BROKER_PORT}")
        else:
            self.connected = False
            logger.error(f"Failed to connect to MQTT broker, return code {rc}")
    
    def _on_disconnect(self, client, userdata, rc):
        """Callback when disconnected from broker."""
        self.connected = False
        logger.warning(f"Disconnected from MQTT broker, return code {rc}")
    
    def _on_publish(self, client, userdata, mid):
        """Callback when message is published."""
        logger.debug(f"Message published with mid: {mid}")
    
    def connect(self):
        """Connect to MQTT broker."""
        if self.client and not self.connected:
            try:
                self.client.connect(settings.MQTT_BROKER_HOST, settings.MQTT_BROKER_PORT, 60)
                self.client.loop_start()
            except Exception as e:
                logger.error(f"Error connecting to MQTT broker: {e}")
    
    def disconnect(self):
        """Disconnect from MQTT broker."""
        if self.client and self.connected:
            self.client.loop_stop()
            self.client.disconnect()
            self.connected = False
            logger.info("Disconnected from MQTT broker")
    
    def publish(self, topic: str, payload: Dict[str, Any], qos: int = 0):
        """Publish message to topic."""
        if not self.connected:
            logger.warning("MQTT not connected, attempting to connect...")
            self.connect()
            if not self.connected:
                logger.error("Failed to connect to MQTT broker, message not published")
                return False
        
        try:
            message = json.dumps(payload, default=str)
            result = self.client.publish(topic, message, qos=qos)
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                logger.debug(f"Published to {topic}: {payload}")
                return True
            else:
                logger.error(f"Failed to publish to {topic}, return code: {result.rc}")
                return False
        except Exception as e:
            logger.error(f"Error publishing to {topic}: {e}")
            return False
    
    def publish_map_event(self, event: str, data: Dict[str, Any]):
        """Publish map-related event."""
        topic = f"{settings.MQTT_TOPIC_PREFIX}/{event}"
        payload = {
            "event": event,
            # Canonical UTC ISO8601 timestamp with timezone
            "timestamp": isoformat_utc(),
            **data
        }
        return self.publish(topic, payload)
    
    def publish_vehicle_pose(self, vehicle_id: str, pose: Dict[str, Any]):
        """Publish vehicle pose update."""
        topic = f"{settings.MQTT_TOPIC_VEHICLES}/{vehicle_id}/pose"
        payload = {
            "event": "vehicle.pose.update",
            "vehicle_id": vehicle_id,
            "pose": pose,
            # Canonical UTC ISO8601 timestamp for the pose event
            "timestamp": isoformat_utc()
        }
        return self.publish(topic, payload)
    
    def is_connected(self) -> bool:
        """Check if connected to broker."""
        return self.connected


# Global MQTT service instance
mqtt_service = MQTTService()

