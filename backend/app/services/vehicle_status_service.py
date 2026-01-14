"""Vehicle status monitoring service with background task for timeout detection."""
import asyncio
import logging
from datetime import timedelta
from typing import List, Dict, Any

from app.config import settings
from app.services.database_service import database_service
from app.services.mqtt_service import mqtt_service
from app.utils.time import now_utc

logger = logging.getLogger(__name__)


class VehicleStatusService:
    """Service for monitoring vehicle status and detecting timeouts."""
    
    def __init__(self):
        self._task: asyncio.Task = None
        self._running: bool = False
    
    async def start(self):
        """Start background task for checking vehicle timeouts."""
        if not self._running:
            self._running = True
            self._task = asyncio.create_task(self._run_loop())
            logger.info("VehicleStatusService started")
    
    async def stop(self):
        """Stop background task."""
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            logger.info("VehicleStatusService stopped")
    
    async def _run_loop(self):
        """Main loop checking timeout every configured interval."""
        while self._running:
            try:
                await self._check_timeout()
                await asyncio.sleep(settings.VEHICLE_STATUS_CHECK_INTERVAL_SECONDS)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in vehicle status check loop: {e}")
                # Continue running even if there's an error
                await asyncio.sleep(settings.VEHICLE_STATUS_CHECK_INTERVAL_SECONDS)
    
    async def _check_timeout(self):
        """Check for timed out vehicles and update their status to offline."""
        try:
            # Calculate cutoff time: now - timeout threshold
            cutoff_time = now_utc() - timedelta(seconds=settings.VEHICLE_HEARTBEAT_TIMEOUT_SECONDS)
            
            # Query vehicles with status="online" and updated_at < cutoff_time
            query = {
                "status": "online",
                "updated_at": {"$lt": cutoff_time}
            }
            
            # Find timed out vehicles
            if database_service.vehicles_collection is None:
                logger.warning("Database not connected, skipping timeout check")
                return
            
            timed_out_vehicles = await database_service.vehicles_collection.find(query).to_list(length=None)
            
            if not timed_out_vehicles:
                return
            
            logger.info(f"Found {len(timed_out_vehicles)} timed out vehicles")
            
            # Update each timed out vehicle to offline
            for vehicle in timed_out_vehicles:
                vehicle_id = vehicle.get("vehicle_id")
                if not vehicle_id:
                    continue
                
                # Update status to offline
                success = await database_service.update_vehicle_status(vehicle_id, "offline")
                
                if success:
                    logger.info(f"Vehicle {vehicle_id} timed out, status changed to offline")
                    
                    # Publish MQTT event
                    mqtt_service.publish_map_event("vehicle.status.updated", {
                        "vehicle_id": vehicle_id,
                        "status": "offline"
                    })
                else:
                    logger.warning(f"Failed to update status for timed out vehicle {vehicle_id}")
        
        except Exception as e:
            logger.error(f"Error checking vehicle timeout: {e}")


# Global vehicle status service instance
vehicle_status_service = VehicleStatusService()
