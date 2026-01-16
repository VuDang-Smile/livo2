"""MongoDB database service."""
import logging
from datetime import datetime
from typing import Optional, List, Dict, Any

from motor.motor_asyncio import AsyncIOMotorClient, AsyncIOMotorDatabase, AsyncIOMotorCollection
from bson import ObjectId

from app.config import settings
from app.models.database import MapDocument, VehicleDocument, VehiclePoseDocument
from app.utils.time import now_utc, ensure_utc

logger = logging.getLogger(__name__)


class DatabaseService:
    """Service for MongoDB operations."""
    
    def __init__(self):
        self.client: Optional[AsyncIOMotorClient] = None
        self.db: Optional[AsyncIOMotorDatabase] = None
        self.maps_collection: Optional[AsyncIOMotorCollection] = None
        self.vehicles_collection: Optional[AsyncIOMotorCollection] = None
        self.vehicle_poses_collection: Optional[AsyncIOMotorCollection] = None
    
    async def connect(self):
        """Connect to MongoDB."""
        try:
            # Build connection string
            if settings.MONGODB_USERNAME and settings.MONGODB_PASSWORD:
                # Extract host:port/db from MONGODB_URL
                url_parts = settings.MONGODB_URL.replace('mongodb://', '').split('/')
                host_port = url_parts[0]
                connection_string = f"mongodb://{settings.MONGODB_USERNAME}:{settings.MONGODB_PASSWORD}@{host_port}"
            else:
                connection_string = settings.MONGODB_URL
            
            self.client = AsyncIOMotorClient(connection_string, serverSelectionTimeoutMS=3000)
            self.db = self.client[settings.MONGODB_DB_NAME]
            self.maps_collection = self.db.maps
            self.vehicles_collection = self.db.vehicles
            self.vehicle_poses_collection = self.db.vehicle_poses
            
            # Create indexes
            await self._create_indexes()
            
            logger.info(f"Connected to MongoDB: {settings.MONGODB_DB_NAME}")
        except Exception as e:
            logger.error(f"Error connecting to MongoDB: {e}")
            raise
    
    async def disconnect(self):
        """Disconnect from MongoDB."""
        if self.client:
            self.client.close()
            logger.info("Disconnected from MongoDB")
    
    async def _create_indexes(self):
        """Create necessary indexes."""
        try:
            # Maps collection indexes
            await self.maps_collection.create_index("is_current", unique=True, partialFilterExpression={"is_current": True})
            await self.maps_collection.create_index("upload_id", unique=True)
            await self.maps_collection.create_index("status")
            
            # Vehicles collection indexes
            await self.vehicles_collection.create_index("vehicle_id", unique=True)
            
            # Vehicle poses collection indexes
            await self.vehicle_poses_collection.create_index([("vehicle_id", 1), ("timestamp", -1)])
            
            logger.info("MongoDB indexes created")
        except Exception as e:
            logger.warning(f"Error creating indexes (may already exist): {e}")
    
    # Map methods
    async def get_current_map(self) -> Optional[Dict[str, Any]]:
        """Get current map (is_current: true)."""
        try:
            doc = await self.maps_collection.find_one({"is_current": True})
            return doc
        except Exception as e:
            logger.error(f"Error getting current map: {e}")
            return None
    
    async def get_map_by_upload_id(self, upload_id: str) -> Optional[Dict[str, Any]]:
        """Get map by upload_id."""
        try:
            doc = await self.maps_collection.find_one({"upload_id": upload_id})
            return doc
        except Exception as e:
            logger.error(f"Error getting map by upload_id: {e}")
            return None
    
    async def create_map(self, map_data: Dict[str, Any]) -> Optional[str]:
        """Create new map."""
        try:
            # Unset is_current for all existing maps if single version mode
            if settings.SINGLE_VERSION_MODE:
                await self.maps_collection.update_many(
                    {"is_current": True},
                    {"$set": {"is_current": False}}
                )
            
            result = await self.maps_collection.insert_one(map_data)
            return str(result.inserted_id)
        except Exception as e:
            logger.error(f"Error creating map: {e}")
            return None
    
    async def update_map(self, upload_id: str, updates: Dict[str, Any]) -> bool:
        """Update map."""
        try:
            # Always store timestamps as UTC
            updates["updated_at"] = now_utc()
            result = await self.maps_collection.update_one(
                {"upload_id": upload_id},
                {"$set": updates}
            )
            return result.modified_count > 0
        except Exception as e:
            logger.error(f"Error updating map: {e}")
            return False
    
    async def delete_current_map(self) -> bool:
        """Delete current map."""
        try:
            result = await self.maps_collection.delete_one({"is_current": True})
            return result.deleted_count > 0
        except Exception as e:
            logger.error(f"Error deleting current map: {e}")
            return False
    
    # Vehicle methods
    async def create_vehicle(self, vehicle_data: Dict[str, Any]) -> bool:
        """Create a new vehicle."""
        try:
            result = await self.vehicles_collection.insert_one(vehicle_data)
            return result.inserted_id is not None
        except Exception as e:
            logger.error(f"Error creating vehicle: {e}")
            return False
    
    async def get_vehicle(self, vehicle_id: str) -> Optional[Dict[str, Any]]:
        """Get vehicle info and latest pose."""
        try:
            doc = await self.vehicles_collection.find_one({"vehicle_id": vehicle_id})
            return doc
        except Exception as e:
            logger.error(f"Error getting vehicle: {e}")
            return None
    
    async def get_all_vehicles(self) -> List[Dict[str, Any]]:
        """Get all vehicles."""
        try:
            cursor = self.vehicles_collection.find({})
            vehicles = await cursor.to_list(length=None)
            return vehicles
        except Exception as e:
            logger.error(f"Error getting all vehicles: {e}")
            return []
    
    async def update_vehicle_pose(self, vehicle_id: str, pose_data: Dict[str, Any]) -> bool:
        """Update vehicle pose (upsert if not exists)."""
        try:
            latest_pose = {
                "position": pose_data["position"],
                "orientation": pose_data["orientation"],
                # Assume pose_data["timestamp"] is already UTC-aware datetime
                "timestamp": ensure_utc(pose_data["timestamp"])
            }
            
            result = await self.vehicles_collection.update_one(
                {"vehicle_id": vehicle_id},
                {
                    "$set": {
                        "latest_pose": latest_pose,
                        "updated_at": now_utc()
                    },
                    "$setOnInsert": {
                        "created_at": now_utc()
                    }
                },
                upsert=True
            )
            return True
        except Exception as e:
            logger.error(f"Error updating vehicle pose: {e}")
            return False
    
    async def update_vehicle_status(self, vehicle_id: str, status: str) -> bool:
        """Update vehicle status."""
        try:
            result = await self.vehicles_collection.update_one(
                {"vehicle_id": vehicle_id},
                {
                    "$set": {
                        "status": status,
                        "updated_at": now_utc()
                    }
                }
            )
            return result.modified_count > 0 or result.matched_count > 0
        except Exception as e:
            logger.error(f"Error updating vehicle status: {e}")
            return False
    
    async def update_vehicle(self, vehicle_id: str, updates: Dict[str, Any]) -> bool:
        """Update vehicle information."""
        try:
            # Prepare update data, excluding vehicle_id and created_at
            update_data = {}
            if "name" in updates:
                update_data["name"] = updates["name"]
            if "description" in updates:
                update_data["description"] = updates["description"]
            if "vehicle_type" in updates:
                update_data["vehicle_type"] = updates["vehicle_type"]
            if "status" in updates:
                update_data["status"] = updates["status"]
            if "metadata" in updates:
                update_data["metadata"] = updates["metadata"]
            if "vehicle_category" in updates:
                update_data["vehicle_category"] = updates["vehicle_category"]
            
            # Always update updated_at in UTC
            update_data["updated_at"] = now_utc()
            
            result = await self.vehicles_collection.update_one(
                {"vehicle_id": vehicle_id},
                {"$set": update_data}
            )
            return result.modified_count > 0 or result.matched_count > 0
        except Exception as e:
            logger.error(f"Error updating vehicle: {e}")
            return False
    
    async def save_pose_history(self, vehicle_id: str, pose_data: Dict[str, Any]) -> Optional[str]:
        """Save pose to history collection."""
        try:
            doc = {
                "vehicle_id": vehicle_id,
                "position": pose_data["position"],
                "orientation": pose_data["orientation"],
                # Store pose timestamp in UTC
                "timestamp": ensure_utc(pose_data["timestamp"]),
                "created_at": now_utc()
            }
            result = await self.vehicle_poses_collection.insert_one(doc)
            return str(result.inserted_id)
        except Exception as e:
            logger.error(f"Error saving pose history: {e}")
            return None
    
    async def get_pose_history(
        self,
        vehicle_id: str,
        limit: int = 100,
        start_time: Optional[datetime] = None,
        end_time: Optional[datetime] = None
    ) -> List[Dict[str, Any]]:
        """Get pose history for vehicle."""
        try:
            query = {"vehicle_id": vehicle_id}
            if start_time:
                query["timestamp"] = {"$gte": start_time}
            if end_time:
                if "timestamp" in query:
                    query["timestamp"]["$lte"] = end_time
                else:
                    query["timestamp"] = {"$lte": end_time}
            
            cursor = self.vehicle_poses_collection.find(query).sort("timestamp", -1).limit(limit)
            poses = await cursor.to_list(length=limit)
            return poses
        except Exception as e:
            logger.error(f"Error getting pose history: {e}")
            return []
    
    async def check_connection(self) -> bool:
        """Check MongoDB connection."""
        try:
            await self.client.admin.command('ping')
            return True
        except Exception as e:
            logger.error(f"MongoDB connection check failed: {e}")
            return False


# Global database service instance
database_service = DatabaseService()

