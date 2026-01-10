"""MongoDB document models/schemas."""
from datetime import datetime
from typing import Optional, Dict, Any
from bson import ObjectId


class MapDocument:
    """Map document schema for MongoDB."""
    def __init__(
        self,
        upload_id: str,
        filename: str,
        status: str,
        is_current: bool,
        metadata: Dict[str, Any],
        file_path: str,
        uploaded_at: datetime,
        processed_at: Optional[datetime] = None,
        created_at: Optional[datetime] = None,
        updated_at: Optional[datetime] = None,
        _id: Optional[ObjectId] = None
    ):
        self._id = _id or ObjectId()
        self.upload_id = upload_id
        self.filename = filename
        self.status = status
        self.is_current = is_current
        self.metadata = metadata
        self.file_path = file_path
        self.uploaded_at = uploaded_at
        self.processed_at = processed_at
        self.created_at = created_at or datetime.utcnow()
        self.updated_at = updated_at or datetime.utcnow()
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for MongoDB."""
        return {
            "_id": self._id,
            "upload_id": self.upload_id,
            "filename": self.filename,
            "status": self.status,
            "is_current": self.is_current,
            "metadata": self.metadata,
            "file_path": self.file_path,
            "uploaded_at": self.uploaded_at,
            "processed_at": self.processed_at,
            "created_at": self.created_at,
            "updated_at": self.updated_at
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MapDocument":
        """Create from dictionary."""
        return cls(
            _id=data.get("_id"),
            upload_id=data["upload_id"],
            filename=data["filename"],
            status=data["status"],
            is_current=data["is_current"],
            metadata=data["metadata"],
            file_path=data["file_path"],
            uploaded_at=data["uploaded_at"],
            processed_at=data.get("processed_at"),
            created_at=data.get("created_at"),
            updated_at=data.get("updated_at")
        )


class VehicleDocument:
    """Vehicle document schema for MongoDB."""
    def __init__(
        self,
        vehicle_id: str,
        latest_pose: Dict[str, Any],
        name: Optional[str] = None,
        description: Optional[str] = None,
        vehicle_type: Optional[str] = None,
        status: Optional[str] = "active",
        metadata: Optional[Dict[str, Any]] = None,
        created_at: Optional[datetime] = None,
        updated_at: Optional[datetime] = None,
        _id: Optional[ObjectId] = None
    ):
        self._id = _id or ObjectId()
        self.vehicle_id = vehicle_id
        self.latest_pose = latest_pose
        self.name = name
        self.description = description
        self.vehicle_type = vehicle_type
        self.status = status
        self.metadata = metadata or {}
        self.created_at = created_at or datetime.utcnow()
        self.updated_at = updated_at or datetime.utcnow()
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for MongoDB."""
        return {
            "_id": self._id,
            "vehicle_id": self.vehicle_id,
            "latest_pose": self.latest_pose,
            "name": self.name,
            "description": self.description,
            "vehicle_type": self.vehicle_type,
            "status": self.status,
            "metadata": self.metadata,
            "created_at": self.created_at,
            "updated_at": self.updated_at
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "VehicleDocument":
        """Create from dictionary."""
        return cls(
            _id=data.get("_id"),
            vehicle_id=data["vehicle_id"],
            latest_pose=data["latest_pose"],
            name=data.get("name"),
            description=data.get("description"),
            vehicle_type=data.get("vehicle_type"),
            status=data.get("status", "active"),
            metadata=data.get("metadata", {}),
            created_at=data.get("created_at"),
            updated_at=data.get("updated_at")
        )


class VehiclePoseDocument:
    """Vehicle pose history document schema for MongoDB."""
    def __init__(
        self,
        vehicle_id: str,
        position: Dict[str, float],
        orientation: Dict[str, float],
        timestamp: datetime,
        created_at: Optional[datetime] = None,
        _id: Optional[ObjectId] = None
    ):
        self._id = _id or ObjectId()
        self.vehicle_id = vehicle_id
        self.position = position
        self.orientation = orientation
        self.timestamp = timestamp
        self.created_at = created_at or datetime.utcnow()
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for MongoDB."""
        return {
            "_id": self._id,
            "vehicle_id": self.vehicle_id,
            "position": self.position,
            "orientation": self.orientation,
            "timestamp": self.timestamp,
            "created_at": self.created_at
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "VehiclePoseDocument":
        """Create from dictionary."""
        return cls(
            _id=data.get("_id"),
            vehicle_id=data["vehicle_id"],
            position=data["position"],
            orientation=data["orientation"],
            timestamp=data["timestamp"],
            created_at=data.get("created_at")
        )

