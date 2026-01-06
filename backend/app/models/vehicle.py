"""Vehicle and pose models."""
from pydantic import BaseModel, Field
from datetime import datetime
from typing import Optional, List


class Position(BaseModel):
    """3D position."""
    x: float
    y: float
    z: float


class Orientation(BaseModel):
    """Quaternion orientation."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


class Pose(BaseModel):
    """Pose data model."""
    position: Position
    orientation: Orientation
    timestamp: datetime


class PoseUpdateRequest(BaseModel):
    """Request model for pose update."""
    position: Position
    orientation: Orientation
    timestamp: datetime


class PoseUpdateResponse(BaseModel):
    """Response model for pose update."""
    success: bool
    vehicle_id: str
    message: str


class VehicleResponse(BaseModel):
    """Response model for vehicle info."""
    vehicle_id: str
    latest_pose: Pose
    created_at: datetime
    updated_at: datetime


class VehicleListItem(BaseModel):
    """Vehicle item in list response."""
    vehicle_id: str
    latest_pose: Pose
    updated_at: datetime


class VehiclesListResponse(BaseModel):
    """Response model for vehicles list."""
    vehicles: List[VehicleListItem]
    total: int


class VehicleRegisterRequest(BaseModel):
    """Request model for vehicle registration."""
    vehicle_id: str = Field(..., description="MAC address (đã bỏ dấu :, ví dụ: aa11bb22cc33dd44)")
    name: Optional[str] = Field(None, description="Vehicle name")
    description: Optional[str] = Field(None, description="Vehicle description")
    vehicle_type: Optional[str] = Field(None, description="Vehicle type (e.g., robot, drone, car)")
    metadata: Optional[dict] = Field(None, description="Additional metadata")


class VehicleRegisterResponse(BaseModel):
    """Response model for vehicle registration."""
    success: bool
    vehicle_id: str
    message: str
    created: bool  # True if newly created, False if already exists

