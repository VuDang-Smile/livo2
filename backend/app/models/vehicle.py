"""Vehicle and pose models."""
from pydantic import BaseModel, Field
from datetime import datetime
from typing import Optional, List
from enum import Enum


class VehicleStatus(str, Enum):
    """Vehicle status enum."""
    ONLINE = "online"
    OFFLINE = "offline"


class VehicleType(str, Enum):
    """Specific vehicle roles."""
    SCANNER = "scanner"
    WORKER = "worker"


class MissionType(str, Enum):
    """Mission types for vehicles."""
    TUNNEL_LINE_1 = "tunnel_line_1"
    TUNNEL_LINE_2 = "tunnel_line_2"
    MATERIAL_TRANSPORT = "material_transport"
    CONCRETE_WALL_PUMP = "concrete_wall_pump"
    SOIL_TRANSPORT = "soil_transport"
    EQUIPMENT_INSTALLATION = "equipment_installation"
    TUNNEL_MAINTENANCE = "tunnel_maintenance"
    GEOLOGICAL_SURVEY = "geological_survey"
    OTHER_MISSION = "other_mission"


class VehicleCategory(str, Enum):
    """High-level vehicle category for tunnel/mining equipment."""
    ROADHEADER = "roadheader"
    DRILL_JUMBO = "drill_jumbo"
    SHOTCRETE_MACHINE = "shotcrete_machine"
    CONCRETE_MIXER_TRUCK = "concrete_mixer_truck"
    WHEEL_LOADER = "wheel_loader"
    DUMP_TRUCK = "dump_truck"
    BACKHOE = "backhoe"
    ROCK_BREAKER = "rock_breaker"
    OTHER_NAMED = "other_named"


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
    name: Optional[str] = None
    description: Optional[str] = None
    vehicle_type: Optional[VehicleType] = None
    mission: Optional[MissionType] = None
    vehicle_category: Optional[VehicleCategory] = None
    status: VehicleStatus = VehicleStatus.OFFLINE
    metadata: Optional[dict] = None
    created_at: datetime
    updated_at: datetime


class VehicleListItem(BaseModel):
    """Vehicle item in list response."""
    vehicle_id: str
    latest_pose: Pose
    name: Optional[str] = None
    vehicle_type: Optional[VehicleType] = None
    mission: Optional[MissionType] = None
    vehicle_category: Optional[VehicleCategory] = None
    status: VehicleStatus = VehicleStatus.OFFLINE
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
    vehicle_type: Optional[VehicleType] = Field(None, description="Vehicle type (scanner, worker, etc.)")
    mission: Optional[MissionType] = Field(None, description="Mission type")
    vehicle_category: Optional[VehicleCategory] = Field(
        None,
        description=(
            "High-level vehicle category "
            "(roadheader, drill_jumbo, shotcrete_machine, concrete_mixer_truck, "
            "wheel_loader, dump_truck, backhoe, rock_breaker, other_named)"
        ),
    )
    status: Optional[VehicleStatus] = Field(VehicleStatus.OFFLINE, description="Vehicle status: online or offline")
    metadata: Optional[dict] = Field(None, description="Additional metadata")


class VehicleRegisterResponse(BaseModel):
    """Response model for vehicle registration."""
    success: bool
    vehicle_id: str
    message: str
    created: bool  # True if newly created, False if already exists


class VehicleStatusUpdateRequest(BaseModel):
    """Request model for updating vehicle status."""
    status: VehicleStatus = Field(..., description="Vehicle status: online or offline")


class VehicleUpdateRequest(BaseModel):
    """Request model for updating vehicle information."""
    name: Optional[str] = Field(None, description="Vehicle name")
    description: Optional[str] = Field(None, description="Vehicle description")
    vehicle_type: Optional[VehicleType] = Field(None, description="Vehicle type")
    mission: Optional[MissionType] = Field(None, description="Mission type")
    status: Optional[VehicleStatus] = Field(None, description="Vehicle status: online or offline")
    metadata: Optional[dict] = Field(None, description="Additional metadata")
    vehicle_category: Optional[VehicleCategory] = Field(
        None,
        description=(
            "High-level vehicle category "
            "(roadheader, drill_jumbo, shotcrete_machine, concrete_mixer_truck, "
            "wheel_loader, dump_truck, backhoe, rock_breaker, other_named)"
        ),
    )


class VehicleUpdateResponse(BaseModel):
    """Response model for vehicle update."""
    success: bool
    vehicle_id: str
    message: str

