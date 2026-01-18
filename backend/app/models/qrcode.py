"""QR Code models."""
from pydantic import BaseModel, Field
from typing import List, Optional, Dict


class QRCode(BaseModel):
    """QR Code data model."""
    id: str = Field(..., description="QR code ID (e.g., qr-TM-0001)")
    position: List[float] = Field(..., description="3D position [x, y, z]", min_length=3, max_length=3)
    isActive: bool = Field(default=False, description="Whether QR code is active")
    surface: str = Field(default="floor", description="Surface type (e.g., floor, wall, ceiling)")


class QRCodeResponse(BaseModel):
    """Response model for single QR code."""
    id: str
    position: List[float]
    isActive: bool
    surface: str


class QRCodeListResponse(BaseModel):
    """Response model for QR codes list."""
    qrcodes: List[QRCodeResponse]
    total: int


# QRCodeCreateRequest is now just a type alias for Dict[str, List[float]]
# Format: {"TM:0001": [x, y, z]} or {"TM:0001": [x, y, z], "TM:0002": [x, y, z]}
QRCodeCreateRequest = Dict[str, List[float]]


class QRCodeUpdateRequest(BaseModel):
    """Request model for updating QR code (all fields optional except id in path)."""
    position: Optional[List[float]] = Field(None, description="3D position [x, y, z]")
    isActive: Optional[bool] = Field(None, description="Whether QR code is active")
    surface: Optional[str] = Field(None, description="Surface type (e.g., floor, wall, ceiling)")
