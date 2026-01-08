"""Upload request/response models."""
from pydantic import BaseModel, Field
from datetime import datetime
from typing import Optional
from uuid import UUID


class UploadResponse(BaseModel):
    """Response model for upload endpoint."""
    upload_id: UUID
    filename: str
    status: str
    uploaded_at: datetime
    size: int
    file_path: str


class MapMetadata(BaseModel):
    """Map metadata model."""
    size: int
    # Tạm thời: Chỉ có size. Các fields khác sẽ thêm sau


class CurrentMapResponse(BaseModel):
    """Response model for current map."""
    upload_id: UUID
    filename: str
    status: str
    metadata: MapMetadata
    uploaded_at: datetime
    file_path: str
    is_current: bool
    processed_at: Optional[datetime] = None

