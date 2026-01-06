"""Map metadata models."""
from pydantic import BaseModel
from typing import Optional, Dict, Any


class MapMetadata(BaseModel):
    """Map metadata model."""
    size: int
    # Tạm thời: Chỉ có size. Các fields khác sẽ thêm sau khi có processor

