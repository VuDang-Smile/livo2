"""Upload API endpoints."""
import logging
from uuid import UUID
from fastapi import APIRouter, HTTPException, UploadFile, File
from app.models.upload import UploadResponse, CurrentMapResponse, MapMetadata
from app.services.map_processor import MapProcessor
from app.services.database_service import database_service
from app.services.mqtt_service import mqtt_service
from app.config import settings

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/maps", tags=["maps"])
map_processor = MapProcessor()


@router.post("/upload", response_model=UploadResponse)
async def upload_map(file: UploadFile = File(...)):
    """
    Upload zip file (map output from Livo).
    
    Tạm thời: Chỉ nhận và lưu file, không process/extract/validate.
    """
    try:
        # Validate file type
        if not file.filename.endswith('.zip'):
            raise HTTPException(status_code=400, detail="Only zip files are allowed")
        
        # Read file content
        file_content = await file.read()
        file_size = len(file_content)
        
        # Validate file size
        if file_size > settings.MAX_UPLOAD_SIZE:
            raise HTTPException(
                status_code=413,
                detail=f"File size exceeds maximum allowed size of {settings.MAX_UPLOAD_SIZE} bytes"
            )
        
        # Publish MQTT event: upload started
        mqtt_service.publish_map_event("map.upload.started", {
            "filename": file.filename,
            "size": file_size
        })
        
        # Check for old map and publish event if exists
        old_map = await database_service.get_current_map()
        if old_map and settings.SINGLE_VERSION_MODE:
            mqtt_service.publish_map_event("map.old.removed", {
                "previous_upload_id": old_map.get("upload_id"),
                "reason": "new_map_uploaded"
            })
        
        # Process upload (save file and metadata)
        result = await map_processor.process_upload(file_content, file.filename)
        if not result:
            raise HTTPException(status_code=500, detail="Failed to process upload")
        
        # Publish MQTT events
        mqtt_service.publish_map_event("map.upload.completed", {
            "upload_id": result["upload_id"],
            "filename": result["filename"],
            "size": result["size"]
        })
        
        if old_map and settings.SINGLE_VERSION_MODE:
            mqtt_service.publish_map_event("map.replaced", {
                "new_upload_id": result["upload_id"],
                "previous_upload_id": old_map.get("upload_id")
            })
        
        return UploadResponse(
            upload_id=UUID(result["upload_id"]),
            filename=result["filename"],
            status=result["status"],
            uploaded_at=result["uploaded_at"],
            size=result["size"],
            file_path=result["file_path"]
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error uploading map: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/current", response_model=CurrentMapResponse)
async def get_current_map():
    """Get current map (chỉ có 1 map duy nhất)."""
    try:
        map_doc = await database_service.get_current_map()
        if not map_doc:
            raise HTTPException(status_code=404, detail="No map found")
        
        return CurrentMapResponse(
            upload_id=UUID(map_doc["upload_id"]),
            filename=map_doc["filename"],
            status=map_doc["status"],
            metadata=MapMetadata(**map_doc["metadata"]),
            uploaded_at=map_doc["uploaded_at"],
            file_path=map_doc["file_path"],
            is_current=map_doc["is_current"],
            processed_at=map_doc.get("processed_at")
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting current map: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("", response_model=CurrentMapResponse)
async def get_maps():
    """Alias for /current (backward compatibility)."""
    return await get_current_map()


@router.delete("/current")
async def delete_current_map():
    """Delete current map."""
    try:
        map_doc = await database_service.get_current_map()
        if not map_doc:
            raise HTTPException(status_code=404, detail="No map found")
        
        # Delete file
        if map_doc.get("file_path"):
            from pathlib import Path
            file_path = Path(map_doc["file_path"])
            from app.services.storage_service import storage_service
            storage_service.delete_file(file_path)
        
        # Delete from database
        success = await database_service.delete_current_map()
        if not success:
            raise HTTPException(status_code=500, detail="Failed to delete map")
        
        return {"success": True, "message": "Map deleted successfully"}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting current map: {e}")
        raise HTTPException(status_code=500, detail=str(e))

