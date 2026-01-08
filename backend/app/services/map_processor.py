"""Map processing logic (minimal implementation - just save file)."""
import logging
from datetime import datetime
from typing import Optional, Dict, Any
from uuid import uuid4
from app.services.storage_service import storage_service
from app.services.database_service import database_service
from app.config import settings

logger = logging.getLogger(__name__)


class MapProcessor:
    """Service for processing uploaded maps."""
    
    async def process_upload(
        self,
        file_content: bytes,
        filename: str
    ) -> Optional[Dict[str, Any]]:
        """
        Process uploaded map file.
        
        Tạm thời: Chỉ lưu file, không extract/validate.
        """
        try:
            # Check and delete old map if single version mode
            if settings.SINGLE_VERSION_MODE:
                old_map = await database_service.get_current_map()
                if old_map:
                    logger.warning(f"Replacing old map: {old_map.get('upload_id')}")
                    # Delete old file
                    if old_map.get("file_path"):
                        from pathlib import Path
                        old_file_path = Path(old_map["file_path"])
                        storage_service.delete_file(old_file_path)
                    # Delete old map from database
                    await database_service.delete_current_map()
            
            # Save new file
            file_path = storage_service.save_file(file_content, filename)
            file_size = len(file_content)
            
            # Create map document
            upload_id = str(uuid4())
            map_data = {
                "upload_id": upload_id,
                "filename": filename,
                "status": "uploaded",
                "is_current": True,
                "metadata": {
                    "size": file_size
                },
                "file_path": str(file_path),
                "uploaded_at": datetime.utcnow(),
                "processed_at": None,
                "created_at": datetime.utcnow(),
                "updated_at": datetime.utcnow()
            }
            
            # Save to database
            map_id = await database_service.create_map(map_data)
            if not map_id:
                logger.error("Failed to save map to database")
                # Cleanup file
                storage_service.delete_file(file_path)
                return None
            
            logger.info(f"Map processed successfully: {upload_id}")
            
            return {
                "upload_id": upload_id,
                "filename": filename,
                "status": "uploaded",
                "uploaded_at": map_data["uploaded_at"],
                "size": file_size,
                "file_path": str(file_path)
            }
        except Exception as e:
            logger.error(f"Error processing map upload: {e}")
            return None

