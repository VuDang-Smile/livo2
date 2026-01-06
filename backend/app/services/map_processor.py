"""Map processing logic including PCD to 2D map conversion."""
import logging
import subprocess
import json
import zipfile
import tempfile
import shutil
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, Any
from uuid import uuid4
from app.services.storage_service import storage_service
from app.services.database_service import database_service
from app.config import settings

logger = logging.getLogger(__name__)


class MapProcessor:
    """Service for processing uploaded maps."""
    
    def _find_pcd_in_zip(self, zip_path: Path) -> Optional[Path]:
        """Extract ZIP and find PCD file."""
        try:
            with tempfile.TemporaryDirectory() as temp_dir:
                with zipfile.ZipFile(zip_path, 'r') as zip_ref:
                    zip_ref.extractall(temp_dir)
                
                # Find PCD file
                temp_path = Path(temp_dir)
                pcd_files = list(temp_path.rglob("*.pcd"))
                
                if not pcd_files:
                    logger.warning(f"No PCD file found in ZIP: {zip_path}")
                    return None
                
                if len(pcd_files) > 1:
                    logger.warning(f"Multiple PCD files found, using first: {pcd_files[0]}")
                
                # Copy PCD to processed directory
                pcd_file = pcd_files[0]
                processed_pcd = storage_service.get_processed_path(pcd_file.name)
                shutil.copy2(pcd_file, processed_pcd)
                logger.info(f"Extracted PCD file: {processed_pcd}")
                
                return processed_pcd
        except Exception as e:
            logger.error(f"Error extracting PCD from ZIP: {e}")
            return None
    
    def _generate_2d_maps(self, pcd_path: Path, output_dir: Path) -> Optional[Dict[str, Any]]:
        """Call Python script to generate 3 2D map views."""
        try:
            # Get path to Python script
            script_path = Path(__file__).parent.parent.parent / "example" / "pcd_to_floorplan.py"
            
            if not script_path.exists():
                logger.error(f"Python script not found: {script_path}")
                return None
            
            # Import the function directly
            import sys
            sys.path.insert(0, str(script_path.parent))
            
            from pcd_to_floorplan import pcd_to_3_views
            
            # Generate 3 views
            result = pcd_to_3_views(
                str(pcd_path),
                output_dir=str(output_dir),
                resolution=0.05,
                colormap='binary',
                invert_colors=True,
                auto_crop=True,
                crop_margin=5,
                border_margin=20,
                outlier_filter=True,
                outlier_percentile=1.0
            )
            
            logger.info(f"Generated 3 views successfully: {result['image_paths']}")
            return result
            
        except ImportError as e:
            logger.error(f"Failed to import pcd_to_floorplan: {e}")
            # Fallback: try calling as subprocess
            try:
                script_path = Path(__file__).parent.parent.parent / "example" / "pcd_to_floorplan.py"
                python_cmd = ["python3", str(script_path)]
                
                # This won't work directly, need to modify script to support 3 views via CLI
                # For now, return None and log error
                logger.error("Subprocess call not implemented yet. Please ensure pcd_to_floorplan.py is importable.")
                return None
            except Exception as e2:
                logger.error(f"Subprocess fallback also failed: {e2}")
                return None
        except Exception as e:
            logger.error(f"Error generating 2D maps: {e}")
            return None
    
    async def process_upload(
        self,
        file_content: bytes,
        filename: str
    ) -> Optional[Dict[str, Any]]:
        """
        Process uploaded map file.
        
        - If ZIP: Extract and find PCD file
        - Generate 3 2D map views (top, side_x, side_y)
        - Save metadata and image paths
        """
        try:
            # Check and delete old map if single version mode
            if settings.SINGLE_VERSION_MODE:
                old_map = await database_service.get_current_map()
                if old_map:
                    logger.warning(f"Replacing old map: {old_map.get('upload_id')}")
                    # Delete old files
                    if old_map.get("file_path"):
                        old_file_path = Path(old_map["file_path"])
                        storage_service.delete_file(old_file_path)
                    # Delete old processed files
                    if old_map.get("metadata", {}).get("image_paths"):
                        for view_path in old_map["metadata"]["image_paths"].values():
                            storage_service.delete_file(Path(view_path))
                    # Delete old map from database
                    await database_service.delete_current_map()
            
            # Save uploaded file
            file_path = storage_service.save_file(file_content, filename)
            file_size = len(file_content)
            
            # Find PCD file
            pcd_path = None
            if filename.endswith('.zip'):
                logger.info(f"Extracting ZIP file: {filename}")
                pcd_path = self._find_pcd_in_zip(file_path)
                if not pcd_path:
                    logger.error("Failed to extract PCD from ZIP")
                    storage_service.delete_file(file_path)
                    return None
            elif filename.endswith('.pcd'):
                # Direct PCD file
                pcd_path = file_path
            else:
                logger.error(f"Unsupported file type: {filename}")
                storage_service.delete_file(file_path)
                return None
            
            # Generate 3 2D map views
            upload_id = str(uuid4())
            output_dir = storage_service.get_processed_path(str(upload_id))
            output_dir.mkdir(parents=True, exist_ok=True)
            
            logger.info(f"Generating 2D maps for PCD: {pcd_path}")
            map_result = self._generate_2d_maps(pcd_path, output_dir)
            
            if not map_result:
                logger.error("Failed to generate 2D maps")
                storage_service.delete_file(file_path)
                if pcd_path != file_path:
                    storage_service.delete_file(pcd_path)
                return None
            
            # Prepare metadata
            metadata = {
                "size": file_size,
                "pcd_path": str(pcd_path),
                "image_paths": {
                    view_id: str(Path(path).relative_to(storage_service.processed_dir))
                    for view_id, path in map_result["image_paths"].items()
                },
                "metadata_path": str(Path(map_result["metadata_path"]).relative_to(storage_service.processed_dir)),
                "map_metadata": map_result["metadata"]  # Full metadata JSON
            }
            
            # Create map document
            map_data = {
                "upload_id": upload_id,
                "filename": filename,
                "status": "processed",
                "is_current": True,
                "metadata": metadata,
                "file_path": str(file_path),
                "uploaded_at": datetime.utcnow(),
                "processed_at": datetime.utcnow(),
                "created_at": datetime.utcnow(),
                "updated_at": datetime.utcnow()
            }
            
            # Save to database
            map_id = await database_service.create_map(map_data)
            if not map_id:
                logger.error("Failed to save map to database")
                # Cleanup files
                storage_service.delete_file(file_path)
                if pcd_path != file_path:
                    storage_service.delete_file(pcd_path)
                return None
            
            logger.info(f"Map processed successfully: {upload_id}")
            
            return {
                "upload_id": upload_id,
                "filename": filename,
                "status": "processed",
                "uploaded_at": map_data["uploaded_at"],
                "processed_at": map_data["processed_at"],
                "size": file_size,
                "file_path": str(file_path),
                "metadata": metadata
            }
        except Exception as e:
            logger.error(f"Error processing map upload: {e}", exc_info=True)
            return None

