"""Map processing logic including map extraction and processing."""
import logging
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
    
    def _extract_zip(self, zip_path: Path) -> Optional[Path]:
        """Extract ZIP file to temporary directory and return temp_dir path."""
        temp_dir = None
        temp_path = None
        try:
            temp_dir = tempfile.mkdtemp()
            temp_path = Path(temp_dir)
            
            with zipfile.ZipFile(zip_path, 'r') as zip_ref:
                zip_ref.extractall(temp_dir)
            
            logger.info(f"Extracted ZIP to temporary directory: {temp_dir}")
            return temp_path
        except Exception as e:
            logger.error(f"Error extracting ZIP: {e}")
            # Cleanup temp directory if extraction failed
            if temp_dir and temp_path and temp_path.exists():
                try:
                    shutil.rmtree(temp_dir, ignore_errors=True)
                    logger.info(f"Cleaned up temporary directory: {temp_dir}")
                except Exception as cleanup_error:
                    logger.warning(f"Error cleaning up temp directory {temp_dir}: {cleanup_error}")
            return None
    
    def _find_pcd_in_extracted(self, temp_dir: Path) -> Optional[Path]:
        """Find PCD file in extracted ZIP directory and copy to processed directory."""
        try:
            # Find PCD file
            pcd_files = list(temp_dir.rglob("*.pcd"))
            
            if not pcd_files:
                logger.warning(f"No PCD file found in extracted ZIP")
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
            logger.error(f"Error finding PCD in extracted ZIP: {e}")
            return None
    
    def _find_floorplan_in_zip(self, temp_dir: Path) -> Optional[Dict[str, Any]]:
        """Find floorplan files in extracted ZIP directory."""
        try:
            # Tìm thư mục floorplan (có thể ở root hoặc trong Log/floorplan/)
            floorplan_dirs = []
            
            # Tìm trong root
            root_floorplan = temp_dir / "floorplan"
            if root_floorplan.exists() and root_floorplan.is_dir():
                floorplan_dirs.append(root_floorplan)
            
            # Tìm trong Log/floorplan/
            log_floorplan = temp_dir / "Log" / "floorplan"
            if log_floorplan.exists() and log_floorplan.is_dir():
                floorplan_dirs.append(log_floorplan)
            
            if not floorplan_dirs:
                logger.warning("No floorplan directory found in ZIP")
                return None
            
            # Sử dụng thư mục đầu tiên tìm thấy
            floorplan_dir = floorplan_dirs[0]
            logger.info(f"Found floorplan directory: {floorplan_dir}")
            
            # Tìm các file PNG và JSON metadata
            png_files = {
                'top': None,
                'side_x': None,
                'side_y': None
            }
            metadata_file = None
            
            for file_path in floorplan_dir.iterdir():
                if file_path.is_file():
                    file_name = file_path.name.lower()
                    if file_name.endswith('_top.png'):
                        png_files['top'] = file_path
                    elif file_name.endswith('_side_x.png'):
                        png_files['side_x'] = file_path
                    elif file_name.endswith('_side_y.png'):
                        png_files['side_y'] = file_path
                    elif file_name.endswith('_metadata.json'):
                        metadata_file = file_path
            
            # Validate đủ files
            missing_files = []
            if not png_files['top']:
                missing_files.append('*_top.png')
            if not png_files['side_x']:
                missing_files.append('*_side_x.png')
            if not png_files['side_y']:
                missing_files.append('*_side_y.png')
            if not metadata_file:
                missing_files.append('*_metadata.json')
            
            if missing_files:
                logger.error(f"Missing floorplan files: {missing_files}")
                return None
            
            # Extract base name từ metadata file
            base_name = metadata_file.stem.replace('_metadata', '')
            
            logger.info(f"Found all floorplan files. Base name: {base_name}")
            
            return {
                'image_paths': {
                    'top': png_files['top'],
                    'side_x': png_files['side_x'],
                    'side_y': png_files['side_y']
                },
                'metadata_path': metadata_file,
                'base_name': base_name
            }
        except Exception as e:
            logger.error(f"Error finding floorplan in extracted ZIP: {e}")
            return None
    
    def _extract_floorplan_files(self, floorplan_info: Dict[str, Any], upload_id: str) -> Optional[Dict[str, Any]]:
        """Copy floorplan files from temp directory to processed directory."""
        try:
            output_dir = storage_service.get_processed_path(upload_id)
            output_dir.mkdir(parents=True, exist_ok=True)
            
            # Copy image files
            image_paths = {}
            for view_id, source_path in floorplan_info['image_paths'].items():
                dest_file = output_dir / source_path.name
                shutil.copy2(source_path, dest_file)
                image_paths[view_id] = str(Path(upload_id) / dest_file.name)
                logger.info(f"Copied {view_id} image: {dest_file}")
            
            # Copy metadata file
            metadata_source = floorplan_info['metadata_path']
            metadata_dest = output_dir / metadata_source.name
            shutil.copy2(metadata_source, metadata_dest)
            metadata_path = str(Path(upload_id) / metadata_dest.name)
            logger.info(f"Copied metadata: {metadata_dest}")
            
            # Read metadata JSON
            with open(metadata_dest, 'r') as f:
                map_metadata = json.load(f)
            
            return {
                'image_paths': image_paths,
                'metadata_path': metadata_path,
                'map_metadata': map_metadata
            }
        except Exception as e:
            logger.error(f"Error extracting floorplan files: {e}")
            return None
    
    async def process_upload(
        self,
        file_content: bytes,
        filename: str
    ) -> Optional[Dict[str, Any]]:
        """
        Process uploaded map file.
        
        - If ZIP: Extract ZIP, find PCD file and floorplan files
        - Extract floorplan files (images + metadata) from ZIP to processed directory
        - Reject ZIP if floorplan files are missing (no fallback generation)
        - Direct PCD upload is not supported (must be in ZIP with floorplan)
        - Save metadata and image paths to database
        """
        try:
            # Check and delete old map if single version mode
            if settings.SINGLE_VERSION_MODE:
                old_map = await database_service.get_current_map()
                if old_map:
                    logger.warning(f"Replacing old map: {old_map.get('upload_id')}")
                    old_metadata = old_map.get("metadata", {})
                    
                    # Delete old uploaded file
                    if old_map.get("file_path"):
                        old_file_path = Path(old_map["file_path"])
                        storage_service.delete_file(old_file_path)
                    
                    # Delete old processed files (images, PCD, metadata)
                    if old_metadata.get("image_paths"):
                        for view_id, rel_path in old_metadata["image_paths"].items():
                            image_path = storage_service.get_processed_path(rel_path)
                            storage_service.delete_file(image_path)
                    
                    # Delete old PCD file
                    if old_metadata.get("pcd_path"):
                        old_pcd_path = Path(old_metadata["pcd_path"])
                        storage_service.delete_file(old_pcd_path)
                    
                    # Delete old metadata file
                    if old_metadata.get("metadata_path"):
                        old_metadata_path = storage_service.get_processed_path(old_metadata["metadata_path"])
                        storage_service.delete_file(old_metadata_path)
                    
                    # Delete old map from database
                    await database_service.delete_current_map()
            
            # Save uploaded file
            file_path = storage_service.save_file(file_content, filename)
            file_size = len(file_content)
            
            # Process ZIP file
            pcd_path = None
            upload_id = str(uuid4())
            temp_dir = None
            map_result = None
            
            if filename.endswith('.zip'):
                logger.info(f"Extracting ZIP file: {filename}")
                
                # Extract ZIP to temp directory
                temp_dir = self._extract_zip(file_path)
                if not temp_dir:
                    logger.error("Failed to extract ZIP file")
                    storage_service.delete_file(file_path)
                    return None
                
                # Find PCD file
                pcd_path = self._find_pcd_in_extracted(temp_dir)
                if not pcd_path:
                    logger.error("Failed to find PCD file in ZIP")
                    storage_service.delete_file(file_path)
                    if temp_dir:
                        shutil.rmtree(temp_dir, ignore_errors=True)
                    return None
                
                # Find floorplan files
                logger.info("Looking for floorplan files in ZIP...")
                floorplan_info = self._find_floorplan_in_zip(temp_dir)
                
                if not floorplan_info:
                    logger.error("ZIP does not contain floorplan files. Please regenerate ZIP with floorplan.")
                    storage_service.delete_file(file_path)
                    storage_service.delete_file(pcd_path)
                    if temp_dir:
                        shutil.rmtree(temp_dir, ignore_errors=True)
                    return None
                
                # Extract floorplan files to processed directory
                logger.info("Extracting floorplan files...")
                map_result = self._extract_floorplan_files(floorplan_info, upload_id)
                
                if not map_result:
                    logger.error("Failed to extract floorplan files")
                    storage_service.delete_file(file_path)
                    storage_service.delete_file(pcd_path)
                    if temp_dir:
                        shutil.rmtree(temp_dir, ignore_errors=True)
                    return None
                
                # Cleanup temp directory
                if temp_dir:
                    try:
                        shutil.rmtree(temp_dir)
                    except Exception as e:
                        logger.warning(f"Error cleaning up temp directory: {e}")
                
            elif filename.endswith('.pcd'):
                # Direct PCD file - not supported anymore
                logger.error("Direct PCD upload not supported. Please upload ZIP file with floorplan.")
                storage_service.delete_file(file_path)
                return None
            else:
                logger.error(f"Unsupported file type: {filename}")
                storage_service.delete_file(file_path)
                return None
            
            # Prepare metadata
            metadata = {
                "size": file_size,
                "pcd_path": str(pcd_path),
                "image_paths": map_result["image_paths"],
                "metadata_path": map_result["metadata_path"],
                "map_metadata": map_result["map_metadata"]  # Full metadata JSON
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
                
                # Cleanup floorplan files (images + metadata) that were already extracted
                if map_result:
                    # Delete floorplan image files
                    if map_result.get("image_paths"):
                        for view_id, rel_path in map_result["image_paths"].items():
                            image_path = storage_service.get_processed_path(rel_path)
                            storage_service.delete_file(image_path)
                            logger.info(f"Cleaned up floorplan image ({view_id}): {image_path}")
                    
                    # Delete metadata file
                    if map_result.get("metadata_path"):
                        metadata_path = storage_service.get_processed_path(map_result["metadata_path"])
                        storage_service.delete_file(metadata_path)
                        logger.info(f"Cleaned up metadata file: {metadata_path}")
                    
                    # Delete upload_id directory if it exists and is empty
                    upload_dir = storage_service.get_processed_path(upload_id)
                    try:
                        if upload_dir.exists() and upload_dir.is_dir():
                            # Try to remove directory (will fail if not empty, which is fine)
                            upload_dir.rmdir()
                            logger.info(f"Cleaned up upload directory: {upload_dir}")
                    except Exception as e:
                        # Directory not empty or other error - that's okay, individual files were deleted
                        logger.debug(f"Could not remove upload directory {upload_dir}: {e}")
                
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

