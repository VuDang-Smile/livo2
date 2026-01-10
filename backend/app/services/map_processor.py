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
    
    async def extract_zip_to_storage(self, zip_file_path: Path) -> Optional[Dict[str, Any]]:
        """
        Extract ZIP file directly to storage without processing metadata.
        Used for auto-extraction on startup or manual extraction.
        """
        try:
            if not zip_file_path.exists():
                logger.error(f"ZIP file not found: {zip_file_path}")
                return None
            
            if not zip_file_path.name.endswith('.zip'):
                logger.error(f"File is not a ZIP: {zip_file_path.name}")
                return None
            
            logger.info("=" * 60)
            logger.info(f"📦 EXTRACTING ZIP TO STORAGE: {zip_file_path.name}")
            logger.info("=" * 60)
            
            # Extract to temp first
            temp_dir = self._extract_zip(zip_file_path)
            if not temp_dir:
                logger.error("Failed to extract ZIP to temp directory")
                return None
            
            # Extract to storage
            try:
                storage_paths = self._extract_to_storage(temp_dir)
                if not storage_paths:
                    logger.error("No files extracted to storage")
                    if temp_dir:
                        shutil.rmtree(temp_dir, ignore_errors=True)
                    return None
                
                logger.info(f"✅ Successfully extracted {len(storage_paths)} items to storage")
                return {"storage_paths": storage_paths}
            finally:
                # Cleanup temp
                if temp_dir:
                    shutil.rmtree(temp_dir, ignore_errors=True)
                    
        except Exception as e:
            logger.error(f"Error extracting ZIP to storage: {e}", exc_info=True)
            return None
    
    def _extract_to_storage(self, temp_dir: Path) -> Dict[str, Any]:
        """Extract folders/files from temp to storage directory."""
        storage_paths = {}
        
        try:
            logger.info(f"Starting extraction to storage from temp directory: {temp_dir}")
            
            # Cleanup old storage first
            logger.info("Cleaning up old storage...")
            storage_service.cleanup_storage()
            
            # Ensure storage directories exist
            storage_service.ensure_storage_directories()
            
            # Copy merged_map
            merged_map_source = None
            # Try root first
            if (temp_dir / "merged_map").exists():
                merged_map_source = temp_dir / "merged_map"
                logger.info(f"Found merged_map at root: {merged_map_source}")
            # Try Log/merged_map
            elif (temp_dir / "Log" / "merged_map").exists():
                merged_map_source = temp_dir / "Log" / "merged_map"
                logger.info(f"Found merged_map in Log/: {merged_map_source}")
            
            if merged_map_source:
                merged_map_dest = storage_service.get_storage_path("merged_map")
                if merged_map_dest.exists():
                    shutil.rmtree(merged_map_dest)
                shutil.copytree(merged_map_source, merged_map_dest, dirs_exist_ok=True)
                storage_paths['merged_map'] = str(merged_map_dest)
                logger.info(f"✅ Copied merged_map to storage: {merged_map_dest}")
            else:
                logger.warning("⚠️ merged_map not found in ZIP")
            
            # Copy fastloc_map
            fastloc_map_source = None
            if (temp_dir / "fastloc_map").exists():
                fastloc_map_source = temp_dir / "fastloc_map"
                logger.info(f"Found fastloc_map at root: {fastloc_map_source}")
            elif (temp_dir / "Log" / "fastloc_map").exists():
                fastloc_map_source = temp_dir / "Log" / "fastloc_map"
                logger.info(f"Found fastloc_map in Log/: {fastloc_map_source}")
            
            if fastloc_map_source:
                fastloc_map_dest = storage_service.get_storage_path("fastloc_map")
                if fastloc_map_dest.exists():
                    shutil.rmtree(fastloc_map_dest)
                shutil.copytree(fastloc_map_source, fastloc_map_dest, dirs_exist_ok=True)
                storage_paths['fastloc_map'] = str(fastloc_map_dest)
                logger.info(f"✅ Copied fastloc_map to storage: {fastloc_map_dest}")
            else:
                logger.warning("⚠️ fastloc_map not found in ZIP")
            
            # Copy floorplan_2d
            floorplan_source = None
            if (temp_dir / "floorplan_2d").exists():
                floorplan_source = temp_dir / "floorplan_2d"
                logger.info(f"Found floorplan_2d at root: {floorplan_source}")
            elif (temp_dir / "Log" / "floorplan_2d").exists():
                floorplan_source = temp_dir / "Log" / "floorplan_2d"
                logger.info(f"Found floorplan_2d in Log/: {floorplan_source}")
            
            if floorplan_source:
                floorplan_dest = storage_service.get_storage_path("floorplan_2d")
                if floorplan_dest.exists():
                    shutil.rmtree(floorplan_dest)
                shutil.copytree(floorplan_source, floorplan_dest, dirs_exist_ok=True)
                storage_paths['floorplan_2d'] = str(floorplan_dest)
                logger.info(f"✅ Copied floorplan_2d to storage: {floorplan_dest}")
            else:
                logger.warning("⚠️ floorplan_2d not found in ZIP")
            
            # Copy QR_detect.json
            qr_source = None
            if (temp_dir / "QR_detect.json").exists():
                qr_source = temp_dir / "QR_detect.json"
                logger.info(f"Found QR_detect.json at root: {qr_source}")
            elif (temp_dir / "Log" / "QR_detect.json").exists():
                qr_source = temp_dir / "Log" / "QR_detect.json"
                logger.info(f"Found QR_detect.json in Log/: {qr_source}")
            
            if qr_source:
                qr_dest = storage_service.get_storage_path("QR_detect.json")
                shutil.copy2(qr_source, qr_dest)
                storage_paths['qr_detect'] = str(qr_dest)
                logger.info(f"✅ Copied QR_detect.json to storage: {qr_dest}")
            else:
                logger.warning("⚠️ QR_detect.json not found in ZIP")
            
            logger.info(f"✅ Extraction to storage completed. Extracted {len(storage_paths)} items: {list(storage_paths.keys())}")
            return storage_paths
        except Exception as e:
            logger.error(f"❌ Error extracting to storage: {e}", exc_info=True)
            raise  # Re-raise to fail the upload if extraction fails
    
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
                    
                    # Cleanup old storage (will be done in _extract_to_storage, but doing it here too for safety)
                    storage_service.cleanup_storage()
                    
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
            storage_paths = {}
            
            if filename.endswith('.zip'):
                logger.info("=" * 60)
                logger.info(f"📦 PROCESSING ZIP FILE: {filename}")
                logger.info("=" * 60)
                
                # Extract ZIP to temp directory
                logger.info("Step 1/2: Extracting ZIP to temporary directory...")
                temp_dir = self._extract_zip(file_path)
                if not temp_dir:
                    logger.error("❌ FAILED: Could not extract ZIP file to temp directory")
                    storage_service.delete_file(file_path)
                    return None
                logger.info(f"✅ ZIP extracted to temp: {temp_dir}")
                
                # Extract to storage directory (REQUIRED - must succeed)
                logger.info("=" * 60)
                logger.info("Step 2/2: EXTRACTING FILES TO STORAGE (REQUIRED)")
                logger.info("=" * 60)
                try:
                    storage_paths = self._extract_to_storage(temp_dir)
                    if not storage_paths:
                        logger.error("❌ FAILED: No files extracted to storage!")
                        logger.error("   This means the ZIP file structure is invalid or extraction failed.")
                        storage_service.delete_file(file_path)
                        if temp_dir:
                            shutil.rmtree(temp_dir, ignore_errors=True)
                        return None
                    
                    # Verify at least one folder was extracted
                    required_folders = ["merged_map", "fastloc_map", "floorplan_2d"]
                    extracted_folders = [k for k in storage_paths.keys() if k in required_folders]
                    if not extracted_folders:
                        logger.error("❌ FAILED: No required folders (merged_map, fastloc_map, floorplan_2d) were extracted!")
                        storage_service.delete_file(file_path)
                        if temp_dir:
                            shutil.rmtree(temp_dir, ignore_errors=True)
                        return None
                    
                    logger.info("=" * 60)
                    logger.info(f"✅ SUCCESS: Extracted {len(storage_paths)} items to storage")
                    logger.info(f"   Folders: {', '.join(extracted_folders)}")
                    logger.info("=" * 60)
                except Exception as e:
                    logger.error("=" * 60)
                    logger.error(f"❌ FAILED: Exception during storage extraction: {e}")
                    logger.error("=" * 60)
                    import traceback
                    logger.error(traceback.format_exc())
                    storage_service.delete_file(file_path)
                    if temp_dir:
                        shutil.rmtree(temp_dir, ignore_errors=True)
                    return None
                
                # Get PCD file from storage (no longer copy to processed)
                pcd_path = None
                merged_pcd = storage_service.get_storage_path("merged_map", "merged_all.pcd")
                if merged_pcd.exists():
                    pcd_path = merged_pcd
                    logger.info(f"Using merged_all.pcd from storage: {pcd_path}")
                else:
                    logger.warning("No merged_all.pcd found in storage")
                
                # Find floorplan files (for backward compatibility with old format)
                logger.info("Looking for floorplan files in ZIP...")
                floorplan_info = self._find_floorplan_in_zip(temp_dir)
                
                # Extract floorplan files to processed directory (for backward compatibility)
                map_result = None
                if floorplan_info:
                    logger.info("Extracting floorplan files to processed directory...")
                    map_result = self._extract_floorplan_files(floorplan_info, upload_id)
                else:
                    # Try to find floorplan_2d in storage
                    floorplan_2d_dir = storage_service.get_storage_path("floorplan_2d")
                    if floorplan_2d_dir.exists():
                        logger.info("Found floorplan_2d in storage, using it...")
                        # Create minimal map_result for backward compatibility
                        map_result = {
                            'image_paths': {},
                            'metadata_path': None,
                            'map_metadata': {}
                        }
                    else:
                        logger.warning("No floorplan files found in ZIP or storage")
                
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
                "pcd_path": str(pcd_path) if pcd_path else None,
                "image_paths": map_result["image_paths"] if map_result else {},
                "metadata_path": map_result["metadata_path"] if map_result else None,
                "map_metadata": map_result["map_metadata"] if map_result else {},  # Full metadata JSON
                "storage_paths": storage_paths if filename.endswith('.zip') else {}  # Storage paths for new structure
            }
            
            # Log storage extraction summary
            if filename.endswith('.zip') and storage_paths:
                logger.info("=" * 60)
                logger.info("📦 STORAGE EXTRACTION SUMMARY")
                logger.info("=" * 60)
                for key, path in storage_paths.items():
                    storage_path_obj = Path(path)
                    if storage_path_obj.exists():
                        if storage_path_obj.is_file():
                            size = storage_path_obj.stat().st_size
                            logger.info(f"✅ {key}: {path} ({size / 1024 / 1024:.2f} MB)")
                        else:
                            file_count = len(list(storage_path_obj.rglob("*")))
                            logger.info(f"✅ {key}: {path} ({file_count} files)")
                    else:
                        logger.warning(f"⚠️ {key}: {path} (NOT FOUND)")
                logger.info("=" * 60)
            
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

