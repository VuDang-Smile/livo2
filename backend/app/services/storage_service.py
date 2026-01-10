"""File storage management service."""
import os
import shutil
import logging
from pathlib import Path
from typing import Optional
from app.config import settings

logger = logging.getLogger(__name__)


class StorageService:
    """Service for managing file storage."""
    
    def __init__(self):
        self.upload_dir = Path(settings.UPLOAD_DIR)
        self.processed_dir = Path(settings.PROCESSED_DIR)
        self.storage_dir = Path(settings.STORAGE_DIR)
        self._ensure_directories()
    
    def _ensure_directories(self):
        """Ensure upload and processed directories exist."""
        self.upload_dir.mkdir(parents=True, exist_ok=True)
        self.processed_dir.mkdir(parents=True, exist_ok=True)
        self.ensure_storage_directories()
        logger.info(f"Storage directories initialized: upload={self.upload_dir}, processed={self.processed_dir}, storage={self.storage_dir}")
    
    def get_upload_path(self, filename: str) -> Path:
        """Get full path for uploaded file."""
        return self.upload_dir / filename
    
    def get_processed_path(self, *paths: str) -> Path:
        """Get full path in processed directory."""
        return self.processed_dir / Path(*paths)
    
    def get_storage_path(self, *paths: str) -> Path:
        """Get full path in storage directory."""
        return self.storage_dir / Path(*paths)
    
    def ensure_storage_directories(self):
        """Ensure storage subdirectories exist."""
        self.storage_dir.mkdir(parents=True, exist_ok=True)
        (self.storage_dir / "merged_map").mkdir(exist_ok=True)
        (self.storage_dir / "fastloc_map").mkdir(exist_ok=True)
        (self.storage_dir / "floorplan_2d").mkdir(exist_ok=True)
        logger.debug(f"Storage subdirectories ensured: {self.storage_dir}")
    
    def cleanup_storage(self) -> bool:
        """Cleanup all files in storage directory."""
        try:
            if self.storage_dir.exists():
                for item in self.storage_dir.iterdir():
                    if item.is_file():
                        item.unlink()
                        logger.info(f"Deleted file from storage: {item}")
                    elif item.is_dir():
                        shutil.rmtree(item)
                        logger.info(f"Deleted directory from storage: {item}")
                # Recreate subdirectories
                self.ensure_storage_directories()
            return True
        except Exception as e:
            logger.error(f"Error cleaning up storage: {e}")
            return False
    
    def save_file(self, file_content: bytes, filename: str) -> Path:
        """Save file to upload directory."""
        file_path = self.get_upload_path(filename)
        with open(file_path, 'wb') as f:
            f.write(file_content)
        logger.info(f"File saved: {file_path}")
        return file_path
    
    def delete_file(self, file_path: Path) -> bool:
        """Delete a file."""
        try:
            if file_path.exists():
                file_path.unlink()
                logger.info(f"File deleted: {file_path}")
                return True
            return False
        except Exception as e:
            logger.error(f"Error deleting file {file_path}: {e}")
            return False
    
    def cleanup_uploads(self) -> bool:
        """Cleanup all files in uploads directory (single version constraint)."""
        try:
            if self.upload_dir.exists():
                for file_path in self.upload_dir.iterdir():
                    if file_path.is_file():
                        file_path.unlink()
                        logger.info(f"Deleted old upload: {file_path}")
            return True
        except Exception as e:
            logger.error(f"Error cleaning up uploads: {e}")
            return False
    
    def cleanup_processed(self) -> bool:
        """Cleanup all files in processed directory."""
        try:
            if self.processed_dir.exists():
                for item in self.processed_dir.iterdir():
                    if item.is_file():
                        item.unlink()
                    elif item.is_dir():
                        shutil.rmtree(item)
                        logger.info(f"Deleted processed directory: {item}")
            return True
        except Exception as e:
            logger.error(f"Error cleaning up processed: {e}")
            return False
    
    def file_exists(self, file_path: Path) -> bool:
        """Check if file exists."""
        return file_path.exists() and file_path.is_file()
    
    def get_file_size(self, file_path: Path) -> Optional[int]:
        """Get file size in bytes."""
        try:
            if file_path.exists():
                return file_path.stat().st_size
            return None
        except Exception as e:
            logger.error(f"Error getting file size for {file_path}: {e}")
            return None


# Global storage service instance
storage_service = StorageService()

