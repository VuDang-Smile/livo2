"""QR Code service for managing QR codes in JSON file."""
import json
import logging
from pathlib import Path
from typing import List, Optional
from app.models.qrcode import QRCode
from app.services.storage_service import storage_service

logger = logging.getLogger(__name__)


class QRCodeService:
    """Service for managing QR codes stored in JSON file."""
    
    def __init__(self):
        self.file_path = storage_service.get_storage_path("web_qrcode.json")
        self._ensure_file_exists()
    
    def _ensure_file_exists(self):
        """Ensure the QR code file exists, create empty array if not."""
        if not self.file_path.exists():
            try:
                storage_service.ensure_storage_directories()
                self.file_path.write_text(
                    json.dumps([], ensure_ascii=False, indent=2),
                    encoding="utf-8"
                )
                logger.info(f"Created QR code file at {self.file_path}")
            except Exception as e:
                logger.error(f"Error creating QR code file: {e}")
                raise
    
    def _read_qrcodes(self) -> List[dict]:
        """Read QR codes from file, return list of dicts."""
        try:
            if not self.file_path.exists():
                return []
            
            content = self.file_path.read_text(encoding="utf-8")
            if not content.strip():
                return []
            
            data = json.loads(content)
            if not isinstance(data, list):
                logger.warning("QR code file is not a list, converting to list")
                return []
            
            return data
        except json.JSONDecodeError as e:
            logger.error(f"Error parsing QR code file: {e}")
            raise ValueError(f"Invalid JSON in QR code file: {e}")
        except Exception as e:
            logger.error(f"Error reading QR code file: {e}")
            raise
    
    def _write_qrcodes(self, qrcodes: List[dict]) -> bool:
        """Write QR codes to file."""
        try:
            # Ensure directory exists
            storage_service.ensure_storage_directories()
            
            # Write with pretty formatting
            self.file_path.write_text(
                json.dumps(qrcodes, ensure_ascii=False, indent=2),
                encoding="utf-8"
            )
            logger.debug(f"Wrote {len(qrcodes)} QR codes to {self.file_path}")
            return True
        except Exception as e:
            logger.error(f"Error writing QR code file: {e}")
            raise
    
    def get_all(self) -> List[QRCode]:
        """Get all QR codes."""
        try:
            data = self._read_qrcodes()
            qrcodes = []
            for item in data:
                try:
                    qrcode = QRCode(**item)
                    qrcodes.append(qrcode)
                except Exception as e:
                    logger.warning(f"Skipping invalid QR code item: {item}, error: {e}")
                    continue
            return qrcodes
        except Exception as e:
            logger.error(f"Error getting all QR codes: {e}")
            raise
    
    def get_by_id(self, qr_id: str) -> Optional[QRCode]:
        """Get QR code by ID."""
        try:
            qrcodes = self.get_all()
            for qrcode in qrcodes:
                if qrcode.id == qr_id:
                    return qrcode
            return None
        except Exception as e:
            logger.error(f"Error getting QR code by ID {qr_id}: {e}")
            raise
    
    def create_or_update(self, qrcode: QRCode) -> QRCode:
        """Create new QR code or update existing one if ID matches."""
        try:
            data = self._read_qrcodes()
            
            # Find existing QR code with same ID
            found_index = None
            for i, item in enumerate(data):
                if item.get("id") == qrcode.id:
                    found_index = i
                    break
            
            # Convert QRCode to dict
            qrcode_dict = qrcode.dict()
            
            if found_index is not None:
                # Update existing
                data[found_index] = qrcode_dict
                logger.info(f"Updated QR code with ID: {qrcode.id}")
            else:
                # Add new
                data.append(qrcode_dict)
                logger.info(f"Created new QR code with ID: {qrcode.id}")
            
            # Write back to file
            self._write_qrcodes(data)
            
            return qrcode
        except Exception as e:
            logger.error(f"Error creating/updating QR code: {e}")
            raise
    
    def update(self, qr_id: str, qrcode_update: dict) -> Optional[QRCode]:
        """Update QR code by ID."""
        try:
            data = self._read_qrcodes()
            
            # Find QR code with matching ID
            found_index = None
            for i, item in enumerate(data):
                if item.get("id") == qr_id:
                    found_index = i
                    break
            
            if found_index is None:
                return None
            
            # Update fields (only update provided fields)
            existing_item = data[found_index]
            for key, value in qrcode_update.items():
                if value is not None:
                    existing_item[key] = value
            
            # Validate updated item
            updated_qrcode = QRCode(**existing_item)
            
            # Write back to file
            data[found_index] = updated_qrcode.dict()
            self._write_qrcodes(data)
            
            logger.info(f"Updated QR code with ID: {qr_id}")
            return updated_qrcode
        except Exception as e:
            logger.error(f"Error updating QR code {qr_id}: {e}")
            raise
    
    def delete(self, qr_id: str) -> bool:
        """Delete QR code by ID."""
        try:
            data = self._read_qrcodes()
            
            # Find and remove QR code with matching ID
            original_length = len(data)
            data = [item for item in data if item.get("id") != qr_id]
            
            if len(data) == original_length:
                # No item was removed
                return False
            
            # Write back to file
            self._write_qrcodes(data)
            logger.info(f"Deleted QR code with ID: {qr_id}")
            return True
        except Exception as e:
            logger.error(f"Error deleting QR code {qr_id}: {e}")
            raise


# Global QR code service instance
qrcode_service = QRCodeService()
