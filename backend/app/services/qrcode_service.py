"""QR Code service for managing QR codes in JSON file."""
import json
import logging
from pathlib import Path
from typing import List, Optional, Dict
from app.models.qrcode import QRCode
from app.services.storage_service import storage_service

logger = logging.getLogger(__name__)


class QRCodeService:
    """Service for managing QR codes stored in QR_detect.json file.
    
    File format: { "TM:0001": [x, y, z], "TM:0002": [x, y, z] }
    Only position [x, y, z] is stored in the file.
    """
    
    def __init__(self):
        self.file_path = storage_service.get_storage_path("QR_detect.json")
        self._ensure_file_exists()
    
    def _ensure_file_exists(self):
        """Ensure the QR code file exists, create empty object if not."""
        if not self.file_path.exists():
            try:
                storage_service.ensure_storage_directories()
                self.file_path.write_text(
                    json.dumps({}, ensure_ascii=False, indent=2),
                    encoding="utf-8"
                )
                logger.info(f"Created QR code file at {self.file_path}")
            except Exception as e:
                logger.error(f"Error creating QR code file: {e}")
                raise
    
    def _read_qrcodes(self) -> Dict[str, List[float]]:
        """Read QR codes from file, return dict with ID as key and position as value."""
        try:
            if not self.file_path.exists():
                return {}
            
            content = self.file_path.read_text(encoding="utf-8")
            if not content.strip():
                return {}
            
            data = json.loads(content)
            if not isinstance(data, dict):
                logger.warning("QR code file is not a dict, converting to dict")
                return {}
            
            # Validate format: each value should be a list of 3 numbers
            validated_data = {}
            for key, value in data.items():
                if isinstance(value, list) and len(value) == 3:
                    # Ensure all elements are numbers
                    if all(isinstance(v, (int, float)) for v in value):
                        validated_data[key] = value
                    else:
                        logger.warning(f"Invalid position format for {key}: {value}")
                else:
                    logger.warning(f"Invalid position format for {key}: {value}")
            
            return validated_data
        except json.JSONDecodeError as e:
            logger.error(f"Error parsing QR code file: {e}")
            raise ValueError(f"Invalid JSON in QR code file: {e}")
        except Exception as e:
            logger.error(f"Error reading QR code file: {e}")
            raise
    
    def _write_qrcodes(self, qrcodes_dict: Dict[str, List[float]]) -> bool:
        """Write QR codes to file. Only position [x, y, z] is written."""
        try:
            # Ensure directory exists
            storage_service.ensure_storage_directories()
            
            # Write with pretty formatting
            self.file_path.write_text(
                json.dumps(qrcodes_dict, ensure_ascii=False, indent=2),
                encoding="utf-8"
            )
            logger.debug(f"Wrote {len(qrcodes_dict)} QR codes to {self.file_path}")
            return True
        except Exception as e:
            logger.error(f"Error writing QR code file: {e}")
            raise
    
    def get_all_raw(self) -> Dict[str, List[float]]:
        """Get all QR codes in raw format (same as file format)."""
        return self._read_qrcodes()
    
    def get_all(self) -> List[QRCode]:
        """Get all QR codes. Convert from file format to QRCode objects."""
        try:
            data = self._read_qrcodes()
            qrcodes = []
            for qr_id, position in data.items():
                try:
                    # Convert from file format to QRCode object
                    # File only has position, use defaults for isActive and surface
                    qrcode = QRCode(
                        id=qr_id,
                        position=position,
                        isActive=False,  # Default value
                        surface="floor"  # Default value
                    )
                    qrcodes.append(qrcode)
                except Exception as e:
                    logger.warning(f"Skipping invalid QR code {qr_id}: {position}, error: {e}")
                    continue
            return qrcodes
        except Exception as e:
            logger.error(f"Error getting all QR codes: {e}")
            raise
    
    def get_by_id(self, qr_id: str) -> Optional[QRCode]:
        """Get QR code by ID."""
        try:
            data = self._read_qrcodes()
            
            if qr_id not in data:
                return None
            
            position = data[qr_id]
            # Convert from file format to QRCode object
            return QRCode(
                id=qr_id,
                position=position,
                isActive=False,  # Default value
                surface="floor"  # Default value
            )
        except Exception as e:
            logger.error(f"Error getting QR code by ID {qr_id}: {e}")
            raise
    
    def create_or_update(self, qrcode: QRCode) -> QRCode:
        """Create new QR code or update existing one if ID matches.
        
        Only position [x, y, z] is saved to file.
        """
        try:
            data = self._read_qrcodes()
            
            # Check if exists before updating
            is_update = qrcode.id in data
            
            # Only save position to file
            data[qrcode.id] = qrcode.position
            
            # Write back to file
            self._write_qrcodes(data)
            
            if is_update:
                logger.info(f"Updated QR code with ID: {qrcode.id}")
            else:
                logger.info(f"Created new QR code with ID: {qrcode.id}")
            
            return qrcode
        except Exception as e:
            logger.error(f"Error creating/updating QR code: {e}")
            raise
    
    def update(self, qr_id: str, qrcode_update: dict) -> Optional[QRCode]:
        """Update QR code by ID. Only position is saved to file."""
        try:
            data = self._read_qrcodes()
            
            if qr_id not in data:
                return None
            
            # Only update position if provided
            if "position" in qrcode_update and qrcode_update["position"] is not None:
                data[qr_id] = qrcode_update["position"]
            # Note: isActive and surface are not saved to file
            
            # Write back to file
            self._write_qrcodes(data)
            
            # Return updated QRCode object
            updated_qrcode = QRCode(
                id=qr_id,
                position=data[qr_id],
                isActive=qrcode_update.get("isActive", False),
                surface=qrcode_update.get("surface", "floor")
            )
            
            logger.info(f"Updated QR code with ID: {qr_id}")
            return updated_qrcode
        except Exception as e:
            logger.error(f"Error updating QR code {qr_id}: {e}")
            raise
    
    def delete(self, qr_id: str) -> bool:
        """Delete QR code by ID."""
        try:
            data = self._read_qrcodes()
            
            if qr_id not in data:
                return False
            
            # Remove from dict
            del data[qr_id]
            
            # Write back to file
            self._write_qrcodes(data)
            logger.info(f"Deleted QR code with ID: {qr_id}")
            return True
        except Exception as e:
            logger.error(f"Error deleting QR code {qr_id}: {e}")
            raise


# Global QR code service instance
qrcode_service = QRCodeService()
