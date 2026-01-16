"""QR Code API endpoints."""
import logging
from fastapi import APIRouter, HTTPException
from app.models.qrcode import (
    QRCode,
    QRCodeResponse,
    QRCodeListResponse,
    QRCodeCreateRequest,
    QRCodeUpdateRequest,
)
from app.services.qrcode_service import qrcode_service

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/qrcodes", tags=["qrcodes"])


@router.post("", response_model=QRCodeResponse)
async def create_qrcode(qrcode_request: QRCodeCreateRequest):
    """
    Create or update QR code.
    
    If QR code with the same ID already exists, it will be overwritten.
    """
    try:
        # Validate ID is not empty
        if not qrcode_request.id or not qrcode_request.id.strip():
            raise HTTPException(status_code=400, detail="ID cannot be empty")
        
        # Validate position has 3 elements
        if len(qrcode_request.position) != 3:
            raise HTTPException(
                status_code=400,
                detail="Position must have exactly 3 elements [x, y, z]"
            )
        
        # Create QRCode object
        qrcode = QRCode(
            id=qrcode_request.id.strip(),
            position=qrcode_request.position,
            isActive=qrcode_request.isActive,
            surface=qrcode_request.surface
        )
        
        # Create or update
        result = qrcode_service.create_or_update(qrcode)
        
        return QRCodeResponse(
            id=result.id,
            position=result.position,
            isActive=result.isActive,
            surface=result.surface
        )
    except HTTPException:
        raise
    except ValueError as e:
        logger.error(f"Validation error creating QR code: {e}")
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"Error creating QR code: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("", response_model=QRCodeListResponse)
async def get_all_qrcodes():
    """Get all QR codes."""
    try:
        qrcodes = qrcode_service.get_all()
        
        qrcode_responses = [
            QRCodeResponse(
                id=qrcode.id,
                position=qrcode.position,
                isActive=qrcode.isActive,
                surface=qrcode.surface
            )
            for qrcode in qrcodes
        ]
        
        return QRCodeListResponse(
            qrcodes=qrcode_responses,
            total=len(qrcode_responses)
        )
    except Exception as e:
        logger.error(f"Error getting all QR codes: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/{qr_id}", response_model=QRCodeResponse)
async def get_qrcode_by_id(qr_id: str):
    """Get QR code by ID."""
    try:
        if not qr_id or not qr_id.strip():
            raise HTTPException(status_code=400, detail="ID cannot be empty")
        
        qrcode = qrcode_service.get_by_id(qr_id.strip())
        
        if not qrcode:
            raise HTTPException(status_code=404, detail=f"QR code with ID '{qr_id}' not found")
        
        return QRCodeResponse(
            id=qrcode.id,
            position=qrcode.position,
            isActive=qrcode.isActive,
            surface=qrcode.surface
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting QR code by ID {qr_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.put("/{qr_id}", response_model=QRCodeResponse)
async def update_qrcode(qr_id: str, qrcode_update: QRCodeUpdateRequest):
    """Update QR code by ID."""
    try:
        if not qr_id or not qr_id.strip():
            raise HTTPException(status_code=400, detail="ID cannot be empty")
        
        # Validate position if provided
        if qrcode_update.position is not None and len(qrcode_update.position) != 3:
            raise HTTPException(
                status_code=400,
                detail="Position must have exactly 3 elements [x, y, z]"
            )
        
        # Prepare update dict (only include non-None fields)
        update_dict = {}
        if qrcode_update.position is not None:
            update_dict["position"] = qrcode_update.position
        if qrcode_update.isActive is not None:
            update_dict["isActive"] = qrcode_update.isActive
        if qrcode_update.surface is not None:
            update_dict["surface"] = qrcode_update.surface
        
        # Check if there's anything to update
        if not update_dict:
            raise HTTPException(status_code=400, detail="No fields to update")
        
        # Update QR code
        updated_qrcode = qrcode_service.update(qr_id.strip(), update_dict)
        
        if not updated_qrcode:
            raise HTTPException(status_code=404, detail=f"QR code with ID '{qr_id}' not found")
        
        return QRCodeResponse(
            id=updated_qrcode.id,
            position=updated_qrcode.position,
            isActive=updated_qrcode.isActive,
            surface=updated_qrcode.surface
        )
    except HTTPException:
        raise
    except ValueError as e:
        logger.error(f"Validation error updating QR code: {e}")
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"Error updating QR code {qr_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.delete("/{qr_id}")
async def delete_qrcode(qr_id: str):
    """Delete QR code by ID."""
    try:
        if not qr_id or not qr_id.strip():
            raise HTTPException(status_code=400, detail="ID cannot be empty")
        
        success = qrcode_service.delete(qr_id.strip())
        
        if not success:
            raise HTTPException(status_code=404, detail=f"QR code with ID '{qr_id}' not found")
        
        return {
            "success": True,
            "message": f"QR code with ID '{qr_id}' deleted successfully"
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting QR code {qr_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))
