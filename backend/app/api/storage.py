"""Storage API endpoints for JSON file uploads."""
import json
import logging
import re
from datetime import datetime
from pathlib import Path
from typing import Optional, List, Dict, Any, Union
from fastapi import APIRouter, HTTPException, UploadFile, File, Form, Body
from fastapi.responses import JSONResponse
from app.services.storage_service import storage_service
from app.utils.time import now_utc, isoformat_utc

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/storage", tags=["storage"])


def _sanitize_filename(filename: str) -> str:
    """Sanitize filename to be filesystem-safe."""
    # Remove or replace invalid characters
    safe_name = re.sub(r"[^A-Za-z0-9._-]+", "_", filename).strip("._-")
    # Ensure it's not empty
    if not safe_name:
        safe_name = "file"
    # Ensure it ends with .json if not already
    if not safe_name.endswith(".json"):
        safe_name += ".json"
    return safe_name


@router.post("/json")
async def upload_json_file(
    file: Optional[UploadFile] = File(None),
    filename: Optional[str] = Form(None),
    json_data: Optional[str] = Form(None)
):
    """
    Upload JSON file to storage directory.
    
    Có thể upload theo 2 cách:
    1. Upload file JSON (multipart/form-data với file)
    2. Gửi JSON data trực tiếp (multipart/form-data với json_data)
    
    Parameters:
    - file: File JSON để upload (optional nếu dùng json_data)
    - filename: Tên file để lưu (optional, sẽ tự động tạo nếu không có)
    - json_data: JSON data dạng string (optional nếu dùng file)
    """
    try:
        # Đảm bảo storage directory tồn tại
        storage_service.ensure_storage_directories()
        
        json_content = None
        final_filename = None
        
        # Xử lý upload file
        if file:
            if not file.filename:
                raise HTTPException(status_code=400, detail="File must have a filename")
            
            # Đọc nội dung file
            content = await file.read()
            try:
                # Validate JSON
                json_content = json.loads(content.decode('utf-8'))
            except json.JSONDecodeError as e:
                raise HTTPException(status_code=400, detail=f"Invalid JSON file: {str(e)}")
            
            # Xác định tên file
            if filename:
                final_filename = _sanitize_filename(filename)
            else:
                # Sử dụng tên file gốc (đã sanitize)
                final_filename = _sanitize_filename(file.filename)
        
        # Xử lý JSON data trực tiếp
        elif json_data:
            try:
                json_content = json.loads(json_data)
            except json.JSONDecodeError as e:
                raise HTTPException(status_code=400, detail=f"Invalid JSON data: {str(e)}")
            
            # Xác định tên file
            if filename:
                final_filename = _sanitize_filename(filename)
            else:
                # Tạo tên file tự động với timestamp
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                final_filename = f"upload_{timestamp}.json"
        
        else:
            raise HTTPException(
                status_code=400, 
                detail="Either 'file' or 'json_data' must be provided"
            )
        
        # Lưu file vào storage
        file_path = storage_service.get_storage_path(final_filename)
        
        # Ghi file với format đẹp (indent=2)
        with open(file_path, 'w', encoding='utf-8') as f:
            json.dump(json_content, f, ensure_ascii=False, indent=2)
        
        logger.info(f"JSON file saved to storage: {file_path}")
        
        # Trả về thông tin file đã lưu
        file_size = file_path.stat().st_size
        
        return JSONResponse(content={
            "success": True,
            "message": "JSON file uploaded successfully",
            "filename": final_filename,
            "file_path": str(file_path),
            "size_bytes": file_size,
            "uploaded_at": isoformat_utc(now_utc())
        })
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error uploading JSON file: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Failed to upload JSON file: {str(e)}")


@router.post("/json/body")
async def upload_json_from_body(
    json_data: dict,
    filename: Optional[str] = None
):
    """
    Upload JSON data từ request body (application/json).
    
    Parameters:
    - json_data: JSON object trong request body
    - filename: Tên file để lưu (optional, sẽ tự động tạo nếu không có)
    """
    try:
        # Đảm bảo storage directory tồn tại
        storage_service.ensure_storage_directories()
        
        # Xác định tên file
        if filename:
            final_filename = _sanitize_filename(filename)
        else:
            # Tạo tên file tự động với timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            final_filename = f"upload_{timestamp}.json"
        
        # Lưu file vào storage
        file_path = storage_service.get_storage_path(final_filename)
        
        # Ghi file với format đẹp (indent=2)
        with open(file_path, 'w', encoding='utf-8') as f:
            json.dump(json_data, f, ensure_ascii=False, indent=2)
        
        logger.info(f"JSON file saved to storage: {file_path}")
        
        # Trả về thông tin file đã lưu
        file_size = file_path.stat().st_size
        
        return JSONResponse(content={
            "success": True,
            "message": "JSON file uploaded successfully",
            "filename": final_filename,
            "file_path": str(file_path),
            "size_bytes": file_size,
            "uploaded_at": isoformat_utc(now_utc())
        })
    
    except Exception as e:
        logger.error(f"Error uploading JSON file: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Failed to upload JSON file: {str(e)}")


@router.get("/json/{filename}")
async def get_json_file(filename: str):
    """
    Lấy nội dung file JSON từ storage.
    
    Parameters:
    - filename: Tên file JSON (phải có extension .json)
    """
    try:
        # Sanitize filename để tránh path traversal
        safe_filename = _sanitize_filename(filename)
        
        file_path = storage_service.get_storage_path(safe_filename)
        
        if not file_path.exists():
            raise HTTPException(status_code=404, detail=f"File {safe_filename} not found")
        
        # Đọc và parse JSON
        with open(file_path, 'r', encoding='utf-8') as f:
            json_content = json.load(f)
        
        return JSONResponse(content=json_content)
    
    except HTTPException:
        raise
    except json.JSONDecodeError as e:
        raise HTTPException(status_code=500, detail=f"Invalid JSON in file: {str(e)}")
    except Exception as e:
        logger.error(f"Error reading JSON file: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Failed to read JSON file: {str(e)}")


@router.delete("/json/{filename}")
async def delete_json_file(filename: str):
    """
    Xóa file JSON từ storage directory.
    
    Parameters:
    - filename: Tên file JSON cần xóa (phải có extension .json)
    
    Lưu ý: Không thể xóa QR_detect.json qua API này, hãy dùng DELETE /api/v1/storage/qr/{qr_id} để xóa từng QR code
    """
    try:
        # Sanitize filename để tránh path traversal
        safe_filename = _sanitize_filename(filename)
        
        # Bảo vệ file QR_detect.json - không cho xóa qua API này
        if safe_filename == "QR_detect.json":
            raise HTTPException(
                status_code=403, 
                detail="Cannot delete QR_detect.json via this endpoint. Use DELETE /api/v1/storage/qr/{qr_id} to delete individual QR codes"
            )
        
        file_path = storage_service.get_storage_path(safe_filename)
        
        if not file_path.exists():
            raise HTTPException(status_code=404, detail=f"File {safe_filename} not found")
        
        # Xóa file
        if storage_service.delete_file(file_path):
            logger.info(f"Deleted JSON file: {file_path}")
            return JSONResponse(content={
                "success": True,
                "message": f"File {safe_filename} deleted successfully",
                "filename": safe_filename
            })
        else:
            raise HTTPException(status_code=500, detail=f"Failed to delete file {safe_filename}")
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting JSON file: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Failed to delete JSON file: {str(e)}")


@router.get("/json")
async def list_json_files():
    """Liệt kê tất cả các file JSON trong storage directory."""
    try:
        storage_dir = storage_service.storage_dir
        
        if not storage_dir.exists():
            return JSONResponse(content={
                "success": True,
                "files": [],
                "count": 0
            })
        
        # Lấy tất cả file .json trong storage (không đệ quy vào subdirectories)
        json_files = [
            f.name for f in storage_dir.iterdir() 
            if f.is_file() and f.name.endswith('.json')
        ]
        
        # Sắp xếp theo tên
        json_files.sort()
        
        return JSONResponse(content={
            "success": True,
            "files": json_files,
            "count": len(json_files),
            "storage_dir": str(storage_dir)
        })
    
    except Exception as e:
        logger.error(f"Error listing JSON files: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Failed to list JSON files: {str(e)}")


# ========== QR Code APIs ==========

def _load_qr_detect_file() -> List[Dict[str, Any]]:
    """
    Load và parse file QR_detect.json, trả về danh sách QR codes dạng array.
    
    File được load từ: backend/storage/QR_detect.json
    """
    qr_path = storage_service.get_storage_path("QR_detect.json")
    absolute_path = qr_path.resolve()
    
    if not qr_path.exists():
        logger.debug(f"QR_detect.json not found at {absolute_path}")
        return []
    
    try:
        with open(qr_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        # Chuyển đổi format cũ sang format mới nếu cần
        qr_list = []
        
        if isinstance(data, dict):
            # Format cũ: {"TM:0001": [x, y, z], ...}
            for qr_id, position in data.items():
                if isinstance(position, list) and len(position) >= 3:
                    # Chuyển đổi sang format mới
                    qr_list.append({
                        "id": qr_id if qr_id.startswith("qr-") else f"qr-{qr_id}",
                        "position": position[:3],  # Chỉ lấy x, y, z
                        "isActive": True,
                        "surface": "floor"
                    })
                elif isinstance(position, dict):
                    # Đã là format mới
                    qr_list.append(position)
        elif isinstance(data, list):
            # Đã là array
            qr_list = data
        else:
            logger.warning(f"Unexpected QR_detect.json format: {type(data)}")
            return []
        
        return qr_list
    except json.JSONDecodeError as e:
        logger.error(f"Error parsing QR_detect.json: {e}")
        return []
    except Exception as e:
        logger.error(f"Error loading QR_detect.json: {e}")
        return []


def _save_qr_detect_file(qr_list: List[Dict[str, Any]]) -> bool:
    """
    Lưu danh sách QR codes vào file QR_detect.json.
    
    Luôn lưu vào: backend/storage/QR_detect.json
    Luôn lưu theo format mới (array), không phải format cũ (dict).
    Format: [{"id": "qr-xxx", "position": [x, y, z], "isActive": bool, "surface": "floor"}, ...]
    """
    try:
        # Lấy đường dẫn đầy đủ đến file QR_detect.json trong backend/storage
        qr_path = storage_service.get_storage_path("QR_detect.json")
        
        # Đảm bảo thư mục storage tồn tại
        storage_service.ensure_storage_directories()
        
        # Đảm bảo qr_list là list (format mới)
        if not isinstance(qr_list, list):
            logger.error(f"qr_list must be a list, got {type(qr_list)}")
            return False
        
        # Lưu dạng array (format mới) - luôn luôn
        with open(qr_path, 'w', encoding='utf-8') as f:
            json.dump(qr_list, f, ensure_ascii=False, indent=2)
        
        # Log đường dẫn đầy đủ để đảm bảo lưu đúng file
        absolute_path = qr_path.resolve()
        logger.info(f"✅ QR_detect.json saved successfully")
        logger.info(f"   Path: {absolute_path}")
        logger.info(f"   QR codes count: {len(qr_list)}")
        logger.info(f"   Format: array")
        
        # Verify file đã được tạo
        if not qr_path.exists():
            logger.error(f"❌ File was not created at {absolute_path}")
            return False
        
        return True
    except Exception as e:
        logger.error(f"❌ Error saving QR_detect.json: {e}", exc_info=True)
        return False


@router.post("/qr")
async def upload_qr_code(
    qr_data: Dict[str, Any] = Body(...)
):
    """
    Upload hoặc cập nhật QR code vào QR_detect.json.
    
    Nghiệp vụ:
    - Ghi nội dung QR được upload vào file QR_detect.json
    - Kiểm tra trùng ID để replace nội dung QR trùng
    - File luôn được lưu theo format mới (array)
    
    Kiểm tra xem QR code với ID đã tồn tại chưa:
    - Nếu tồn tại: cập nhật/replace thông tin QR code đó
    - Nếu chưa tồn tại: thêm mới vào file
    
    Format QR code:
    {
        "id": "qr-TM-0001",
        "position": [28.11602014937347, -16.198274724176674, 0.4078582429272276],
        "isActive": false,
        "surface": "floor"
    }
    
    Parameters:
    - qr_data: JSON object chứa thông tin QR code
    """
    try:
        # Validate required fields
        if "id" not in qr_data:
            raise HTTPException(status_code=400, detail="Field 'id' is required")
        
        if "position" not in qr_data:
            raise HTTPException(status_code=400, detail="Field 'position' is required")
        
        qr_id = qr_data["id"]
        position = qr_data["position"]
        
        # Validate position
        if not isinstance(position, list) or len(position) < 3:
            raise HTTPException(
                status_code=400, 
                detail="Field 'position' must be an array with at least 3 elements [x, y, z]"
            )
        
        # Đảm bảo ID có format đúng (thêm prefix "qr-" nếu chưa có)
        if not qr_id.startswith("qr-"):
            qr_id = f"qr-{qr_id}"
            qr_data["id"] = qr_id
        
        # Load file hiện tại (tự động convert format cũ sang mới nếu cần)
        qr_list = _load_qr_detect_file()
        
        # Tìm QR code với ID tương ứng để kiểm tra trùng
        found_index = None
        for i, qr in enumerate(qr_list):
            if qr.get("id") == qr_id:
                found_index = i
                break
        
        # Tạo QR code object với các field mặc định
        qr_code = {
            "id": qr_id,
            "position": position[:3],  # Chỉ lấy x, y, z
            "isActive": qr_data.get("isActive", True),
            "surface": qr_data.get("surface", "floor")
        }
        
        # Thêm các field bổ sung nếu có
        for key, value in qr_data.items():
            if key not in ["id", "position", "isActive", "surface"]:
                qr_code[key] = value
        
        # Update hoặc thêm mới
        if found_index is not None:
            # Replace QR code hiện có (check trùng ID)
            old_qr = qr_list[found_index]
            qr_list[found_index] = qr_code
            action = "updated"
            logger.info(f"Replaced QR code '{qr_id}' in QR_detect.json (old: {old_qr}, new: {qr_code})")
        else:
            # Thêm QR code mới
            qr_list.append(qr_code)
            action = "added"
            logger.info(f"Added new QR code '{qr_id}' to QR_detect.json")
        
        # Lưu file QR_detect.json với format mới (array) vào backend/storage/QR_detect.json
        qr_file_path = storage_service.get_storage_path("QR_detect.json")
        absolute_file_path = qr_file_path.resolve()
        
        if not _save_qr_detect_file(qr_list):
            raise HTTPException(status_code=500, detail=f"Failed to save QR_detect.json to {absolute_file_path}")
        
        return JSONResponse(content={
            "success": True,
            "message": f"QR code {action} successfully in QR_detect.json",
            "action": action,
            "qr_code": qr_code,
            "total_qr_codes": len(qr_list),
            "file_path": str(absolute_file_path),
            "relative_path": "backend/storage/QR_detect.json"
        })
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error uploading QR code: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Failed to upload QR code: {str(e)}")


@router.get("/qr")
async def get_all_qr_codes():
    """Lấy tất cả QR codes từ QR_detect.json."""
    try:
        qr_list = _load_qr_detect_file()
        
        return JSONResponse(content={
            "success": True,
            "qr_codes": qr_list,
            "count": len(qr_list)
        })
    except Exception as e:
        logger.error(f"Error getting QR codes: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Failed to get QR codes: {str(e)}")


@router.get("/qr/{qr_id}")
async def get_qr_code(qr_id: str):
    """Lấy thông tin QR code theo ID."""
    try:
        # Đảm bảo ID có format đúng
        if not qr_id.startswith("qr-"):
            qr_id = f"qr-{qr_id}"
        
        qr_list = _load_qr_detect_file()
        
        # Tìm QR code
        for qr in qr_list:
            if qr.get("id") == qr_id:
                return JSONResponse(content={
                    "success": True,
                    "qr_code": qr
                })
        
        raise HTTPException(status_code=404, detail=f"QR code with ID '{qr_id}' not found")
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting QR code: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Failed to get QR code: {str(e)}")


@router.delete("/qr/{qr_id}")
async def delete_qr_code(qr_id: str):
    """
    Xóa QR code theo ID trong file QR_detect.json.
    
    Chỉ xóa nội dung QR code đó trong file, không xóa toàn bộ file.
    File QR_detect.json vẫn tồn tại với danh sách QR codes còn lại.
    
    Parameters:
    - qr_id: ID của QR code cần xóa (có thể có hoặc không có prefix "qr-")
    
    Returns:
    - success: True nếu xóa thành công
    - message: Thông báo kết quả
    - total_qr_codes: Tổng số QR codes còn lại trong file
    """
    try:
        # Đảm bảo ID có format đúng
        if not qr_id.startswith("qr-"):
            qr_id = f"qr-{qr_id}"
        
        # Load danh sách QR codes từ file QR_detect.json
        qr_list = _load_qr_detect_file()
        
        # Tìm và xóa QR code (chỉ xóa nội dung trong danh sách, không xóa file)
        original_count = len(qr_list)
        qr_list = [qr for qr in qr_list if qr.get("id") != qr_id]
        
        # Kiểm tra xem có tìm thấy QR code để xóa không
        if len(qr_list) == original_count:
            raise HTTPException(status_code=404, detail=f"QR code with ID '{qr_id}' not found")
        
        # Lưu lại danh sách đã cập nhật vào file QR_detect.json (file vẫn tồn tại)
        if not _save_qr_detect_file(qr_list):
            raise HTTPException(status_code=500, detail="Failed to save QR_detect.json")
        
        logger.info(f"Deleted QR code '{qr_id}' from QR_detect.json. Remaining: {len(qr_list)} QR codes")
        
        return JSONResponse(content={
            "success": True,
            "message": f"QR code '{qr_id}' deleted successfully from QR_detect.json",
            "deleted_qr_id": qr_id,
            "total_qr_codes": len(qr_list),
            "file_path": str(storage_service.get_storage_path("QR_detect.json"))
        })
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting QR code: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Failed to delete QR code: {str(e)}")
