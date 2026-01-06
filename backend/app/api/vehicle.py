"""Vehicle and pose API endpoints."""
import logging
from datetime import datetime
from typing import Optional, List
from fastapi import APIRouter, HTTPException, Query
from app.models.vehicle import (
    PoseUpdateRequest,
    PoseUpdateResponse,
    VehicleResponse,
    VehiclesListResponse,
    VehicleListItem,
    VehicleRegisterRequest,
    VehicleRegisterResponse,
    Pose,
    Position,
    Orientation
)
from app.services.database_service import database_service
from app.services.mqtt_service import mqtt_service

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/vehicles", tags=["vehicles"])


@router.post("", response_model=VehicleRegisterResponse)
async def register_vehicle(vehicle_request: VehicleRegisterRequest):
    """
    Register/create a new vehicle.
    
    vehicle_id: MAC address (đã bỏ dấu :, ví dụ: aa11bb22cc33dd44)
    """
    try:
        vehicle_id = vehicle_request.vehicle_id
        
        # Check if vehicle already exists
        existing_vehicle = await database_service.get_vehicle(vehicle_id)
        if existing_vehicle:
            return VehicleRegisterResponse(
                success=True,
                vehicle_id=vehicle_id,
                message="Vehicle already exists",
                created=False
            )
        
        # Create new vehicle with initial pose (default values)
        initial_pose = {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "timestamp": datetime.utcnow()
        }
        
        # Create vehicle document
        vehicle_data = {
            "vehicle_id": vehicle_id,
            "latest_pose": initial_pose,
            "name": vehicle_request.name,
            "description": vehicle_request.description,
            "vehicle_type": vehicle_request.vehicle_type,
            "metadata": vehicle_request.metadata or {},
            "created_at": datetime.utcnow(),
            "updated_at": datetime.utcnow()
        }
        
        # Insert vehicle
        success = await database_service.create_vehicle(vehicle_data)
        if not success:
            raise HTTPException(status_code=500, detail="Failed to create vehicle")
        
        # Publish MQTT event
        mqtt_service.publish_map_event("vehicle.registered", {
            "vehicle_id": vehicle_id,
            "name": vehicle_request.name,
            "vehicle_type": vehicle_request.vehicle_type
        })
        
        return VehicleRegisterResponse(
            success=True,
            vehicle_id=vehicle_id,
            message="Vehicle registered successfully",
            created=True
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error registering vehicle: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/{vehicle_id}/pose", response_model=PoseUpdateResponse)
async def update_vehicle_pose(
    vehicle_id: str,
    pose_request: PoseUpdateRequest
):
    """
    Update vehicle pose localization.
    
    vehicle_id: MAC address (đã bỏ dấu :, ví dụ: aa11bb22cc33dd44)
    """
    try:
        # Prepare pose data for database
        pose_data = {
            "position": pose_request.position.dict(),
            "orientation": pose_request.orientation.dict(),
            "timestamp": pose_request.timestamp
        }
        
        # Update vehicle in database
        success = await database_service.update_vehicle_pose(vehicle_id, pose_data)
        if not success:
            raise HTTPException(status_code=500, detail="Failed to update vehicle pose")
        
        # Save to history
        await database_service.save_pose_history(vehicle_id, pose_data)
        
        # Publish MQTT event
        mqtt_pose = {
            "position": pose_data["position"],
            "orientation": pose_data["orientation"],
            "timestamp": pose_request.timestamp.isoformat()
        }
        mqtt_service.publish_vehicle_pose(vehicle_id, mqtt_pose)
        
        return PoseUpdateResponse(
            success=True,
            vehicle_id=vehicle_id,
            message="Pose updated successfully"
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error updating vehicle pose: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/{vehicle_id}", response_model=VehicleResponse)
async def get_vehicle(vehicle_id: str):
    """Get vehicle info and latest pose."""
    try:
        vehicle = await database_service.get_vehicle(vehicle_id)
        if not vehicle:
            raise HTTPException(status_code=404, detail="Vehicle not found")
        
        latest_pose = vehicle.get("latest_pose", {})
        # Handle case where vehicle exists but has no pose yet
        if not latest_pose:
            # Return default pose
            latest_pose = {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                "timestamp": datetime.utcnow()
            }
        
        return VehicleResponse(
            vehicle_id=vehicle["vehicle_id"],
            latest_pose=Pose(
                position=Position(**latest_pose["position"]),
                orientation=Orientation(**latest_pose["orientation"]),
                timestamp=latest_pose["timestamp"]
            ),
            created_at=vehicle.get("created_at", datetime.utcnow()),
            updated_at=vehicle.get("updated_at", datetime.utcnow())
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting vehicle: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("", response_model=VehiclesListResponse)
async def get_all_vehicles():
    """List all vehicles with latest pose."""
    try:
        vehicles = await database_service.get_all_vehicles()
        
        vehicle_items = []
        for vehicle in vehicles:
            latest_pose = vehicle.get("latest_pose", {})
            # Handle case where vehicle has no pose yet
            if not latest_pose:
                latest_pose = {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    "timestamp": datetime.utcnow()
                }
            
            vehicle_items.append(
                VehicleListItem(
                    vehicle_id=vehicle["vehicle_id"],
                    latest_pose=Pose(
                        position=Position(**latest_pose["position"]),
                        orientation=Orientation(**latest_pose["orientation"]),
                        timestamp=latest_pose["timestamp"]
                    ),
                    updated_at=vehicle.get("updated_at", datetime.utcnow())
                )
            )
        
        return VehiclesListResponse(
            vehicles=vehicle_items,
            total=len(vehicle_items)
        )
    except Exception as e:
        logger.error(f"Error getting all vehicles: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/{vehicle_id}/poses")
async def get_pose_history(
    vehicle_id: str,
    limit: int = Query(default=100, ge=1, le=1000),
    start_time: Optional[datetime] = None,
    end_time: Optional[datetime] = None
):
    """Get pose history for vehicle."""
    try:
        poses = await database_service.get_pose_history(
            vehicle_id,
            limit=limit,
            start_time=start_time,
            end_time=end_time
        )
        return {"vehicle_id": vehicle_id, "poses": poses, "count": len(poses)}
    except Exception as e:
        logger.error(f"Error getting pose history: {e}")
        raise HTTPException(status_code=500, detail=str(e))

