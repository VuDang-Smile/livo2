"""FastAPI application entry point."""
import logging
import time
from contextlib import asynccontextmanager
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from app.config import settings
from app.services.database_service import database_service
from app.services.mqtt_service import mqtt_service
from app.api import upload, vehicle

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Track startup time for uptime calculation
startup_time = time.time()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifespan context manager for startup and shutdown events."""
    # Startup
    logger.info("Starting up...")
    
    # Connect to MongoDB with timeout
    import asyncio
    try:
        await asyncio.wait_for(database_service.connect(), timeout=3.0)
        logger.info("MongoDB connected")
    except asyncio.TimeoutError:
        logger.warning("MongoDB connection timeout (continuing without MongoDB)")
    except Exception as e:
        logger.warning(f"Failed to connect to MongoDB: {e} (continuing without MongoDB)")
        # Don't raise - allow server to start without MongoDB for testing
    
    # Connect to MQTT
    try:
        mqtt_service.connect()
        logger.info("MQTT service initialized")
    except Exception as e:
        logger.warning(f"Failed to connect to MQTT (will retry): {e}")
    
    # Auto-extract current map on startup if storage is empty
    try:
        from app.services.map_processor import MapProcessor
        from app.services.storage_service import storage_service
        from pathlib import Path
        
        # Check if storage is empty
        storage_dir = storage_service.storage_dir
        has_storage_files = False
        if storage_dir.exists():
            for folder in ["merged_map", "fastloc_map", "floorplan_2d"]:
                folder_path = storage_dir / folder
                if folder_path.exists() and any(folder_path.iterdir()):
                    has_storage_files = True
                    break
            if not has_storage_files:
                qr_file = storage_dir / "QR_detect.json"
                if qr_file.exists():
                    has_storage_files = True
        
        if not has_storage_files:
            logger.info("=" * 60)
            logger.info("🔄 AUTO-EXTRACTION: Storage is empty, checking for uploaded map...")
            logger.info("=" * 60)
            
            # Get current map from database
            current_map = await database_service.get_current_map()
            if current_map and current_map.get("file_path"):
                map_file = Path(current_map["file_path"])
                if map_file.exists() and map_file.name.endswith('.zip'):
                    logger.info(f"📦 Found map file: {map_file.name}")
                    logger.info("🔄 Auto-extracting to storage (extraction only, no metadata update)...")
                    
                    # Use extract_zip_to_storage to avoid deleting old map
                    map_processor = MapProcessor()
                    result = await map_processor.extract_zip_to_storage(map_file)
                    
                    if result and result.get("storage_paths"):
                        logger.info("✅ Auto-extraction completed successfully!")
                        logger.info(f"   Extracted {len(result['storage_paths'])} items")
                    else:
                        logger.warning("⚠️ Auto-extraction failed or returned no results")
                else:
                    logger.info("ℹ️ Current map is not a ZIP file or file not found")
            else:
                logger.info("ℹ️ No current map found in database")
            
            logger.info("=" * 60)
    except Exception as e:
        logger.warning(f"Auto-extraction on startup failed: {e}")
        # Don't fail startup if auto-extraction fails
    
    yield
    
    # Shutdown
    logger.info("Shutting down...")
    await database_service.disconnect()
    mqtt_service.disconnect()
    logger.info("Shutdown complete")


# Create FastAPI app
app = FastAPI(
    title=settings.SERVICE_NAME,
    version=settings.SERVICE_VERSION,
    description="FastAPI Backend for Livo System",
    lifespan=lifespan
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify allowed origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(upload.router, prefix="/api/v1")
app.include_router(vehicle.router, prefix="/api/v1")


# Health check endpoints
@app.get("/health")
async def health_check_simple():
    """Simple health check endpoint."""
    return {
        "status": "healthy",
        "service": settings.SERVICE_NAME,
        "timestamp": time.time()
    }


@app.get("/api/v1/health")
async def health_check_detailed():
    """Detailed health check endpoint."""
    # Check MongoDB
    mongodb_status = "disconnected"
    mongodb_latency_ms = None
    try:
        start = time.time()
        mongodb_connected = await database_service.check_connection()
        mongodb_latency_ms = int((time.time() - start) * 1000)
        mongodb_status = "connected" if mongodb_connected else "disconnected"
    except Exception as e:
        logger.error(f"MongoDB health check failed: {e}")
    
    # Check MQTT
    mqtt_status = "connected" if mqtt_service.is_connected() else "disconnected"
    mqtt_broker = f"{settings.MQTT_BROKER_HOST}:{settings.MQTT_BROKER_PORT}"
    
    # Calculate uptime
    uptime_seconds = int(time.time() - startup_time)
    
    overall_status = "healthy" if mongodb_status == "connected" else "degraded"
    
    return {
        "status": overall_status,
        "service": settings.SERVICE_NAME,
        "version": settings.SERVICE_VERSION,
        "timestamp": time.time(),
        "uptime_seconds": uptime_seconds,
        "services": {
            "mongodb": {
                "status": mongodb_status,
                "latency_ms": mongodb_latency_ms
            },
            "mqtt": {
                "status": mqtt_status,
                "broker": mqtt_broker
            }
        }
    }


# Error handlers
@app.exception_handler(Exception)
async def global_exception_handler(request, exc):
    """Global exception handler."""
    logger.error(f"Unhandled exception: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={"detail": "Internal server error"}
    )


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "app.main:app",
        host=settings.HOST,
        port=settings.PORT,
        reload=True
    )

