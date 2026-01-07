#!/usr/bin/env python3
"""
Simple wrapper for ros2_numpy functionality
"""
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import struct

def numpify(pc_msg):
    """
    Convert PointCloud2 message to numpy array
    """
    # Get the point cloud data
    data = pc_msg.data
    
    # Calculate number of points
    point_step = pc_msg.point_step
    num_points = len(data) // point_step
    
    # Create structured array
    dtype = []
    offset = 0
    
    # Parse fields
    for field in pc_msg.fields:
        if field.name == 'x':
            dtype.append(('x', 'f4'))
        elif field.name == 'y':
            dtype.append(('y', 'f4'))
        elif field.name == 'z':
            dtype.append(('z', 'f4'))
        elif field.name == 'intensity':
            dtype.append(('intensity', 'f4'))
        elif field.name == 'ring':
            dtype.append(('ring', 'u2'))
        elif field.name == 'time':
            dtype.append(('time', 'f4'))
    
    # Create numpy array
    points = np.frombuffer(data, dtype=dtype)
    
    # Extract xyz coordinates
    xyz = np.column_stack([points['x'], points['y'], points['z']])
    
    return {"xyz": xyz}

def msgify(msg_type, data):
    """
    Convert numpy array or dictionary to PointCloud2 message
    """
    if msg_type == PointCloud2:
        if isinstance(data, dict):
            return dict_to_pointcloud2(data)
        else:
            return numpy_to_pointcloud2(data)
    else:
        raise NotImplementedError(f"msgify not implemented for {msg_type}")

def dict_to_pointcloud2(data):
    """
    Convert dictionary to PointCloud2 message
    """
    msg = PointCloud2()
    
    # Extract xyz coordinates
    if "xyz" in data:
        xyz = data["xyz"]
        n_points = xyz.shape[0]
        
        # Set header
        msg.header.frame_id = "camera_init"
        
        # Set fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Add intensity field if available
        if "intensity" in data:
            msg.fields.append(PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1))
        
        # Set point step and row step
        msg.point_step = 4 * len(msg.fields)  # 4 bytes per float
        msg.row_step = msg.point_step * n_points
        
        # Prepare data array
        if "intensity" in data:
            points_array = np.column_stack([xyz, data["intensity"]])
        else:
            points_array = xyz
        
        # Convert points to bytes
        points_float32 = points_array.astype(np.float32)
        msg.data = points_float32.tobytes()
        
        # Set width and height
        msg.width = n_points
        msg.height = 1
        
        # Set is_bigendian and is_dense
        msg.is_bigendian = False
        msg.is_dense = True
        
    else:
        raise ValueError("Dictionary must contain 'xyz' key")
    
    return msg

def numpy_to_pointcloud2(points):
    """
    Convert numpy array to PointCloud2 message
    """
    msg = PointCloud2()
    
    if len(points.shape) == 2 and points.shape[1] >= 3:
        # Points is a Nx3 or Nx4 array
        n_points = points.shape[0]
        
        # Set header
        msg.header.frame_id = "camera_init"
        
        # Set fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Add intensity field if available
        if points.shape[1] >= 4:
            msg.fields.append(PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1))
        
        # Set point step and row step
        msg.point_step = 4 * points.shape[1]  # 4 bytes per float
        msg.row_step = msg.point_step * n_points
        
        # Convert points to bytes
        points_float32 = points.astype(np.float32)
        msg.data = points_float32.tobytes()
        
        # Set width and height
        msg.width = n_points
        msg.height = 1
        
        # Set is_bigendian and is_dense
        msg.is_bigendian = False
        msg.is_dense = True
        
    else:
        raise ValueError(f"Invalid points shape: {points.shape}")
    
    return msg
