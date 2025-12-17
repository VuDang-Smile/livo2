#!/usr/bin/env python3
"""
Script to create a calibration bag from MCAP recording

This script:
1. Reads MCAP recording
2. Extracts first 5 seconds
3. Converts CustomMsg to PointCloud2
4. Creates CameraInfo from Image messages
5. Writes new bag with required topics for calibration

Usage:
    python3 create_calibration_bag.py <input_recording_path> [--output <output_bag_path>] [--duration <seconds>]

Example:
    python3 create_calibration_bag.py /path/to/recording_20251210_165248 --output calibration_bag --duration 5
"""

import sys
import argparse
from pathlib import Path
import struct
import time

try:
    from rosbags.rosbag2 import Reader, Writer
    from rosbags.serde import deserialize_cdr, serialize_cdr
    from rosbags.typesys import Stores, get_typestore
except ImportError:
    print("Error: rosbags library not found.")
    print("Attempting to install rosbags...")
    try:
        import subprocess
        import sys
        subprocess.check_call([sys.executable, "-m", "pip", "install", "rosbags", "--user"])
        print("Successfully installed rosbags. Please run the script again.")
        sys.exit(0)
    except Exception as e:
        print(f"Failed to install rosbags automatically: {e}")
        print("Please install it manually:")
        print("  pip install rosbags")
        print("  or")
        print("  python3 -m pip install rosbags --user")
        sys.exit(1)


def convert_custommsg_to_pointcloud2(custom_msg, frame_id="livox_frame"):
    """
    Convert Livox CustomMsg to PointCloud2 message
    
    Args:
        custom_msg: CustomMsg message object
        frame_id: Frame ID for PointCloud2
        
    Returns:
        PointCloud2 message dict
    """
    # Get typestore
    typestore = get_typestore(Stores.LATEST)
    PointCloud2 = typestore.types['sensor_msgs/msg/PointCloud2']
    PointField = typestore.types['sensor_msgs/msg/PointField']
    
    # Create PointCloud2 message
    pointcloud2 = {
        'header': {
            'stamp': custom_msg['header']['stamp'],
            'frame_id': frame_id
        },
        'height': 1,
        'width': custom_msg['point_num'],
        'is_dense': True,
        'is_bigendian': False,
        'point_step': 26,  # 4+4+4+4+1+1+8 = 26 bytes
        'row_step': custom_msg['point_num'] * 26,
        'fields': [
            {'name': 'x', 'offset': 0, 'datatype': 7, 'count': 1},  # FLOAT32
            {'name': 'y', 'offset': 4, 'datatype': 7, 'count': 1},
            {'name': 'z', 'offset': 8, 'datatype': 7, 'count': 1},
            {'name': 'intensity', 'offset': 12, 'datatype': 7, 'count': 1},
            {'name': 'tag', 'offset': 16, 'datatype': 1, 'count': 1},  # UINT8
            {'name': 'line', 'offset': 17, 'datatype': 1, 'count': 1},
            {'name': 'timestamp', 'offset': 18, 'datatype': 8, 'count': 1}  # FLOAT64
        ],
        'data': bytearray()
    }
    
    # Convert points
    data = bytearray()
    for point in custom_msg['points']:
        # x, y, z (float32)
        data.extend(struct.pack('<f', point['x']))
        data.extend(struct.pack('<f', point['y']))
        data.extend(struct.pack('<f', point['z']))
        # reflectivity (float32)
        data.extend(struct.pack('<f', float(point['reflectivity'])))
        # tag (uint8)
        data.extend(struct.pack('<B', point['tag']))
        # line (uint8)
        data.extend(struct.pack('<B', point['line']))
        # timestamp (float64) - convert from nanoseconds to seconds
        timestamp_sec = (custom_msg['timebase'] + point['offset_time']) / 1e9
        data.extend(struct.pack('<d', timestamp_sec))
    
    pointcloud2['data'] = bytes(data)
    
    return pointcloud2


def create_camera_info_from_image(image_msg, frame_id="camera_link"):
    """
    Create CameraInfo message from Image message
    
    Args:
        image_msg: Image message object
        frame_id: Frame ID for CameraInfo
        
    Returns:
        CameraInfo message dict
    """
    width = image_msg['width']
    height = image_msg['height']
    
    # Default intrinsic parameters (for uncalibrated camera)
    default_fx = width * 0.8
    default_fy = height * 0.8
    default_cx = width / 2.0
    default_cy = height / 2.0
    
    camera_info = {
        'header': {
            'stamp': image_msg['header']['stamp'],
            'frame_id': frame_id
        },
        'height': height,
        'width': width,
        'distortion_model': 'plumb_bob',
        'd': [0.0, 0.0, 0.0, 0.0, 0.0],  # k1, k2, p1, p2, k3
        'k': [
            default_fx, 0.0, default_cx,
            0.0, default_fy, default_cy,
            0.0, 0.0, 1.0
        ],
        'r': [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ],
        'p': [
            default_fx, 0.0, default_cx, 0.0,
            0.0, default_fy, default_cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
    }
    
    return camera_info


def create_calibration_bag(input_path, output_path, duration_seconds=5):
    """
    Create calibration bag from MCAP recording
    
    Args:
        input_path: Path to input MCAP recording directory
        output_path: Path to output bag directory
        duration_seconds: Duration to extract (seconds)
    """
    input_path = Path(input_path)
    output_path = Path(output_path)
    
    if not input_path.exists():
        print(f"Error: Input path does not exist: {input_path}")
        return False
    
    # Create output directory
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Get typestore
    typestore = get_typestore(Stores.LATEST)
    
    # Message types
    Image = typestore.types['sensor_msgs/msg/Image']
    CameraInfo = typestore.types['sensor_msgs/msg/CameraInfo']
    PointCloud2 = typestore.types['sensor_msgs/msg/PointCloud2']
    CustomMsg = typestore.types['livox_ros_driver2/msg/CustomMsg']
    
    print(f"Reading recording from: {input_path}")
    print(f"Output bag will be written to: {output_path}")
    print(f"Extracting first {duration_seconds} seconds...")
    
    # Read input recording
    with Reader(input_path) as reader:
        # Get start time
        start_time = None
        end_time = None
        duration_ns = int(duration_seconds * 1e9)
        
        # Find start time from first message
        for connection, timestamp, rawdata in reader.messages():
            if start_time is None:
                start_time = timestamp
                end_time = start_time + duration_ns
                print(f"Start time: {start_time} ns")
                print(f"End time: {end_time} ns")
            break
        
        if start_time is None:
            print("Error: No messages found in recording")
            return False
        
        # Create output bag
        with Writer(output_path) as writer:
            # Add connections for topics we'll write
            image_conn = writer.add_connection(
                '/image_raw',
                Image.__msgtype__,
                typestore=typestore
            )
            camera_info_conn = writer.add_connection(
                '/camera_info',
                CameraInfo.__msgtype__,
                typestore=typestore
            )
            pointcloud2_conn = writer.add_connection(
                '/livox/point2',
                PointCloud2.__msgtype__,
                typestore=typestore
            )
            
            # Process messages
            image_count = 0
            lidar_count = 0
            camera_info_count = 0
            pointcloud2_count = 0
            
            # Reset reader to beginning
            reader.close()
            reader.open(input_path)
            
            for connection, timestamp, rawdata in reader.messages():
                # Skip messages outside duration
                if timestamp < start_time or timestamp > end_time:
                    continue
                
                # Process /image_raw
                if connection.topic == '/image_raw':
                    try:
                        image_msg = deserialize_cdr(rawdata, connection.msgtype)
                        # Write image
                        writer.write(image_conn, timestamp, rawdata)
                        image_count += 1
                        
                        # Create and write camera_info
                        camera_info_dict = create_camera_info_from_image(image_msg)
                        camera_info_raw = serialize_cdr(camera_info_dict, CameraInfo.__msgtype__)
                        writer.write(camera_info_conn, timestamp, camera_info_raw)
                        camera_info_count += 1
                    except Exception as e:
                        print(f"Warning: Error processing image message: {e}")
                        continue
                
                # Process /livox/lidar (CustomMsg)
                elif connection.topic == '/livox/lidar':
                    try:
                        custom_msg = deserialize_cdr(rawdata, connection.msgtype)
                        # Convert to PointCloud2
                        pointcloud2_dict = convert_custommsg_to_pointcloud2(custom_msg)
                        pointcloud2_raw = serialize_cdr(pointcloud2_dict, PointCloud2.__msgtype__)
                        writer.write(pointcloud2_conn, timestamp, pointcloud2_raw)
                        pointcloud2_count += 1
                        lidar_count += 1
                    except Exception as e:
                        print(f"Warning: Error processing lidar message: {e}")
                        continue
            
            print("\n" + "="*70)
            print("Conversion completed!")
            print("="*70)
            print(f"Images processed: {image_count}")
            print(f"CameraInfo created: {camera_info_count}")
            print(f"Lidar messages processed: {lidar_count}")
            print(f"PointCloud2 messages created: {pointcloud2_count}")
            print(f"\nOutput bag: {output_path}")
            print("="*70)
            
            return True


def main():
    parser = argparse.ArgumentParser(
        description='Create calibration bag from MCAP recording',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
    python3 create_calibration_bag.py /path/to/recording_20251210_165248 --output calibration_bag --duration 5
        """
    )
    parser.add_argument('input', type=str, help='Input MCAP recording directory path')
    parser.add_argument('--output', '-o', type=str, default=None,
                       help='Output bag directory path (default: <input>_calibration)')
    parser.add_argument('--duration', '-d', type=float, default=5.0,
                       help='Duration to extract in seconds (default: 5.0)')
    
    args = parser.parse_args()
    
    # Determine output path
    if args.output is None:
        input_path = Path(args.input)
        output_path = input_path.parent / f"{input_path.name}_calibration"
    else:
        output_path = Path(args.output)
    
    # Create calibration bag
    success = create_calibration_bag(args.input, output_path, args.duration)
    
    if not success:
        sys.exit(1)


if __name__ == '__main__':
    main()
