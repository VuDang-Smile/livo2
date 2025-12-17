# Manual Commands for Bag Mapping

## Quick Start

### Option 1: Using the script (Recommended)

```bash
cd /home/ubuntu/Documents/code/livo2
# With rate 0.5x (slow playback)
./scripts/run_bag_mapping.sh /path/to/bag_folder True 0.5

# With rate 1.0x (normal speed)
./scripts/run_bag_mapping.sh /path/to/bag_folder True 1.0
```

### Option 2: Manual commands (Step by step)

#### Terminal 1: Start Mapping Node

```bash
# Source environment
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/Documents/code/livo2/dependencies/drive_ws/install/setup.sh  # Optional, for CustomMsg
source /home/ubuntu/Documents/code/livo2/ws/install/setup.sh

# Launch mapping (with RViz2)
ros2 launch fast_livo mapping_mid360_equirectangular.launch.py use_rviz:=True

# Or without RViz2
ros2 launch fast_livo mapping_mid360_equirectangular.launch.py use_rviz:=False
```

#### Terminal 2: Play Bag File

```bash
# Source environment
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/Documents/code/livo2/dependencies/drive_ws/install/setup.sh  # Optional, for CustomMsg
source /home/ubuntu/Documents/code/livo2/ws/install/setup.sh

# Play bag with rate 0.5x (slow playback)
ros2 bag play /path/to/bag_folder --rate 0.5

# Play bag with normal speed
ros2 bag play /path/to/bag_folder --rate 1.0

# With loop option
ros2 bag play /path/to/bag_folder --rate 0.5 --loop
```

## Commands Summary

### 1. Launch Mapping Only

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/Documents/code/livo2/ws/install/setup.sh
ros2 launch fast_livo mapping_mid360_equirectangular.launch.py use_rviz:=True
```

### 2. Play Bag Only

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/Documents/code/livo2/ws/install/setup.sh
ros2 bag play /path/to/bag_folder
```

### 3. Combined (in background)

```bash
# Terminal 1: Start mapping
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/Documents/code/livo2/ws/install/setup.sh
ros2 launch fast_livo mapping_mid360_equirectangular.launch.py use_rviz:=True &
MAPPING_PID=$!

# Wait for mapping to initialize
sleep 5

# Terminal 2: Play bag
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/Documents/code/livo2/ws/install/setup.sh
ros2 bag play /path/to/bag_folder &
BAG_PID=$!

# To stop:
# kill $MAPPING_PID $BAG_PID
```

## Configuration Files

- **Unified config**: `ws/src/FAST-LIVO2/config/mid360_equirectangular.yaml` (includes camera and mapping parameters)
- **Launch file**: `ws/src/FAST-LIVO2/launch/mapping_mid360_equirectangular.launch.py`

## Required Topics

Make sure your bag file contains:
- `/livox/lidar` (CustomMsg)
- `/livox/imu`
- `/image_raw` (equirectangular image, 2880x1440)

## Troubleshooting

### Check if topics are available:
```bash
ros2 topic list
ros2 topic echo /livox/lidar --once
ros2 topic echo /image_raw --once
```

### Check bag info:
```bash
ros2 bag info /path/to/bag_folder
```

### Kill processes:
```bash
# Find processes
ps aux | grep fastlivo_mapping
ps aux | grep "bag play"

# Kill by name
pkill -f fastlivo_mapping
pkill -f "bag play"
```
