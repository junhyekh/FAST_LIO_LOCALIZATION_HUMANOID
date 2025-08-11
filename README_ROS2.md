# FAST-LIO Localization for Humanoid Robots - ROS2 Version

## NOTE
This README is automatically generated through GPT. Not fully tested yet.

## Overview

This is the ROS2 Humble version of the FAST-LIO localization system for humanoid robots. The system provides robust localization based on offline pointcloud maps using Open3D for point cloud registration.

## Features

- **ROS2 Humble Support**: Fully compatible with ROS2 Humble
- **Robust Localization**: Handles rough initial poses and provides robust localization
- **Offline Map Support**: Uses pre-built pointcloud maps to avoid accumulated errors
- **Open3D Integration**: Leverages Open3D for efficient point cloud registration
- **Kalman Filtering**: Includes Kalman filtering for pose estimation smoothing

## Prerequisites

### 1.1 Ubuntu and ROS2
- **Ubuntu 22.04** for ROS2 Humble
- Install ROS2 Humble following the [official guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### 1.2 System Dependencies
```bash
# Install required packages
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libpcl-dev \
    python3-colcon-common-extensions \
    python3-vcstool
```

### 1.3 Open3D
We recommend using the precompiled Open3D library. You can download it from:
- [Baidu Netdisk](https://pan.baidu.com/s/1vTLXVYJ6JBlbhNpDf87Cdg?pwd=spdg) (pwd: spdg)
- `open3d141.zip` for x86 architecture
- `open3d141_arm.zip` for ARM architecture

Extract and update the path in `open3d_loc/CMakeLists.txt`:
```cmake
set(Open3D_DIR "/path/to/your/open3d141/lib/cmake/Open3D")
```

Alternatively, build Open3D from source following the [official documentation](https://www.open3d.org/docs/release/compilation.html).

### 1.4 Livox-SDK2
Follow the [Livox-SDK2 installation guide](https://github.com/Livox-SDK/Livox-SDK2)

### 1.5 Livox ROS2 Driver
The project includes the Livox ROS2 driver as a submodule. Make sure to build it:
```bash
cd third_party/livox_ros_driver2
colcon build --symlink-install
source install/setup.bash
```

## 2. Build

### 2.1 Create ROS2 Workspace
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID.git
cd ..

# Initialize and update submodules
git submodule update --init --recursive
```

### 2.2 Build Dependencies
```bash
# Install ROS2 dependencies
rosdep install --from-paths src --ignore-src -y

# Build Livox ROS2 driver first
cd src/FAST_LIO_LOCALIZATION_HUMANOID/third_party/livox_ros_driver2
colcon build --symlink-install
source install/setup.bash
cd ../../../..

# Build FAST-LIO ROS2
cd src/FAST_LIO_LOCALIZATION_HUMANOID/third_party/FAST_LIO_ROS2
colcon build --symlink-install
source install/setup.bash
cd ../../../..
```

### 2.3 Build Main Package
```bash
# Build the main localization package
colcon build --symlink-install --packages-select open3d_loc

# Source the workspace
source install/setup.bash
```

## 3. Usage

### 3.1 Launch Localization
```bash
# Launch the localization system
ros2 launch open3d_loc open3d_loc_g1.launch.py

# With custom parameters
ros2 launch open3d_loc open3d_loc_g1.launch.py \
    config_path:=/path/to/config.yaml \
    map_path:=/path/to/map.ply
```

### 3.2 Launch FAST-LIO Mapping
```bash
# Launch FAST-LIO for mapping
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

### 3.3 Launch Livox Driver
```bash
# Launch Livox ROS2 driver (example for MID360)
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

## 4. Configuration

### 4.1 Localization Parameters
Key parameters in the launch file:
- `path_map`: Path to the offline map file (.ply format)
- `voxelsize_coarse`: Coarse voxel size for registration (default: 0.15)
- `voxelsize_fine`: Fine voxel size for registration (default: 0.1)
- `threshold_fitness`: Registration fitness threshold (default: 0.5)
- `loc_frequence`: Localization frequency in Hz (default: 2.5)

### 4.2 FAST-LIO Configuration
Edit `third_party/FAST_LIO_ROS2/config/mid360.yaml` for your specific LiDAR:
- `lid_topic`: LiDAR point cloud topic
- `imu_topic`: IMU topic
- `extrinsic_T`: Translation extrinsic parameters
- `extrinsic_R`: Rotation extrinsic parameters

## 5. Topics

### 5.1 Subscribed Topics
- `/Odometry`: FAST-LIO odometry output
- `/cloud_registered`: Registered point cloud from FAST-LIO
- `/initialpose`: Initial pose estimate (for pose initialization)

### 5.2 Published Topics
- `/baselink2map`: Base link to map transform
- `/motionlink2map`: Motion link to map transform
- `/odom2map`: Odom to map transform
- `/localization_3d`: 3D localization pose
- `/localization_3d_confidence`: Localization confidence
- `/map`: Published map point cloud
- `/scan`: Current scan point cloud

## 6. TF Frames

The system uses the following TF frame hierarchy:
- `map` ← `odom` ← `camera_init` ← `base_link` ← `imu_link`
- `motion_link` ← `base_link`

## 7. Troubleshooting

### 7.1 Build Issues
- Ensure Open3D path is correctly set in CMakeLists.txt
- Make sure all submodules are properly initialized
- Check that ROS2 Humble is properly installed

### 7.2 Runtime Issues
- Verify LiDAR and IMU topics are correctly configured
- Check that the map file path is accessible
- Ensure TF frames are properly configured

### 7.3 Performance Issues
- Adjust voxel sizes for better performance/accuracy trade-off
- Modify localization frequency based on your requirements
- Consider reducing point cloud density for faster processing

## 8. Contributing

This is a ROS2 port of the original ROS1 implementation. When contributing:
- Follow ROS2 best practices
- Use ament_cmake build system
- Ensure compatibility with ROS2 Humble
- Test with the provided launch files

## 9. License

BSD License - see LICENSE file for details.

## 10. Acknowledgments

- Original FAST-LIO authors
- Open3D development team
- Livox SDK team
- ROS2 community

## 11. References

- [FAST-LIO ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)
- [Livox ROS2 Driver](https://github.com/Livox-SDK/livox_ros_driver2)
- [Open3D Documentation](https://www.open3d.org/docs/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
