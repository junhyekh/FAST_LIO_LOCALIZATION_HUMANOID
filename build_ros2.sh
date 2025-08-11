#!/bin/bash

# FAST-LIO Localization ROS2 Build Script
# This script builds the entire project for ROS2 Humble

set -e  # Exit on any error

echo "=== FAST-LIO Localization ROS2 Build Script ==="
echo "Building for ROS2 Humble..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if we're in the right directory
if [ ! -f "README.MD" ] || [ ! -d "open3d_loc" ]; then
    print_error "Please run this script from the project root directory"
    exit 1
fi

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    print_error "ROS2 environment not sourced. Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

if [ "$ROS_DISTRO" != "humble" ]; then
    print_warning "ROS2 distribution is $ROS_DISTRO, but this project is designed for Humble"
fi

# Check if colcon is available
if ! command -v colcon &> /dev/null; then
    print_error "colcon not found. Please install it: sudo apt install python3-colcon-common-extensions"
    exit 1
fi

print_status "Initializing submodules..."
git submodule update --init --recursive

print_status "Installing ROS2 dependencies..."
rosdep install --from-paths . --ignore-src -y

# Build Livox ROS2 driver first
print_status "Building Livox ROS2 driver..."
cd third_party/livox_ros_driver2
if [ -d "build" ] || [ -d "install" ]; then
    print_warning "Cleaning previous build..."
    rm -rf build install log
fi
colcon build --symlink-install
source install/setup.bash
cd ../..

# Build FAST-LIO ROS2
print_status "Building FAST-LIO ROS2..."
cd third_party/FAST_LIO_ROS2
if [ -d "build" ] || [ -d "install" ]; then
    print_warning "Cleaning previous build..."
    rm -rf build install log
fi
colcon build --symlink-install
source install/setup.bash
cd ../..

# Build main localization package
print_status "Building main localization package..."
if [ -d "build" ] || [ -d "install" ]; then
    print_warning "Cleaning previous build..."
    rm -rf build install log
fi

# Check if Open3D path is set correctly
if [ ! -f "open3d_loc/CMakeLists.txt" ]; then
    print_error "open3d_loc/CMakeLists.txt not found"
    exit 1
fi

# Try to build
colcon build --symlink-install --packages-select open3d_loc

print_status "Build completed successfully!"
print_status "To use the system, run:"
echo "  source install/setup.bash"
echo "  ros2 launch open3d_loc open3d_loc_g1.launch.py"

# Check if build was successful
if [ -d "install" ]; then
    print_status "Installation directory created successfully"
else
    print_error "Build failed - installation directory not created"
    exit 1
fi

print_status "Build script completed!"
