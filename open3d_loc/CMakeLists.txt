# =============================================================================
# CMakeLists.txt for open3d_loc ROS2 Package
# 
# This file configures the build system for the FAST-LIO localization package
# that provides global localization for humanoid robots using Open3D point
# cloud registration.
# 
# Dependencies:
# - ROS2 Humble
# - Open3D (point cloud processing and registration)
# - PCL (Point Cloud Library)
# - Eigen3 (linear algebra)
# - TF2 (transform handling)
# =============================================================================

cmake_minimum_required(VERSION 3.8)
project(open3d_loc)

# Set build type to Release for optimized performance
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags for optimization and C++17 standard
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ===== Open3D Configuration =====
# Avoid hardcoding Open3D_DIR; rely on system or user-provided CMAKE_PREFIX_PATH
find_package(Open3D REQUIRED)

# ===== ROS2 Dependencies =====
# Core ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# ROS2 message packages
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# TF2 packages for transform handling
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Point cloud processing packages
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# Interface generation
# find_package(rosidl_default_generators REQUIRED)

# ===== Third-party Libraries =====
# Linear algebra library
find_package(Eigen3 REQUIRED)

# Point Cloud Library components
find_package(PCL REQUIRED COMPONENTS common io)

# ===== CUDA Toolkit (optional, for CUDA-enabled Open3D builds) =====
# If Open3D was built with CUDA and exports static CUDA libs, linking these resolves cudart/cublas symbols
find_package(CUDAToolkit QUIET)

# ===== Package Dependencies List =====
# All dependencies required by this package
set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
)

# ===== Interface Generation =====
# Generate ROS2 interfaces (messages, services, actions)
# Currently no custom interfaces defined, but structure is ready
# rosidl_generate_interfaces(${PROJECT_NAME}
#   # Add any custom messages here if needed
#   # Example: "msg/CustomMessage.msg"
# )
# ament_export_dependencies(rosidl_default_runtime)

# ===== Include Directories =====
# Set up include paths for headers and libraries
include_directories(
  ${PROJECT_SOURCE_DIR}      # Package source directory
  ${Open3D_INCLUDE_DIRS}     # Open3D headers
  include                    # Package include directory
)

# ===== Library Targets =====
# Create shared libraries for reusable components

# Open3D registration library - handles point cloud registration algorithms
add_library(open3d_registration src/open3d_registration/open3d_registration.cpp)
target_link_libraries(open3d_registration Open3D::Open3D)
ament_target_dependencies(open3d_registration ${dependencies})
if(CUDAToolkit_FOUND)
  target_link_libraries(open3d_registration CUDA::cudart CUDA::cublas CUDA::cublasLt CUDA::cusolver CUDA::nppc)
endif()

# Open3D conversions library - handles data format conversions
add_library(open3d_conversions src/open3d_conversions/open3d_conversions.cpp)
target_link_libraries(open3d_conversions Open3D::Open3D)
ament_target_dependencies(open3d_conversions ${dependencies})
if(CUDAToolkit_FOUND)
  target_link_libraries(open3d_conversions CUDA::cudart CUDA::cublas CUDA::cublasLt CUDA::cusolver CUDA::nppc)
endif()

# ===== Executable Target =====
# Main localization node executable
add_executable(global_localization_node src/global_localization_ros2.cpp)
target_link_libraries(global_localization_node Open3D::Open3D)
target_link_libraries(global_localization_node open3d_registration)
target_link_libraries(global_localization_node open3d_conversions)
ament_target_dependencies(global_localization_node ${dependencies})
if(CUDAToolkit_FOUND)
  target_link_libraries(global_localization_node CUDA::cudart CUDA::cublas CUDA::cublasLt CUDA::cusolver CUDA::nppc)
endif()

# ===== Installation Configuration =====
# Install targets to appropriate locations

# Install library targets
install(TARGETS
  open3d_registration
  open3d_conversions
  global_localization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install package directories (launch files, configs, etc.)
install(DIRECTORY
  launch      # Launch files for starting the system
  config      # Configuration files (YAML parameters)
  rviz_cfg    # RViz configuration files for visualization
  DESTINATION share/${PROJECT_NAME}
)

# ===== Export Configuration =====
# Export package information for other packages to use

# Export include directories for other packages
ament_export_include_directories(include)

# Export libraries for linking
ament_export_libraries(open3d_registration open3d_conversions)

# Export dependencies for other packages
ament_export_dependencies(${dependencies})

# ===== Package Finalization =====
# Required call to finalize the ROS2 package
ament_package()