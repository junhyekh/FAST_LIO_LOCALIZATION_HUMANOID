#!/usr/bin/env bash
source /root/GP/src/FAST_LIO_LOCALIZATION_HUMANOID/install/setup.bash
ros2 launch open3d_loc open3d_loc_g1.launch.py rviz:=True map_path:=/root/GP/map_build/0212_align.pcd