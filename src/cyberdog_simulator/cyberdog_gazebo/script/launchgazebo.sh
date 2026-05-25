#!/bin/bash

# 默认参数
WORLD="race"
USE_LIDAR="false"
USE_CAMERA="false"
INIT_X="${INIT_X:-0.0}"
INIT_Y="${INIT_Y:-0.0}"
INIT_YAW="${INIT_YAW:-0.0}"

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --world)
            WORLD="$2"
            shift 2
            ;;
        --lidar)
            USE_LIDAR="true"
            shift
            ;;
        --camera)
            USE_CAMERA="true"
            shift
            ;;
        *)
            echo "未知参数: $1"
            echo "用法: $0 [--world <name>] [--lidar] [--camera]"
            exit 1
            ;;
    esac
done

source /opt/ros/galactic/setup.bash
source install/setup.bash
chmod +x src/cyberdog_simulator/cyberdog_gazebo/script/gazebolauncher.py

echo "启动 Gazebo 仿真..."
echo "地图: $WORLD"
echo "激光雷达: $USE_LIDAR"
echo "RGB摄像头: $USE_CAMERA"

python3 src/cyberdog_simulator/cyberdog_gazebo/script/gazebolauncher.py \
    ros2 launch cyberdog_gazebo race_gazebo.launch.py \
    wname:=$WORLD use_lidar:=$USE_LIDAR use_camera:=$USE_CAMERA initial_x:=$INIT_X initial_y:=$INIT_Y initial_yaw:=$INIT_YAW
