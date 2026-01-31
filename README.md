# LiDAR Object Processing & Visualization

[![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)](https://www.python.org/)
[![ROS2](https://img.shields.io/badge/ROS2-F05032?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/foxy/index.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)](https://opensource.org/licenses/MIT)

## Overview
- This repository contains **LiDAR preprocessing and visualization tools** for 3D point cloud data.  
- The project provides **Python scripts** for point cloud handling and **ROS 2 nodes** for visualization and message handling.  
- It is intended as a **foundation for future LiDAR object detection pipelines**.

![LiDAR](https://keymakr.com/blog/content/images/2023/06/3D-Point-cloud.jpg)

## Features
- LiDAR point cloud preprocessing in Python
- Visualization of LiDAR messages
- ROS 2 nodes for real-time LiDAR message handling
- Modular and readable code structure
- Python and C++ support in ROS 2

## Repository Structure

```text
.
├── machine_learning_lidar_detection/src/   # Python scripts for LiDAR processing
│   ├── dataset.py                           # Loading and preprocessing LiDAR data
│   ├── main.py                              # Script for running preprocessing and visualization
│   └── visualization.py                     # LiDAR point cloud visualization utilities
├── ros2/src/
│   ├── lidar_perception/                    # ROS 2 Python package for LiDAR nodes
│   └── lidar_perception_cpp/                # ROS 2 C++ package for LiDAR nodes
├── .gitignore                               # Ignored files
└── README.md                                # Project overview and instructions
```

## Installation

- Clone the repository
```bash
git clone https://github.com/yourusername/lidar-object-processing.git
cd lidar-object-processing
```
- Install Python dependencies
```bash
pip install -r requirements.txt
```

## Usage

- Run the preprocessing and visualization:
```bash
python3 machine_learning_lidar_detection/src/main.py
```
- Individual Utilities
```bash
python3 machine_learning_lidar_detection/src/dataset.py        # Preprocess point clouds
python3 machine_learning_lidar_detection/src/visualization.py  # Visualize point clouds
```
- Python ROS 2 package:
```bash
colcon build
source install/setup.bash
ros2 launch lidar_perception lidar_perception_launch.py
```
- C++ ROS 2 package:
```bash
colcon build --packages-select lidar_perception_cpp
source install/setup.bash
ros2 run lidar_perception_cpp lidar_perception_node
```

## Limitations & Future Work

- Currently only preprocessing and visualization is implemented
- No object detection or segmentation pipeline yet
- Future improvements:
  - Integrate ML-based LiDAR object detection
  - Add tracking of objects across frames
  - Real-time ROS 2 integration with detection and planning modules

