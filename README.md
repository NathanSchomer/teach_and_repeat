# Teach and Repeat

## Overview

**teach_and_repeat** is a ROS 2 package for implementing teach-and-repeat navigation using RGB-D images and odometry. It provides an action server that records synchronized sensor data (color images, depth images, and odometry) during a "teach" phase, and can later use this data for localization or navigation.

## Features

- Synchronized subscription to color, depth, and odometry topics
- Keypoint extraction and descriptor computation using OpenCV (ORB)
- Frame caching and saving to disk (YAML format)
- Action interface for starting/stopping the teach process

## Dependencies

- ROS 2 (tested on Jazzy)
- OpenCV
- [DBoW2](https://github.com/dorian3d/DBoW2) (included as a submodule)
- `cv_bridge`
- `sensor_msgs`, `nav_msgs`, `std_msgs`

## How to use this code

### 1. Move into ros workspace
```
cd ~/ros2_ws
```

### 2. Clone respository/extract archive into ros workspace and update submodules
```
git clone https://github.com/nathanschomer/teach_and_repeat.git ./src/teach_and_repeat
bash ./src/teach_and_repeat/utils/setup.sh
```

### 3. Build and source
```
colcon build
source ./install/setup.bash
```

### 5. Launch Teacher

```
ros2 launch teach_and_repeat teach_server.py
```

### 6. Send Goal to Teacher

this will save key frames to ./frames.yml

```
ros2 action send_goal /teach teach_and_repeat_interfaces/action/Teach "{path_name: 'frames.yml'}"
```
