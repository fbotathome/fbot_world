<div align="center">
<img width="3805" height="719" alt="fbot_world" src="https://github.com/user-attachments/assets/04408a27-c5b0-40f3-8424-e21c23659687" />

![UBUNTU](https://img.shields.io/badge/UBUNTU-22.04-orange?style=for-the-badsge&logo=ubuntu)
![python](https://img.shields.io/badge/python-3.10-blue?style=for-the-badsge&logo=python)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badsge&logo=ros)
[![Last Commit](https://img.shields.io/github/last-commit/fbotathome/fbot_world.svg?style=for-the-badsge)](https://github.com/fbotathome/fbot_world/commits/main)
[![GitHub issues](https://img.shields.io/github/issues/fbotathome/fbot_world)](https://github.com/fbotathome/fbot_world/issues)
[![GitHub pull requests](https://img.shields.io/github/issues-pr/fbotathome/fbot_world)](https://github.com/fbotathome/fbot_world/pulls)
[![Contributors](https://img.shields.io/github/contributors/fbotathome/fbot_world.svg)](https://github.com/fbotathome/fbot_world/graphs/contributors)

**A ROS 2 vision system for robotics applications featuring object detection, face recognition, person tracking, and vision-language model integration.**

[Overview](#overview) • [Architecture](#architecture) • [Installation](#installation) • [Usage](#usage) • [fbot_world message and services](#fbot_world-message-and-services) • [Contributing](#contributing)

</div>

## Overview
######
```fbot_world``` is a ROS 2 package suite designed for robotic applications in real world scenarios. 

. It was designed for the RoboCup@Home and the robot BORIS competition but is adaptable to various robotics scenarios.

---

## Architecture

The system consists of three main packages:

```
fbot_world/
├── 📁 fbot_world/          # Core fbot_world files
|   ├── 📁 fbot_world/      # node files
│   ├── 📁 launch/      # launch files
│   ├── 📁 scripts/    # script files
└── 📁 fbot_world_msgs/          # Custom ROS message definitions
```

---

## Installation

### Prerequisites

- ROS2 Humble
- Python 3.10+
- Ubuntu 22.04
- Dependencies listed in `package.xml` and `requirements.txt`

### Setup

1. **Clone the repository into your ROS workspace:**
   ```bash
   cd ~/fbot_ws/src
   git clone https://github.com/fbotathome/fbot_world.git
   ```

2. **Install dependencies:**
   ```bash
   cd ~/fbot_ws
   sudo rosdep init  # Skip if already initialized
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   pip install -r src/fbot_world/requirements.txt
   ```

3. **Build the workspace:**
   ```bash
   cd ~/fbot_ws
   colcon build --packages-select fbot_world fbot_world_msgs
   source install/setup.bash
   ```

---

## Usage

### Pose Node
```bash
# Launch pose node
ros2 launch fbot_world pose.launch.py config_file_name:=file_name_without_dot_yaml
```

### Pose Writer Node
```bash
# Launch YOLO tracker with pose estimation
ros2 run fbot_world pose_writer
```

## fbot_world message and services
### Services

| Service | Type | Description |
|---------|------|-------------|
| `/fbot_world/get_pose` | [`GetPose`](fbot_world_msgs/srv/GetPose.srv) | Service callback to return the pose and size for a requested target key |
| `/fbot_world/get_set` | [`GetPoseFromSet`](fbot_world_msgs/srv/GetPoseFromSet.srv) | Service callback to return all poses for a requested group name key |
| `/fbot_world/get_groups_names` | [`GetSets`](fbot_world_msgs/srv/GetSets.srv) | Service callback to return all gorup names in redis |

---

## Contributing

1. Create a feature branch (`git checkout -b feat/amazing-feature`)
2. Commit your changes (`git commit -m 'Add amazing feature'`)
3. Push to the branch (`git push origin feat/amazing-feature`)
4. Open a Pull Request