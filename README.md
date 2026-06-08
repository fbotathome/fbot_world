<div align="center">
<img width="3805" height="719" alt="fbot_world" src="https://github.com/user-attachments/assets/04408a27-c5b0-40f3-8424-e21c23659687" />

![UBUNTU](https://img.shields.io/badge/UBUNTU-22.04-orange?style=for-the-badsge&logo=ubuntu)
![python](https://img.shields.io/badge/python-3.10-blue?style=for-the-badsge&logo=python)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badsge&logo=ros)
[![Last Commit](https://img.shields.io/github/last-commit/fbotathome/fbot_world.svg?style=for-the-badsge)](https://github.com/fbotathome/fbot_world/commits/main)
[![GitHub issues](https://img.shields.io/github/issues/fbotathome/fbot_world)](https://github.com/fbotathome/fbot_world/issues)
[![GitHub pull requests](https://img.shields.io/github/issues-pr/fbotathome/fbot_world)](https://github.com/fbotathome/fbot_world/pulls)
[![Contributors](https://img.shields.io/github/contributors/fbotathome/fbot_world.svg)](https://github.com/fbotathome/fbot_world/graphs/contributors)

**A ROS 2 world system for robotics applications featuring new poses.**

[Overview](#overview) • [Architecture](#architecture) • [Installation](#installation) • [Usage](#usage) • [fbot_world message and services](#fbot_world-message-and-services) • [Contributing](#contributing)

</div>

## Overview
######
```fbot_world``` is a ROS 2 package for managing poses in world map scenarios. It allows loading and saving poses for reuse across different robotic applications.  

   It was designed for the RoboCup@Home and the robot BORIS competition but is adaptable to various robotics scenarios.  

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
Loads a poses/rooms YAML file and serves it through the `/fbot_world/*` services
(see the table below). It also publishes RViz markers of every room and object
sub-area on `/fbot_world/debug_markers`, and answers "which room am I in?" queries
via `/fbot_world/get_room`.
```bash
# Launch pose node
ros2 launch fbot_world pose.launch.py config_file_name:=file_name_without_dot_yaml
```

### Pose Writer Node
Interactively records the robot's **base poses** and saves them under the
`targets` group of a YAML file. Drive/teleop the robot to a spot, give the pose a
name, and the current `/amcl_pose` is captured (position + orientation). New poses
are appended to the chosen file, so existing entries are preserved.
```bash
ros2 run fbot_world pose_writer
```

### Place Pose Writer Node
Records fixed **end-effector "place" poses** (e.g. a shelf or bin drop point) under
the `place_poses` group. It reads the gripper pose from TF (in the `map` frame) and
**disables arm torque** so you can hand-guide the arm to the target, name it, and
capture it; torque is re-enabled on exit.

> ⚠️ The arm goes limp when torque is cut — hold it before confirming the prompt.

Configurable via parameters: `reference_frame` (default `map`), `ee_frame`
(default `wx200/ee_gripper_link`), `group_set` (default `place_poses`),
`robot_name` (default `wx200`) and `torque_group` (default `arm`).
```bash
ros2 run fbot_world place_pose_writer
```

### Room Writer Node
Annotates **room boundaries and object sub-areas** by clicking points in RViz, and
writes them to the `rooms` section of a YAML file — the same structure the Pose
Node loads (used by `/fbot_world/get_room`). Only the `rooms` section is written;
any existing `poses`/`targets` are left untouched.

Workflow: launch navigation with your map and RViz, run the node, then use the
RViz **Publish Point** tool to click each corner of a room. The in-progress polygon
is previewed live; when you close it, the finished area is drawn filled, labelled
and coloured. You can then type the pose names that belong to the room and draw any
object sub-areas the same way.

In RViz, add a **MarkerArray** display on `/room_writer/preview` (fixed frame
`map`). Terminal commands while drawing: `Enter` closes the polygon, `u` undoes the
last point, `r` resets, `a` aborts.
```bash
ros2 run fbot_world room_writer
```

## fbot_world message and services
### Services

| Service | Type | Description |
|---------|------|-------------|
| `/fbot_world/get_pose` | [`GetPose`](fbot_world_msgs/srv/GetPose.srv) | Service callback to return the pose and size for a requested target key |
| `/fbot_world/get_set` | [`GetPoseFromSet`](fbot_world_msgs/srv/GetPoseFromSet.srv) | Service callback to return all poses for a requested group name key |
| `/fbot_world/get_groups_names` | [`GetSets`](fbot_world_msgs/srv/GetSets.srv) | Service callback to return all gorup names in poses and rooms names with postions and objetcs in yaml file |
| `/fbot_world/get_room ` | [`GetRoom`](fbot_world_msgs/srv/GetRoom.srv) | Service that returns the room and place name where the robot is, based on its current position |

---

## Contributing

1. Create a feature branch (`git checkout -b feat/amazing-feature`)
2. Commit your changes (`git commit -m 'Add amazing feature'`)
3. Push to the branch (`git push origin feat/amazing-feature`)
4. Open a Pull Request