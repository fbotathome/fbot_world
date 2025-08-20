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

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/fbot_vision/fr/object_recognition` | [`Detection3DArray`](fbot_vision_msgs/msg/Detection3DArray.msg) | 3D object detections |
| `/fbot_vision/pt/tracking3D` | [`Detection3DArray`](fbot_vision_msgs/msg/Detection3DArray.msg) | 3D person tracking |
| `/fbot_vision/fr/face_recognition` | [`Detection3DArray`](fbot_vision_msgs/msg/Detection3DArray.msg) | 3D face recognition |
| `/fbot_vision/vlm/question_answering/query` | [`VLMQuestion`](fbot_vision_msgs/msg/VLMQuestion.msg) | VLM questions |
| `/fbot_vision/vlm/question_answering/answer` | [`VLMAnswer`](fbot_vision_msgs/msg/VLMAnswer.msg) | VLM responses |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/fbot_vision/fr/object_start` | [`std_srvs/Empty`](http://docs.ros.org/en/api/std_srvs/html/srv/Empty.html) | Start object detection |
| `/fbot_vision/fr/object_stop` | [`std_srvs/Empty`](http://docs.ros.org/en/api/std_srvs/html/srv/Empty.html) | Stop object detection |
| `/fbot_vision/pt/start` | [`std_srvs/Empty`](http://docs.ros.org/en/api/std_srvs/html/srv/Empty.html) | Start person tracking |
| `/fbot_vision/pt/stop` | [`std_srvs/Empty`](http://docs.ros.org/en/api/std_srvs/html/srv/Empty.html) | Stop person tracking |
| `/fbot_vision/vlm/question_answering/query` | [`VLMQuestionAnswering`](fbot_vision_msgs/srv/VLMQuestionAnswering.srv) | Ask VLM questions |
| `/fbot_vision/vlm/answer_history/query` | [`VLMAnswerHistory`](fbot_vision_msgs/srv/VLMAnswerHistory.srv) | Get VLM conversation history |
| `/fbot_vision/face_recognition/people_introducing` | [`PeopleIntroducing`](fbot_vision_msgs/srv/PeopleIntroducing.srv) | Register new person |
| `/fbot_vision/face_recognition/people_forgetting` | [`PeopleForgetting`](fbot_vision_msgs/srv/PeopleForgetting.srv) | Forget an existing person |
| `/fbot_vision/look_at_description` | [`LookAtDescription3D`](fbot_vision_msgs/srv/LookAtDescription3D.srv) | Look at specific 3D detection |

---

## Contributing

1. Create a feature branch (`git checkout -b feat/amazing-feature`)
2. Commit your changes (`git commit -m 'Add amazing feature'`)
3. Push to the branch (`git push origin feat/amazing-feature`)
4. Open a Pull Request