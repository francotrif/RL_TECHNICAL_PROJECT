# Autonomous Airport Baggage Handling System

A multi-robot coordination system for autonomous airport baggage handling using ROS2, Gazebo, Nav2, and ArUco marker detection.


## 🎯 Overview

This project implements an autonomous baggage handling system for airport environments, featuring:
- **1 Fra2mo mobile robot** for autonomous navigation between aircraft stations
- **6 Armando manipulator robots** for coordinated baggage handling
- **ArUco marker detection** for precise positioning and station identification
- **Nav2 stack** for autonomous navigation and path planning
- **Event-driven architecture** for scalable multi-robot coordination

The system demonstrates seamless integration of mobile navigation, computer vision, and multi-manipulator coordination in a simulated airport environment.


---

## 🏗️ System Architecture

### ROS2 Packages

The project consists of 3 main ROS2 packages:

#### 1. **ros2_fra2mo** (Mobile Robot Package)
- Fra2mo robot URDF model
- Custom airport world (Gazebo)
- Navigation stack integration (Nav2)
- SLAM & mapping tools
- Mission execution nodes

#### 2. **final_project** (Manipulation Package)
- Armando manipulator URDF model
- Multi-robot spawning system
- Controller nodes (event-driven)
- Baggage handling sequences

#### 3. **aruco_ros** (Computer Vision - External Library)
- ArUco marker detection
- 6-DOF pose estimation
- Real-time marker tracking

---


## 🎮 Usage

### Quick Start Guide

Launch the system using **7 terminals** in the following order:

#### **Terminal 1: Gazebo World & Fra2mo**
Launches the custom airport environment and spawns the Fra2mo mobile robot.

```bash
ros2 launch ros2_fra2mo gazebo_fra2mo.launch.py
```

**What it does:**
- Loads custom airport world (20m x 14m)
- Spawns 6 aircraft models with ArUco markers
- Spawns Fra2mo robot 
- Activates sensors (Lidar, Camera, IMU)

---

#### **Terminal 2: Armando Manipulators Spawn**
Spawns 6 Armando manipulator robots at designated stations.

```bash
ros2 launch final_project spawn_6_armando.launch.py
```

**What it does:**
- Sequential spawning of 6 Armando robots (5s intervals)
- Activates joint and gripper controllers

**Wait ~60 seconds for all robots to spawn completely.**

---

#### **Terminal 3: Armando Controllers Activation**
Launches 6 times a controller node that listen for mission triggers.

```bash
ros2 launch final_project armando_controllers.launch.py
```

**What it does:**
- Spawns 6 independent controller nodes (armando_controller_1 ... armando_controller_6)
- All controllers subscribe to `/target_station` and `/num_bags` topics
- Event-driven activation: only the matching station controller will execute
- Remains active between missions (no restart needed)

---

#### **Terminal 4: Navigation Stack**
Activates the Nav2 navigation stack with AMCL localization.

```bash
ros2 launch ros2_fra2mo fra2mo_navigation.launch.py
```

**What it does:**
- Launches RViz2 for visualization
- Loads pre-built map (`airport_map.yaml`)
- Starts AMCL localization 
- Activates Nav2 components:


---

#### **Terminal 5: Mission Execution**
Executes a mission to a specific station with a defined number of bags.

```bash
ros2 launch ros2_fra2mo mission_to_station.launch.py station:=X num_bags:=Y
```

**Parameters:**
- `station`: Target aircraft station (1-6)
  - Station 1: (3.8, -6.4) → ArUco ID 1
  - Station 2: (3.8, +6.4) → ArUco ID 2
  - Station 3: (9.8, -6.4) → ArUco ID 3
  - Station 4: (9.8, +6.4) → ArUco ID 4
  - Station 5: (15.8, -6.4) → ArUco ID 5
  - Station 6: (15.8, +6.4) → ArUco ID 6
- `num_bags`: Number of baggage items to handle (1-N)

**Example:**
```bash
ros2 launch ros2_fra2mo mission_to_station.launch.py station:=3 num_bags:=5
```

**What it does:**
- Activates ArUco detection for marker ID (station)
- Sends navigation goal to Nav2 (target waypoint)
- Monitors navigation progress
- On arrival, publishes triggers:
  - `/target_station` = 3
  - `/num_bags` = 5
- Activates Armando_3 controller automatically

---

#### **Terminal 6: ArUco Detection Visualization** 
Displays real-time ArUco marker detection from camera feed.

```bash
ros2 run rqt_image_view rqt_image_view
```

**Then select topic:** `/aruco_single/result`

**What it does:**
- Visualizes camera feed with detected ArUco markers
- Shows bounding boxes and marker IDs
- Useful for debugging detection issues

---

#### **Terminal 7: ArUco Pose Monitor** 
Subscribes to ArUco pose topic and prints detection data.

```bash
ros2 run ros2_fra2mo aruco_pose_subscriber.py
```

**What it does:**
- Subscribes to `/aruco_single/pose` topic
- Prints marker pose (position + orientation)
- Displays detection timestamp and frame_id
- Useful for monitoring pose estimation accuracy

---

### Mission Examples

#### Example 1: Station 1 with 3 bags
```bash
ros2 launch ros2_fra2mo mission_to_station.launch.py station:=1 num_bags:=3
```
**Result:** Fra2mo navigates to station 1, Armando_1 handles 3 bags (~150s total)

---

#### Example 2: Station 4 with 2 bags
```bash
ros2 launch ros2_fra2mo mission_to_station.launch.py station:=4 num_bags:=2
```
**Result:** Fra2mo navigates to station 4, Armando_4 handles 2 bags (~120s total)

---

#### Example 3: Sequential missions (no restart needed!)
```bash
# Mission 1
ros2 launch ros2_fra2mo mission_to_station.launch.py station:=1 num_bags:=3
# Wait for completion...

# Mission 2 (path automatically replanned)
ros2 launch ros2_fra2mo mission_to_station.launch.py station:=6 num_bags:=5
# Wait for completion...

# Mission 3
ros2 launch ros2_fra2mo mission_to_station.launch.py station:=3 num_bags:=2
```

**Note:** Only Terminal 5 needs to be restarted between missions. All other components remain active!

---


### Key Topics

#### Published Topics
- `/target_station` (std_msgs/Int32) - Triggers Armando controller
- `/num_bags` (std_msgs/Int32) - Specifies baggage quantity
- `/navigation_status` (std_msgs/String) - Navigation state
- `/aruco_single/pose` (geometry_msgs/PoseStamped) - Marker pose
- `/cmd_vel` (geometry_msgs/Twist) - Fra2mo velocity commands

#### Subscribed Topics
- `/fra2mo/camera` (sensor_msgs/Image) - Camera feed for ArUco
- `/fra2mo/camera_info` (sensor_msgs/CameraInfo) - Camera calibration
- `/scan` (sensor_msgs/LaserScan) - Lidar data for navigation
- `/armando_N/position_controller/commands` - Joint control
- `/armando_N/gripper_controller/commands` - Gripper control


---

## 🎥 Demo Video

[![Demo Video](https://youtu.be/DqaR3WdtpHM?si=FV887ZaNlWFL4FWx)](https://youtu.be/DqaR3WdtpHM?si=FV887ZaNlWFL4FWx)

**Watch the full system demonstration** showing:
- Complete system setup (all 7 terminals)
- Mission to Station 2 (3 bags)
- Mission to Station 5 (2 bags)
- RViz monitoring and visualization
- Sequential mission execution

---



## 📚 Dependencies

### ROS2 Packages
- `nav2_bringup`
- `slam_toolbox`
- `ros2_control`
- `ros2_controllers`
- `gz_ros2_control`
- `aruco_ros`
- `cv_bridge`
- `image_transport`

### Python Libraries
- `rclpy`
- `nav2_simple_commander`
- `opencv-python`
- `numpy`

---

#

## 👨‍💻 Author

**Franco Trifuoggi**
- GitHub: [@francotrif](https://github.com/francotrif)
- Email: franco.trifuoggi@live.it


**⭐ If you find this project useful, please consider giving it a star!**
