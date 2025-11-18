# Maze Solver on Gazebo

ROS 2 based autonomous maze solver robot simulation project. This project includes a robot simulation that performs autonomous navigation in mazes of different difficulty levels in Gazebo simulation environment using wall following algorithm.

## About the Project

This project enables a mobile robot to perform autonomous navigation in mazes using LiDAR sensor. The robot finds its way out of mazes using wall-following algorithm. The project is built on ROS 2 and Gazebo simulation environment and has a modular structure.

## Features

- Realistic physics simulation in Gazebo simulation environment
- LiDAR-based perception and obstacle detection
- Autonomous navigation with wall following algorithm
- 3 different difficulty levels (Easy, Medium, Hard)
- SLAM (Simultaneous Localization and Mapping) support
- Visualization with RViz
- Modular and extensible architecture

## Package Descriptions

### 1. `robot_simulation`
Main package where robot model and simulation environment are defined.

**Contents:**
- **URDF/Xacro files**: Robot physical model and sensor configurations
- **World files**: Maze worlds at 3 different difficulty levels (easy.sdf, medium.sdf, hard.sdf)
- **Launch files**: Launch files required to start the simulation
- **Mesh files**: 3D meshes for robot visual model

**Tasks:**
- Starting Gazebo simulation
- Spawning robot model
- Publishing TF with robot state publisher

### 2. `perception_pkg`
Perception package where sensor data is processed.

**Contents:**
- `lidar_processor_node.cpp`: Node that processes LiDAR data

**Tasks:**
- Receiving LiDAR data from `/scan` topic
- Calculating minimum distance in 4 directions (front, right, left, back)
- Sectoral scanning with 20° FOV (field of view)
- Publishing processed obstacle data to `/obstacle_data` topic

**Published Topic:**
- `/obstacle_data` (Float32MultiArray): [front_distance, right_distance, left_distance, back_distance]

### 3. `control_pkg`
Package that provides robot motion control.

**Contents:**
- `wall_follower_node.cpp`: Wall following algorithm node

**Tasks:**
- Applying right wall following algorithm using obstacle data
- Publishing robot velocity commands (`/cmd_vel`)
- Position tracking using odometry data
- State machine based control (GO, TURN RIGHT, TURN LEFT, STOP)
- Making smooth turns at 90° angles

**State Machine:**
- **GO**: Forward movement, heading correction
- **TURN_RIGHT**: 90° right turn if there is space on the right
- **TURN_LEFT**: 90° left turn if there is obstacle in front
- **STOP**: Stopping (not currently used)

### 4. `mapping_pkg`
Mapping and visualization package.

**Contents:**
- SLAM configuration (`slam.yaml`)
- RViz configuration (`rviz_config.rviz`)
- Map visualization launch file

**Tasks:**
- Real-time map creation with SLAM
- Visualizing robot's position and map in RViz

## Requirements

- **Operating System**: Ubuntu 22.04 (Jammy)
- **ROS 2**: Humble Hawksbill
- **Gazebo**: Harmonic (or compatible version)
- **Dependencies**:
  - `ros-humble-desktop`
  - `ros-humble-ros-gz-sim`
  - `ros-humble-ros-gz-bridge`
  - `ros-humble-slam-toolbox` (optional for SLAM)
  - `ros-humble-navigation2` (optional for navigation)

## Installation

### 1. Creating Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Cloning the Project
```bash
git clone https://github.com/username/maze_solver_on_gazebo.git
cd ~/ros2_ws
```

### 3. Installing Dependencies
```bash
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Building Workspace
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Basic Usage

To start the simulation (default: easy level):

```bash
ros2 launch robot_simulation bring_up.launch.py
```

To start with different difficulty levels:

```bash
# Easy level
ros2 launch robot_simulation bring_up.launch.py difficulty:=easy

# Medium level
ros2 launch robot_simulation bring_up.launch.py difficulty:=medium

# Hard level
ros2 launch robot_simulation bring_up.launch.py difficulty:=hard
```

### Starting Control Node

Start the wall following algorithm in a new terminal:

```bash
ros2 run control_pkg wall_follower_node
```

### SLAM and Visualization (Optional)

For map creation and RViz visualization:

```bash
ros2 launch mapping_pkg map_visulator.launch.py
```

## Architecture

### Node Structure

```
┌─────────────────────┐
│  Gazebo Simulation  │
│   (World + Robot)   │
└──────────┬──────────┘
           │
           │ /scan (LaserScan)
           │
┌──────────▼──────────┐
│  lidar_processor    │
│  (perception_pkg)   │
└──────────┬──────────┘
           │
           │ /obstacle_data
           │
┌──────────▼──────────┐       ┌─────────────┐
│  wall_follower      │◄──────┤   /odom     │
│  (control_pkg)      │       └─────────────┘
└──────────┬──────────┘
           │
           │ /cmd_vel
           │
┌──────────▼──────────┐
│   Robot Actuators   │
└─────────────────────┘
```

### Topic List

| Topic | Message Type | Publisher | Subscriber | Description |
|-------|-----------|-----------|---------|----------|
| `/scan` | sensor_msgs/LaserScan | Gazebo LiDAR | lidar_processor | Raw LiDAR data |
| `/obstacle_data` | std_msgs/Float32MultiArray | lidar_processor | wall_follower | Processed obstacle distances |
| `/cmd_vel` | geometry_msgs/Twist | wall_follower | Gazebo | Robot velocity commands |
| `/odom` | nav_msgs/Odometry | Gazebo | wall_follower | Robot odometry data |

### Algorithm Logic

**Wall Following Algorithm (Right-Hand Rule):**

1. **Forward Movement**: Go straight if no obstacle ahead
2. **Heading Correction**: Robot always aligns to multiples of 90°
3. **Right Priority**: Turn right if there is space on the right and sufficient progress
4. **Left Turn**: Turn left if obstacle ahead and left is clear
5. **Distance Thresholds**:
   - Minimum front distance: 0.45m
   - Right-left difference threshold: 0.20m
   - Minimum progress: 0.30m

## Development

### Changing Parameters

Adjustable parameters in `wall_follower_node.cpp`:

```cpp
float min_front_distance = 0.45;    // Front obstacle threshold
float min_side_difference = 0.20;   // Side wall detection threshold
double linear_velocity = 0.2;       // Forward velocity
double angular_velocity = 0.25;     // Turning velocity
```

In `lidar_processor_node.cpp`:

```cpp
float fov = 20.0 * M_PI / 180.0;   // Field of view (degrees)
```
