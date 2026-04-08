# ME5413 Final Project

Autonomous mapping and navigation system for the NUS ME5413 final project using a Clearpath Jackal mobile robot in Gazebo.

> Replace the names below with your actual group members before publishing.
>
> **Group Members:** Member A, Member B, Member C

---

## 1. Project Overview

This repository contains our final implementation for the ME5413 Autonomous Mobile Robotics project. The objective is to build a robot software stack that can map the environment, navigate autonomously across two floors, avoid obstacles, and complete the required task sequence in simulation.

Our final pipeline is organized into two main stages:

1. **Mapping stage** using **RTAB-Map** with RGB-D, point cloud, and filtered odometry inputs.
2. **Navigation stage** using **AMCL + move_base + DWA**, together with custom task-execution scripts for floor-1 scanning, dynamic obstacle handling, and floor-2 room selection.

---

## 2. Implemented System

### 2.1 Mapping

We use **RTAB-Map** as the SLAM backend for map building.

**Sensor inputs used in mapping:**
- RGB image: `/front_rgbd/rgb/image_raw`
- Depth image: `/front_rgbd/depth/image_raw`
- 3D point cloud: `/mid/points`
- Filtered odometry: `/odometry/filtered`

**Key mapping characteristics:**
- 3D grid generation enabled
- Depth-based occupancy integration enabled
- Ray tracing enabled
- Visual + ICP registration
- Robust graph optimization
- Parameters adjusted for ramp/slope handling

The mapping launch file is:

```bash
roslaunch me5413_world mapping.launch
```

### 2.2 Navigation

For navigation on the saved 2D map, we use:
- **map_server** for loading the generated map
- **AMCL** for localization
- **move_base** for path planning and local motion control
- **DWA local planner** for real-time trajectory generation

The navigation stack is extended with three custom modules:

#### (a) `floor1_auto_scan.py`
This node performs the lower-floor exploration routine. It navigates along predefined scan regions, detects numbered boxes using RGB-D perception, and records the occurrence count of each label.

#### (b) `dynamic_obstacle_predictor.py`
This node tracks moving obstacles from the front laser scan, predicts their short-horizon motion, and publishes a predicted obstacle cloud for the local costmap.

#### (c) `floor2_room_selector.py`
This node waits for the floor-1 counting result, opens the timed blockade by publishing to `/cmd_unblock`, travels up the ramp, checks which corridor entrance is blocked by the cone, chooses the feasible route, observes room labels, and stops in the room corresponding to the least frequent box number found on floor 1.

The final navigation launch file is:

```bash
roslaunch me5413_world navigation.launch
```

---

## 3. Repository Structure

```text
ME5413_Final_Project/
├── src/
│   ├── interactive_tools/
│   ├── jackal_description/
│   └── me5413_world/
│       ├── config/
│       ├── launch/
│       ├── maps/
│       ├── media/
│       ├── models/
│       ├── rviz/
│       ├── scripts/
│       └── worlds/
├── LICENSE
└── README.md
```

Important files for our final solution:

- `src/me5413_world/launch/world.launch`
- `src/me5413_world/launch/mapping.launch`
- `src/me5413_world/launch/navigation.launch`
- `src/me5413_world/scripts/floor1_auto_scan.py`
- `src/me5413_world/scripts/floor2_room_selector.py`
- `src/me5413_world/scripts/dynamic_obstacle_predictor.py`
- `src/me5413_world/config/nav_global_costmap.yaml`
- `src/me5413_world/config/nav_local_costmap.yaml`
- `src/me5413_world/config/floor1_auto_scan.yaml`
- `src/me5413_world/config/floor2_room_selector.yaml`

---

## 4. Requirements

### System
- Ubuntu 20.04
- ROS Noetic
- Catkin workspace tools
- Gazebo

### ROS packages
Typical dependencies include:
- `roscpp`
- `rospy`
- `rviz`
- `std_msgs`
- `nav_msgs`
- `geometry_msgs`
- `visualization_msgs`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `pluginlib`
- `map_server`
- `gazebo_ros`
- `jsk_rviz_plugins`
- `jackal_gazebo`
- `jackal_navigation`
- `velodyne_simulator`
- `teleop_twist_keyboard`
- `rtabmap_ros`

---

## 5. Installation

Clone the repository:

```bash
cd ~
git clone https://github.com/<YOUR_USERNAME>/ME5413_Final_Project.git
cd ME5413_Final_Project
```

Install dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

If some simulation-related packages are still missing, install them manually as needed:

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-jackal-gazebo \
  ros-noetic-jackal-navigation \
  ros-noetic-jsk-rviz-plugins \
  ros-noetic-teleop-twist-keyboard \
  ros-noetic-velodyne-simulator \
  ros-noetic-rtabmap-ros
```

Build the workspace:

```bash
catkin_make
source devel/setup.bash
```

Copy Gazebo models if required:

```bash
mkdir -p ~/.gazebo/models
cp -r ~/ME5413_Final_Project/src/me5413_world/models/* ~/.gazebo/models/
```

---

## 6. How to Run

### Step 1: Launch the world

```bash
roslaunch me5413_world world.launch
```

### Step 2: Build the map

In a new terminal:

```bash
cd ~/ME5413_Final_Project
source devel/setup.bash
roslaunch me5413_world mapping.launch
```

### Step 3: Save the 2D map

After mapping is completed:

```bash
cd ~/ME5413_Final_Project
source devel/setup.bash
roscd me5413_world/maps
rosrun map_server map_saver -f my_map map:=/map
```

### Step 4: Launch autonomous navigation

Stop the mapping node, then run:

```bash
cd ~/ME5413_Final_Project
source devel/setup.bash
roslaunch me5413_world navigation.launch
```

### Step 5: Initialize localization

In RViz, use **“2D Pose Estimate”** to set the robot’s initial pose on the saved map before starting the autonomous routine.

---

## 7. Expected Task Flow

The final autonomous pipeline is designed to execute the following sequence:

1. Scan floor 1 and count numbered boxes.
2. Publish to `/cmd_unblock` to open the timed blockade.
3. Exit the lower floor and travel up the ramp.
4. Detect which upper-floor entrance is blocked by the cone.
5. Select the open gap and enter the second-floor main area.
6. Use room observations to identify the destination room.
7. Stop at the room associated with the least frequent box label found on floor 1.

---

## 8. Generated Outputs

The system stores intermediate and final results in JSON files:

- Floor-1 counting result:
  ```text
  ~/.ros/me5413_floor1_counts.json
  ```

- Floor-2 selection result:
  ```text
  ~/.ros/me5413_floor2_result.json
  ```

Saved maps are stored in:

```text
src/me5413_world/maps/
```

---

## 9. Notes for Evaluation

- The mapping pipeline uses RTAB-Map with fused sensing inputs.
- The navigation pipeline uses AMCL and move_base on the generated 2D map.
- Dynamic obstacle avoidance is enhanced through short-horizon prediction.
- The system includes task-level logic for both floors rather than pure point-to-point navigation only.

---

## 10. GitHub Submission Notes

Before submission, make sure the repository includes:

- Source code
- Final `README.md`
- Saved map files
- Report PDF
- Presentation slides
- Demo video or link to the demo asset if stored externally

It is also recommended to keep commit history clean and descriptive, for example:

```text
Add RTAB-Map mapping pipeline
Tune local/global costmaps for ramp and corridor navigation
Implement floor-1 automatic box counting
Add dynamic obstacle predictor for moving person avoidance
Implement floor-2 room selection logic
Finalize README and submission assets
```

---

## 11. Acknowledgements

This project is based on the ME5413 final project simulation framework and extends it with a custom mapping and autonomous navigation pipeline.

