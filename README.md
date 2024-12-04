# ARIAC Orders Subscriber and Trajectory Execution Node

This repository contains a ROS package designed to handle orders and execute trajectories in the ARIAC (Agile Robotics for Industrial Automation Competition) simulation environment. The node subscribes to the `/ariac/orders` topic to receive incoming orders, queries the `/ariac/material_locations` service for part locations, generates trajectories to pick up parts, and sends these trajectories to the Action Server for execution.

---

## Introduction

This ROS package integrates ARIAC simulation features into custom ROS nodes for industrial automation. It processes orders and uses the `/ariac/arm/follow_joint_trajectory` Action Server to command the robot to execute trajectories based on detected parts.

---

## Objectives

This project aims to:
1. Demonstrate proficiency in ROS topics, services, and actions.
2. Develop a trajectory execution pipeline using the Action Server.
3. Implement logical camera-based part detection and transformation to generate robot trajectories.

---

##File Structure

ariac_entry
├── launch/
│   └── competition.launch
├── src/
│   └── node_source.cpp
├── CMakeLists.txt
├── package.xml
└── README.md

---

## Table of Contents
- [Features](#features)
- [System Requirements](#system-requirements)
- [Setup and Installation](#setup-and-installation)
- [Usage Instructions](#usage-instructions)
- [Node Description](#node-description)
- [Testing](#testing)


---

## Features

- **Order Processing**: Automatically processes incoming orders from the `/ariac/orders` topic.
- **Part Detection**: Uses logical cameras to detect parts and calculate their poses relative to the robot.
- **Trajectory Execution**: Generates and sends trajectories to the Action Server for robot motion.
- **Service Integration**: Utilizes the `/ariac/material_locations` service for storage locations.

---

## System Requirements

- **Operating System**: Ubuntu 20.04
- **ROS Distribution**: ROS Noetic
- **ARIAC Environment**: Ensure the ARIAC simulation is installed and configured.
- **Dependencies**:
  - `osrf_gear` package
  - `roscpp`
  - `actionlib`
  - `control_msgs`
  - `trajectory_msgs`
  - `rosgraph_msgs`

---

## Setup and Installation
1. Clone this repository into your catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone <repository_url>
2. Build the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```
3. Verify that the package is built correctly:
   ```bash
    rospack find ariac_entry
   ```

---

## Usage Instructions
### Launch ARIAC Simulation
Start the ARIAC environment:
```bash
roslaunch ecse_373_ariac ecse_373_ariac.launch
```
### Run the Node
Start the **`orders_subscriber_node`**:
```bash
rosrun ariac_entry ariac_entry_node
```
### Manually start the competition if needed:
```bash
rosservice call /ariac/start_competition "{}"
```
### Publish Test Orders
Publish a test order to verify the node functionality:
```bash
rostopic pub /ariac/orders osrf_gear/Order "{order_id: 'test_order', shipments: [{shipment_type: 'test_shipment', products: [{type: 'gear_part', pose: {position: {x: 0.1, y: 0.2, z: 0.0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}]}]}"
```
Manually publish a trajectory to confirm the robot can move:
```bash
rostopic pub /ariac/arm1/arm/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
joint_names: ['linear_arm_actuator_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
points:
- positions: [0.0, 0.0, -1.57, 0.0, 0.0, 0.0, 0.0]
  velocities: []
  accelerations: []
  time_from_start: {secs: 1, nsecs: 0}"
```

---

##Node Description
**`orders_subscriber_node`**
- **Topic Subscribed**:
  - **`/ariac/orders`** (**`osrf_gear/Order`**): Receives order messages from the ARIAC simulation.
- **Service Called**:
  - **`/ariac/material_locations`** (**`osrf_gear/GetMaterialLocations`**): Queries storage locations for specified parts.
- **Workflow**:
  1. Subscribe to **`/ariac/orders`** to receive order data.
  2. Parse each product in the order and query **`/ariac/material_locations`** for its storage location.
  3. Log the results for further use.
  
---

## Testing
1. **Verify Node Subscription**: Check that the node subscribes to **`/ariac/orders`**:
   ```
   rostopic info /ariac/orders
   ```
2. **Publish Test Orders**: Send a test order to the topic:
   ```bash
   rostopic pub /ariac/orders osrf_gear/Order "{order_id: 'test_order', shipments: [{shipment_type: 'test_shipment', products: [{type: 'gear_part', pose: {position: {x: 0.1, y: 0.2, z: 0.0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}]}]}"
   ```
3. **Check Action Server**: Verify the robot executes trajectories.
   ```bash
   rostopic echo /ariac/arm/follow_joint_trajectory/status
   ```
4. **Inspect Logs**: Use **`rqt_console`** to monitor the node's behavior.

---

## Known Issues
- Simulated Time: Ensure **`/clock`** is publishing by running the simulation before the node.
- Gazebo Performance: Run Gazebo with sufficient resources in a native or well-configured VM environment.
   
