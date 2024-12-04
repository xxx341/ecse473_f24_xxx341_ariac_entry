# ARIAC Orders Subscriber Node

This repository contains a ROS package that implements a node to handle orders in the ARIAC (Agile Robotics for Industrial Automation Competition) simulation environment. The node subscribes to the `/ariac/orders` topic to receive incoming orders, queries the `/ariac/material_locations` service to retrieve storage locations for parts, and logs the results for further processing.

---

## Introduction

The ARIAC Orders Subscriber Node is designed to support a simulated industrial automation system. It processes orders by:
1. Subscribing to the `/ariac/orders` topic to receive order details.
2. Querying the `/ariac/material_locations` service to locate required parts.
3. Logging the results to assist in planning the robotic operations for fulfilling the orders.

---

## Objectives

This project aims to:
- Demonstrate proficiency in ROS topics and services.
- Integrate ARIAC simulation features into custom ROS nodes.
- Develop a system capable of parsing and processing industrial automation orders.

---

##File Structure

ariac_entry
├── launch/
│   └── Entry.launch
├── src/
│   └── node_entry.cpp
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

- **Order Processing**: Automatically processes incoming orders by subscribing to the `/ariac/orders` topic.
- **Material Location Query**: Utilizes the `/ariac/material_locations` service to identify the storage locations of required parts.
- **Extensibility**: Provides a modular foundation for extending functionality to robotic motion planning or task execution.

---

## System Requirements

- **Operating System**: Ubuntu 20.04
- **ROS Distribution**: ROS Noetic
- **ARIAC Environment**: Ensure the ARIAC simulation environment is installed and configured.
- **Dependencies**:
  - `osrf_gear` package
  - `roscpp`
  - `std_msgs`

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
rosrun ariac_entry orders_subscriber_node
```
### Publish Test Orders
Publish a test order to verify the node functionality:
```bash
rostopic pub /ariac/orders osrf_gear/Order "{order_id: 'test_order', shipments: [{shipment_type: 'test_shipment', products: [{type: 'gear_part', pose: {position: {x: 0.1, y: 0.2, z: 0.0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}]}]}"
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
3. **Check Logs**: Confirm the node logs the part locations retrieved from the service.
4. **Service Test**: Test the **`/ariac/material_locations`** service independently:
   ```bash
   rosservice call /ariac/material_locations "{material_type: 'gear_part'}"
   ```
