# 🤖 ROS2 Robot Navigation Logger

A reactive ROS2 node that navigates a mobile robot using laser scan and odometry feedback. It logs the robot’s trajectory, velocities, and laser scan data into CSV files for offline visualization and analysis. Designed for robotics researchers, students, and developers working with obstacle avoidance and autonomous motion.

---

## ✨ Features

- 🛑 **Obstacle Detection & Avoidance**  
  Simple finite state machine using LIDAR scans to navigate forward and turn when obstacles are detected.

- 📡 **Real-Time Logging to CSV**  
  Logs key data to disk:
  - Robot position trajectory
  - Linear velocity
  - Raw laser scan readings
  - Transformed laser scan points (map view)

- 🧭 **Odometry Feedback Integration**  
  Converts quaternion to Euler angles to extract yaw for robot orientation tracking.

- 🗺️ **Laser Scan Mapping**  
  Converts polar scan data to global cartesian coordinates for basic 2D mapping.

- 🧪 **Easy to Visualize and Debug**  
  CSV outputs can be plotted using Python, Excel, MATLAB, or tools like Jupyter Notebooks.

---

## 🚀 Usage

### 🧰 Prerequisites

- ROS 2 Foxy, Galactic, Humble, or newer installed
- A robot or simulator publishing:
  - `/scan` (sensor_msgs/msg/LaserScan)
  - `/odom` (nav_msgs/msg/Odometry)
- A velocity controller listening on:
  - `/cmd_vel` (geometry_msgs/msg/Twist)

---


## 🧾 CSV Output Format

Logs are saved to: `/tmp/ros_logs/`

| **File Name**             | **Columns**                                                  | **Description**                                                  |
|--------------------------|--------------------------------------------------------------|------------------------------------------------------------------|
| `odom_trajectory.csv`     | `x_position`, `y_position`                                   | Robot’s position in 2D space, from odometry                      |
| `velocity_data.csv`       | `cycle_number`, `linear_velocity`                            | Current forward speed and loop count                             |
| `laser_scan.csv`          | `left`, `mid_left`, `front`, `mid_right`, `right`, `index`   | 5-directional LIDAR scan distances with time-step index          |
| `laser_map.csv`           | `x_transformed`, `y_transformed`                             | Laser scan points projected into global coordinate frame         |



## 🧠 Future Improvements

### 🌀 Add Rotation in Place
Enable better cornering and more flexible obstacle avoidance by allowing the robot to rotate in place when necessary.

### 🧠 Dynamic Thresholding
Adapt obstacle detection thresholds dynamically based on robot speed, direction, or the density of nearby objects.

### 🗺️ Real-Time Visualization
Use tools like `rqt_plot`, or publish `visualization_msgs/Marker` data for live display in RViz during operation.

### 📊 ROS2 Logging and Bag File Support
Replace CSV logging with ROS2 bag recording for higher efficiency, built-in tooling, and replay support.

### 🎯 Goal-Oriented Navigation
Enhance autonomy by incorporating planned path navigation using the `nav2` stack or custom goal planners.

### 🧪 Test Suite with Gazebo or Ignition
Integrate simulation testing using Gazebo or Ignition for CI/CD pipelines and safe experimentation.


