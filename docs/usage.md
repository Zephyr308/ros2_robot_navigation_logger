# 🚀 Usage & Setup

## 🧰 Requirements

- ROS 2 (Foxy, Galactic, Humble, or newer)
- A robot/simulator that:
  - Publishes `/scan` (sensor_msgs/msg/LaserScan)
  - Publishes `/odom` (nav_msgs/msg/Odometry)
  - Accepts velocity commands on `/cmd_vel` (geometry_msgs/msg/Twist)

---

## 🛠️ Build Instructions

Assuming a workspace at `~/ros2_ws`:

```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/ros2_robot_navigation_logger.git
cd ..
colcon build --packages-select stopper
source install/setup.bash
