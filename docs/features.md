# ✨ Features

### 🛑 Obstacle Detection & Avoidance
Implements a finite state machine based on LIDAR ranges to detect obstacles and navigate accordingly.

### 📡 Real-Time CSV Logging
Logs key runtime data to `/tmp/ros_logs/`:
- Odometry-based position
- Linear velocity
- Raw laser scan values
- Transformed 2D laser scan points (map projection)

### 🧭 Odometry Feedback
Extracts yaw from orientation quaternions for heading estimation.

### 🗺️ Scan Mapping
Projects polar laser data into global cartesian coordinates for simple 2D mapping.

### 🧪 Easy to Debug and Visualize
Logs are stored in CSV format, easily plotted using:
- Python (matplotlib, pandas)
- Excel
- MATLAB
