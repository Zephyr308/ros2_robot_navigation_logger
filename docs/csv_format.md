# 📂 CSV Output Format

Logs are saved to: `/tmp/ros_logs/`

| **File Name**           | **Columns**                                                  | **Description**                                                  |
|------------------------|--------------------------------------------------------------|------------------------------------------------------------------|
| `odom_trajectory.csv`   | `x_position`, `y_position`                                   | Robot’s position in 2D space from odometry                      |
| `velocity_data.csv`     | `cycle_number`, `linear_velocity`                            | Current forward velocity over time                              |
| `laser_scan.csv`        | `left`, `mid_left`, `front`, `mid_right`, `right`, `index`   | LIDAR scan distances at 5 angles with time step                 |
| `laser_map.csv`         | `x_transformed`, `y_transformed`                             | Projected 2D coordinates of laser hits in global frame          |

Visualize using:
- 📈 Python: matplotlib + pandas
- 📊 Excel
- 🔬 MATLAB
