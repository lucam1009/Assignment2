This repository contains a ROS 2 C++ package that implements a robotic control system featuring manual velocity control, automated obstacle avoidance, and safety monitoring. The system consists of two main nodes communicating via topics and custom services.

## System Architecture

The project is structured around two primary nodes:

1.  **`movement`**: Handles user input, commands the robot's velocity, and calculates velocity statistics.
2.  **`distance_check`**: Monitors the environment via LaserScan, manages emergency stops, and hosts configuration services.

### Custom Interfaces
The package relies on the following custom definitions in `assignment2_msgs`:

* **Msg `ObstaclePosition`**:
    * `float32 distance`: Distance to the closest obstacle.
    * `float32 pos_x`, `float32 pos_y`: Obstacle coordinates (sensor frame).
    * `float32 threshold`: Current safety threshold.
* **Srv `Threshold`**:
    * Request: `float32 threshold` (New safety distance).
    * Response: `float32 x` (Confirmed value).
* **Srv `Average`**:
    * Request: `float32 lin_average`, `float32 ang_average` (Computed averages).
    * Response: `float32 lin_average`, `float32 ang_average`.

---

## Node Details

### 1. Node: `movement`
This node acts as the user interface and high-level controller.

* **User Interface (Multithreading)**: The node spawns a detached `std::thread` to handle `std::cin` input. This allows the user to interact with the terminal without blocking the main ROS event loop (`rclcpp::spin`).
* **Velocity Control**: It publishes to `/cmd_vel`. When a user inputs a velocity, the robot moves for a limited duration (~4 seconds) or stops immediately if the safety flag is triggered.
* **Statistics**: Uses a `std::deque` to store the last 5 velocity inputs. It calculates the moving average for both linear and angular velocities and sends them to the `generate_average` service.

**User Menu Options:**
1.  **Input Velocity**: Sets linear/angular velocity. Displays info about the closest obstacle.
2.  **Change Threshold**: Calls the `generate_threshold` service to update the safety distance.
3.  **Send Average**: Computes averages of the last 5 inputs and sends them to the server.

### 2. Node: `distance_check`
This node handles perception and active safety.

* **LaserScan Processing**: Subscribes to `/scan`, iterates through the ranges, and identifies the closest point.
* **Coordinate Conversion**: Converts the polar coordinates (distance, angle) from the laser scanner into Cartesian coordinates ($x, y$) relative to the sensor frame:
    $$x = d \cdot \cos(\theta), \quad y = d \cdot \sin(\theta)$$
* **Safety State Machine (Hysteresis)**:
    * **Emergency**: If `min_distance < threshold`, the node enters emergency mode. It publishes `1` to `/safety_status` and forces a **movement on the opposite direction** on `/cmd_vel`.
    * **Recovery**: To prevent oscillation, the robot only exits emergency mode when `min_distance > threshold + 0.5`.
 
## Usage

### Prerequisites
* **Ensure your workspace is built and sourced:**
1. colcon build
2. source install/setup.bash
* **launch the gazebo and rviz simulation:**
1. ros2 launch bme_gazebo_sensors spawn_robot.launch.py
* **then run the two codes:**
1. ros2 run bme_gazebo_sensors distance
2. ros2 run bme_gazebo_sensors movement
