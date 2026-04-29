# Gap Follower - ROS2 Autonomous Navigation Package

A ROS2 package implementing the gap following algorithm for autonomous vehicle navigation using LiDAR data. This package enables F1/10 scale autonomous vehicles to navigate through environments by identifying and following the largest gaps in LiDAR scan data.

## Table of Contents

- [Overview](#overview)
- [Algorithm Explanation](#algorithm-explanation)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Node Architecture](#node-architecture)
- [Topics and Messages](#topics-and-messages)
- [Contributing](#contributing)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Overview

The Gap Follower package implements a reactive navigation algorithm that processes LiDAR data to identify obstacles and navigate through the largest available gap. The algorithm is particularly effective for high-speed autonomous racing scenarios where quick reaction times are crucial.

### Key Features

- **Real-time LiDAR processing**: Filters and processes laser scan data in real-time
- **Obstacle detection**: Identifies critical edges and obstacles in the environment
- **Adaptive speed control**: Adjusts vehicle speed based on steering angle and obstacle proximity
- **Customizable parameters**: Extensive configuration options for different racing scenarios
- **Safety mechanisms**: Includes collision avoidance and emergency stopping
- **Joy control integration**: Manual override capabilities with joystick control

## Algorithm Explanation

### Gap Following Algorithm

The gap following algorithm operates in several key stages:

1. **Scan Processing**: 
   - Limits the LiDAR field of view to a configurable angle range
   - Filters out readings beyond the specified distance thresholds

2. **Edge Detection**:
   - Identifies discontinuities in LiDAR readings as potential obstacles
   - Sorts edges by proximity to determine critical obstacles

3. **Gap Creation**:
   - Creates artificial obstacles around detected edges using arc-length calculations
   - Filters the scan to create "gaps" in the environment

4. **Target Selection**:
   - Selects the direction of the longest available ray as the target
   - Calculates the steering angle to follow the identified gap

5. **Speed Control**:
   - Implements adaptive speed control based on:
     - Steering angle (sigmoid function)
     - Obstacle proximity
     - Safety constraints

### Safety Features

- **Emergency stopping**: Automatic braking when obstacles are too close
- **Collision avoidance**: Maintains safe distances from detected obstacles
- **Manual override**: Joystick control for emergency situations

## Installation

### Prerequisites

- ROS2 (tested with Humble/Iron)
- Python 3.8+
- Required ROS2 packages:
  - `rclpy`
  - `geometry_msgs`
  - `ackermann_msgs`
  - `sensor_msgs`
  - `std_msgs`

### Build Instructions

1. Clone the repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository_url>
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select gap_follower
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Basic Usage

1. Launch the gap follower node:
```bash
ros2 run gap_follower steering_speed_exe
```

2. Or use with custom configuration:
```bash
ros2 run gap_follower steering_speed_exe --ros-args --params-file src/planning/gap_follower/config/gap_follower_config.yaml
```

### With Launch Files

```bash
ros2 launch gap_follower gap_follower.launch.py
```

### Control Interface

- **Autonomous Mode**: Activated via joystick button 4 (L1/LB)
- **Manual Override**: Available through joystick control
- **Toggle**: Use `/gap_follower_toggle` topic to enable/disable the algorithm

## Configuration

### Parameters

The package behavior can be customized through various parameters:

#### Core Algorithm Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `limit_angle` | 85.0 | LiDAR field of view limit (degrees) |
| `kp` | 0.51 | Proportional gain for steering control |
| `kd` | 1.15 | Derivative gain for steering control |
| `arc_length` | 0.48 | Arc length for obstacle expansion |
| `number_of_critical_edges` | 3 | Number of critical obstacles to consider |
| `obstacle_distance_thresh` | 0.4 | Distance threshold for edge detection |

#### Speed Control Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_vel` | 1.5 | Minimum vehicle speed (m/s) |
| `max_vel` | 3.5 | Maximum vehicle speed (m/s) |
| `min_distance` | 0.5 | Minimum safe distance to obstacles |
| `max_distance` | 10.0 | Maximum consideration distance |
| `k_sigmoid` | 8.0 | Sigmoid function steepness for speed control |

#### Topic Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `scan_topic` | "/scan" | Input LiDAR topic |
| `drive_topic` | "/ackermann_cmd" | Output drive command topic |
| `filtered_scan_topic` | "/filtered_scan" | Filtered LiDAR output topic |

### Configuration File

Edit `config/gap_follower_config.yaml` to customize parameters for your specific setup.

## Node Architecture

### Main Node: `SteeringSpeedNode`

The primary node that implements the gap following algorithm.

#### Subscriptions
- `/scan` (sensor_msgs/LaserScan): Input LiDAR data
- `/joy` (sensor_msgs/Joy): Joystick input for manual control
- `/gap_follower_toggle` (std_msgs/Bool): Algorithm enable/disable

#### Publications
- `/ackermann_cmd` (ackermann_msgs/AckermannDriveStamped): Drive commands
- `/filtered_scan` (sensor_msgs/LaserScan): Processed LiDAR data

#### Key Methods

- `filter_scan_cb()`: Processes incoming LiDAR data
- `get_theta_target_5()`: Implements the gap following algorithm
- `find_linear_vel_steering_controlled_sigmoidally()`: Calculates adaptive speed
- `follow_the_gap()`: Main control loop with PD controller

## Topics and Messages

### Input Topics

- **`/scan`** (sensor_msgs/LaserScan): Raw LiDAR data
- **`/joy`** (sensor_msgs/Joy): Joystick input
- **`/gap_follower_toggle`** (std_msgs/Bool): Algorithm control

### Output Topics

- **`/ackermann_cmd`** (ackermann_msgs/AckermannDriveStamped): Vehicle control commands
- **`/filtered_scan`** (sensor_msgs/LaserScan): Processed LiDAR data for visualization

## Contributing

We welcome contributions to improve the gap follower algorithm! Please follow these guidelines:

### Development Setup

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/amazing-feature`
3. Follow the existing code style and conventions
4. Add tests for new functionality
5. Update documentation as needed

### Code Style

- Follow PEP 8 for Python code
- Use meaningful variable names
- Add docstrings for functions and classes
- Include inline comments for complex logic

### Testing

- Test your changes with both simulation and real hardware
- Verify parameter changes don't break existing functionality
- Test edge cases and safety scenarios

### Pull Request Process

1. Ensure all tests pass
2. Update README.md with details of changes if needed
3. Update version numbers in relevant files
4. Submit pull request with clear description of changes

### Areas for Contribution

- **Algorithm improvements**: Enhanced gap detection and selection
- **Performance optimization**: Faster processing for real-time applications
- **Safety features**: Additional collision avoidance mechanisms
- **Documentation**: Code comments, examples, and tutorials
- **Testing**: Unit tests and integration tests
- **Visualization**: RViz plugins and debugging tools

## Troubleshooting

### Common Issues

1. **No movement**: Check if autonomous mode is activated with joystick
2. **Erratic behavior**: Verify LiDAR data quality and parameter tuning
3. **Compilation errors**: Ensure all dependencies are installed
4. **Performance issues**: Check CPU usage and reduce processing frequency if needed

### Debug Information

Enable debug logging to see algorithm state:
```bash
ros2 run gap_follower steering_speed_exe --ros-args --log-level debug
```

### Parameter Tuning Tips

- Start with conservative speed settings
- Adjust `kp` and `kd` gradually for stable control
- Modify `arc_length` based on vehicle size and environment
- Tune `k_sigmoid` for desired speed response

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Maintainers

- **George Hany** - georgehany064@gmail.com
- **Fam Shihata** - fam@awadlouis.com

## Acknowledgments

- F1/10 community for the autonomous racing platform
- ROS2 community for the excellent robotics framework
- Contributors and testers who helped improve this package

---

For questions, issues, or contributions, please open an issue on the repository or contact the maintainers directly.
