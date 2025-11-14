# Mower Path Planning System

<div align="center">
  <img src=“img/mower.png" alt="Mower System" width="400">
</div>

An autonomous lawn mower path planning and navigation solution based on ROS2 Jazzy.

## 📋 Overview

This project provides a complete path planning system for lawn mowers, supporting multiple coverage path generation algorithms, GPS waypoint following, path recording, and more. The system is developed on ROS2 Jazzy and supports Docker containerized deployment.

## 🚀 Features

### Core Modules

- **boustrophedon_coverage**: Boustrophedon coverage path planning algorithm
  - Multiple path generation modes (boustrophedon, spiral, zigzag)
  - Zone map management and path optimization
  - Integration with Nav2 for autonomous navigation

- **nav2_gps_waypoint_follower**: GPS waypoint following system
  - GPS localization and waypoint navigation
  - Dual EKF fusion for localization
  - Waypoint path tracking

- **path_record**: Path recording and playback
  - Real-time path recording
  - Path data management

- **nav2_straightline_planner**: Straight-line path planner
  - Nav2 global path planning plugin
  - Straight-line path generation

- **maphub**: Map management service
  - Map data management
  - Polygon mask generation

## 🛠️ Requirements

- **ROS2**: Jazzy Jalisco
- **OS**: Ubuntu 22.04 (Jammy)
- **Docker**: Optional, for containerized deployment
- **Python**: 3.10+

## 📦 Installation & Build

### Method 1: Local Build

1. **Clone the repository**
```bash
cd ~/car_ws/src
git clone <repository-url> mower_path_planning
```

2. **Install dependencies**
```bash
cd ~/car_ws/src/mower_path_planning
make deps
```

3. **Build workspace**
```bash
# Development build
make build

# Or release build
make build-release
```

4. **Source environment**
```bash
source ~/car_ws/install/setup.bash
```

### Method 2: Docker Deployment

1. **Build Docker image**
```bash
docker-compose build
```

2. **Run container**
```bash
# Set ROS_DOMAIN_ID (optional)
export ROS_DOMAIN_ID=0

# Start container
docker-compose up -d
```

3. **Enter container**
```bash
docker-compose exec lawan_node bash
```

## 🎮 Usage

### Start Test System

```bash
make test-system
```

Or using launch file:

```bash
ros2 launch nav2_gps_waypoint_follower test_system.launch.py
```

### Start Coverage Path Planning

```bash
ros2 launch boustrophedon_coverage nav2.launch.py
```

### Start GPS Waypoint Following

```bash
ros2 launch nav2_gps_waypoint_follower gps_waypoint_follower.launch.py
```

## 📁 Project Structure

```
mower_path_planning/
├── boustrophedon_coverage/          # Coverage path planning core module
│   ├── path_generators/             # Path generation algorithms
│   └── utils/                       # Utility functions
├── nav2_gps_waypoint_follower/      # GPS waypoint following
├── path_record/                     # Path recording module
├── nav2_straightline_planner/       # Straight-line path planner
├── maphub/                          # Map management service
├── Dockerfile                       # Docker build file
├── docker-compose.yaml              # Docker Compose configuration
└── Makefile                         # Build scripts
```

## 🔧 Makefile Commands

- `make build`: Development build
- `make deps`: Install dependencies
- `make build-release`: Release build
- `make test-system`: Start test system

## 🐳 Docker Configuration

Docker configuration supports:
- Automatic dependency installation
- ROS2 workspace building
- Runtime environment configuration
- Network host mode (for ROS2 communication)

## 📝 Configuration

### ROS_DOMAIN_ID

The system uses ROS_DOMAIN_ID for node isolation. Set via environment variable:

```bash
export ROS_DOMAIN_ID=0
```

### Path Planning Parameters

Main parameters can be configured in the `boustrophedon_coverage` node:
- `strip_width_m`: Mower effective cutting width (default: 0.2m)
- `waypoint_spacing_m`: Waypoint spacing (default: 0.1m)
- `unknown_as_obstacle`: Treat unknown areas as obstacles (default: true)

## 🤝 Contributing

Contributions are welcome! Please feel free to submit Issues and Pull Requests.

## 📧 Contact

- Maintainer: fxrbindi
- Email: edwards940428@gmail.com

## 📄 License

TODO: License declaration

## 🙏 Acknowledgments

This project is developed based on the ROS2 Navigation2 framework.
