# NavActionServer Node Documentation

## Overview

`NavActionServer` is a ROS 2 node that provides action servers for robot navigation. It integrates with Nav2's `BasicNavigator` to execute waypoint-based navigation tasks, including path following with split points for coverage path planning.

**Node Name**: `nav_action_server`

---

## File Structure

```
NavActionServer (Node)
├── __init__()                          # Initialize node, action servers, publishers
├── Navigation Actions
│   ├── execute_callback()              # Action callback: Navigate through waypoints
│   └── single_path_execute_callback()  # Action callback: Follow single path with split points
├── Visualization
│   └── publish_split_points_marker()   # Publish split points markers for RViz
└── Cleanup
    └── destroy_node()                  # Clean up action servers and node

main()                                  # Main function: Initialize and run node
```

---

## Parameters

| Parameter Name | Type | Default | Description |
|---------------|------|---------|-------------|
| `use_sim_time` | bool | True | Whether to use simulation time |

---

## Action Servers

| Action Name | Action Type | Description |
|------------|-------------|-------------|
| `nav_action` | `nav2_action_interfaces/action/Waypoint` | Navigate through a series of waypoints with path planning and smoothing |
| `nav_action_follow_path` | `nav2_action_interfaces/action/Waypoint` | Follow a single path with coverage split points |

### Action Details

#### 1. `nav_action`
**Purpose**: Navigate through waypoints by planning and smoothing paths between consecutive poses.

**Request**:
- `path` (nav_msgs/Path): List of waypoints to navigate through

**Workflow**:
1. For each consecutive pair of poses in the path:
   - Calculate yaw angle between poses
   - Convert to quaternion orientation
   - Plan path using Nav2's planner
   - Smooth the planned path
   - Follow the smoothed path
2. Wait for each segment to complete before moving to next
3. Return success when all waypoints are reached

**Features**:
- Automatic orientation calculation based on movement direction
- Path planning between waypoints
- Path smoothing for better trajectory
- Real-time feedback on distance to goal

#### 2. `nav_action_follow_path`
**Purpose**: Follow a pre-planned path with designated split points for coverage planning.

**Request**:
- `path` (nav_msgs/Path): The complete path to follow
- `poses` (geometry_msgs/Pose[]): Coverage split points

**Workflow**:
1. Navigate to first pose using `goToPose()`
2. Split the path at designated split points
3. For each path segment:
   - Collect poses until reaching a split point
   - Publish the segment path
   - Follow the segment using `followPath()`
   - Wait for completion
4. Return success when entire path is completed

**Features**:
- Path segmentation at split points
- Sequential execution of path segments
- Real-time feedback during navigation
- Split path visualization

---

## Published Topics

| Topic Name | Message Type | QoS | Description |
|-----------|-------------|-----|-------------|
| `/split_path` | `nav_msgs/msg/Path` | depth=1 | Current path segment being executed |
| `/coverage_split_points` | `visualization_msgs/msg/Marker` | depth=1 | Visualization markers for coverage split points in RViz |

### Topic Details

#### `/split_path`
- **Purpose**: Visualize and debug the current path segment
- **Frame**: `map`
- **Published**: During `nav_action_follow_path` execution at each split point

#### `/coverage_split_points`
- **Purpose**: Visualize split points in RViz for debugging
- **Marker Type**: `SPHERE_LIST`
- **Color**: Red (rgba: 1.0, 0.0, 0.0, 0.8)
- **Size**: 0.15m diameter spheres
- **Height**: 0.1m above ground

---

## Dependencies

### ROS 2 Messages and Actions
- `nav_msgs/msg/Path`
- `geometry_msgs/msg/Point`
- `visualization_msgs/msg/Marker`
- `std_msgs/msg/ColorRGBA`
- `nav2_action_interfaces/action/Waypoint`

### ROS 2 Packages
- `nav2_simple_commander`: Provides `BasicNavigator` for navigation control
- `rclpy`: ROS 2 Python client library

### Python Libraries
- `math`: Mathematical operations for orientation calculations

---

## Main Functionality

### 1. Waypoint Navigation (`nav_action`)

**Use Case**: Navigate through a series of waypoints with automatic path planning.

**Process**:
1. Receives a path with multiple waypoints
2. For each consecutive pair:
   - Calculates heading direction (yaw)
   - Converts Euler angles to quaternion
   - Plans optimal path using Nav2 planner
   - Smooths path for better trajectory
   - Executes navigation
3. Provides feedback on distance to goal

**Key Features**:
- **Automatic Orientation**: Calculates robot orientation based on movement direction
- **Path Planning**: Uses Nav2's global planner to find optimal paths
- **Path Smoothing**: Applies smoothing for better robot motion
- **Sequential Execution**: Completes one segment before starting next

### 2. Coverage Path Following (`nav_action_follow_path`)

**Use Case**: Execute coverage paths with designated split points for better control.

**Process**:
1. Receives complete coverage path and split points
2. Navigates to starting position
3. Splits path into segments at designated points
4. Executes each segment sequentially
5. Publishes segment paths for visualization

**Key Features**:
- **Split Point Management**: Divides long paths into manageable segments
- **Segment Visualization**: Publishes each segment for monitoring
- **Coverage Split Points**: Stores and tracks split locations
- **Sequential Execution**: Ensures proper coverage by completing segments in order

### 3. Visualization

**Split Points Marker**:
- Displays coverage split points as red spheres in RViz
- Helps visualize path segmentation
- Useful for debugging coverage planning

---

## Helper Functions

### `euler_to_quaternion(roll, pitch, yaw)`
Converts Euler angles to quaternion representation.

**Parameters**:
- `roll` (float): Roll angle in radians
- `pitch` (float): Pitch angle in radians
- `yaw` (float): Yaw angle in radians

**Returns**:
- `tuple`: (qx, qy, qz, qw) quaternion components

**Usage**: Automatically calculates robot orientation between waypoints.

---

## Usage Examples

### Launch Node
```bash
ros2 run nav_robot nav_action_server
```

### Call Action: Navigate Through Waypoints
```bash
ros2 action send_goal /nav_action nav2_action_interfaces/action/Waypoint "{path: {header: {frame_id: 'map'}, poses: [...]}}"
```

### Call Action: Follow Coverage Path
```bash
ros2 action send_goal /nav_action_follow_path nav2_action_interfaces/action/Waypoint "{path: {header: {frame_id: 'map'}, poses: [...]}, poses: [...]}"
```

### Monitor Split Path
```bash
ros2 topic echo /split_path
```

### Visualize Split Points in RViz
1. Add Marker display
2. Set topic to `/coverage_split_points`
3. Red spheres will appear at split point locations

---

## Integration with Nav2

This node uses Nav2's `BasicNavigator` which provides:

### Navigation Methods Used

| Method | Purpose |
|--------|---------|
| `goToPose(pose)` | Navigate to a single pose |
| `followPath(path)` | Follow a pre-planned path |
| `getPath(start, goal)` | Plan a path between two poses |
| `smoothPath(path)` | Smooth a planned path |
| `isTaskComplete()` | Check if current task is done |
| `getFeedback()` | Get navigation feedback |

### Required Nav2 Stack
- **Planner**: Global path planner (e.g., NavFn, Smac)
- **Controller**: Local trajectory controller (e.g., DWB, TEB)
- **Smoother**: Path smoother (e.g., Simple Smoother)
- **Behavior Trees**: Nav2 behavior tree navigator

---

## Member Variables

| Variable Name | Type | Description |
|--------------|------|-------------|
| `navigator` | `BasicNavigator` | Nav2 navigation interface |
| `action_server` | `ActionServer` | Action server for waypoint navigation |
| `action_server_follow_path` | `ActionServer` | Action server for path following |
| `split_path_pub` | `Publisher` | Publisher for split path segments |
| `coverage_split_points_pub` | `Publisher` | Publisher for split points markers |
| `coverage_split_points` | `List` | List of coverage split point coordinates |

---

## Coordinate System

- **Frame ID**: `map`
- All poses and paths use the `map` frame
- Coordinates are in meters
- Orientation uses quaternions (x, y, z, w)

---

## Error Handling

The node handles:
- Navigation failures (checked via `isTaskComplete()`)
- Empty paths
- Invalid poses
- Navigation feedback timeouts

All errors are logged through ROS 2 logging system.

---

## Important Notes

1. **Nav2 Dependency**: Requires Nav2 stack to be running (planner, controller, etc.)
2. **Sequential Execution**: Waypoints are executed sequentially, not in parallel
3. **Blocking Operations**: Action callbacks block until navigation completes
4. **Split Points**: Must be exact coordinate matches in the path
5. **Orientation**: Automatically calculated for `nav_action`, should be pre-set for `nav_action_follow_path`
6. **Simulation Time**: Uses simulation time by default (set `use_sim_time` parameter)

---

## Typical Workflow

### For Coverage Path Planning:
1. Generate coverage path with split points
2. Call `nav_action_follow_path` action
3. Node navigates to start position
4. Executes path segments sequentially
5. Publishes split paths for monitoring
6. Returns success when complete

### For Waypoint Navigation:
1. Define waypoints (positions only, orientation auto-calculated)
2. Call `nav_action` action
3. Node plans and smooths paths between waypoints
4. Executes navigation sequentially
5. Provides distance feedback
6. Returns success when all waypoints reached

---

## Performance Considerations

- **Path Planning**: Each waypoint pair requires path planning (can be slow)
- **Path Smoothing**: Adds processing time but improves trajectory
- **Split Points**: More split points = more segments = more control but slower execution
- **Feedback**: Continuous feedback can generate high log volume

---

## Copyright

Copyright 2024 fxrbindi  
Licensed under the Apache License, Version 2.0

