# MapManage Node Documentation

## Overview

`MapManage` is a ROS 2 node responsible for managing and generating various types of maps, including free space maps, risk maps, and channel maps. The node uses OpenCV for image processing and morphological operations.

**Node Name**: `map_manage`

---

## File Structure

```
MapManage (Node)
├── __init__()                          # Initialize node, services, clients, publishers
├── Risk Map Related
│   ├── create_risk_map_srv()           # Service callback: Create risk map
│   ├── _start_create_risk_map_async()  # Async start risk map creation
│   ├── _handle_risk_zone_list_response() # Handle risk zone list response
│   ├── _generate_risk_map()            # Generate risk map
│   └── _create_risk_map_inflated()     # Inflate risk map (outward expansion)
├── Free Space Related
│   ├── create_free_space_srv()         # Service callback: Create free space
│   ├── _start_create_free_space_async() # Async start free space creation
│   ├── _handle_zone_list_response()    # Handle zone list response
│   ├── _create_zone_maps_and_freespace() # Create zone maps and free space
│   └── _create_free_space_inflated()   # Inflate free space (inward shrink)
├── Channel Map Related
│   ├── create_chennal_map_srv()        # Service callback: Create channel map
│   ├── _start_create_chennal_map_async() # Async start channel map creation
│   ├── _handle_chennal_path_list_response() # Handle channel path list response
│   ├── _generate_chennal_map()         # Generate channel map
│   └── _create_chennal_map_inflated()  # Inflate channel map (inward shrink)
└── Other
    ├── get_zone_map_list_srv()         # Service callback: Get zone map list
    └── get_zone_cell_maps()            # Get zone cell maps list

main()                                  # Main function: Initialize and run node
```

---

## Parameters

| Parameter Name | Type | Default | Description |
|---------------|------|---------|-------------|
| `inflate_radius_m` | float | 0.08 | Inflation radius (meters) for map inflation/erosion operations |
| `chennal_width_m` | float | 0.6 | Channel width (meters) |
| `use_sim_time` | bool | True | Whether to use simulation time |

---

## Services

### Provided Services (Server)

| Service Name | Service Type | Description |
|-------------|-------------|-------------|
| `/create_risk_map` | `std_srvs/srv/Trigger` | Create risk map, marking risk zones as obstacles |
| `/create_free_space` | `std_srvs/srv/Trigger` | Create free space map based on recorded zone list |
| `/create_chennal_map` | `std_srvs/srv/Trigger` | Create channel map based on channel paths |
| `/get_zone_map_list_srv` | `boustrophedon_coverage_interfaces/srv/ZoneMapList` | Get current zone map list |

### Client Services

| Service Name | Service Type | Description |
|-------------|-------------|-------------|
| `/get_risk_zone_list` | `boustrophedon_coverage_interfaces/srv/GetZoneList` | Get risk zone list |
| `/get_record_zone_list` | `boustrophedon_coverage_interfaces/srv/GetZoneList` | Get recorded zone list |
| `/get_chennal_path_list` | `path_record_interface/srv/ChennalPathList` | Get channel path list |

---

## Published Topics

| Topic Name | Message Type | QoS | Description |
|-----------|-------------|-----|-------------|
| `/free_space` | `nav_msgs/msg/OccupancyGrid` | TRANSIENT_LOCAL, RELIABLE | Free space map (original) |
| `/free_space_inflated` | `nav_msgs/msg/OccupancyGrid` | TRANSIENT_LOCAL, RELIABLE | Free space map (inflated, inward shrink) |
| `/risk_map` | `nav_msgs/msg/OccupancyGrid` | TRANSIENT_LOCAL, RELIABLE | Risk map (original) |
| `/risk_map_inflated` | `nav_msgs/msg/OccupancyGrid` | TRANSIENT_LOCAL, RELIABLE | Risk map (inflated, outward expansion) |
| `/chennal_map` | `nav_msgs/msg/OccupancyGrid` | TRANSIENT_LOCAL, RELIABLE | Channel map (original) |
| `/chennal_map_inflated` | `nav_msgs/msg/OccupancyGrid` | TRANSIENT_LOCAL, RELIABLE | Channel map (inflated, inward shrink) |

**QoS Settings**:
- `depth`: 1
- `durability`: TRANSIENT_LOCAL (late subscribers can receive previously published messages)
- `reliability`: RELIABLE (guaranteed message delivery)

---

## Main Functionality

### 1. Risk Map

**Workflow**:
1. Call `/create_risk_map` service
2. Asynchronously get risk zone list (via `/get_risk_zone_list` service)
3. Generate map based on risk zone polygons, marking risk areas as obstacles (100)
4. Perform **outward dilation** on risk map to increase safety margin
5. Publish original and inflated risk maps

**Features**:
- Uses same origin and resolution as `base_map`
- Risk zones expand outward, increasing obstacle coverage

### 2. Free Space Map

**Workflow**:
1. Call `/create_free_space` service
2. Asynchronously get recorded zone list (via `/get_record_zone_list` service)
3. Create individual `ZoneMap` objects for each zone
4. Generate overall free space map: zones are free space (0), outside is obstacle (100)
5. Perform **inward erosion** on free space to make it more conservative
6. Publish original and inflated free space maps

**Features**:
- Resolution: 0.05m (5cm)
- Boundary margin: 1.0m
- Free space shrinks inward for safety
- Generates `zone_map_list` with individual mask for each zone

### 3. Channel Map

**Workflow**:
1. Call `/create_chennal_map` service
2. Asynchronously get channel path list (via `/get_chennal_path_list` service)
3. Generate channels based on path points using OpenCV line drawing
4. Channel areas marked as free space (0), others as obstacles (100)
5. Perform **inward erosion** on channels to make them more conservative
6. Publish original and inflated channel maps

**Features**:
- Channel width controlled by `chennal_width_m` parameter
- Uses same coordinate system as `base_map`
- Channels shrink inward for safe passage

---

## Inflation Operations

### Risk Map Inflation (Dilation - Outward Expansion)
- **Purpose**: Increase risk zone coverage for safety
- **Method**: Use `cv2.dilate()` to dilate risk areas
- **Effect**: Obstacle areas become larger

### Free Space/Channel Inflation (Erosion - Inward Shrink)
- **Purpose**: Reduce available space for conservative path planning
- **Method**: Use `cv2.erode()` to erode free areas
- **Effect**: Free space becomes smaller, boundaries shrink inward

**Structuring Element**:
- Shape: Ellipse (`cv2.MORPH_ELLIPSE`)
- Size: `(2 * r_cells + 1, 2 * r_cells + 1)`
- `r_cells = ceil(inflate_radius_m / resolution)`

---

## Data Structures

### Member Variables

| Variable Name | Type | Description |
|--------------|------|-------------|
| `base_map` | `OccupancyGrid` | Base map (free space map) |
| `zone_map_list` | `List[ZoneMap]` | Zone map list, each zone has individual mask |
| `zone_list` | `List` | Zone list |
| `latest_map` | `OccupancyGrid` | Latest map |
| `service_callback_group` | `ReentrantCallbackGroup` | Reentrant callback group for service calls |

---

## Dependencies

### ROS 2 Messages and Services
- `nav_msgs/msg/OccupancyGrid`
- `std_srvs/srv/Trigger`
- `boustrophedon_coverage_interfaces/msg/ZoneMap`
- `boustrophedon_coverage_interfaces/srv/GetZoneList`
- `boustrophedon_coverage_interfaces/srv/ZoneMapList`
- `path_record_interface/srv/ChennalPathList`

### Python Libraries
- `rclpy`: ROS 2 Python client library
- `numpy`: Numerical computing
- `cv2` (OpenCV): Image processing and morphological operations
- `math`: Mathematical operations

---

## Usage Examples

### Launch Node
```bash
ros2 run maphub map_manage
```

### Call Services

#### Create Free Space Map
```bash
ros2 service call /create_free_space std_srvs/srv/Trigger
```

#### Create Risk Map
```bash
ros2 service call /create_risk_map std_srvs/srv/Trigger
```

#### Create Channel Map
```bash
ros2 service call /create_chennal_map std_srvs/srv/Trigger
```

#### Get Zone Map List
```bash
ros2 service call /get_zone_map_list_srv boustrophedon_coverage_interfaces/srv/ZoneMapList
```

### Subscribe to Map Topics
```bash
# View free space map
ros2 topic echo /free_space

# View risk map
ros2 topic echo /risk_map

# View channel map
ros2 topic echo /chennal_map
```

---

## Important Notes

1. **Execution Order**: Must call `/create_free_space` first to create `base_map` before creating risk maps and channel maps
2. **Async Processing**: All map creation operations are asynchronous; services return immediately while map generation happens in background
3. **Coordinate System**: All maps use the same coordinate system (frame_id: 'map')
4. **Map Value Convention**:
   - `0`: Free space (traversable)
   - `100`: Obstacle (non-traversable)
   - `-1`: Unknown area (not used)
5. **QoS Settings**: Uses TRANSIENT_LOCAL to ensure late subscribers can receive maps

---

## Error Handling

The node logs the following error conditions:
- Service unavailable (5 second timeout)
- Failed to get zone list
- Map generation failure
- No valid data points
- No base map available (when creating risk or channel maps)

All errors are logged through the ROS 2 logging system.

---

## Copyright

Copyright 2024 fxrbindi  
Licensed under the Apache License, Version 2.0
