# M20 SDK Deploy

This package runs a DeepRobotics M20 robot in Gazebo Sim and connects it to
the CMU autonomy stack for LiDAR-inertial mapping, route planning, local
planning, and velocity control.

Current main pipeline:

```text
Gazebo Sim
  -> /M20/IMU, /M20/LIDAR/FRONT
  -> pointcloud_lio_adapter.py
  -> Point-LIO
  -> lio_to_cmu_bridge.py
  -> /state_estimation, /registered_scan
  -> CMU terrain/local/FAR planners
  -> /cmd_vel
  -> cmd_vel_to_raw_relay.py
  -> /cmd_vel_raw
  -> rl_deploy --autonomy
  -> /M20/JOINTS_CMD
  -> gazebo_controller_ros2.py
  -> Gazebo joint force commands
```

## Workspaces

This setup assumes two workspaces are available:

```text
/home/cjy/deeprobotics_ws      # this package, rl_deploy
/home/cjy/autonomy_stack_go2   # CMU autonomy stack and Point-LIO packages
```

Before using any terminal, source ROS and the relevant workspace:

```bash
source /opt/ros/humble/setup.bash
cd /home/cjy/deeprobotics_ws
source install/setup.bash
```

For terminals that launch CMU packages, also source:

```bash
source /opt/ros/humble/setup.bash
cd /home/cjy/autonomy_stack_go2
source install/setup.bash
cd /home/cjy/deeprobotics_ws
source install/setup.bash
```

Sourcing `deeprobotics_ws` after `autonomy_stack_go2` makes sure the local
`rl_deploy` launch files and scripts are used.

## Build

Build `rl_deploy` after changing C++ files, launch installs, or Python scripts:

```bash
cd /home/cjy/deeprobotics_ws
colcon build --packages-select rl_deploy --symlink-install
source install/setup.bash
```

If the CMU workspace was changed:

```bash
cd /home/cjy/autonomy_stack_go2
colcon build --symlink-install
source install/setup.bash
```

## Run The Full System

Use three terminals. Start Gazebo first, then the M20 controller, then the
autonomy stack.

### Terminal 1: Gazebo Sim

```bash
cd /home/cjy/deeprobotics_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rl_deploy gazebo.launch.py
```

Default launch behavior:

- world: `model/indoor_large/indoor.world`
- robot model: `model/M20_urdf/urdf/M20.sdf`
- robot name / namespace: `M20`
- front LiDAR adapter: enabled by default
- main LiDAR output for Point-LIO: `/M20/LIDAR/FRONT_LIO`

Useful launch arguments:

```bash
ros2 launch rl_deploy gazebo.launch.py x:=0.0 y:=0.0 z:=0.57
ros2 launch rl_deploy gazebo.launch.py lidar_preprocess:=false
ros2 launch rl_deploy gazebo.launch.py robot_name:=M20_A
```

For multi-robot tests, keep `robot_name` unique. Most Gazebo topics are built
from this robot name.

### Terminal 2: M20 RL Controller

```bash
cd /home/cjy/deeprobotics_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rl_deploy rl_deploy --autonomy --ros-args -r __ns:=/M20
```

The `--autonomy` flag enables the ROS velocity command interface:

```text
/cmd_vel      geometry_msgs/msg/TwistStamped
/cmd_vel_raw  geometry_msgs/msg/Twist
/m20_mode     std_msgs/msg/String
```

Bring the robot up:

```bash
ros2 topic pub --once /m20_mode std_msgs/msg/String "{data: stand}"
sleep 3
ros2 topic pub --once /m20_mode std_msgs/msg/String "{data: control}"
```

Mode meanings:

```text
damping  -> joint damping
stand    -> stand up
control  -> RL locomotion control
```

### Terminal 3: Point-LIO + CMU Autonomy

```bash
cd /home/cjy/autonomy_stack_go2
source /opt/ros/humble/setup.bash
source install/setup.bash
cd /home/cjy/deeprobotics_ws
source install/setup.bash
ros2 launch rl_deploy autonomy_sim.launch.py
```

Current defaults in `autonomy_sim.launch.py`:

```text
sim:=false              # Gazebo is launched separately
point_lio:=true
bridge:=true
cmu:=true
route_planner:=true
cmd_vel_relay:=true
rl_deploy:=false        # rl_deploy is launched separately
rviz:=true
lidar_preprocess:=false # Gazebo launch already starts the adapter
```

If you want this launch file to start only selected parts:

```bash
ros2 launch rl_deploy autonomy_sim.launch.py point_lio:=true bridge:=true cmu:=false route_planner:=false
ros2 launch rl_deploy autonomy_sim.launch.py rviz:=false
```

## Send Goals

In RViz, use the `Goalpoint` tool from the FAR planner RViz config.

Important topics:

```text
/goal_point  -> FAR planner goal input
/way_point   -> local planner waypoint input
/path        -> local planner path
/cmd_vel     -> pathFollower velocity output
/cmd_vel_raw -> relay output consumed by M20
```

For route planning, prefer RViz `Goalpoint` or publish `/goal_point`:

```bash
ros2 topic pub --once /goal_point geometry_msgs/msg/PointStamped \
"{header: {frame_id: map}, point: {x: 3.0, y: 0.0, z: 0.0}}"
```

For local planner-only tests, publish `/way_point`:

```bash
ros2 topic pub --once /way_point geometry_msgs/msg/PointStamped \
"{header: {frame_id: map}, point: {x: 2.0, y: 0.0, z: 0.0}}"
```

## Quick Velocity Tests

Use these tests with Gazebo and `rl_deploy --autonomy` running. CMU can be
stopped for cleaner debugging.

Forward:

```bash
ros2 topic pub -r 50 /cmd_vel_raw geometry_msgs/msg/Twist \
"{linear: {x: 0.24, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Yaw left:

```bash
ros2 topic pub -r 50 /cmd_vel_raw geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.5}}"
```

Yaw right:

```bash
ros2 topic pub -r 50 /cmd_vel_raw geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.5}}"
```

Check command scaling:

```bash
ros2 topic echo /m20_cmdvel_debug --once
```

Debug layout:

```text
[source, forward_scale, side_scale, yaw_scale, target_mode, safe_mode,
 current_state, current_gait]

source = 0  -> /cmd_vel
source = 1  -> /cmd_vel_raw
target_mode/current_state = 6 -> RLControlMode
```

Check Gazebo actuator execution:

```bash
ros2 topic echo /M20/exec_debug --once
```

Debug layout:

```text
[cmd_received,
 effective_wheel_vel_cmd(FL, FR, HL, HR),
 wheel_vel_actual(FL, FR, HL, HR),
 wheel_torque(FL, FR, HL, HR),
 wheel_kd(FL, FR, HL, HR)]
```

Use this to decide where a motion problem is:

- command scale is zero: `/cmd_vel_raw` is not reaching `rl_deploy`
- wheel velocity command is zero: policy is not producing wheel motion
- wheel command is nonzero but actual wheel velocity is small: Gazebo actuator/contact issue
- wheel velocity is nonzero but body does not move: friction/collision/world contact issue

## Mapping And Autonomy Topics

Core robot topics:

| Topic | Type | Description |
|---|---|---|
| `/M20/IMU` | `sensor_msgs/msg/Imu` | Gazebo IMU |
| `/M20/LIDAR/FRONT` | `sensor_msgs/msg/PointCloud2` | Raw front LiDAR |
| `/M20/LIDAR/FRONT_LIO` | `sensor_msgs/msg/PointCloud2` | LiDAR with `time` and `ring` fields |
| `/M20/JOINTS_DATA` | `drdds/msg/JointsData` | Joint feedback to `rl_deploy` |
| `/M20/JOINTS_CMD` | `drdds/msg/JointsDataCmd` | Joint command from `rl_deploy` |
| `/M20/exec_debug` | `std_msgs/msg/Float32MultiArray` | Gazebo actuator debug |

Point-LIO outputs:

| Topic | Type | Description |
|---|---|---|
| `/aft_mapped_to_init` | `nav_msgs/msg/Odometry` | LIO odometry |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | Registered current cloud |
| `/path` or `/lio_path` | `nav_msgs/msg/Path` | LIO path |

CMU bridge outputs:

| Topic | Type | Description |
|---|---|---|
| `/state_estimation` | `nav_msgs/msg/Odometry` | Odometry for CMU stack |
| `/registered_scan` | `sensor_msgs/msg/PointCloud2` | Filtered cloud for CMU stack |
| `/overall_map` | `sensor_msgs/msg/PointCloud2` | Accumulated map from bridge |

CMU planning outputs:

| Topic | Type | Description |
|---|---|---|
| `/terrain_map` | `sensor_msgs/msg/PointCloud2` | Local terrain map |
| `/terrain_map_ext` | `sensor_msgs/msg/PointCloud2` | Extended terrain map |
| `/free_paths` | `sensor_msgs/msg/PointCloud2` | Candidate local paths |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Local planner velocity |
| `/cmd_vel_raw` | `geometry_msgs/msg/Twist` | M20 velocity command after relay |

## Expected Health Checks

After Gazebo starts:

```bash
ros2 topic hz /M20/IMU
ros2 topic hz /M20/LIDAR/FRONT
ros2 topic hz /M20/LIDAR/FRONT_LIO
```

After Point-LIO starts:

```bash
ros2 topic hz /aft_mapped_to_init
ros2 topic hz /cloud_registered
```

After CMU bridge starts:

```bash
ros2 topic hz /state_estimation
ros2 topic hz /registered_scan
```

After planning starts and a goal is sent:

```bash
ros2 topic hz /cmd_vel
ros2 topic hz /cmd_vel_raw
```

## Troubleshooting

### `ros2 node list` or `ros2 daemon stop` hangs

Kill stale ROS/Gazebo processes and restart the daemon:

```bash
pkill -9 -f ros2daemon
pkill -9 -f _ros2_daemon
pkill -9 -f "ign gazebo"
pkill -9 -f "parameter_bridge"
pkill -9 -f "gazebo_controller_ros2.py"
pkill -9 -f "pointcloud_lio_adapter.py"
pkill -9 -f "rl_deploy"
rm -rf ~/.ros/ros2_daemon
```

Then open a new terminal and source again.

### Gazebo launches but LiDAR has no messages

Check that `indoor.world` has Gazebo Sim system plugins and that the bridge is
running:

```bash
ign topic -l | grep front_lidar
ros2 topic info /M20/LIDAR/FRONT
ros2 topic hz /M20/LIDAR/FRONT
```

If `Publisher count` is nonzero but no messages arrive, restart Gazebo and make
sure no old `ign gazebo server` process is still running.

### Point-LIO says `Failed to find match for field 'time'` or `ring`

Use `/M20/LIDAR/FRONT_LIO`, not raw `/M20/LIDAR/FRONT`.
The adapter adds `time` and `ring` fields.

```bash
ros2 topic echo /M20/LIDAR/FRONT_LIO --once
```

The fields should include:

```text
x, y, z, intensity, time, ring
```

### RViz shows no map

Use `camera_init` or `map` as the fixed frame depending on the display:

- Point-LIO: `/cloud_registered`, `/aft_mapped_to_init`, frame `camera_init`
- CMU stack: `/registered_scan`, `/terrain_map`, `/overall_map`, frame `map`

Check:

```bash
ros2 topic echo /cloud_registered --once
ros2 topic echo /registered_scan --once
```

### Robot does not react to autonomy commands

Check the command chain:

```bash
ros2 topic echo /cmd_vel --once
ros2 topic echo /cmd_vel_raw --once
ros2 topic echo /m20_cmdvel_debug --once
ros2 topic echo /M20/exec_debug --once
```

`/m20_cmdvel_debug` only publishes when `/cmd_vel` or `/cmd_vel_raw` is
received. `/M20/exec_debug` should publish periodically while
`gazebo_controller_ros2.py` is running.

### Robot turns weakly or asymmetrically

The M20 policy and `policy.onnx` are unchanged from the original deploy
package. Turning behavior in this setup is affected by:

- `/cmd_vel_raw` scaling in `ros2_cmdvel_interface.hpp`
- relay scale in `cmd_vel_to_raw_relay.py`
- wheel velocity/torque execution in `gazebo_controller_ros2.py`
- Gazebo wheel collision and ground friction in `M20.sdf`

Use `/m20_cmdvel_debug` and `/M20/exec_debug` to isolate whether the problem is
command scaling, policy output, or Gazebo contact.

## Important Files

```text
launch/gazebo.launch.py
  Starts Gazebo Sim, robot spawn, ros_gz_bridge, Gazebo controller,
  and front LiDAR LIO adapter.

launch/autonomy_sim.launch.py
  Starts Point-LIO, LIO-to-CMU bridge, CMU terrain/local/FAR planners,
  cmd_vel relay, and RViz.

interface/user_command/ros2_cmdvel_interface.hpp
  Converts /cmd_vel and /cmd_vel_raw into M20 RL command scales.

interface/robot/simulation/gazebo_controller_ros2.py
  Converts M20 joint commands into Gazebo joint force commands and publishes
  /M20/exec_debug.

scripts/pointcloud_lio_adapter.py
  Adds time/ring fields to Gazebo point clouds for Point-LIO.

scripts/lio_to_cmu_bridge.py
  Converts Point-LIO outputs into CMU stack topics and publishes /overall_map.

scripts/cmd_vel_to_raw_relay.py
  Converts CMU /cmd_vel TwistStamped into /cmd_vel_raw Twist for M20.

model/M20_urdf/urdf/M20.sdf
  Gazebo Sim robot model with IMU, front/rear LiDAR, collisions, and friction.

model/indoor_large/indoor.world
  Main Gazebo Sim indoor environment.
```

## Notes

- The official M20 RL policy is not modified in this package.
- The original `M20.urdf` matches the official model at
  `/home/cjy/M20/M20_urdf/urdf/M20.urdf`.
- `M20.sdf` is the Gazebo Sim model used by `gazebo.launch.py`.
- `autonomy_sim.launch.py` does not launch Gazebo by default; this is
  intentional so Gazebo can be restarted independently.
- For real robot use, keep the namespace and topic structure, but review
  the simulation-only bridge/controller scripts before deployment.
