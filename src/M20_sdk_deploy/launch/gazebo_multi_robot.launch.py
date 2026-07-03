"""
Multi-Robot Gazebo Launch File

Launches two M20 robots (M20_A and M20_B) in the same Gazebo world.

Topic structure:
  Gazebo topics (per robot):
    /{robot_name}/joint_states
    /{robot_name}/IMU
    /{robot_name}/LIDAR/FRONT
    /{robot_name}/LIDAR/REAR
    /model/{robot_name}/joint/*/cmd_force

  DDS topics (per robot, namespaced):
    /{robot_name}/JOINTS_DATA
    /{robot_name}/IMU_DATA
    /{robot_name}/JOINTS_CMD

Usage:
  ros2 launch rl_deploy gazebo_multi_robot.launch.py

  # Then in separate terminals:
  ros2 run rl_deploy rl_deploy --ros-args -r __ns:=/M20_A
  ros2 run rl_deploy rl_deploy --ros-args -r __ns:=/M20_B
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import os

# Path to the model directory containing Depot_simple
MODEL_PATH = "/home/cjy/deeprobotics_ws/src/M20_sdk_deploy/model"
# Path to the simplified world file
WORLD_FILE = "/home/cjy/deeprobotics_ws/src/M20_sdk_deploy/model/Edifice_simple/edifice_simple.sdf"
# Path to the robot SDF
ROBOT_SDF = "/home/cjy/deeprobotics_ws/src/M20_sdk_deploy/model/M20_urdf/urdf/M20.sdf"

# Robot configurations
ROBOTS = [
    {"name": "M20_A", "x": "-2.0", "y": "2.0", "z": "0.57", "yaw": "0.0"},
    {"name": "M20_B", "x": "-2.0", "y": "-2.0", "z": "0.57", "yaw": "0.0"},
]

# Joint names for bridge setup
JOINT_NAMES = [
    'fl_hipx_joint', 'fl_hipy_joint', 'fl_knee_joint', 'fl_wheel_joint',
    'fr_hipx_joint', 'fr_hipy_joint', 'fr_knee_joint', 'fr_wheel_joint',
    'hl_hipx_joint', 'hl_hipy_joint', 'hl_knee_joint', 'hl_wheel_joint',
    'hr_hipx_joint', 'hr_hipy_joint', 'hr_knee_joint', 'hr_wheel_joint'
]


def create_robot_spawn(robot_config, world_name="Edifice"):
    """Create spawn node for a robot"""
    return Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", world_name,
            "-name", robot_config["name"],
            "-file", ROBOT_SDF,
            "-x", robot_config["x"],
            "-y", robot_config["y"],
            "-z", robot_config["z"],
            "-Y", robot_config.get("yaw", "0.0"),
        ],
    )


def create_robot_bridges(robot_name, world_name="Edifice"):
    """Create all bridge nodes for a robot"""
    bridges = []

    # Joint states bridge
    bridges.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{robot_name}_joint_state_bridge',
        arguments=[
            f'/world/{world_name}/model/{robot_name}/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model'
        ],
        remappings=[
            (f'/world/{world_name}/model/{robot_name}/joint_state', f'/{robot_name}/joint_states')
        ],
        output='screen'
    ))

    # IMU bridge - note: sensor topics use /world/{world}/model/{robot}/... format
    bridges.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{robot_name}_imu_bridge',
        arguments=[
            f'/world/{world_name}/model/{robot_name}/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'
        ],
        remappings=[
            (f'/world/{world_name}/model/{robot_name}/link/base_link/sensor/imu_sensor/imu', f'/{robot_name}/IMU')
        ],
        output='screen'
    ))

    # Front LiDAR bridge - note: sensor topics use /world/{world}/model/{robot}/... format
    bridges.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{robot_name}_front_lidar_bridge',
        arguments=[
            f'/world/{world_name}/model/{robot_name}/link/base_link/sensor/front_lidar/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        ],
        remappings=[
            (f'/world/{world_name}/model/{robot_name}/link/base_link/sensor/front_lidar/scan/points', f'/{robot_name}/LIDAR/FRONT')
        ],
        output='screen'
    ))

    # Rear LiDAR bridge - note: sensor topics use /world/{world}/model/{robot}/... format
    bridges.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{robot_name}_rear_lidar_bridge',
        arguments=[
            f'/world/{world_name}/model/{robot_name}/link/base_link/sensor/rear_lidar/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        ],
        remappings=[
            (f'/world/{world_name}/model/{robot_name}/link/base_link/sensor/rear_lidar/scan/points', f'/{robot_name}/LIDAR/REAR')
        ],
        output='screen'
    ))

    # Joint force command bridges
    for joint_name in JOINT_NAMES:
        bridges.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{robot_name}_{joint_name}_bridge',
            arguments=[
                f'/model/{robot_name}/joint/{joint_name}/cmd_force@std_msgs/msg/Float64]ignition.msgs.Double'
            ],
            output='screen'
        ))

    return bridges


def create_robot_controller(robot_name, world_name="Edifice"):
    """Create the gazebo controller node for a robot with namespace"""
    return GroupAction([
        PushRosNamespace(robot_name),
        Node(
            package='rl_deploy',
            executable='gazebo_controller_ros2.py',
            name='gazebo_controller',
            output='screen',
            parameters=[{
                'robot_name': robot_name,
                'world_name': world_name,
            }],
        )
    ])


def generate_launch_description():
    world_name = "Edifice"

    # Get existing IGN_GAZEBO_RESOURCE_PATH and append our model path
    existing_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    new_resource_path = f"{MODEL_PATH}:{existing_path}" if existing_path else MODEL_PATH

    # Set environment variable for Gazebo to find our custom models
    set_ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=new_resource_path
    )

    # 1) Start Gazebo with simplified world (auto-run, not paused)
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-v", "4", "-r", WORLD_FILE],
        output="screen",
        additional_env={"IGN_GAZEBO_RESOURCE_PATH": new_resource_path},
    )

    # 2) GPU monitoring
    gpu_monitor = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "sleep 10 && while true; do "
            "echo '' && "
            "echo '[GPU STATS] '$(date '+%H:%M:%S')' -------------------------------------------' && "
            "nvidia-smi --query-gpu=utilization.gpu,utilization.memory,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits | "
            "awk -F', ' '{printf \"  GPU Util: %3d%% | Mem Util: %3d%% | VRAM: %s/%s MiB | Temp: %s C\\n\", $1, $2, $3, $4, $5}' && "
            "sleep 5; "
            "done"
        ],
        output="screen",
    )

    # 3) Spawn robots (staggered to avoid collision during spawn)
    spawn_actions = []
    for i, robot in enumerate(ROBOTS):
        spawn_delay = 8.0 + (i * 3.0)  # M20_A at 8s, M20_B at 11s
        spawn_actions.append(
            TimerAction(
                period=spawn_delay,
                actions=[create_robot_spawn(robot, world_name)]
            )
        )

    # 4) Create bridges and controllers for all robots (after spawning)
    bridge_controller_actions = []
    for i, robot in enumerate(ROBOTS):
        robot_name = robot["name"]
        start_delay = 15.0 + (i * 2.0)  # M20_A bridges at 15s, M20_B at 17s

        # Bridges for this robot
        bridges = create_robot_bridges(robot_name, world_name)

        # Controller for this robot
        controller = create_robot_controller(robot_name, world_name)

        bridge_controller_actions.append(
            TimerAction(
                period=start_delay,
                actions=bridges + [controller]
            )
        )

    # Combine all actions
    return LaunchDescription([
        set_ign_resource_path,
        gazebo,
        gpu_monitor,
    ] + spawn_actions + bridge_controller_actions)
