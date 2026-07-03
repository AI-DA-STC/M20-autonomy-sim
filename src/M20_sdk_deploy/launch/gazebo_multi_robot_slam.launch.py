"""
Gazebo Multi-Robot Simulation with Single-Robot SLAM

Launches Gazebo with both robots but runs GLIM SLAM for only ONE robot at a time.
This allows sequential mapping followed by offline map merging.

Usage:
  # SLAM for M20_A (default):
  ros2 launch rl_deploy gazebo_multi_robot_slam.launch.py robot_name:=M20_A

  # SLAM for M20_B:
  ros2 launch rl_deploy gazebo_multi_robot_slam.launch.py robot_name:=M20_B

  # Then control the robot:
  ros2 run rl_deploy rl_deploy --ros-args -r __ns:=/M20_A

Workflow:
  1. Launch with robot_name:=M20_A, map the environment, save map
  2. Restart with robot_name:=M20_B, map the environment, save map
  3. Use GLIM offline viewer to merge maps:
     https://koide3.github.io/glim/merge.html
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction,
    SetEnvironmentVariable, GroupAction, OpaqueFunction
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import os

# Paths
MODEL_PATH = "/home/cjy/deeprobotics_ws/src/M20_sdk_deploy/model"
WORLD_FILE = "/home/cjy/deeprobotics_ws/src/M20_sdk_deploy/model/Edifice_simple/edifice_simple.sdf"
ROBOT_SDF = "/home/cjy/deeprobotics_ws/src/M20_sdk_deploy/model/M20_urdf/urdf/M20.sdf"
CONFIG_BASE_PATH = "/home/cjy/deeprobotics_ws/src/M20_sdk_deploy/config/glim"

# Robot spawn positions
ROBOT_CONFIGS = {
    "M20_A": {"x": "-2.0", "y": "2.0", "z": "0.57", "yaw": "0.0"},
    "M20_B": {"x": "-2.0", "y": "-2.0", "z": "0.57", "yaw": "0.0"},
}

# Joint names for bridge setup
JOINT_NAMES = [
    'fl_hipx_joint', 'fl_hipy_joint', 'fl_knee_joint', 'fl_wheel_joint',
    'fr_hipx_joint', 'fr_hipy_joint', 'fr_knee_joint', 'fr_wheel_joint',
    'hl_hipx_joint', 'hl_hipy_joint', 'hl_knee_joint', 'hl_wheel_joint',
    'hr_hipx_joint', 'hr_hipy_joint', 'hr_knee_joint', 'hr_wheel_joint'
]


def create_robot_spawn(robot_name, world_name="Edifice"):
    """Create spawn node for a robot"""
    config = ROBOT_CONFIGS[robot_name]
    return Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", world_name,
            "-name", robot_name,
            "-file", ROBOT_SDF,
            "-x", config["x"],
            "-y", config["y"],
            "-z", config["z"],
            "-Y", config.get("yaw", "0.0"),
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

    # IMU bridge
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

    # Front LiDAR bridge
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

    # Rear LiDAR bridge
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
    """Create the gazebo controller node for a robot"""
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


def create_static_tf_publishers(robot_name):
    """Create static transform publishers for sensor frames"""
    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{robot_name}_front_lidar_tf',
            arguments=[
                '0.32028', '0', '-0.013', '0', '0', '0',
                f'{robot_name}/base_link', f'{robot_name}/front_lidar'
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{robot_name}_rear_lidar_tf',
            arguments=[
                '-0.32028', '0', '-0.013', '0', '0', '3.14159',
                f'{robot_name}/base_link', f'{robot_name}/rear_lidar'
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{robot_name}_imu_tf',
            arguments=[
                '0.0632', '-0.0268', '-0.0435', '0', '0', '0',
                f'{robot_name}/base_link', f'{robot_name}/imu'
            ],
        ),
    ]


def create_glim_nodes(robot_name):
    """Create GLIM SLAM nodes for the specified robot only"""
    config_path = os.path.join(CONFIG_BASE_PATH, robot_name)

    nodes = []

    # Static TF publishers for this robot
    nodes.extend(create_static_tf_publishers(robot_name))

    # GLIM ROS node - uses front lidar directly (no merger needed)
    nodes.append(
        Node(
            package='glim_ros',
            executable='glim_rosnode',
            name=f'{robot_name}_glim_slam',
            output='screen',
            parameters=[
                {'config_path': config_path},
                {'use_sim_time': True},
            ],
        )
    )

    return nodes


def launch_setup(context, *args, **kwargs):
    """Setup launch based on robot_name argument"""
    robot_name = LaunchConfiguration('robot_name').perform(context)
    world_name = "Edifice"

    print(f"\n{'='*60}")
    print(f"  GLIM SLAM launching for: {robot_name}")
    print(f"  Map will be saved to: {CONFIG_BASE_PATH}/{robot_name}/")
    print(f"{'='*60}\n")

    actions = []

    # Spawn BOTH robots (so world is consistent across runs)
    for i, name in enumerate(["M20_A", "M20_B"]):
        spawn_delay = 8.0 + (i * 3.0)
        actions.append(
            TimerAction(
                period=spawn_delay,
                actions=[create_robot_spawn(name, world_name)]
            )
        )

    # Bridges and controllers for BOTH robots (so either can be controlled)
    for i, name in enumerate(["M20_A", "M20_B"]):
        start_delay = 15.0 + (i * 2.0)
        bridges = create_robot_bridges(name, world_name)
        controller = create_robot_controller(name, world_name)
        actions.append(
            TimerAction(
                period=start_delay,
                actions=bridges + [controller]
            )
        )

    # GLIM SLAM for ONLY the specified robot (single viewer)
    glim_delay = 25.0
    glim_nodes = create_glim_nodes(robot_name)
    actions.append(
        TimerAction(
            period=glim_delay,
            actions=glim_nodes
        )
    )

    return actions


def generate_launch_description():
    # Environment setup
    existing_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    new_resource_path = f"{MODEL_PATH}:{existing_path}" if existing_path else MODEL_PATH

    return LaunchDescription([
        # Declare robot_name argument
        DeclareLaunchArgument(
            'robot_name',
            default_value='M20_A',
            description='Robot to run SLAM for (M20_A or M20_B)'
        ),

        # Set Gazebo resource path
        SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=new_resource_path
        ),

        # Start Gazebo
        ExecuteProcess(
            cmd=["ign", "gazebo", "-v", "4", "-r", WORLD_FILE],
            output="screen",
            additional_env={"IGN_GAZEBO_RESOURCE_PATH": new_resource_path},
        ),

        # GPU monitoring
        ExecuteProcess(
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
        ),

        # Dynamic launch setup based on robot_name
        OpaqueFunction(function=launch_setup),
    ])
