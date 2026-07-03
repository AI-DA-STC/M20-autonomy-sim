from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

# Path to the model directory containing Depot_simple
MODEL_PATH = "/home/cjy/deeprobotics_ws/src/M20_sdk_deploy/model"
# Path to the simplified world file
WORLD_FILE = "/home/cjy/deeprobotics_ws/src/M20_sdk_deploy/model/Edifice_simple/edifice_simple.sdf"


def generate_launch_description():
    world_name = LaunchConfiguration("world_name")
    robot_name = LaunchConfiguration("robot_name")
    robot_sdf = LaunchConfiguration("robot_sdf")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    # Get existing IGN_GAZEBO_RESOURCE_PATH and append our model path
    existing_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    new_resource_path = f"{MODEL_PATH}:{existing_path}" if existing_path else MODEL_PATH

    declare_args = [
        DeclareLaunchArgument("world_name", default_value="Edifice"),
        DeclareLaunchArgument("robot_name", default_value="M20"),
        DeclareLaunchArgument(
            "robot_sdf",
            default_value="/home/cjy/deeprobotics_ws/src/M20_sdk_deploy/model/M20_urdf/urdf/M20_velodyne.sdf",
        ),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.57"),  # Higher spawn to prevent immediate fall
    ]

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

    # 2) GPU monitoring - shows GPU utilization, VRAM, and temperature every 3 seconds
    gpu_monitor = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "sleep 8 && while true; do "
            "echo '' && "
            "echo '[GPU STATS] '$(date '+%H:%M:%S')' -------------------------------------------' && "
            "nvidia-smi --query-gpu=utilization.gpu,utilization.memory,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits | "
            "awk -F', ' '{printf \"  GPU Util: %3d%% | Mem Util: %3d%% | VRAM: %s/%s MiB | Temp: %s C\\n\", $1, $2, $3, $4, $5}' && "
            "sleep 3; "
            "done"
        ],
        output="screen",
    )

    # 3) Gazebo stats monitor - shows real-time factor and sim time
    gazebo_stats_monitor = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "sleep 10 && while true; do "
            "ign topic -e -t /stats -n 1 2>/dev/null | "
            "grep -E '(real_time_factor|sim_time|iterations)' | head -6 | "
            "awk '/real_time_factor/ {rtf=$2} /sim_time.*sec:/ {st=$2} /iterations/ {iter=$2} "
            "END {if(rtf) printf \"[SIM STATS] RTF: %.2f | Iterations: %s\\n\", rtf, iter}' && "
            "sleep 3; "
            "done"
        ],
        output="screen",
    )

    # 4) Spawn robot with ros_gz_sim create (delayed to let world load)
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", world_name,
            "-name", robot_name,
            "-file", robot_sdf,
            "-x", x,
            "-y", y,
            "-z", z,
        ],
    )

    # Spawn robot after 8 seconds to let world fully load
    delayed_spawn = TimerAction(period=8.0, actions=[spawn_robot])

    # 3) Bridge joint states from Gazebo to ROS2
    # Note: Topics are namespaced with /M20/ to be consistent with multi-robot setup
    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/Edifice/model/M20/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model'
        ],
        remappings=[
            ('/world/Edifice/model/M20/joint_state', '/M20/joint_states')
        ],
        output='screen'
    )

    # 4) Bridge IMU data from Gazebo to ROS2
    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/Edifice/model/M20/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'
        ],
        remappings=[
            ('/world/Edifice/model/M20/link/base_link/sensor/imu_sensor/imu', '/M20/IMU')
        ],
        output='screen'
    )

    # 5) Bridge Velodyne HDL-32E point cloud data from Gazebo to ROS2
    bridge_velodyne = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/Edifice/model/M20/link/base_link/sensor/velodyne_hdl32e/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        ],
        remappings=[
            ('/world/Edifice/model/M20/link/base_link/sensor/velodyne_hdl32e/scan/points', '/M20/LIDAR/VELODYNE')
        ],
        output='screen'
    )

    # 5) Bridge joint force commands for each joint
    bridge_joints = []
    joint_names = [
        'fl_hipx_joint', 'fl_hipy_joint', 'fl_knee_joint', 'fl_wheel_joint',
        'fr_hipx_joint', 'fr_hipy_joint', 'fr_knee_joint', 'fr_wheel_joint',
        'hl_hipx_joint', 'hl_hipy_joint', 'hl_knee_joint', 'hl_wheel_joint',
        'hr_hipx_joint', 'hr_hipy_joint', 'hr_knee_joint', 'hr_wheel_joint'
    ]
    
    for joint_name in joint_names:
        bridge_joints.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    f'/model/M20/joint/{joint_name}/cmd_force@std_msgs/msg/Float64]ignition.msgs.Double'
                ],
                output='screen'
            )
        )

    # 6) Controller node - Python script with parameters
    # Note: Using /M20 namespace for DDS topics (JOINTS_DATA, IMU_DATA, JOINTS_CMD)
    controller_node = Node(
        package='rl_deploy',
        executable='gazebo_controller_ros2.py',
        output='screen',
        name='gazebo_controller',
        namespace='M20',  # DDS topics will be /M20/JOINTS_DATA, /M20/IMU_DATA, etc.
        parameters=[{
            'robot_name': 'M20',
            'world_name': 'Edifice',
        }],
    )

    # Start bridges and controller after robot spawns
    delayed_bridges_controller = TimerAction(
        period=15.0, 
        actions=[bridge_joint_states, bridge_imu] + bridge_joints + [controller_node] + [bridge_velodyne]
    )

    return LaunchDescription(declare_args + [set_ign_resource_path, gazebo, gpu_monitor, gazebo_stats_monitor, delayed_spawn, delayed_bridges_controller])
