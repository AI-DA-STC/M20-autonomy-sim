from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

PKG = get_package_share_directory("rl_deploy")
SOURCE_PKG = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
MODEL_ROOT = SOURCE_PKG if os.path.isdir(os.path.join(SOURCE_PKG, "model", "indoor_large")) else PKG
MODEL_PATH = os.path.join(MODEL_ROOT, "model")
WORLD_FILE = os.path.join(MODEL_PATH, "indoor_large", "indoor.world")
ROBOT_SDF = os.path.join(MODEL_PATH, "M20_urdf", "urdf", "M20.sdf")


def generate_launch_description():
    world_name = LaunchConfiguration("world_name")
    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")
    robot_sdf = LaunchConfiguration("robot_sdf")
    lidar_preprocess = LaunchConfiguration("lidar_preprocess")
    wheel_kd_scale = LaunchConfiguration("wheel_kd_scale")
    wheel_vel_scale = LaunchConfiguration("wheel_vel_scale")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    joint_state_gz_topic = PythonExpression([
        "'/world/", world_name, "/model/", robot_name, "/joint_state'"
    ])
    joint_state_bridge = PythonExpression([
        "'/world/", world_name, "/model/", robot_name,
        "/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model'"
    ])
    imu_gz_topic = PythonExpression([
        "'/world/", world_name, "/model/", robot_name,
        "/link/base_link/sensor/imu_sensor/imu'"
    ])
    imu_bridge = PythonExpression([
        "'/world/", world_name, "/model/", robot_name,
        "/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'"
    ])
    front_lidar_gz_topic = PythonExpression([
        "'/world/", world_name, "/model/", robot_name,
        "/link/base_link/sensor/front_lidar/scan/points'"
    ])
    front_lidar_bridge = PythonExpression([
        "'/world/", world_name, "/model/", robot_name,
        "/link/base_link/sensor/front_lidar/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'"
    ])
    rear_lidar_gz_topic = PythonExpression([
        "'/world/", world_name, "/model/", robot_name,
        "/link/base_link/sensor/rear_lidar/scan/points'"
    ])
    rear_lidar_bridge = PythonExpression([
        "'/world/", world_name, "/model/", robot_name,
        "/link/base_link/sensor/rear_lidar/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'"
    ])
    ros_joint_state_topic = PythonExpression(["'/", robot_name, "/joint_states'"])
    ros_imu_topic = PythonExpression(["'/", robot_name, "/IMU'"])
    ros_front_lidar_topic = PythonExpression(["'/", robot_name, "/LIDAR/FRONT'"])
    ros_rear_lidar_topic = PythonExpression(["'/", robot_name, "/LIDAR/REAR'"])

    # Get existing Gazebo resource paths and append our model paths.
    # indoor_large/indoor.world uses model://indoor, so the parent directory
    # of the indoor model must be in the resource path.
    existing_ign_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    indoor_large_model_path = os.path.join(MODEL_PATH, "indoor_large")
    base_resource_path = f"{MODEL_PATH}:{indoor_large_model_path}"
    new_ign_resource_path = (
        f"{base_resource_path}:{existing_ign_path}"
        if existing_ign_path else base_resource_path
    )
    new_gz_resource_path = (
        f"{base_resource_path}:{existing_gz_path}"
        if existing_gz_path else base_resource_path
    )

    declare_args = [
        DeclareLaunchArgument("world", default_value=WORLD_FILE),
        DeclareLaunchArgument("world_name", default_value="default"),
        DeclareLaunchArgument("robot_name", default_value="M20"),
        DeclareLaunchArgument("robot_sdf", default_value=ROBOT_SDF),
        DeclareLaunchArgument("lidar_preprocess", default_value="true"),
        DeclareLaunchArgument("wheel_kd_scale", default_value="1.0"),
        DeclareLaunchArgument("wheel_vel_scale", default_value="1.0"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.57"),  # Higher spawn to prevent immediate fall
    ]

    # Set environment variable for Gazebo to find our custom models
    set_ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=new_ign_resource_path
    )
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=new_gz_resource_path
    )

    # 1) Start Gazebo with selected world (auto-run, not paused)
    # Force NVIDIA GPU via PRIME render offload (Intel Arc 0x7d67 is unsupported by Mesa)
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-v", "4", "-r", world],
        output="screen",
        additional_env={
            "IGN_GAZEBO_RESOURCE_PATH": new_ign_resource_path,
            "GZ_SIM_RESOURCE_PATH": new_gz_resource_path,
            "__NV_PRIME_RENDER_OFFLOAD": "1",
            "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
            "__EGL_VENDOR_LIBRARY_FILENAMES": "/usr/share/glvnd/egl_vendor.d/10_nvidia.json",
        },
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

    # 3) Bridge Gazebo <-> ROS topics in one parameter_bridge process.
    # Starting one bridge per joint creates many duplicate /ros_gz_bridge nodes
    # and can make ROS graph discovery sluggish or stuck after shutdown.
    bridge_args = [joint_state_bridge, imu_bridge]
    bridge_remappings = [
        (joint_state_gz_topic, ros_joint_state_topic),
        (imu_gz_topic, ros_imu_topic),
        (front_lidar_gz_topic, ros_front_lidar_topic),
        (rear_lidar_gz_topic, ros_rear_lidar_topic),
    ]
    joint_names = [
        'fl_hipx_joint', 'fl_hipy_joint', 'fl_knee_joint', 'fl_wheel_joint',
        'fr_hipx_joint', 'fr_hipy_joint', 'fr_knee_joint', 'fr_wheel_joint',
        'hl_hipx_joint', 'hl_hipy_joint', 'hl_knee_joint', 'hl_wheel_joint',
        'hr_hipx_joint', 'hr_hipy_joint', 'hr_knee_joint', 'hr_wheel_joint'
    ]
    
    for joint_name in joint_names:
        bridge_args.append(
            PythonExpression([
                "'/model/", robot_name, "/joint/", joint_name,
                "/cmd_force@std_msgs/msg/Float64]ignition.msgs.Double'"
            ])
        )

    bridge_args.extend([front_lidar_bridge, rear_lidar_bridge])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=bridge_args,
        remappings=bridge_remappings,
        output='screen'
    )

    # 4) Controller node - Python script with parameters
    # Uses /<robot_name> namespace for DDS topics (JOINTS_DATA, IMU_DATA, JOINTS_CMD)
    controller_node = Node(
        package='rl_deploy',
        executable='gazebo_controller_ros2.py',
        output='screen',
        name='gazebo_controller',
        namespace=robot_name,  # DDS topics will be /<robot_name>/JOINTS_DATA, /<robot_name>/IMU_DATA, etc.
        parameters=[{
            'robot_name': robot_name,
            'world_name': world_name,
            'wheel_kd_scale': wheel_kd_scale,
            'wheel_vel_scale': wheel_vel_scale,
        }],
    )

    pointcloud_lio_adapter = Node(
        package='rl_deploy',
        executable='pointcloud_lio_adapter.py',
        output='screen',
        name='pointcloud_lio_adapter',
        parameters=[
            {
                'input_topic': ros_front_lidar_topic,
                'output_topic': PythonExpression(["'/", robot_name, "/LIDAR/FRONT_LIO'"]),
                'scan_rate': 10.0,
                'default_ring_count': 16,
                'horizontal_samples': 512,
            },
        ],
    )

    # Start bridges and controller after robot spawns
    delayed_bridges_controller = TimerAction(
        period=15.0, 
        actions=[bridge, controller_node]
    )

    delayed_lidar_adapter = TimerAction(
        period=17.0,
        actions=[pointcloud_lio_adapter],
        condition=IfCondition(lidar_preprocess),
    )

    return LaunchDescription(declare_args + [
        set_ign_resource_path,
        set_gz_resource_path,
        gazebo,
        gpu_monitor,
        gazebo_stats_monitor,
        delayed_spawn,
        delayed_bridges_controller,
        delayed_lidar_adapter,
    ])
