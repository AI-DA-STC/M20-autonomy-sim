from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

PKG = get_package_share_directory('rl_deploy')
DEFAULT_WORLD_FILE = os.path.join(PKG, 'model', 'indoor_nav', 'indoor_nav.world')
XACRO_FILE = os.path.join(PKG, 'model', 'M20_urdf', 'urdf', 'M20_classic.urdf.xacro')
RVIZ_FILE = os.path.join(PKG, 'rviz', 'm20_gazebo_classic.rviz')


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    world = LaunchConfiguration('world')
    rviz_enabled = LaunchConfiguration('rviz')
    lidar_preprocess_enabled = LaunchConfiguration('lidar_preprocess')
    wheel_kd_scale = LaunchConfiguration('wheel_kd_scale')
    wheel_vel_scale = LaunchConfiguration('wheel_vel_scale')

    declare_args = [
        DeclareLaunchArgument('robot_name', default_value='M20'),
        DeclareLaunchArgument('world', default_value=DEFAULT_WORLD_FILE),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('lidar_preprocess', default_value='true'),
        DeclareLaunchArgument('wheel_kd_scale', default_value='1.0'),
        DeclareLaunchArgument('wheel_vel_scale', default_value='1.0'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-2.5'),
        DeclareLaunchArgument('z', default_value='0.30'),
    ]

    robot_description = Command(['xacro ', XACRO_FILE])

    # 1) Gazebo Classic
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )

    # 2) Robot State Publisher — makes robot_description available on the parameter server
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # 3) Spawn robot (delayed to let Gazebo start)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-topic', 'robot_description',
            '-x', x, '-y', y, '-z', z,
        ],
    )
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_robot])

    # 4) ros2_control spawners (after robot is in the world)
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    effort_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_effort_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    delayed_controllers = TimerAction(
        period=10.0,
        actions=[joint_state_broadcaster, effort_controller],
    )

    # 5) Effort bridge — translates per-joint Float64 → Float64MultiArray for effort controller
    #    and relays /joint_states → /M20/joint_states
    effort_bridge = Node(
        package='rl_deploy',
        executable='effort_bridge.py',
        output='screen',
        name='effort_bridge',
    )

    # 7) Gazebo controller (same as before — unchanged interface)
    controller_node = Node(
        package='rl_deploy',
        executable='gazebo_controller_ros2.py',
        output='screen',
        name='gazebo_controller',
        namespace='M20',
        parameters=[{
            'robot_name': 'M20',
            'world_name': 'default',
            'wheel_kd_scale': wheel_kd_scale,
            'wheel_vel_scale': wheel_vel_scale,
        }],
    )

    delayed_nodes = TimerAction(
        period=12.0,
        actions=[effort_bridge, controller_node],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', RVIZ_FILE],
        condition=IfCondition(rviz_enabled),
    )

    pointcloud_lio_adapter = Node(
        package='rl_deploy',
        executable='pointcloud_lio_adapter.py',
        output='screen',
        name='pointcloud_lio_adapter',
        parameters=[
            {'input_topic': '/M20/LIDAR/FRONT'},
            {'output_topic': '/M20/LIDAR/FRONT_LIO'},
            {'scan_rate': 10.0},
            {'default_ring_count': 16},
            {'horizontal_samples': 512},
        ],
        condition=IfCondition(lidar_preprocess_enabled),
    )

    return LaunchDescription(
        declare_args + [
            gazebo,
            robot_state_publisher,
            delayed_spawn,
            delayed_controllers,
            delayed_nodes,
            pointcloud_lio_adapter,
            rviz,
        ]
    )
