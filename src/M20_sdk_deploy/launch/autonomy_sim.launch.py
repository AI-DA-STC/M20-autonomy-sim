"""
Full autonomy simulation stack for M20 + CMU autonomy.

Pipeline:
  Gazebo Classic (indoor_nav world)
    → M20 LiDAR (/M20/LIDAR/FRONT) + IMU (/M20/IMU)
    → Point-LIO  →  /aft_mapped_to_init, /cloud_registered
    → lio_to_cmu_bridge  →  /state_estimation, /registered_scan, TF(map→sensor)
    → CMU: sensor_scan_generation, terrain_analysis, local_planner
    → /cmd_vel (TwistStamped)
    → rl_deploy (--autonomy)  →  joint commands  →  M20 robot

Recommended split usage:
  Terminal 1:
    ros2 launch rl_deploy gazebo_classic.launch.py lidar_preprocess:=true rviz:=false

  Terminal 2:
    ros2 launch rl_deploy autonomy_sim.launch.py route_planner:=true

Then in a separate terminal, stand the robot up:
  ros2 run rl_deploy rl_deploy --ros-args -r __ns:=/M20 -- --autonomy
  ros2 topic pub /m20_mode std_msgs/msg/String "{data: stand}"   (once)
  ros2 topic pub /m20_mode std_msgs/msg/String "{data: control}" (once)

Set a navigation goal in RViz with the 2D Nav Goal button,
or publish directly:
  ros2 topic pub /way_point geometry_msgs/msg/PointStamped \
    "{header: {frame_id: 'map'}, point: {x: 5.0, y: 3.0, z: 0.0}}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

RL_PKG      = get_package_share_directory('rl_deploy')
CMU_LP_PKG  = get_package_share_directory('local_planner')
FAR_PKG     = get_package_share_directory('far_planner')
GRAPH_PKG   = get_package_share_directory('graph_decoder')

WORLD_FILE  = os.path.join(RL_PKG, 'model', 'indoor_nav', 'indoor_nav.world')
XACRO_FILE  = os.path.join(RL_PKG, 'model', 'M20_urdf', 'urdf', 'M20_classic.urdf.xacro')
POINTLIO_CFG = os.path.join(RL_PKG, 'config', 'm20_gazebo_lio.yaml')
FAR_CFG = os.path.join(FAR_PKG, 'config', 'default.yaml')
GRAPH_CFG = os.path.join(GRAPH_PKG, 'config', 'default.yaml')


def generate_launch_description():
    # ── Arguments ────────────────────────────────────────────────────────────
    robot_name = LaunchConfiguration('robot_name')
    spawn_x    = LaunchConfiguration('x')
    spawn_y    = LaunchConfiguration('y')
    spawn_z    = LaunchConfiguration('z')
    route_planner = LaunchConfiguration('route_planner')
    sim = LaunchConfiguration('sim')
    lidar_preprocess = LaunchConfiguration('lidar_preprocess')
    point_lio_enabled = LaunchConfiguration('point_lio')
    bridge_enabled = LaunchConfiguration('bridge')
    cmu_enabled = LaunchConfiguration('cmu')
    cmd_vel_relay_enabled = LaunchConfiguration('cmd_vel_relay')
    rl_deploy_enabled = LaunchConfiguration('rl_deploy')
    rviz_enabled = LaunchConfiguration('rviz')

    declare_args = [
        DeclareLaunchArgument('robot_name', default_value='M20'),
        DeclareLaunchArgument('x', default_value='-8.0'),
        DeclareLaunchArgument('y', default_value='-3.0'),
        DeclareLaunchArgument('z', default_value='0.20'),
        DeclareLaunchArgument('route_planner', default_value='true'),
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('lidar_preprocess', default_value='false'),
        DeclareLaunchArgument('point_lio', default_value='true'),
        DeclareLaunchArgument('bridge', default_value='true'),
        DeclareLaunchArgument('cmu', default_value='true'),
        DeclareLaunchArgument('cmd_vel_relay', default_value='true'),
        DeclareLaunchArgument('rl_deploy', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    robot_description = Command(['xacro ', XACRO_FILE])
    use_sim_time = {'use_sim_time': True}

    # ── Gazebo Classic ───────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', WORLD_FILE,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        condition=IfCondition(sim),
    )

    # ── Robot state publisher ────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, **use_sim_time}],
        condition=IfCondition(sim),
    )

    # ── Spawn robot (wait for Gazebo) ────────────────────────────────────────
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', robot_name,
                   '-topic', 'robot_description',
                   '-x', spawn_x, '-y', spawn_y, '-z', spawn_z],
    )
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_robot], condition=IfCondition(sim))

    # ── ros2_control spawners ────────────────────────────────────────────────
    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    eff_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_effort_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    delayed_ctrl = TimerAction(
        period=10.0,
        actions=[jsb_spawner, eff_spawner],
        condition=IfCondition(sim),
    )

    # ── Effort bridge + Gazebo controller ───────────────────────────────────
    effort_bridge = Node(
        package='rl_deploy', executable='effort_bridge.py',
        output='screen', name='effort_bridge',
    )
    gazebo_controller = Node(
        package='rl_deploy', executable='gazebo_controller_ros2.py',
        output='screen', name='gazebo_controller',
        namespace='M20',
        parameters=[{'robot_name': 'M20', 'world_name': 'default', **use_sim_time}],
    )
    delayed_hw = TimerAction(
        period=12.0,
        actions=[effort_bridge, gazebo_controller],
        condition=IfCondition(sim),
    )

    # Add Velodyne-style ring/time fields to the front LiDAR before Point-LIO.
    lidar_lio_adapter = Node(
        package='rl_deploy',
        executable='pointcloud_lio_adapter.py',
        output='screen',
        name='pointcloud_lio_adapter',
        parameters=[
            use_sim_time,
            {
                'input_topic': '/M20/LIDAR/FRONT',
                'output_topic': '/M20/LIDAR/FRONT_LIO',
                'scan_rate': 10.0,
                'default_ring_count': 16,
                'horizontal_samples': 512,
            },
        ],
    )
    delayed_lidar_adapter = TimerAction(
        period=7.0,
        actions=[lidar_lio_adapter],
        condition=IfCondition(lidar_preprocess),
    )

    # ── rl_deploy state machine (autonomy mode) ──────────────────────────────
    # Starts after hw bridge is ready.
    # Operator must then: publish "stand" then "control" to /m20_mode
    rl_deploy = Node(
        package='rl_deploy', executable='rl_deploy',
        output='screen', name='rl_deploy',
        namespace='M20',          # relative topics → /M20/JOINTS_DATA etc.
        arguments=['--autonomy'],  # absolute topics (/cmd_vel, /m20_mode) unaffected
        parameters=[use_sim_time],
    )
    delayed_rl = TimerAction(
        period=14.0,
        actions=[rl_deploy],
        condition=IfCondition(rl_deploy_enabled),
    )

    # ── Point-LIO SLAM ───────────────────────────────────────────────────────
    point_lio = Node(
        package='point_lio_unilidar',
        executable='pointlio_mapping',
        output='screen',
        name='laserMapping',
        parameters=[POINTLIO_CFG, use_sim_time],
        remappings=[
            ('/path', '/lio_path'),
        ],
    )
    delayed_lio = TimerAction(
        period=8.0,
        actions=[point_lio],
        condition=IfCondition(point_lio_enabled),
    )

    # ── LIO → CMU bridge (remap frames, republish topics) ────────────────────
    lio_bridge = Node(
        package='rl_deploy', executable='lio_to_cmu_bridge.py',
        output='screen', name='lio_cmu_bridge',
        parameters=[use_sim_time],
    )
    delayed_bridge = TimerAction(
        period=9.0,
        actions=[lio_bridge],
        condition=IfCondition(bridge_enabled),
    )

    # ── CMU autonomy nodes ───────────────────────────────────────────────────
    sensor_scan_gen = Node(
        package='sensor_scan_generation',
        executable='sensorScanGeneration',
        output='screen', name='sensorScanGeneration',
        parameters=[use_sim_time],
    )

    terrain_analysis = Node(
        package='terrain_analysis',
        executable='terrainAnalysis',
        output='screen', name='terrainAnalysis',
        parameters=[
            use_sim_time,
            {'scanVoxelSize': 0.05},
            {'decayTime': 1.0},
            {'noDecayDis': 0.0},
            {'clearingDis': 8.0},
            {'useSorting': True},
            {'quantileZ': 0.25},
            {'considerDrop': False},
            {'limitGroundLift': False},
            {'maxGroundLift': 0.15},
            {'clearDyObs': True},
            {'minDyObsDis': 0.3},
            {'minDyObsAngle': 0.0},
            {'minDyObsRelZ': -0.3},
            {'absDyObsRelZThre': 0.2},
            {'minDyObsVFOV': -16.0},
            {'maxDyObsVFOV': 16.0},
            {'minDyObsPointNum': 1},
            {'noDataObstacle': False},
            {'noDataBlockSkipNum': 0},
            {'minBlockPointNum': 10},
            {'maxElevBelowVeh': -0.6},
            {'noDataAreaMinX': 0.3},
            {'noDataAreaMaxX': 1.8},
            {'noDataAreaMinY': -0.9},
            {'noDataAreaMaxY': 0.9},
            {'vehicleHeight': 1.5},
            {'voxelPointUpdateThre': 100},
            {'voxelTimeUpdateThre': 2.0},
            {'minRelZ': -1.5},
            {'maxRelZ': 0.5},
            {'disRatioZ': 0.2},
        ],
    )

    terrain_analysis_ext = Node(
        package='terrain_analysis_ext',
        executable='terrainAnalysisExt',
        output='screen', name='terrainAnalysisExt',
        parameters=[
            use_sim_time,
            {'scanVoxelSize': 0.1},
            {'decayTime': 1.5},
            {'noDecayDis': 0.0},
            {'clearingDis': 30.0},
            {'useSorting': True},
            {'quantileZ': 0.1},
            {'vehicleHeight': 1.5},
            {'voxelPointUpdateThre': 100},
            {'voxelTimeUpdateThre': 0.5},
            {'lowerBoundZ': -2.5},
            {'upperBoundZ': 1.0},
            {'disRatioZ': 0.1},
            {'checkTerrainConn': True},
            {'terrainConnThre': 0.5},
            {'terrainUnderVehicle': -0.75},
            {'ceilingFilteringThre': 2.0},
            {'localTerrainMapRadius': 4.0},
        ],
        condition=IfCondition(route_planner),
    )

    local_planner = Node(
        package='local_planner',
        executable='localPlanner',
        output='screen', name='localPlanner',
        parameters=[
            use_sim_time,
            {'pathFolder': os.path.join(CMU_LP_PKG, 'paths')},
            {'vehicleLength': 0.3},
            {'vehicleWidth': 0.7},
            {'laserVoxelSize': 0.05},
            {'terrainVoxelSize': 0.2},
            {'useTerrainAnalysis': True},
            {'checkObstacle': True},
            {'checkRotObstacle': False},
            {'adjacentRange': 3.0},
            {'obstacleHeightThre': 0.3},
            {'groundHeightThre': 0.1},
            {'costHeightThre': 0.1},
            {'costScore': 0.02},
            {'useCost': False},
            {'pointPerPathThre': 2},
            {'minRelZ': -0.5},
            {'maxRelZ': 0.25},
            {'maxSpeed': 1.0},
            {'dirWeight': 0.02},
            {'dirThre': 90.0},
            {'dirToVehicle': False},
            {'pathScale': 0.75},
            {'minPathScale': 0.5},
            {'pathScaleStep': 0.25},
            {'pathScaleBySpeed': True},
            {'minPathRange': 1.0},
            {'pathRangeStep': 0.5},
            {'pathRangeBySpeed': True},
            {'pathCropByGoal': True},
            {'autonomyMode': False},
            {'autonomySpeed': 1.0},
            {'joyToSpeedDelay': 2.0},
            {'joyToCheckObstacleDelay': 5.0},
            {'goalClearRange': 1.0},
            {'goalX': 0.0},
            {'goalY': 0.0},
            {'twoWayDrive': True},
        ],
    )

    path_follower = Node(
        package='local_planner',
        executable='pathFollower',
        output='screen', name='pathFollower',
        parameters=[
            use_sim_time,
            {'maxSpeed': 1.0},
            {'pubSkipNum': 1},
            {'maxYawRate': 60.0},
            {'lookAheadDis': 0.35},
            {'yawRateGain': 4.0},
            {'stopYawRateGain': 6.0},
            {'maxAccel': 2.0},
            {'switchTimeThre': 1.0},
            {'dirDiffThre': 0.15},
            {'omniDirDiffThre': 1.5},
            {'noRotSpeed': 10.0},
            {'stopDisThre': 0.3},
            {'slowDwnDisThre': 1.0},
            {'useInclRateToSlow': False},
            {'inclRateThre': 120.0},
            {'slowRate1': 0.25},
            {'slowRate2': 0.5},
            {'slowTime1': 2.0},
            {'slowTime2': 2.0},
            {'useInclToStop': False},
            {'inclThre': 45.0},
            {'stopTime': 5.0},
            {'noRotAtStop': False},
            {'noRotAtGoal': True},
            {'autonomyMode': False},
            {'autonomySpeed': 1.0},
            {'joyToSpeedDelay': 2.0},
            {'goalCloseDis': 0.4},
            {'twoWayDrive': True},
            {'is_real_robot': False},  # publish /cmd_vel only, skip Go2 sport API
        ],
    )

    cmd_vel_relay = Node(
        package='rl_deploy',
        executable='cmd_vel_to_raw_relay.py',
        output='screen',
        name='cmd_vel_to_raw_relay',
        parameters=[
            use_sim_time,
            {
                'input_topic': '/cmd_vel',
                'output_topic': '/cmd_vel_raw',
                'linear_x_scale': 1.0,
                'linear_y_scale': 0.3,
                'yaw_scale': 3.0,
                'max_linear': 1.0,
                'max_yaw': 2.0,
                'deadband': 0.01,
            },
        ],
        condition=IfCondition(cmd_vel_relay_enabled),
    )

    delayed_cmu = TimerAction(
        period=15.0,
        actions=[
            sensor_scan_gen,
            terrain_analysis,
            terrain_analysis_ext,
            local_planner,
            path_follower,
            cmd_vel_relay,
        ],
        condition=IfCondition(cmu_enabled),
    )

    far_planner = Node(
        package='far_planner',
        executable='far_planner',
        name='far_planner',
        output='screen',
        parameters=[
            FAR_CFG,
            use_sim_time,
            {
                'robot_dim': 0.6,
                'local_planner_range': 2.5,
                'util/obs_inflate_size': 1,
                'util/dynamic_obs_dacay_time': 0.25,
                'util/new_points_decay_time': 0.25,
                'g_planner/goal_adjust_radius': 1.0,
            },
        ],
        remappings=[
            ('/odom_world', '/state_estimation'),
            ('/terrain_cloud', '/terrain_map_ext'),
            ('/scan_cloud', '/terrain_map'),
            ('/terrain_local_cloud', '/registered_scan'),
        ],
        condition=IfCondition(route_planner),
    )

    graph_decoder = Node(
        package='graph_decoder',
        executable='graph_decoder',
        name='graph_decoder_node',
        output='screen',
        parameters=[
            use_sim_time,
            {
                'world_frame': 'map',
                'visual_scale_ratio': 0.5,
            },
        ],
        condition=IfCondition(route_planner),
    )

    delayed_route_planner = TimerAction(
        period=17.0,
        actions=[far_planner, graph_decoder],
        condition=IfCondition(route_planner),
    )

    # ── Static TFs for CMU ───────────────────────────────────────────────────
    # sensor → vehicle (M20 sensor is roughly at center, vehicle = base_link equiv)
    sensor_to_vehicle = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='sensor_to_vehicle',
        arguments=['0', '0', '0', '0', '0', '0', 'sensor', 'vehicle'],
        condition=IfCondition(bridge_enabled),
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz_cfg = os.path.join(FAR_PKG, 'rviz', 'default.rviz')

    rviz_args = ['-d', rviz_cfg] if rviz_cfg else []
    rviz = Node(
        package='rviz2', executable='rviz2',
        name='rviz2', output='screen',
        arguments=rviz_args,
        condition=IfCondition(rviz_enabled),
    )

    return LaunchDescription(
        declare_args + [
            gazebo,
            robot_state_publisher,
            delayed_spawn,
            delayed_ctrl,
            delayed_hw,
            delayed_lidar_adapter,
            delayed_lio,
            delayed_bridge,
            delayed_rl,
            delayed_cmu,
            delayed_route_planner,
            sensor_to_vehicle,
            rviz,
        ]
    )
