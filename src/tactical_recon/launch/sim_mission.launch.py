import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('tactical_recon')

    cooperative_mode_arg = DeclareLaunchArgument(
        'cooperative_mode', default_value='true')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true')
    uav_x_arg = DeclareLaunchArgument(
        'uav_x', default_value='-10.0')
    uav_y_arg = DeclareLaunchArgument(
        'uav_y', default_value='-10.0')

    world_file = os.path.join(pkg_share, 'worlds', 'military_sector.world')
    uav_urdf_file = os.path.join(pkg_share, 'urdf', 'scout_uav.urdf')
    ugv_urdf_file = os.path.join(pkg_share, 'urdf', 'recon_ugv.urdf')

    with open(uav_urdf_file, 'r') as f:
        uav_urdf_content = f.read()
    with open(ugv_urdf_file, 'r') as f:
        ugv_urdf_content = f.read()

    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        additional_env={'GAZEBO_MODEL_DATABASE_URI': ''})

    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')))

    uav_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='uav_robot_state_publisher',
        parameters=[{'robot_description': uav_urdf_content, 'use_sim_time': True}],
        remappings=[('/robot_description', '/uav/robot_description')],
        output='screen')

    ugv_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ugv_robot_state_publisher',
        parameters=[{'robot_description': ugv_urdf_content, 'use_sim_time': True}],
        remappings=[('/robot_description', '/ugv/robot_description')],
        output='screen')

    spawn_uav = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_uav',
        arguments=[
            '-entity', 'scout_uav',
            '-topic', '/uav/robot_description',
            '-x', LaunchConfiguration('uav_x'),
            '-y', LaunchConfiguration('uav_y'),
            '-z', '12.0',
            '-Y', '0.0'],
        parameters=[{'use_sim_time': True}],
        output='screen')

    spawn_ugv = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_ugv',
        arguments=[
            '-entity', 'recon_ugv',
            '-topic', '/ugv/robot_description',
            '-x', '-12.0',
            '-y', '-12.0',
            '-z', '0.2',
            '-Y', '0.0'],
        parameters=[{'use_sim_time': True}],
        output='screen')

    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}],
        output='screen')

    uav_patrol_node = Node(
        package='tactical_recon',
        executable='uav_patrol_node',
        name='uav_patrol_node',
        parameters=[{'use_sim_time': True}],
        output='screen')

    terrain_mapper_node = Node(
        package='tactical_recon',
        executable='terrain_mapper_node',
        name='terrain_mapper_node',
        parameters=[{'use_sim_time': True}],
        output='screen')

    global_planner_node = Node(
        package='tactical_recon',
        executable='global_planner_node',
        name='global_planner_node',
        parameters=[{
            'cooperative_mode': LaunchConfiguration('cooperative_mode'),
            'use_sim_time': True
        }],
        output='screen')

    ugv_controller_node = Node(
        package='tactical_recon',
        executable='ugv_controller_node',
        name='ugv_controller_node',
        parameters=[{
            'cooperative_mode': LaunchConfiguration('cooperative_mode'),
            'use_sim_time': True
        }],
        output='screen')

    mission_nodes = TimerAction(
        period=15.0,
        actions=[
            uav_patrol_node,
            terrain_mapper_node,
            global_planner_node,
            ugv_controller_node
        ])

    return LaunchDescription([
        cooperative_mode_arg,
        gui_arg,
        uav_x_arg,
        uav_y_arg,
        gazebo_server,
        gazebo_client,
        uav_rsp,
        ugv_rsp,
        spawn_uav,
        spawn_ugv,
        static_map_to_odom,
        mission_nodes
    ])
