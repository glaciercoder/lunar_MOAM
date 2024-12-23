from launch import LaunchDescription
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationNotEquals, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
import os
import xacro
from ament_index_python.packages import get_package_share_directory

import sys
sys.path.append("src/turtle_bot_description/launch") 
from globals import *


def generate_launch_description():
    robots = [
        {'name': 'robot0', 'x_pos': 0.0, 'y_pos': 0.5, 'z_pos': 1.01,
            "color_name" : 'Yellow', "color_rgb" : "1 1 0 1"},
        {'name': 'robot1', 'x_pos': 0.0, 'y_pos': -0.5, 'z_pos': 1.01,
            "color_name" : 'Blue', "color_rgb" : "0 0 1 1"},
        {'name': 'robot2', 'x_pos': 0.5, 'y_pos': -0.5, 'z_pos': 1.01,
            "color_name" : 'Blue', "color_rgb" : "0 0 1 1"}
    ]

    arrNodes = []

    # namespace = LaunchConfiguration('namespace')
    # use_namespace = LaunchConfiguration('use_namespace')

    world = LaunchConfiguration('world')
    # simulator = LaunchConfiguration('simulator')
    use_simulator = LaunchConfiguration('use_simulator')
    ##headless = LaunchConfiguration('headless')

    #use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    # use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    # Create the launch configuration variables
    
    # simple, slam, map
    mode = LaunchConfiguration('mode')
    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='simple',
        description='Whether run robot in a simple mode, SLAM or Map')
    arrNodes.append(declare_mode_cmd)

    #slam = LaunchConfiguration('slam')
    slam = PythonExpression(["'", mode, "'=='slam'"]) 
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Whether run a SLAM')
    arrNodes.append(declare_slam_cmd)

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='True',
        description='Whether to execute gzclient')
    arrNodes.append(declare_simulator_cmd)
    map_yaml_file = LaunchConfiguration('map')
    keepout_mask = LaunchConfiguration('keepout_mask')

    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    #autostart = LaunchConfiguration('autostart')

    # declare_namespace_cmd = DeclareLaunchArgument(
    #     'namespace',
    #     default_value='',
    #     description='Top-level namespace')
    # arrNodes.append(declare_namespace_cmd)

    # declare_use_namespace_cmd = DeclareLaunchArgument(
    #     'use_namespace',
    #     default_value='false',
    #     description='Whether to apply a namespace to the navigation stack')    
    # arrNodes.append(declare_use_namespace_cmd)

    # def_maps_path declared in globals.py
    # declare_map_yaml_cmd = DeclareLaunchArgument(
    #     'map',
    #     default_value=def_maps_path,
    #     description='Full path to map file to load')
    # arrNodes.append(declare_map_yaml_cmd)
    
    # declare_keepout_mask_yaml_cmd = DeclareLaunchArgument(
    #     'keepout_mask',
    #     default_value=def_keepout_mask_path,
    #     description='Full path to keepout mask file to load')    
    # arrNodes.append(declare_keepout_mask_yaml_cmd)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    arrNodes.append(declare_use_sim_time_cmd)

    # # def_nav2_params_path declared in globals.py
    # declare_params_file_cmd = DeclareLaunchArgument(
    #     'params_file',
    #     default_value=def_nav2_params_path,
    #     description='Full path to the ROS2 parameters file to use for all launched nodes')
    # arrNodes.append(declare_params_file_cmd)

    # # def_nav2_bt_navigator_path declared in globals.py
    # declare_bt_xml_cmd = DeclareLaunchArgument(
    #     'default_bt_xml_filename',
    #     default_value=def_nav2_bt_navigator_path,
    #     description='Full path to the behavior tree xml file to use')
    # arrNodes.append(declare_bt_xml_cmd)

    # declare_autostart_cmd = DeclareLaunchArgument(
    #     'autostart', default_value='True',
    #     description='Automatically startup the nav2 stack')
    # arrNodes.append(declare_autostart_cmd)

    # def_rviz_path defined in globals.py
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=def_rviz_path,
        description='Full path to the RVIZ config file to use')
    arrNodes.append(declare_rviz_config_file_cmd)

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')
    arrNodes.append(declare_use_simulator_cmd)

    # declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    #     'use_robot_state_pub',
    #     default_value='True',
    #     description='Whether to start the robot state publisher')
    # arrNodes.append(declare_use_robot_state_pub_cmd)

    # declare_use_rviz_cmd = DeclareLaunchArgument(
    #     'use_rviz',
    #     default_value='True',
    #     description='Whether to start RVIZ')
    # arrNodes.append(declare_use_rviz_cmd)

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=def_world_path,
        description='Full path to world file to load')
    arrNodes.append(declare_world_cmd)

    # declare_simulator_cmd = DeclareLaunchArgument(
    #     'simulator',
    #     default_value='gazebo',
    #     description='The simulator to use (gazebo or gzserver)')
    # arrNodes.append(declare_simulator_cmd)
   
    # gazebo_node = ExecuteProcess(cmd=['gazebo', '--verbose', world,'-s', 
    #     'libgazebo_ros_factory.so'], output='screen')    
    # arrNodes.append(gazebo_node)

    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
         output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(
            [use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        output='screen')
    
    arrNodes.append(start_gazebo_server_cmd)
    arrNodes.append(start_gazebo_client_cmd)
    
    for robot in robots:
        namespace = "/" + robot['name'] if robot['name'] != "" else ""

        robot_description_config = xacro.process_file(
            def_urdf,
            mappings={
                "robot_name": robot['name'],
                "namespace": namespace,
                "robot_material_name": robot['color_name'],
                "robot_material_color_rgb": robot['color_rgb'],
            },
        )
            
        params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}

    
        start_robot_state_publisher_cmd = Node(
            #condition=IfCondition(use_robot_state_pub),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            #remappings=def_remappings,

            parameters=[params]
        )    
        arrNodes.append(start_robot_state_publisher_cmd)

        # ---
             
        spawn_entity_cmd = Node(package='gazebo_ros', executable='spawn_entity.py',
            namespace=namespace,
            arguments=[
                "-topic", namespace + "/robot_description",
                '-entity', robot['name'],
                '-robot_namespace', robot['name'], #launch.substitutions.LaunchConfiguration('robot_name'),
                '-x', str(robot['x_pos']),
                '-y', str(robot['y_pos']),
                '-z', str(robot['z_pos']),             
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
        arrNodes.append(spawn_entity_cmd)

        pkg_path = os.path.join(get_package_share_directory(def_package_name))
        # robot_localization_node = Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     namespace=namespace,
        #     output='screen',
        #     parameters=[os.path.join(pkg_path, 'config/ekf.yaml')]#, {'use_sim_time': use_sim_time }]
        # )    
        # arrNodes.append(robot_localization_node)

        # bringup_cmd = IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(def_launch_dir, 'bringup_launch.py')),
        #     condition=LaunchConfigurationNotEquals('mode', 'simple'),
        #     launch_arguments={
        #         'namespace': namespace,
        #         'use_namespace': 'True',
        #         'slam': slam,
        #         'map': map_yaml_file,
        #         'keepout_mask': keepout_mask,
        #         'use_sim_time': use_sim_time,
        #         'params_file': params_file,
        #         'default_bt_xml_filename': default_bt_xml_filename,
        #         'autostart': 'True' }.items())
        # bringup_timer_action = launch.actions.TimerAction( period=5.0, actions=[ bringup_cmd ])
        # arrNodes.append(bringup_timer_action)

        params = { 'use_sim_time': use_sim_time}
        joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[params],
            #remappings=def_remappings,
            #condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
        )
        arrNodes.append(joint_state_publisher_node)

        # Lidar odom node
        # lidar_odom_node = Node(
        #     package='lidar_odom',
        #     executable='lidar_odom',
        #     name='lidar_odom',
        #     output='screen',
        #     parameters=[params],
        # )
        # arrNodes.append(lidar_odom_node)

        # Publish world to odom, used when no relative localization is given
        node_tf = launch_ros.actions.Node( 
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[str(robot['x_pos']), str(robot['y_pos']), str(robot['z_pos']), '0', '0', '0', 
                'world', namespace + '/odom'],
            output='screen')

        # arrNodes.append(node_tf)         

    # ---

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',  # Works fine
        ],
        output='screen'
    )

    arrNodes.append(bridge)

    rviz_node = Node(package    ='rviz2',
                     executable ='rviz2',
                     name       ='rviz2',
                     output     ='log',
                     arguments  =['-d', rviz_config_file]) 
    arrNodes.append(rviz_node)

    gazebo_exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_gazebo_client_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='gazebo exited'))))
    arrNodes.append(gazebo_exit_event_handler)

    rviz_exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))
    arrNodes.append(rviz_exit_event_handler)

    # Create the launch description and populate
    ld = LaunchDescription()

    for node in arrNodes:
        ld.add_action(node)

    return ld


