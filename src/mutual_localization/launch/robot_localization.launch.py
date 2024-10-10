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


def robot_localization_node_gen(i, j):
    print(f"i = {i}, j= {j}")
    robot1 = 'robot' + str(i)
    robot2 = 'robot' + str(j)
    odom_parent = '/'.join([robot1, 'odom'])
    odom_child = '/'.join([robot2, 'odom'])
    wheel_rel_odom_topic = '_'.join(['wheel_rel_odom', str(robot1), str(robot2)])
    pointlk_odom_topic = '_'.join(['pointlk_reg', str(robot1), str(robot2)])
    print(f'odom paraent:{odom_parent}, odom_child:{odom_child}')
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='_'.join(['ekf_filter_node', str(i), str(j)]),
        namespace='',
        output='screen',
        remappings=[
            ('/odometry/filtered', '_'.join(['/fused_odom', str(robot1), str(robot2)]))
        ],
        parameters= [{'publish_tf': False,
                      'odom_frame': odom_parent,
                      'base_link_frame': odom_child,
                      'world_frame': odom_parent,
                      'use_sim_time': True,
                      'odom0': pointlk_odom_topic,
                      'odom0_config': [True,  True,  False,
                       False, False, False,
                       False, False, False,
                       False, False, True,
                       False, False, False] ,
                       'odom0_pose_rejection_threshold': 5.0,
                       'odom1': wheel_rel_odom_topic,
                       'odom1_config': [False,  False,  False,
                                        False, False, False,
                                        True, True, False,
                                        False, False, True,
                                        False, False, False],
                       'odom1_twist_rejection_threshold': 1.0
                    }
                    ],
    )
    return robot_localization_node

def tf_glue(i):
    robot_i = 'robot' + str(i)
    base = 'base_footprint'
    odom = 'odom'
    node_tf = launch_ros.actions.Node( 
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0.0', '0.0', '0.0', '0', '0', '0', 
        '/'.join([robot_i, odom]), '/'.join([robot_i, base]) ],
    output='screen')

    return node_tf
    

def generate_launch_description():
    node_list = []
    robot_num = 3
    for i in range(robot_num - 1):
        for j in range(i+1, robot_num):
            node =  robot_localization_node_gen(i, j)
            node_list.append(node)
   
   # Temp attach the odom to base footprint
    for i in range(robot_num):
        node_list.append(tf_glue(i))

    ld = LaunchDescription()

    for node in node_list:
        ld.add_action(node)

    return ld


