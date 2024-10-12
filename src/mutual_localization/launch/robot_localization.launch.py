from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction
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

def robot_localization_gen_func(context: LaunchContext, robot_num_arg: LaunchConfiguration):
    node_list = []
    robot_num = context.perform_substitution(robot_num_arg)
    robot_num = int(robot_num)
    for i in range(robot_num - 1):
        for j in range(i+1, robot_num):
            node =  robot_localization_node_gen(i, j)
            node_list.append(node)
    return node_list


def robot_localization_node_gen(i, j):
    print(f"localization: i = {i}, j= {j}")
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
    

def generate_launch_description():
    robot_num = LaunchConfiguration('robot_num')
    declare_robot_num_cmd = DeclareLaunchArgument(
        'robot_num',
        default_value='3',
        description='Num of simulated robots')
   

    ld = LaunchDescription()
    ld.add_action(declare_robot_num_cmd)
    robot_localization_nodes = OpaqueFunction(function=robot_localization_gen_func, args=[robot_num])
    ld.add_action(robot_localization_nodes)

    return ld


