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


def pointlk_reg_node_gen(i, j):
    robot_i = 'robot' + str(i)
    robot_j = 'robot' + str(j)
    scan = 'mid360_PointCloud2'
    odom = 'odom'
    node =  Node(
            package='pointlk_reg',
            namespace='',
            executable='pointlk_reg',
            name='_'.join(['pointlk_reg', str(i), str(j)]),
            arguments=['-robot1', i,
                       '-robot2', j,
                       '-pretrained_path', '/home//glacier-dssl/results/ex1_pointlk_0915_model_best.pth']
        )
    return node


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
    robot_num = 2

    for i in range(robot_num - 1):
        for j in range(i+1, robot_num):
            node_list.append(pointlk_reg_node_gen(i, j))

    # Temp attach the odom to base footprint
    for i in range(robot_num):
        node_list.append(tf_glue(i))
   

    ld = LaunchDescription()

    for node in node_list:
        ld.add_action(node)

    print(robot_num)

    return ld

    
