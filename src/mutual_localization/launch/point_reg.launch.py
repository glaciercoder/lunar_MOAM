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


def point_reg_node_gen(i, j):
    robot_i = 'robot' + str(i)
    robot_j = 'robot' + str(j)
    scan = 'mid360_PointCloud2'
    odom = 'odom'
    node =  Node(
            package='lidar_odom',
            namespace='',
            executable='lidar_odom',
            name='_'.join(['pointreg', str(i), str(j)]),
            arguments=['-scan_topic1', '/'.join([robot_i, scan]),
                       '-scan_topic2', '/'.join([robot_j, scan]),
                       '-odom_parent', '/'.join([robot_i, odom]),
                       '-odom_child', '/'.joint([robot_j, odom])]
        )
    return node
    

def generate_launch_description():
    node_list = []

    robot_num = LaunchConfiguration('robot_rnum')
    declare_robot_num_cmd = DeclareLaunchArgument(
        'robot_num',
        default_value='2',
        description='Number of robots to be simulated')
    node_list.append(declare_robot_num_cmd)

    for i in range(robot_num - 1):
        for j in range(i+1, robot_num):
            node_list.append(point_reg_node_gen(i, j))



    ld = LaunchDescription()

    for node in node_list:
        ld.add_action(node)

    return ld

    
