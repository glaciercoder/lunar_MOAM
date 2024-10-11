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


def tf_glue_gen_func(context: LaunchContext, robot_num_arg: LaunchConfiguration):
    node_list = []
    robot_num = context.perform_substitution(robot_num_arg)
    robot_num = int(robot_num)
    for i in range(robot_num):
        node =  tf_glue_node_gen(i)
        node_list.append(node)
    return node_list



def tf_glue_node_gen(i):
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
    robot_num = LaunchConfiguration('robot_num')
    declare_robot_num_cmd = DeclareLaunchArgument(
        'robot_num',
        default_value='3',
        description='Num of simulated robots')
   

    ld = LaunchDescription()
    ld.add_action(declare_robot_num_cmd)
    tf_glue_node_nodes = OpaqueFunction(function=tf_glue_gen_func, args=[robot_num])
    ld.add_action(tf_glue_node_nodes)

    return ld


