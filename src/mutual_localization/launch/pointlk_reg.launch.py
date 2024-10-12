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


def pointlk_reg_node_gen(i, j):
    node =  Node(
            package='pointlk_reg',
            namespace='',
            executable='pointlk_reg',
            name='_'.join(['pointlk_reg', str(i), str(j)]),
            parameters=[{'robot1': str(i),
                         'robot2': str(j),
                         'pretrained_path': '/home/glacier-dssl/results/ex1_pointlk_0915_model_best.pth',
                         'use_sim_time': True}]
        )
    return node


def pointlk_reg_gen_func(context: LaunchContext, robot_num_arg: LaunchConfiguration):
    node_list = []
    robot_num = context.perform_substitution(robot_num_arg)
    robot_num = int(robot_num)
    for i in range(robot_num - 1):
        for j in range(i+1, robot_num):
            node =  pointlk_reg_node_gen(i, j)
            node_list.append(node)
    return node_list


    

def generate_launch_description():
    robot_num = LaunchConfiguration('robot_num')
    declare_robot_num_cmd = DeclareLaunchArgument(
        'robot_num',
        default_value='3',
        description='Num of simulated robots')
   

    ld = LaunchDescription()
    ld.add_action(declare_robot_num_cmd)
    pointlk_reg_nodes = OpaqueFunction(function=pointlk_reg_gen_func, args=[robot_num])
    ld.add_action(pointlk_reg_nodes)

    return ld


