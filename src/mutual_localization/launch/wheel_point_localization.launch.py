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

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('mutual_localization')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    robot_num = LaunchConfiguration('robot_num')

    # Declare the launch arguments
    declare_robot_num_cmd = DeclareLaunchArgument(
        'robot_num',
        default_value='3',
        description='Num of simulated robots')

  

    tf_glue_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'tf_glue.launch.py')),
        launch_arguments={'robot_num': robot_num}.items())

    wheel_rel_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'wheel_rel.launch.py')),
        launch_arguments={'robot_num': robot_num}.items())

    pointlk_reg_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'pointlk_reg.launch.py')),
        launch_arguments={'robot_num': robot_num}.items())

    pointlk_reg_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'pointlk_reg.launch.py')),
        launch_arguments={'robot_num': robot_num}.items()) 

    # Create the launch description and populate
    ld = LaunchDescription()


    # Declare the launch options
    ld.add_action(declare_robot_num_cmd)
    ld.add_action(tf_glue_cmd)
    ld.add_action(wheel_rel_cmd)
    ld.add_action(pointlk_reg_cmd)

    return ld
