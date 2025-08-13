import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node




def generate_launch_description():

    goat_nav2_dir = get_package_share_directory('goat_nav2')

    gazebo_sim_resource_path = SetEnvironmentVariable(
            name = 'GZ_SIM_RESOURCE_PATH',
            value=':'.join([
            os.path.join('/opt/ros/humble', 'share'),
            os.path.join(get_package_share_directory('goat_gazebo'), 'models'),
            '/usr/share/gz/gz-sim8/worlds',
            os.path.join(get_package_share_directory('turtlebot3'), "models")
        ])
    )


    gazebo_sim_system_plugin_path =  SetEnvironmentVariable(
            name = 'GZ_SIM_PLUGIN_PATH',
            value = '/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins'
    )

    declare_mapper_online_async_param_cmd = DeclareLaunchArgument(
        'async_param',
        default_value=os.path.join(goat_nav2_dir, 'params', 'mapper_params_online_async.yaml'),
        description='Set mappers online async param file')

    mapper_online_async_param_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'),
        ),
        launch_arguments=[('slam_params_file', LaunchConfiguration('async_param'))],
    )


    mapping_node_rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(goat_nav2_dir, "rviz", "goat_mapping_view.rviz")]
    )


    ld = LaunchDescription()


    ld.add_action(gazebo_sim_resource_path)
    ld.add_action(gazebo_sim_system_plugin_path)
 
    ld.add_action(declare_mapper_online_async_param_cmd)
    ld.add_action(mapper_online_async_param_launch)
    ld.add_action(mapping_node_rviz2)
    


    return ld