import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='goat_hotel_world')

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

    # spawn robot
    # gazebo_sim_urdf_spawner = Node(
    #     package='gazebo_sim_spawner',
    #     executable='urdf_spawner',
    #     output='screen',
    #     parameters=[
    #         {'robot_description_topic': 'robot_description'},
    #         {'world_name': world_name},
    #         {'model_name': 'ant_bot'}
    #     ]
    # )

    # Spawn robot

   


    spawn_hotel_robot = Node(
            package='ros_gz_sim',
            executable='create',
            name='ant_bot',
            output='screen',
            arguments=[
                '-world', world_name,
                '-topic', 'robot_description',
                '-x', '3.64', # '-3.43',
                '-y', '-4.94', # '1.50',
                '-z', '0.01', # '0.05',
                '-name', 'goat_robot'
            ]
    )





    world_only = os.path.join(get_package_share_directory('goat_gazebo'), "worlds", "hotel_world.sdf")



    goat_robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('goat_gazebo'),
                              'launch', 'robot_state_publisher.launch.py')])
    )
    

    gazebo_sim_empty_world = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 3 ' + world_only]),
                              ('gz_version', '8')
                             ]
    )


    ros_gazebo_sim_bridge = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('goat_gazebo'),
                              'launch', 'ros_gz_bridge.launch.py')]),
    )



        



    return LaunchDescription([
        gazebo_sim_resource_path,
        gazebo_sim_system_plugin_path,

        goat_robot_state_publisher,
        gazebo_sim_empty_world,
                  
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),

        DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='World name'),




        ros_gazebo_sim_bridge,
        spawn_hotel_robot,
        # delayed_spawn_ant_bot,
       



    ])
    
