
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

package_name = "goat_nav2"

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    param_file_name = "goat_nav2_params" + '.yaml'
    filtered_param_file_name = "goat_nav2_filtered_params" + '.yaml'


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

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory(package_name),
            'maps',
            'hotel_world.yaml'))
    
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory(package_name),
            'params',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')
    

    map_arg = DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load')

    params_file_arg =  DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load')

    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true')

    nav2_bringup_launch =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                "autostart": "true"}.items(),
    )
    

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(get_package_share_directory(package_name), 'config/ekf.yaml'), {'use_sim_time': use_sim_time}]
    )


    return LaunchDescription([

        gazebo_sim_resource_path,
        gazebo_sim_system_plugin_path,

        map_arg,
        params_file_arg,
        use_sim_time_arg,
        nav2_bringup_launch,

        rviz_node,
        # robot_localization_node,
    ])