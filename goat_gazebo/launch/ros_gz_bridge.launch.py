from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
                # Velocity command (ROS2 -> gz)
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                # Odometry (GZ -> ROS2)
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                # TF (GZ -> ROS2)
                '/odom/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                # Clock (GZ -> ROS2)
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                # Joint states (GZ -> ROS2)
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                # Lidar (GZ -> ROS2)
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                # IMU (GZ -> ROS2)
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                # Camera (GZ -> ROS2)
                '/camera/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                ],
        remappings=[
            ("/odom/tf", "tf"),
        ],
        output='screen'
    )

    map_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'])

    return LaunchDescription([
        bridge,
        # map_static_tf,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
    ])
