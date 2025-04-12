from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    ld = LaunchDescription()

    # LD19 LIDAR node
    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD19'},
            {'topic_name': 'scan'},
            {'frame_id': 'base_laser'},
            {'port_name': '/dev/ttyUSB0'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0}
        ]
    )

    # Static transform: base_link → base_laser
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser',
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'base_laser']
    )

    # Static fallback transform: map → base_link
    static_tf_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_link_fallback',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )

    # SLAM Toolbox node (online sync mode)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'slam_toolbox_params_file': '/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_sync.yaml',
            'scan_topic': '/scan',
            'base_frame': 'base_link',
            'odom_frame': 'base_link',
            'map_frame': 'map'
        }],
        output='screen'
    )

    # RViz with preloaded config (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            os.environ['HOME'],
            'ldlidar_ros_ws/install/ldlidar_stl_ros2/share/ldlidar_stl_ros2/rviz/ldlidar.rviz'
        )],
        output='screen'
    )

    # Add all to launch description
    ld.add_action(ldlidar_node)
    ld.add_action(static_tf_laser)
    ld.add_action(static_tf_map)
    ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)

    return ld

