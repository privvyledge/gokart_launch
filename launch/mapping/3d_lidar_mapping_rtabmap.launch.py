# Example:
#   $ ros2 launch velodyne_driver velodyne_driver_node-VLP16-launch.py
#   $ ros2 launch velodyne_pointcloud velodyne_transform_node-VLP16-launch.py
#
#   SLAM:
#   $ ros2 launch rtabmap_examples vlp16.launch.py


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    deskewing = LaunchConfiguration('deskewing')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    wait_imu_to_init = LaunchConfiguration('wait_imu_to_init')
    imu_topic = LaunchConfiguration('imu_topic')
    gps_topic = LaunchConfiguration('gps_topic')
    database_path = LaunchConfiguration('database_path')

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
                'use_sim_time', default_value='false',
                description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
                'deskewing', default_value='true',
                description='Enable lidar deskewing'),

        DeclareLaunchArgument(
                'lidar_frame_id', default_value='livox_frame',
                description='Enable lidar deskewing'),

        DeclareLaunchArgument(
                'wait_imu_to_init', default_value='livox_frame',
                description=''),

        DeclareLaunchArgument(
                'imu_topic', default_value='livox_frame',
                description='Used with VIO approaches and for SLAM graph optimization (gravity constraints). '),

        DeclareLaunchArgument(
                'gps_topic', default_value='livox_frame',
                description='GPS async subscription. This is used for SLAM graph optimization and loop closure candidates selection.'),

        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db',
                              description='Where is the map saved/loaded.'),

        # Nodes to launch
        Node(
                package='rtabmap_odom', executable='icp_odometry', output='screen',
                parameters=[{
                    'frame_id': lidar_frame_id,  # 'livox_frame'
                    'odom_frame_id': 'odom',
                    'wait_for_transform': 0.2,
                    'expected_update_rate': 50.0,
                    'deskewing': deskewing,
                    'use_sim_time': use_sim_time,
                }],
                remappings=[
                    ('scan_cloud', '/livox/lidar')
                ],
                arguments=[
                    'Icp/PointToPlane', 'true',
                    'Icp/Iterations', '10',
                    'Icp/VoxelSize', '0.1',
                    'Icp/Epsilon', '0.001',
                    'Icp/PointToPlaneK', '20',
                    'Icp/PointToPlaneRadius', '0',
                    'Icp/MaxTranslation', '2',
                    'Icp/MaxCorrespondenceDistance', '1',
                    'Icp/Strategy', '1',
                    'Icp/OutlierRatio', '0.7',
                    'Icp/CorrespondenceRatio', '0.01',
                    'Odom/ScanKeyFrameThr', '0.4',
                    'OdomF2M/ScanSubtractRadius', '0.1',
                    'OdomF2M/ScanMaxSize', '15000',
                    'OdomF2M/BundleAdjustment', 'false',
                ]),

        Node(
                package='rtabmap_util', executable='point_cloud_assembler', output='screen',
                parameters=[{
                    'max_clouds': 10,
                    'fixed_frame_id': '',
                    'use_sim_time': use_sim_time,
                }],
                remappings=[
                    ('cloud', 'odom_filtered_input_scan')
                ]),

        Node(
                package='rtabmap_slam', executable='rtabmap', output='screen',
                parameters=[{
                    'frame_id': lidar_frame_id,  # 'livox_frame'
                    'subscribe_depth': False,
                    'subscribe_rgb': False,
                    'subscribe_scan_cloud': True,
                    'approx_sync': False,
                    'wait_for_transform': 0.2,
                    'use_sim_time': use_sim_time,
                    'wait_imu_to_int': wait_imu_to_init,
                    'gps_topic': gps_topic,
                    'imu_topic': imu_topic,
                    'database_path': database_path,
                }],
                remappings=[
                    ('scan_cloud', 'assembled_cloud')
                ],
                arguments=[
                    '-d',  # This will delete the previous database (~/.ros/rtabmap.db)
                    'RGBD/ProximityMaxGraphDepth', '0',
                    'RGBD/ProximityPathMaxNeighbors', '1',
                    'RGBD/AngularUpdate', '0.05',
                    'RGBD/LinearUpdate', '0.05',
                    'RGBD/CreateOccupancyGrid', 'false',
                    'Mem/NotLinkedNodesKept', 'false',
                    'Mem/STMSize', '30',
                    'Mem/LaserScanNormalK', '20',
                    'Reg/Strategy', '1',
                    'Icp/VoxelSize', '0.1',
                    'Icp/PointToPlaneK', '20',
                    'Icp/PointToPlaneRadius', '0',
                    'Icp/PointToPlane', 'true',
                    'Icp/Iterations', '10',
                    'Icp/Epsilon', '0.001',
                    'Icp/MaxTranslation', '3',
                    'Icp/MaxCorrespondenceDistance', '1',
                    'Icp/Strategy', '1',
                    'Icp/OutlierRatio', '0.7',
                    'Icp/CorrespondenceRatio', '0.2',
                ]),

        Node(
                package='rtabmap_viz', executable='rtabmap_viz', output='screen',
                parameters=[{
                    'frame_id': lidar_frame_id,  # 'livox_frame'
                    'odom_frame_id': 'odom',
                    'subscribe_odom_info': True,
                    'subscribe_scan_cloud': True,
                    'approx_sync': False,
                    'use_sim_time': use_sim_time,
                }],
                remappings=[
                    ('scan_cloud', 'odom_filtered_input_scan')
                ]),
    ])


