from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    args = [
        DeclareLaunchArgument('serial_no', default_value=''),
        DeclareLaunchArgument('usb_port_id', default_value=''),
        DeclareLaunchArgument('device_type', default_value=''),
        DeclareLaunchArgument('json_file_path', default_value=''),
        DeclareLaunchArgument('camera', default_value='camera'),
        DeclareLaunchArgument('tf_prefix', default_value=LaunchConfiguration('camera')),
        DeclareLaunchArgument('external_manager', default_value='false'),
        DeclareLaunchArgument('manager', default_value='realsense2_camera_manager'),
        DeclareLaunchArgument('output', default_value='screen'),
        DeclareLaunchArgument('respawn', default_value='false'),

        DeclareLaunchArgument('fisheye_width', default_value='-1'),
        DeclareLaunchArgument('fisheye_height', default_value='-1'),
        DeclareLaunchArgument('enable_fisheye', default_value='false'),

        DeclareLaunchArgument('depth_width', default_value='-1'),
        DeclareLaunchArgument('depth_height', default_value='-1'),
        DeclareLaunchArgument('enable_depth', default_value='true'),

        DeclareLaunchArgument('confidence_width', default_value='-1'),
        DeclareLaunchArgument('confidence_height', default_value='-1'),
        DeclareLaunchArgument('enable_confidence', default_value='true'),
        DeclareLaunchArgument('confidence_fps', default_value='-1'),

        DeclareLaunchArgument('infra_width', default_value='848'),
        DeclareLaunchArgument('infra_height', default_value='480'),
        DeclareLaunchArgument('enable_infra', default_value='false'),
        DeclareLaunchArgument('enable_infra1', default_value='false'),
        DeclareLaunchArgument('enable_infra2', default_value='false'),
        DeclareLaunchArgument('infra_rgb', default_value='false'),

        DeclareLaunchArgument('color_width', default_value='-1'),
        DeclareLaunchArgument('color_height', default_value='-1'),
        DeclareLaunchArgument('enable_color', default_value='true'),

        DeclareLaunchArgument('fisheye_fps', default_value='-1'),
        DeclareLaunchArgument('depth_fps', default_value='-1'),
        DeclareLaunchArgument('infra_fps', default_value='30'),
        DeclareLaunchArgument('color_fps', default_value='-1'),
        DeclareLaunchArgument('gyro_fps', default_value='-1'),
        DeclareLaunchArgument('accel_fps', default_value='-1'),
        DeclareLaunchArgument('enable_gyro', default_value='false'),
        DeclareLaunchArgument('enable_accel', default_value='false'),

        DeclareLaunchArgument('enable_pointcloud', default_value='true'),
        DeclareLaunchArgument('pointcloud_texture_stream', default_value='RS2_STREAM_COLOR'),
        DeclareLaunchArgument('pointcloud_texture_index', default_value='0'),
        DeclareLaunchArgument('allow_no_texture_points', default_value='false'),
        DeclareLaunchArgument('ordered_pc', default_value='false'),

        DeclareLaunchArgument('enable_sync', default_value='false'),
        DeclareLaunchArgument('align_depth', default_value='false'),

        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('tf_publish_rate', default_value='0'),

        DeclareLaunchArgument('filters', default_value='pointcloud'),
        DeclareLaunchArgument('clip_distance', default_value='-2'),
        DeclareLaunchArgument('linear_accel_cov', default_value='0.01'),
        DeclareLaunchArgument('initial_reset', default_value='true'),
        DeclareLaunchArgument('reconnect_timeout', default_value='6.0'),
        DeclareLaunchArgument('wait_for_device_timeout', default_value='-1.0'),
        DeclareLaunchArgument('unite_imu_method', default_value=''),
        DeclareLaunchArgument('topic_odom_in', default_value='odom_in'),
        DeclareLaunchArgument('calib_odom_file', default_value=''),
        DeclareLaunchArgument('publish_odom_tf', default_value='false'),
    ]

    # Realsense node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace=LaunchConfiguration('camera'),
        parameters=[
            {'serial_no': LaunchConfiguration('serial_no')},
            {'usb_port_id': LaunchConfiguration('usb_port_id')},
            {'device_type': LaunchConfiguration('device_type')},
            {'json_file_path': LaunchConfiguration('json_file_path')},
            {'tf_prefix': LaunchConfiguration('tf_prefix')},
            {'enable_pointcloud': LaunchConfiguration('enable_pointcloud')},
            {'pointcloud_texture_stream': LaunchConfiguration('pointcloud_texture_stream')},
            {'pointcloud_texture_index': LaunchConfiguration('pointcloud_texture_index')},
            {'enable_sync': LaunchConfiguration('enable_sync')},
            {'align_depth': LaunchConfiguration('align_depth')},
            {'enable_depth': LaunchConfiguration('enable_depth')},
            {'enable_confidence': LaunchConfiguration('enable_confidence')},
            {'enable_color': LaunchConfiguration('enable_color')},
            {'enable_infra': LaunchConfiguration('enable_infra')},
            {'enable_infra1': LaunchConfiguration('enable_infra1')},
            {'enable_infra2': LaunchConfiguration('enable_infra2')},
            {'enable_gyro': LaunchConfiguration('enable_gyro')},
            {'enable_accel': LaunchConfiguration('enable_accel')},
            {'filters': LaunchConfiguration('filters')},
            {'clip_distance': LaunchConfiguration('clip_distance')},
        ],
        remappings=[
            ('/camera/depth/image_rect_raw', '/image'),
            ('/camera/depth/camera_info', '/camera_info'),
        ],
        output=LaunchConfiguration('output')
    )

    # Depth Image to Laser Scan Node
    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        parameters=[
            {'scan_height': 1},
            {'scan_time': 0.033},
            {'range_min': 0.2},
            {'range_max': 10.0},
            {'output_frame_id': 'realsense'},
        ],
        remappings=[
            ('image', '/camera/depth/image_rect_raw'),
            ('camera_info', '/camera/depth/camera_info'),
            ('scan', '/scan/realsense'),
        ]
    )

    # Static Transform Nodes
    static_transform_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_realsense',
        arguments=['0.2', '0', '0.7', '0', '0', '0', '/base_link', '/realsense']
    )

    static_transform_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_link',
        arguments=['0.2', '0', '0.7', '0', '0', '0', '/base_link', '/camera_link']
    )

    return LaunchDescription(args + [realsense_node, depth_to_scan_node, static_transform_1, static_transform_2])
