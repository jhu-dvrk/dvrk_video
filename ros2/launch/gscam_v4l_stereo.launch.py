from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    stereo_rig_name = DeclareLaunchArgument('stereo_rig_name', default_value='stereo')
    left_name       = DeclareLaunchArgument('left_name',  default_value='left')
    right_name      = DeclareLaunchArgument('right_name', default_value='right')
    left_device     = DeclareLaunchArgument('left_device',  default_value='/dev/video0')
    right_device    = DeclareLaunchArgument('right_device', default_value='/dev/video2')
    images_per_sec  = DeclareLaunchArgument('images_per_second', default_value='0')
    stereo_proc     = DeclareLaunchArgument('stereo_proc', default_value='False')
    crop_top        = DeclareLaunchArgument('crop_top', default_value='0')
    crop_bottom     = DeclareLaunchArgument('crop_bottom', default_value='0')
    crop_left       = DeclareLaunchArgument('crop_left', default_value='0')
    crop_right      = DeclareLaunchArgument('crop_right', default_value='0')
    deinterlace     = DeclareLaunchArgument('deinterlace', default_value='True')
    glimagesink     = DeclareLaunchArgument('glimagesink', default_value='False')
    left_cam_info   = DeclareLaunchArgument('left_camera_info_url', default_value='')
    right_cam_info  = DeclareLaunchArgument('right_camera_info_url', default_value='')

    # Path to monocular helper (this file) — assumes same package; adjust if different
    mono_launch = PathJoinSubstitution([FindPackageShare('dvrk_video'), 'launch', 'gscam_v4l.launch.py'])

    # define the namespace
    left_include = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mono_launch),
            launch_arguments={
                'camera_name':     LaunchConfiguration('left_name'),
                'device':          LaunchConfiguration('left_device'),
                'camera_info_url': LaunchConfiguration('left_camera_info_url'),
                'images_per_second': LaunchConfiguration('images_per_second'),
                'crop_top':        LaunchConfiguration('crop_top'),
                'crop_bottom':     LaunchConfiguration('crop_bottom'),
                'crop_left':       LaunchConfiguration('crop_left'),
                'crop_right':      LaunchConfiguration('crop_right'),
                'deinterlace':     LaunchConfiguration('deinterlace'),
                'glimagesink':     LaunchConfiguration('glimagesink'),
            }.items(),
        ),
    ], scoped=True,)

    right_include = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mono_launch),
            launch_arguments={
                'camera_name':     LaunchConfiguration('right_name'),
                'device':          LaunchConfiguration('right_device'),
                'camera_info_url': LaunchConfiguration('right_camera_info_url'),
                'images_per_second': LaunchConfiguration('images_per_second'),
                'crop_top':        LaunchConfiguration('crop_top'),
                'crop_bottom':     LaunchConfiguration('crop_bottom'),
                'crop_left':       LaunchConfiguration('crop_left'),
                'crop_right':      LaunchConfiguration('crop_right'),
                'deinterlace':     LaunchConfiguration('deinterlace'),
                'glimagesink':     LaunchConfiguration('glimagesink'),
            }.items(),
        ),
    ], scoped=True,)
    
    left_rect = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='left_rectify',
        namespace=[LaunchConfiguration('stereo_rig_name'), '/', LaunchConfiguration('left_name')],
        remappings=[('image', 'image_raw'), ('image_rect', 'image_rect')],
    )
    right_rect = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='right_rectify',
        namespace=[LaunchConfiguration('stereo_rig_name'), '/', LaunchConfiguration('right_name')],
        remappings=[('image', 'image_raw'), ('image_rect', 'image_rect')],
    )
    # Disparity + point cloud components live in stereo_image_proc pkg
    disparity = ComposableNode(
        package='stereo_image_proc',
        plugin='stereo_image_proc::DisparityNode',
        name='disparity_node',
        namespace=LaunchConfiguration('stereo_rig_name'),
        remappings=[
            ('left/image_rect',  [LaunchConfiguration('left_name'), '/image_rect']),
            ('right/image_rect', [LaunchConfiguration('right_name'), '/image_rect']),
            ('left/camera_info', [LaunchConfiguration('left_name'), '/camera_info']),
            ('right/camera_info',[LaunchConfiguration('right_name'), '/camera_info']),
        ],
    )
    point_cloud = ComposableNode(
        package='stereo_image_proc',
        plugin='stereo_image_proc::PointCloudNode',
        name='point_cloud_node',
        namespace=LaunchConfiguration('stereo_rig_name'),
        remappings=[
            ('left/image_rect_color',  [LaunchConfiguration('left_name'), '/image_rect']),
            ('right/image_rect',       [LaunchConfiguration('right_name'), '/image_rect']),
            ('left/camera_info',       [LaunchConfiguration('left_name'), '/camera_info']),
            ('right/camera_info',      [LaunchConfiguration('right_name'), '/camera_info']),
        ],
    )

    stereo_container = ComposableNodeContainer(
        condition=IfCondition(LaunchConfiguration('stereo_proc')),
        name='stereo_proc_container',
        namespace=LaunchConfiguration('stereo_rig_name'),
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[left_rect, right_rect, disparity, point_cloud],
        output='screen',
    )

    return LaunchDescription([
        stereo_rig_name, left_name, right_name,
        left_device, right_device, images_per_sec, stereo_proc,
        crop_top, crop_bottom, crop_left, crop_right, deinterlace, glimagesink,
        left_cam_info, right_cam_info,
        GroupAction(actions=[PushRosNamespace(LaunchConfiguration('stereo_rig_name')), left_include]),
        GroupAction(actions=[PushRosNamespace(LaunchConfiguration('stereo_rig_name')), right_include]),
        stereo_container,
    ])
