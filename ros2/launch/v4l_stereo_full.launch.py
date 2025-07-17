from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node



# def generate_launch_description():
#     stereo_rig_name = DeclareLaunchArgument('stereo_rig_name', default_value='stereo')
#     # Example: full resolution; you can override on CLI if needed.
#     images_per_sec  = DeclareLaunchArgument('images_per_second', default_value='0')
#     stereo_proc     = DeclareLaunchArgument('stereo_proc', default_value='False')
#     left_device     = DeclareLaunchArgument('left_device', default_value='/dev/video0')
#     right_device    = DeclareLaunchArgument('right_device', default_value='/dev/video2')

#     stereo_launch = PathJoinSubstitution([FindPackageShare('dvrk_video'), 'launch', 'gscam_v4l_stereo.launch.py'])

#     return LaunchDescription([
#         stereo_rig_name, images_per_sec, stereo_proc, left_device, right_device,
#         IncludeLaunchDescription(PythonLaunchDescriptionSource(stereo_launch), launch_arguments={
#             'stereo_rig_name': LaunchConfiguration('stereo_rig_name'),
#             'left_device':     LaunchConfiguration('left_device'),
#             'right_device':    LaunchConfiguration('right_device'),
#             'images_per_second': LaunchConfiguration('images_per_second'),
#             'stereo_proc':     LaunchConfiguration('stereo_proc'),
#             'crop_top':        '0',
#             'crop_bottom':     '0',
#             'crop_left':       '0',
#             'crop_right':      '0',
#             'deinterlace':     'True',
#             'glimagesink':     'False',
#         }.items()),
#     ])


def generate_launch_description():
    #
    # Launch Arguments (pass‑through to the underlying stereo bringup)
    #
    stereo_rig_name = DeclareLaunchArgument('stereo_rig_name', default_value='stereo',
                                            description='Top‑level namespace for the stereo rig.')
    images_per_sec  = DeclareLaunchArgument('images_per_second', default_value='0',
                                            description='FPS throttle; 0=unlimited (device default).')
    stereo_proc     = DeclareLaunchArgument('stereo_proc', default_value='False',
                                            description='Enable downstream stereo processing pipeline (rectify/disparity).')
    left_device     = DeclareLaunchArgument('left_device', default_value='/dev/video0',
                                            description='Left V4L2 device path.')
    right_device    = DeclareLaunchArgument('right_device', default_value='/dev/video2',
                                            description='Right V4L2 device path.')

    left_name  = LaunchConfiguration('left_name')
    right_name = LaunchConfiguration('right_name')

    stereo_launch = PathJoinSubstitution([
        FindPackageShare('dvrk_video'), 'launch', 'gscam_v4l_stereo.launch.py'
    ])

    stereo_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stereo_launch),
        launch_arguments={
            'stereo_rig_name': LaunchConfiguration('stereo_rig_name'),
            'left_device':     LaunchConfiguration('left_device'),
            'right_device':    LaunchConfiguration('right_device'),
            'images_per_second': LaunchConfiguration('images_per_second'),
            'stereo_proc':     LaunchConfiguration('stereo_proc'),
            # Full‑resolution: no cropping, deinterlace on, headless (no glimagesink tee)
            'crop_top':        '0',
            'crop_bottom':     '0',
            'crop_left':       '0',
            'crop_right':      '0',
            'deinterlace':     'True',
            'glimagesink':     'False',
        }.items()
    )

    left_view = Node(
        package='image_view',
        executable='image_view',
        name='left_view',
        namespace=LaunchConfiguration('stereo_rig_name'),
        remappings=[
            ('image', [left_name, TextSubstitution(text='/image_raw')]),
        ],
        output='screen',
    )

    right_view = Node(
        package='image_view',
        executable='image_view',
        name='right_view',
        namespace=LaunchConfiguration('stereo_rig_name'),
        remappings=[
            ('image', [right_name, TextSubstitution(text='/image_raw')]),
        ],
        output='screen',
    )

    #
    # Assemble and return
    #
    return LaunchDescription([
        stereo_rig_name,
        images_per_sec,
        stereo_proc,
        left_device,
        right_device,
        stereo_include,
        left_view,
        right_view,
    ])