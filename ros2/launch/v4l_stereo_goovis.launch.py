from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node



def generate_launch_description():
    stereo_rig_name = DeclareLaunchArgument('stereo_rig_name', default_value='stereo')
    # Example: full resolution; you can override on CLI if needed.
    images_per_sec  = DeclareLaunchArgument('images_per_second', default_value='0')
    stereo_proc     = DeclareLaunchArgument('stereo_proc', default_value='False')
    left_device     = DeclareLaunchArgument('left_device', default_value='/dev/video0')
    right_device    = DeclareLaunchArgument('right_device', default_value='/dev/video2')

    stereo_launch = PathJoinSubstitution([FindPackageShare('dvrk_video'), 'launch', 'gscam_v4l_stereo.launch.py'])

    return LaunchDescription([
        stereo_rig_name, images_per_sec, stereo_proc, left_device, right_device,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(stereo_launch), launch_arguments={
            'stereo_rig_name': LaunchConfiguration('stereo_rig_name'),
            'left_device':     LaunchConfiguration('left_device'),
            'right_device':    LaunchConfiguration('right_device'),
            'images_per_second': LaunchConfiguration('images_per_second'),
            'stereo_proc':     LaunchConfiguration('stereo_proc'),
            'crop_top':        '0',
            'crop_bottom':     '0',
            'crop_left':       '0',
            'crop_right':      '0',
            'deinterlace':     'True',
            'glimagesink':     'True',
        }.items()),
    ])