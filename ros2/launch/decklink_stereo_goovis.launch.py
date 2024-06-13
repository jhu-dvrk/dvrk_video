from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    stereo_rig_name_configuration = LaunchConfiguration('stereo_rig_name')
    stereo_rig_name_argument = DeclareLaunchArgument('stereo_rig_name',
                                                     description = 'stereo right_camera_name')

    left_device_configuration = LaunchConfiguration('left_device')
    left_device_argument = DeclareLaunchArgument(
        'left_device',
        description = 'decklink device to use for left video',
        default_value = '1')

    right_device_configuration = LaunchConfiguration('right_device')
    right_device_argument = DeclareLaunchArgument(
        'right_device',
        description = 'decklink device to use for right video',
        default_value = '0')

    images_per_second_configuration = LaunchConfiguration('images_per_second')
    images_per_second_argument = DeclareLaunchArgument(
        'images_per_second',
        description = 'add rate filter to the gstreamer pipeline',
        default_value = '0')

    stereo_gscam_launch =  [
        PathJoinSubstitution([FindPackageShare('dvrk_video'), 'launch', '']),
        'gscam_decklink_stereo.launch.py',
    ]

    stereo_gscam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stereo_gscam_launch),
        launch_arguments =
        {'stereo_rig_name': stereo_rig_name_configuration,
         'left_device': left_device_configuration,
         'right_device': right_device_configuration,
         'images_per_second': images_per_second_configuration,
         'deinterlace': 'True',
         'glimagesink': 'True',
        }.items())

    return LaunchDescription([
        stereo_rig_name_argument,
        left_device_argument,
        right_device_argument,
        images_per_second_argument,
        stereo_gscam_node,
    ])
