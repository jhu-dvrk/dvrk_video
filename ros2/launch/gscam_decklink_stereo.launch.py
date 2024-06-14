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

    crop_top_configuration = LaunchConfiguration('crop_top')
    crop_top_argument = DeclareLaunchArgument(
        'crop_top',
        description = 'pixels to crop from top of image',
        default_value = '0')

    crop_bottom_configuration = LaunchConfiguration('crop_bottom')
    crop_bottom_argument = DeclareLaunchArgument(
        'crop_bottom',
        description = 'pixels to crop from bottom of image',
        default_value = '0')

    crop_left_configuration = LaunchConfiguration('crop_left')
    crop_left_argument = DeclareLaunchArgument(
        'crop_left',
        description = 'pixels to crop from left of image',
        default_value = '0')

    crop_right_configuration = LaunchConfiguration('crop_right')
    crop_right_argument = DeclareLaunchArgument(
        'crop_right',
        description = 'pixels to crop from right of image',
        default_value = '0')

    deinterlace_configuration = LaunchConfiguration('deinterlace')
    deinterlace_argument = DeclareLaunchArgument(
        'deinterlace',
        description = 'add deinterlace filter to the gstreamer pipeline',
        default_value = 'True',
        choices = ['True', 'False', '0', '1'])

    glimagesink_configuration = LaunchConfiguration('glimagesink')
    glimagesink_argument = DeclareLaunchArgument(
        'glimagesink',
        description = 'add glimagesink to the gstreamer pipeline',
        default_value = 'False',
        choices = ['True', 'False', '0', '1'])

    gscam_launch =  [
        PathJoinSubstitution([FindPackageShare('dvrk_video'), 'launch', '']),
        'gscam_decklink.launch.py',
    ]

    left_gscam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gscam_launch),
        launch_arguments =
        {'camera_name': PythonExpression(['"', stereo_rig_name_configuration, '" + "/left"']),
         'device': left_device_configuration,
         'images_per_second': images_per_second_configuration,
         'crop_top': crop_top_configuration,
         'crop_bottom': crop_bottom_configuration,
         'crop_left': crop_left_configuration,
         'crop_right': crop_right_configuration,
         'deinterlace': deinterlace_configuration,
         'glimagesink': glimagesink_configuration,
        }.items())

    right_gscam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gscam_launch),
        launch_arguments =
        {'camera_name': PythonExpression(['"', stereo_rig_name_configuration, '" + "/right"']),
         'device': right_device_configuration,
         'images_per_second': images_per_second_configuration,
         'crop_top': crop_top_configuration,
         'crop_bottom': crop_bottom_configuration,
         'crop_left': crop_left_configuration,
         'crop_right': crop_right_configuration,
         'deinterlace': deinterlace_configuration,
         'glimagesink': glimagesink_configuration,
        }.items())

    return LaunchDescription([
        stereo_rig_name_argument,
        left_device_argument,
        right_device_argument,
        images_per_second_argument,
        crop_top_argument,
        crop_bottom_argument,
        crop_left_argument,
        crop_right_argument,
        deinterlace_argument,
        glimagesink_argument,
        left_gscam_node,
        right_gscam_node
    ])
