from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
# from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    camera_name_configuration = LaunchConfiguration('camera_name')
    camera_name_argument = DeclareLaunchArgument('camera_name',
                                                 description = 'camera name',
                                                 default_value = 'decklink')

    camera_info_url_configuration = LaunchConfiguration('camera_info_url',
                                                        default = 'package://gscam/examples/uncalibrated_parameters.ini')

    device_configuration = LaunchConfiguration('device')
    device_argument = DeclareLaunchArgument(
        'device',
        description = 'decklink device to use',
        default_value = '0')

    connection_configuration = LaunchConfiguration('connection')
    connection_argument = DeclareLaunchArgument(
        'connection',
        description = 'deckink connection to use',
        default_value = 'sdi')

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

    gscam_node = Node(
        package = 'gscam',
        executable = 'gscam_node',
        output = 'both',
        namespace = camera_name_configuration,
        parameters = [{
            'gscam_config': PythonExpression([
                '"decklinkvideosrc"',
                ' + " device-number=" + "', device_configuration, '"',
                ' + " connection=" + "', connection_configuration, '"',
                
                ' + (" ! videorate ! video/x-raw,framerate=', images_per_second_configuration, '/1" if ', images_per_second_configuration, ' != 0 else "")',

                ' + (" ! videocrop top=', crop_top_configuration, ' bottom=', crop_bottom_configuration,
                ' left=', crop_left_configuration, ' right=', crop_right_configuration,
                '" if ((', crop_top_configuration, ' > 0) or (', crop_bottom_configuration, ' > 0) or (',
                crop_left_configuration, ' > 0) or (', crop_right_configuration, ' > 0)) else "")',
                
                ' + (" ! deinterlace fields=top" if ', deinterlace_configuration, ' else "")',
                
                ' + (" ! tee name=t t. ! queue ! glimagesink force-aspect-ratio=false t. ! queue" if ', glimagesink_configuration, ' else "")',
                
                ' + " ! videoconvert"'
            ]),
            'camera_info_url': camera_info_url_configuration,
            'camera_name': camera_name_configuration,
            'frame_id': PythonExpression(['"/', camera_name_configuration, '" + "_frame"']),
            'synk_sink': 'false',
        }],
        remappings=[
            ('camera/camera_info', 'camera_info'),
            ('camera/image_raw', 'image_raw'),
        ]
        )

    return LaunchDescription([
        camera_name_argument,
        device_argument,
        connection_argument,
        images_per_second_argument,
        crop_top_argument,
        crop_bottom_argument,
        crop_left_argument,
        crop_right_argument,
        deinterlace_argument,
        glimagesink_argument,
        gscam_node
    ])
