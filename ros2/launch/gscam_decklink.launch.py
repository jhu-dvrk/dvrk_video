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
                ' + (" ! deinterlace" if ', deinterlace_configuration, ' else "")',
                ' + (" ! tee name=t t. ! queue ! glimagesink force-aspect-ratio=false t. ! queue" if ', glimagesink_configuration, ' else "")',
                ' + " ! videoconvert"'
            ]),
            'camera_info_url': camera_info_url_configuration,
            'camera_name': camera_name_configuration,
            'frame_id': PythonExpression(['"/', camera_name_configuration, '" + "_frame"']),
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
        deinterlace_argument,
        glimagesink_argument,
        gscam_node
    ])
