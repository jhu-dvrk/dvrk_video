from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
# from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    camera_info_url = LaunchConfiguration('camera_info_url',
                                          default = 'package://gscam/examples/uncalibrated_parameters.ini')

    device_configuration = LaunchConfiguration('device')
    device_argument = DeclareLaunchArgument(
        'device',
        description = 'v4l2 device to use',
        default_value = '/dev/video0')

    deinterlace_configuration = LaunchConfiguration('deinterlace')
    deinterlace_argument = DeclareLaunchArgument(
        'deinterlace',
        description = 'add deinterlace filter to the gstreamer pipeline',
        default_value = 'False',
        choices = ['True', 'False', '1', '0'])

    gscam_node = Node(
        package = 'gscam',
        executable = 'gscam_node',
        output = 'screen',
        parameters = [{
            'gscam_config': PythonExpression([
                '"v4l2src device=" + "', device_configuration, '"',
                ' + (" ! deinterlace" if ', deinterlace_configuration, ' else "")',
                ' + " ! videoconvert"'
            ]),
            'camera_info_url': camera_info_url,
        }],
        )

    return LaunchDescription([
        deinterlace_argument,
        device_argument,
        gscam_node
    ])



    # return LaunchDescription([
    #     launch_argument_device,
    #     ComposableNodeContainer(
    #         name = 'gscam_container',
    #         namespace = '',
    #         package = 'rclcpp_components',
    #         executable = 'component_container',
    #         composable_node_descriptions = [

    #         # GSCam driver
    #         ComposableNode(
    #             package = 'gscam',
    #             plugin = 'gscam::GSCam',
    #             name = 'gscam_node',
    #             parameters = [{
    #                 'gscam_config': gscam_config,
    #                 'camera_info_url': camera_info_url,
    #             }],
    #             extra_arguments = [{
    #                 'use_intra_process_comms': True,
    #             }],
    #         ),

    #         # # Bayer color decoding
    #         # ComposableNode(
    #         #     package = 'image_proc',
    #         #     plugin = 'image_proc::DebayerNode',
    #         #     name = 'debayer_node',
    #         #     namespace = 'camera',
    #         #     extra_arguments = [{
    #         #         'use_intra_process_comms': True,
    #         #     }],
    #         # ),

    #         # # Mono rectification
    #         # ComposableNode(
    #         #     package = 'image_proc',
    #         #     plugin = 'image_proc::RectifyNode',
    #         #     name = 'mono_rectify_node',
    #         #     namespace = 'camera',
    #         #     extra_arguments = [{
    #         #         'use_intra_process_comms': True,
    #         #     }],
    #         #     remappings = [
    #         #         ('image', 'image_mono'),
    #         #         ('image_rect', 'image_rect_mono'),
    #         #     ],
    #         # ),

    #         # # Color rectification
    #         # ComposableNode(
    #         #     package = 'image_proc',
    #         #     plugin = 'image_proc::RectifyNode',
    #         #     name = 'color_rectify_node',
    #         #     namespace = 'camera',
    #         #     extra_arguments = [{
    #         #         'use_intra_process_comms': True,
    #         #     }],
    #         #     remappings = [
    #         #         ('image', 'image_color'),
    #         #         ('image_rect', 'image_rect_color'),
    #         #     ],
    #         # ),
    #     ],
    #     output = 'screen',
    # )])
