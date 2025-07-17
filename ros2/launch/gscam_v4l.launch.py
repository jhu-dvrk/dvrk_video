from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_gst_pipeline(device: str,
                        fps: int,
                        crop_top: int,
                        crop_bottom: int,
                        crop_left: int,
                        crop_right: int,
                        deinterlace: bool,
                        glimagesink: bool) -> str:
    '''
    Build the conditional GStreamer string assembly
    device: device to load
    fps: desired image refreshing rate in frame per second
    crop_top: crop distance in pixel from top edge
    crop_bottom: crop distance in pixel from bottom edge
    crop_left: crop distance in pixel from left edge
    crop_right: crop distance in pixel from right edge
    deinterlace: whether enable deinterlace
    glimagesink: whether using opengl
    output: gstreamer string
    '''
    pipe = f"v4l2src device={device} do-timestamp=true"
    if fps > 0:
        pipe += f" ! videorate ! video/x-raw,framerate={fps}/1"
    if any(v > 0 for v in (crop_top, crop_bottom, crop_left, crop_right)):
        pipe += (f" ! videocrop top={crop_top} bottom={crop_bottom} "
                 f"left={crop_left} right={crop_right}")
    if deinterlace:
        pipe += " ! deinterlace"
        
    pipe += " ! videoconvert"
    if glimagesink:
        pipe += " ! tee name=t t. ! queue ! glimagesink force-aspect-ratio=false t. ! queue"
    return pipe


def _launch_gscam_node(context, *, prefix="", suffix=""):
    # Pull evaluated launch args
    cam_name    = LaunchConfiguration('camera_name').perform(context)
    device      = LaunchConfiguration('device').perform(context)
    cam_info    = LaunchConfiguration('camera_info_url').perform(context)
    fps         = int(LaunchConfiguration('images_per_second').perform(context))
    crop_top    = int(LaunchConfiguration('crop_top').perform(context))
    crop_bottom = int(LaunchConfiguration('crop_bottom').perform(context))
    crop_left   = int(LaunchConfiguration('crop_left').perform(context))
    crop_right  = int(LaunchConfiguration('crop_right').perform(context))
    deint       = LaunchConfiguration('deinterlace').perform(context).lower() in ('true','1','yes')
    glsink      = LaunchConfiguration('glimagesink').perform(context).lower() in ('true','1','yes')

    gst = _build_gst_pipeline(device, fps, crop_top, crop_bottom, crop_left, crop_right, deint, glsink)

    frame_id = f"/{cam_name}_frame"

    # remap gscam's default camera/* topics back to the ROS2 topics like /<ns>/image_raw etc
    remaps = [
        ('camera/image_raw',     'image_raw'),
        ('camera/camera_info',   'camera_info'),
        ('camera/set_camera_info','set_camera_info'),
    ]

    node = Node(
        package='gscam',
        executable='gscam_node',
        name=cam_name if not prefix else f"{prefix}{cam_name}{suffix}",
        namespace=cam_name,  # Push into its own namespace so topics become <name>/image_raw
        output='screen',
        parameters=[{
            'camera_name': cam_name,
            'camera_info_url': cam_info,
            'gscam_config': gst,
            'frame_id': frame_id,
            'sync_sink': False,
            'use_gst_timestamps': True,
        }],
        remappings=remaps,
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('device', default_value='/dev/video0'),
        DeclareLaunchArgument('camera_info_url', default_value=''),
        DeclareLaunchArgument('images_per_second', default_value='30'),
        DeclareLaunchArgument('crop_top', default_value='0'),
        DeclareLaunchArgument('crop_bottom', default_value='0'),
        DeclareLaunchArgument('crop_left', default_value='0'),
        DeclareLaunchArgument('crop_right', default_value='0'),
        DeclareLaunchArgument('deinterlace', default_value='True'),
        DeclareLaunchArgument('glimagesink', default_value='False'),
        OpaqueFunction(function=_launch_gscam_node),
    ])