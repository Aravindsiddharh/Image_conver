from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # USB Cam Node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{'video_device': '/dev/video0',  # Adjust as needed
                         'frame_rate': 30}]
        ),

        # Image Conversion Node
        Node(
            package='image_conversion',
            executable='image_conversion_node',
            name='image_conversion_node',
            output='screen',
            parameters=[
                {'input_image_topic': '/image_raw'},
                {'output_image_topic': '/converted_image'}
            ]
        )
    ])

