# save as: pi_camera.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='libcamera_ros',
            executable='libcamera_ros',
            name='pi_camera',
            output='screen',
            parameters=[{
                'camera_name': 'pi_camera',
                'image_size': [640, 480],   # adjust resolution
                'pixel_format': 'YUV420',   # or 'RGB888'
                'frame_rate': 30,
            }],
            remappings=[
                ('/image', '/camera/image_raw'),
                ('/camera_info', '/camera/camera_info'),
            ]
        )
    ])
