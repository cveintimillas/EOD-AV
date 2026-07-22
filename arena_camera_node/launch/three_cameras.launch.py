from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='arena_camera_node',
            executable='start',
            name='arena_cam_enp6s0',
            parameters=[{
                'qos_reliability': 'reliable',
                'serial': '260500567',
                'frame_id': 'camera_enp6s0',
                'topic': '/arena_camera_node/enp6s0/image',
                'width': 2048,
                'height': 1536,
                'pixelformat': 'rgb8',
                'gain': 13.0,
                'exposure_time': 28000.0,
                'trigger_mode': False,
                'ptp_enable': True
            }]
        ),

        Node(
            package='arena_camera_node',
            executable='start',
            name='arena_cam_enp7s0',
            parameters=[{
                'qos_reliability': 'reliable',
                'serial': '261203985',
                'frame_id': 'camera_enp7s0',
                'topic': '/arena_camera_node/enp7s0/image',
                'width': 2048,
                'height': 1536,
                'pixelformat': 'rgb8',
                'gain': 13.0,
                'exposure_time': 28000.0,
                'trigger_mode': False,
                'ptp_enable': True
            }]
        ),

        Node(
            package='arena_camera_node',
            executable='start',
            name='arena_cam_enp8s0',
            parameters=[{
                'qos_reliability': 'reliable',
                'serial': '260500585',
                'frame_id': 'camera_enp8s0',
                'topic': '/arena_camera_node/enp8s0/image',
                'width': 2048,
                'height': 1536,
                'pixelformat': 'rgb8',
                'gain': 13.0,
                'exposure_time': 28000.0,
                'trigger_mode': False,
                'ptp_enable': True
            }]
        ),
    ])
