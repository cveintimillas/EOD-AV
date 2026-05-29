from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='arena_camera_node',
            executable='start',
            name='arena_cam_enp3s0',
            parameters=[{
                'qos_reliability': 'reliable',
                'serial': '260500567',
                'topic': '/arena_camera_node/enp3s0/image',
                'width': 2048,
                'height': 1536,
                'pixelformat': 'rgb8',
                'gain': 38.0,
                'exposure_time': 28000.0,
                'trigger_mode': False
            }]
        ),

        Node(
            package='arena_camera_node',
            executable='start',
            name='arena_cam_enp4s0',
            parameters=[{
                'qos_reliability': 'reliable',
                'serial': '261203985',
                'topic': '/arena_camera_node/enp4s0/image',
                'width': 2048,
                'height': 1536,
                'pixelformat': 'rgb8',
                'gain': 38.0,
                'exposure_time': 28000.0,
                'trigger_mode': False
            }]
        ),

        Node(
            package='arena_camera_node',
            executable='start',
            name='arena_cam_enp5s0',
            parameters=[{
                'qos_reliability': 'reliable',
                'serial': '260500585',
                'topic': '/arena_camera_node/enp5s0/image',
                'width': 2048,
                'height': 1536,
                'pixelformat': 'rgb8',
                'gain': 18.0,
                'exposure_time': 28000.0,
                'trigger_mode': False
            }]
        ),
    ])
