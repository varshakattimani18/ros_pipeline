from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    drone_id = 'rudra'
    return LaunchDescription([
        Node(package='drone_pipeline', executable='camera_node', name='camera_node',
     parameters=[{'drone_id': drone_id}]),
        Node(package='drone_pipeline', executable='detection_node', name='detection_node',
             parameters=[{'drone_id': drone_id}]),
        Node(package='drone_pipeline', executable='geotag_node', name='geotag_node',
             parameters=[{'drone_id': drone_id}]),
        Node(package='drone_pipeline', executable='mission_control_node', name='mission_control_node',
             parameters=[{'drone_id': drone_id}]),
        Node(package='drone_pipeline', executable='fusion_node', name='fusion_node'),
        Node(package='drone_pipeline', executable='payload_control_node', name='payload_control_node'),
        Node(package='drone_pipeline', executable='nav_delivery_node', name='nav_delivery_node'),
        Node(package='drone_pipeline', executable='csv_logger_node', name='csv_logger_node'),
    ])
