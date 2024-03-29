from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = []

    robot_count = 3

    for i in range(robot_count):
        nodes.append(
            Node(
                namespace= f"Turtle_{i+1}", 
                package='rlb_controller', 
                executable='rlb_controller', 
                output='screen',
                parameters=[
                {'robot_id': f"Turtle_{i+1}"}
                ]
            )
        )
        
    return LaunchDescription(nodes)
