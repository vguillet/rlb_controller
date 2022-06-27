
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = []

    robot_count = 4

    for i in range(robot_count):
        nodes.append(
            Node(
                namespace= f"Turtle_{i+1}", 
                package='Python_nav', 
                executable='turtlebot', 
                output='screen',
                parameters=[
                {'robot_id': f"Turtle_{i+1}"}
                ]
            )
        )

    return LaunchDescription(nodes)