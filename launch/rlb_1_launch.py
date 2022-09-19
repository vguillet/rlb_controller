from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    nodes = []

    communication_range = LaunchConfiguration(
        'communication_range', default='100')

    scenario_path = LaunchConfiguration(
        'scenario_path',
        default='generated_scenarios/grid_graphs_dim=(7, 7)_obsprop=0.15/scenario_0/data.json')

    scenario_id = LaunchConfiguration('scenario_id', default='0')

    # no_com / sheng / ours
    auction_com_cost = LaunchConfiguration('auction_com_cost', default='no_com')

    method = LaunchConfiguration('method', default='SI')

    folder = LaunchConfiguration('folder', default='rlb_tests')

    robot_count = 1

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

        nodes.append(
            Node(
                namespace= f"Turtle_{i+1}", 
                package='rlb_msg_filter', 
                executable='rlb_msg_filter', 
                output='screen',
                parameters=[
                {'robot_id': f"Turtle_{i+1}"}
                ]
            )
        )

        nodes.append(
            Node(
                namespace= f"Turtle_{i+1}", 
                package='rlb_mba_agent', 
                executable='rlb_mba_agent', 
                output='screen',
                parameters=[
                {
                    'robot_id': f"Turtle_{i+1}",
                    "team_members_file_name": "team_spec_inst_caylus_r5_w21.json",
                    "robots_spec_file_name": "robots_spec_inst_5r_20w_01.json",
                    "mission_spec_file_path": "mission_spec_inst_caylus_w21.json",
                    "communication_range": communication_range,
                    "scenario_path": scenario_path,
                    "scenario_id": scenario_id,
                    "auction_com_cost": auction_com_cost,
                    "method": method,
                    "folder": folder,
                    "use_sim_time": True
                }
                ]
            )
        )

    return LaunchDescription(nodes)
