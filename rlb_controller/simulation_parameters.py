
# ------------- Maps
sim_map_image_path = "/home/vguillet/ros2_ws/src/rlb_coordinator/rlb_coordinator/Caylus_map/Caylus_map.png"
paths_image_path = "/home/vguillet/ros2_ws/src/rlb_coordinator/rlb_coordinator/Caylus_map/Caylus_map_paths.png"
hard_obstacles_image_path = "/home/vguillet/ros2_ws/src/rlb_coordinator/rlb_coordinator/Caylus_map/Caylus_map_hard_obstacles.png"
dense_vegetation_img_path = "/home/vguillet/ros2_ws/src/rlb_coordinator/rlb_coordinator/Caylus_map/Caylus_map_dense_vegetation.png"
light_vegetation_img_path = "/home/vguillet/ros2_ws/src/rlb_coordinator/rlb_coordinator/Caylus_map/Caylus_map_light_vegetation.png"

images_shape = (911, 1000)

ref_1_pixel =(7, 193)
ref_1_latlon = (44.27303, 1.72456)

ref_2_pixel = (871, 908)
ref_2_latlon = (44.276598, 1.730598)

# ------------- Obstacles signal blocking probabilities
hard_obstacles_signal_blocking_prob = 1.
dense_vegetation_signal_blocking_prob = .0
light_vegetation_signal_blocking_prob = .0


# ------------- Turtle emulator
agent_count = 4
agent_start_index = 1
agent_spawn_delay = 0.5

real_time_factor = 1.0
auto_dt = True

lin_vel_scaling_factor = 0.1
ang_vel_scaling_factor = 0.07

# -- Meta
timers_period = 0.000001


'''
ros2 topic pub --once /gazebo/set_model_state gazebo_msgs/ModelState "{model_name: turtlebot3_burger, pose: {position: {x: .9, y: -.5}, orientation: {z: 1}}}"

[rlb_controller-2]     raise handler.exception()
[rlb_controller-2]   File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/task.py", line 239, in __call__
[rlb_controller-2]     self._handler.send(None)
[rlb_controller-2]   File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/executors.py", line 429, in handler
[rlb_controller-2]     await call_coroutine(entity, arg)
[rlb_controller-2]   File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/executors.py", line 343, in _execute_timer
[rlb_controller-2]     await await_or_execute(tmr.callback)
[rlb_controller-2]   File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/executors.py", line 118, in await_or_execute
[rlb_controller-2]     return callback(*args)
[rlb_controller-2]   File "/home/vguillet/ros2_ws/build/rlb_controller/rlb_controller/Turtlebot_control_node.py", line 246, in instruction_publisher_callback
[rlb_controller-2]     self.determine_collision_avoidance_instruction()
[rlb_controller-2]   File "/home/vguillet/ros2_ws/build/rlb_controller/rlb_controller/Collision_avoidance.py", line 70, in determine_collision_avoidance_instruction
[rlb_controller-2]     elif not self.cleared_obstacle or self.collision_delay > 0:
[rlb_controller-2]   File "/home/vguillet/ros2_ws/build/rlb_controller/rlb_controller/Collision_avoidance.py", line 136, in cleared_obstacle
[rlb_controller-2]     self.__publish_collision_teamcomm(
[rlb_controller-2]   File "/home/vguillet/ros2_ws/build/rlb_controller/rlb_controller/Collision_avoidance.py", line 210, in __publish_collision_teamcomm
[rlb_controller-2]     self.team_comms_publisher.publish(msg=msg)
[rlb_controller-2]   File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/publisher.py", line 70, in publish
[rlb_controller-2]     _rclpy.rclpy_publish(capsule, msg)
[rlb_controller-2] _rclpy.RCLError: Failed to publish: cannot publish data, at /tmp/binarydeb/ros-foxy-rmw-fastrtps-shared-cpp-1.3.0/src/rmw_publish.cpp:55, at /tmp/binarydeb/ros-foxy-rcl-1.1.13/src/rcl/publisher.c:319
[ERROR] [rlb_controller-2]: process has died [pid 44587, exit code 1, cmd '/home/vguillet/ros2_ws/install/rlb_controller/lib/rlb_controller/rlb_controller --ros-args -r __ns:=/Turtle_2 --params-file /tmp/launch_params_vq2hnp22'].
'''
