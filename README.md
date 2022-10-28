# Package: RLB Controller
Basic controller implementation. Provided to enable testing task allocation algorithms. The controller retreives goal messages, and performs a goto on all goals received in order of priority/FIFO (goals are not preemptive). The controller also contains some basic sensor-based collision detection and avoidance logic.

All properties of the controller can be adjusted in the **robot_parameters.py** script (every modification however requires a re-launch of the controllers and visualizer nodes to be reflected)

To run:
```
ros2 launch rlb_controller rlb_<# turtles>_launch.py
```

*Note: One controller must be launched per agent/turtle*
