# ------------- RLB parameters
goals_topic = "/goals_backlog"
# goals_topic = "/sim_node_publisher/rlb/targets"

# ------------- Controller parameters
BURGER_MAX_LIN_VEL = 0.22
# BURGER_MAX_LIN_VEL = 0.05
BURGER_MAX_ANG_VEL = 2.84
# BURGER_MAX_ANG_VEL = 0.6

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

lazer_scan_range_min = 0.08
safety_zone = 1.

# ------------- Goto specs
success_distance_range = .1 
success_angle_range = 30.  # deg
dynamic_success_angle_range_factor = 3.16

K_l = 1.
K_a1 = 1.5      # Orientation correction
K_a2 = 4.       # On course correction

# ------------- Collision parameters
vision_cones = {
    "very_short_range_cone": {
        "angle": 190,
        "threshold": 0.145
    },


    "short_range_cone4": {
        "angle": 115,
        "threshold": 0.17
    },

    "short_range_cone3": {
        "angle": 90,
        "threshold": 0.205
    },

    "short_range_cone2": {
        "angle": 67,
        "threshold": 0.255
    },

    "short_range_cone1": {
        "angle": 50,
        "threshold": 0.305
    },

    "short_range_cone0": {
        "angle": 40,
        "threshold": 0.355
    },

    "long_range_cone": {
        "angle": 30,
        "threshold": 0.42
    },

    "ulong_range_cone": {
        "angle": 15,
        "threshold": 0.50
    },

    # "SAFETY_cone": {
    #     "angle": 280,
    #     "threshold": 0.12
    # }
}

side_vision_cones = {
    "medium_range_cone": {
        "angle": 1,
        "threshold": 0.35   
    }
}

collision_delay_length = 1.2

collision_rotation_speed_fraction = 0.5

# ------------- Coordinate collision parameters
collsion_ray_length = .4


# ------------- RLB_viz parameters
lazer_scan_plot_range = 1.5