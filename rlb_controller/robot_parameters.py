# ------------- Controller parameters
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

# ------------- Goto specs
success_distance_range = .1 
success_angle_range = 30.  # deg
dynamic_success_angle_range_factor = 3.16

K_l = 1.
K_a1 = 1.5   # Orientation correction
K_a2 = 4.  # On course correction

# ------------- Collision parameters
vision_cones = {
    "long_range_cone": {
        "angle": 40,
        "threshold": 0.40
    },
    "short_range_cone": {
        "angle": 100,
        "threshold": 0.23
    },
    "short_range_cone2": {
        "angle": 65,
        "threshold": 0.3
    },
    "very_short_range_cone": {
        "angle": 140,
        "threshold": 0.18
    },
    "SAFETY_cone": {
        "angle": 280,
        "threshold": 0.13
    }
}

side_vision_cones = {
    "medium_range_cone": {
        "angle": 30,
        "threshold": 0.4   
    }
}

collision_delay_length = 1.2

collision_rotation_speed_fraction = 0.5


# ------------- RLB_viz parameters
lazer_scan_plot_range = 1.5