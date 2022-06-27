

class Collision_avoidance:
    # -> Setup robot collision avoidance spec
    collision_cone_angle = 80 # deg, full cone (/2 for each side)
    collision_treshold = 0.5
    collision_delay_length = 1.8

    def __init__(self):
        # -> Setup state tracker
        self.collision_delay = 0.

        # ----------------------------------- Collision timer
        self.timer_period = .1  # seconds

        self.collision_timer = self.create_timer(
            self.timer_period, 
            self.collision_timer_callback
            )

    # ================================================= Callbacks definition
    def collision_timer_callback(self):
        '''
        Used to increment collision timer if running
        '''
        if self.collision_delay > 0:
            self.collision_delay -= self.timer_period

    # ================================================= Collision avoidance definition    
    def determine_collision_avoidance_instruction(self):
        '''
        Used to determine the collision avoidance instruction based on the state
        of the collision avoidance process
        '''
        # -> If on direct collision course
        if self.on_collision_course:
            self.collision_delay = self.collision_delay_length

            # -> Perform avoidance maneuvre
            self.target_angular_velocity = self.BURGER_MAX_ANG_VEL * 2/3
            self.target_linear_velocity = 0.

        # -> If collision timer is still running
        elif self.collision_delay > 0:
            self.target_angular_velocity = 0.
            self.target_linear_velocity = self.BURGER_MAX_LIN_VEL

        else:
            print("!!!!! Invalide collision avoidance instruction call !!!!!")
            pass

    @property
    def collision_avoidance_mode(self):
        '''
        Used ot check if collision avoidance mode is enabled
        '''
        # Return true if on collision course or collision timer is still running
        if self.collision_delay > 0 or self.on_collision_course:
            return True
        else:
            return False

    # ================================================= Utils
    @property
    def collision_cone_scan(self):
        '''
        Used to retreive collision cone from lazer scan
        '''
        if self.lazer_scan is not None:
            positive_half = self.lazer_scan[:int(self.collision_cone_angle/2)]
            negative_half = self.lazer_scan[-int(self.collision_cone_angle/2):]

            return positive_half + negative_half

        else:
            return []

    @property
    def on_collision_course(self):
        '''
        Used to determine if an obstacle is detected in the collision cone
        '''
        for i in self.collision_cone_scan:
            if i is None:
                pass

            elif i < self.collision_treshold:
                print("!!!!!!!!!!!!!!!!!! ON COLLISION COURSE !!!!!!!!!!!!!!!!!!")
                return True
            else:
                pass

        return False
