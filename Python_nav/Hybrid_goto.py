from math import cos

class Goto:
    # -> Setup goto specs
    success_distance_range = .1 
    success_angle_range = 10.  # deg
    dynamic_success_angle_range_factor = 4

    K_l = 1.
    K_a1 = 1.5
    K_a2 = 3.

    def __init__(self):
        self.dynamic_success_angle_range = self.success_angle_range 

    # ================================================= Goto definition
    def determine_goto_instruction(self):
        # -> Calculate angle difference (negative diff = clockwise turn required)
        angle_diff = self.goal_angle - self.orientation

        # -> Select shorter side of turn
        if abs(angle_diff) > 180:
            new_angle_diff = 180 - (abs(angle_diff) - 180)
            
            if angle_diff < 0:
                angle_diff = new_angle_diff
            
            else:
                angle_diff = -new_angle_diff

        # -> Get angle difference percentage (of 180 degrees)
        angle_diff_percent = abs(angle_diff/180)

        if self.verbose == 3:
            print("\n------------------------------------------------------")
            print("self.position:", self.position, "self.goal", self.goal)
            print("self.path_vector:", self.path_vector)
            print("self.distance_to_goal:", self.distance_to_goal)
            print("self.orientation:", self.orientation)
            print("goal_angle: ", self.goal_angle)        
            print("angle_diff: ", angle_diff)
            print("angle_diff_precent", angle_diff_percent * 100, "%")
            print("success_angle_range", self.success_angle_range)
            print("------------------------------------------------------")

        # ======================================================================== Solving for linear velocity
        # ------------- Initial orientation
        if abs(angle_diff) > self.dynamic_success_angle_range:
            # -> Reduce dynamic_success_angle_range
            self.dynamic_success_angle_range = self.success_angle_range

            # -> halt robot
            self.current_linear_velocity = 0.0
            self.target_linear_velocity = 0.0

        # ------------- Smooth correction
        elif self.distance_to_goal > self.success_distance_range:   
            # -> Widen dynamic_success_angle_range
            self.dynamic_success_angle_range = self.success_angle_range * self.dynamic_success_angle_range_factor            
            
            # -> Solve for linear velocity instruction
            self.target_linear_velocity = self.BURGER_MAX_LIN_VEL * self.K_l

            # -> Apply linear velcity ceiling
            self.target_linear_velocity = \
                self.__check_linear_limit_velocity(self.target_linear_velocity + self.LIN_VEL_STEP_SIZE)

        else:
            # -> Halt robot
            self.current_linear_velocity = 0.0
            self.target_linear_velocity = 0.0
        
        # ======================================================================== Solving for angular velocity
        if self.distance_to_goal > self.success_distance_range:
            if abs(angle_diff) > self.dynamic_success_angle_range:
                                # -> Solve for angular velocity instruction magnitude
                self.target_angular_velocity = self.BURGER_MAX_ANG_VEL * angle_diff_percent * self.K_a1

            else:
                # -> Solve for angular velocity instruction magnitude
                self.target_angular_velocity = self.BURGER_MAX_ANG_VEL * angle_diff_percent * self.K_a2

            # -> Apply robot performance ceiling for angular velocity
            if angle_diff < 0:
                self.target_angular_velocity = \
                    self.__check_angular_limit_velocity(self.target_angular_velocity - self.ANG_VEL_STEP_SIZE)
            else:
                self.target_angular_velocity = \
                    self.__check_angular_limit_velocity(self.target_angular_velocity + self.ANG_VEL_STEP_SIZE)

            # -> Apply robot performance floor for angular velocity
            if abs(self.target_angular_velocity) < 0.1:
                if angle_diff < 0:
                    self.target_angular_velocity = -0.1
                else:
                    self.target_angular_velocity = 0.1

            if angle_diff < 0:
                self.target_angular_velocity = -self.target_angular_velocity
          
        else:
            # -> Halt robot
            self.current_angular_velocity = 0.0
            self.target_angular_velocity = 0.0

    # ================================================= Utils
    @staticmethod
    def __constrain(input_vel, low_bound, high_bound):
        if input_vel < low_bound:
            input_vel = low_bound
        elif input_vel > high_bound:
            input_vel = high_bound
        else:
            input_vel = input_vel

        return input_vel

    def __check_linear_limit_velocity(self, velocity):
        if self.TURTLEBOT3_MODEL == 'burger':
            return self.__constrain(velocity, -self.BURGER_MAX_LIN_VEL, self.BURGER_MAX_LIN_VEL)
        else:
            return self.__constrain(velocity, -self.WAFFLE_MAX_LIN_VEL, self.WAFFLE_MAX_LIN_VEL)

    def __check_angular_limit_velocity(self, velocity):
        if self.TURTLEBOT3_MODEL == 'burger':
            return self.__constrain(velocity, -self.BURGER_MAX_ANG_VEL, self.BURGER_MAX_ANG_VEL)
        else:
            return self.__constrain(velocity, -self.WAFFLE_MAX_ANG_VEL, self.WAFFLE_MAX_ANG_VEL)
