

class Goto:
    # -> Setup goto specs
    success_distance_range = .10 
    success_angle_range = 3.0    # %
    dynamic_success_angle_range = 3.0
    dynamic_success_angle_range_adjustment = 4
    
    def __init__(self):
        '''
        Placeholder __init__
        '''
        pass

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

        # ======================================================================== Solving for angular velocity
        if abs(angle_diff_percent)*100 > self.dynamic_success_angle_range:
            self.dynamic_success_angle_range = self.success_angle_range

            if self.verbose in [1, 2, 3]:
                print("--> Correcting angle")
                
            # -> halt robot
            self.current_linear_velocity = 0.0
            self.target_linear_velocity = 0.0

            # -> Solve for angular velocity instruction magnitude
            self.target_angular_velocity = self.BURGER_MAX_ANG_VEL * (1 - 1/2*(1-angle_diff_percent))

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
            # -> Halt robot rotation
            self.current_angular_velocity = 0.0
            self.target_angular_velocity = 0.0

        # ======================================================================== Solving for linear velocity
        if self.distance_to_goal > self.success_distance_range and self.current_angular_velocity == 0:
            self.dynamic_success_angle_range = self.success_angle_range * self.dynamic_success_angle_range_adjustment

            if self.verbose in [1, 2, 3]:
                print("--> Correction velocity")
                
            # -> Solve for linear velocity instruction
            self.target_linear_velocity = self.BURGER_MAX_LIN_VEL

            self.target_linear_velocity = \
                self.__check_linear_limit_velocity(self.target_linear_velocity + self.LIN_VEL_STEP_SIZE)

        else:
            # -> Halt robot
            self.current_linear_velocity = 0.0
            self.target_linear_velocity = 0.0

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
