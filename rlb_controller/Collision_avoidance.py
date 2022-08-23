import json
from rlb_utils.msg import TeamComm
from rlb_config.robot_parameters import *

class Collision_avoidance:
    # -> Setup robot collision avoidance spec
    collision_delay_length = collision_delay_length

    def __init__(self):
        # -> Setup state tracker
        self.collision_delay = 0.
        self.collision_state = 0  # Used to control collision message
        self.collision_direction = None

        # -> Setup cones dict
        self.vision_cones = vision_cones
        self.side_vision_cones = side_vision_cones
        
        self.l_side_cone_toggle = False
        self.r_side_cone_toggle = True
        
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
            # -> Reset collision delay
            self.collision_delay = self.collision_delay_length

            # -> Set collision state to 1 (direct collision)
            self.collision_state = 1

            # -> Perform avoidance maneuvre
            if self.collision_direction == "l":
                self.target_angular_velocity = -self.BURGER_MAX_ANG_VEL * collision_rotation_speed_fraction

                # -> Toggle left vision cones
                self.l_side_cone_toggle = True

            else:
                self.target_angular_velocity = self.BURGER_MAX_ANG_VEL * collision_rotation_speed_fraction
                
                # -> Toggle right vision cones
                self.r_side_cone_toggle = True

            self.target_linear_velocity = 0.

        # -> If collision timer is still running
        elif not self.cleared_obstacle or self.collision_delay > 0:
            # -> Set collision state to 2 (avoiding collision)
            self.collision_state = 2

            self.target_angular_velocity = 0.
            self.target_linear_velocity = self.BURGER_MAX_LIN_VEL

        else:
            # -> Set collision state to 0 (no collision collision)
            self.collision_state = 0
            self.collision_direction = None

            self.__publish_collision_teamcomm(
                cone_triggered=None)


    @property
    def collision_avoidance_mode(self):
        '''
        Used ot check if collision avoidance mode is enabled
        '''
        # Return true if on collision course or collision timer is still running
        if self.on_collision_course or self.collision_state in [1, 2]:
            return True
        else:
            return False

    # ================================================= Side scan
    def side_collision_cone_scan(self, cone_ref: str) -> list:
        '''
        Used to retrieve side collision cone from lazer scan
        '''

        cone_angle = self.side_vision_cones[cone_ref]["angle"]

        if self.lazer_scan is not None:
            if self.collision_direction == "l":
                positive_half = self.lazer_scan[:int(cone_angle/2 + 90)]
                negative_half = self.lazer_scan[-int(cone_angle/2 + 90):]
            
            else:
                positive_half = self.lazer_scan[:int(cone_angle/2 + 90)]
                negative_half = self.lazer_scan[-int(cone_angle/2 + 90):]  

            return positive_half + negative_half

        else:
            return []

    @property
    def cleared_obstacle(self):
        '''
        Used to determined if obstacle is detected in side collision cone
        '''
        
        for cone_ref, cone_properties in self.side_vision_cones.items():
            cone_scan = self.side_collision_cone_scan(
                cone_ref=cone_ref
                )

            # -> Check collision cone
            for i, val in enumerate(cone_scan):
                if val is None:
                    pass

                elif val < cone_properties["threshold"]:
                    self.__publish_collision_teamcomm(
                        cone_triggered=cone_ref)

                    # -> Return True as obstacles are detected
                    return False
                
                else:
                    pass

        # -> Return True as no more obstacles are detected
        return True


    # ================================================= Front scan
    def collision_cone_scan(self, cone_ref: str) -> list:
        '''
        Used to retrieve collision cone from lazer scan
        '''

        cone_angle = self.vision_cones[cone_ref]["angle"]

        if self.lazer_scan is not None:
            positive_half = self.lazer_scan[:int(cone_angle/2)]
            negative_half = self.lazer_scan[-int(cone_angle/2):]

            return positive_half + negative_half

        else:
            return []

    @property
    def on_collision_course(self):
        '''
        Used to determine if an obstacle is detected in the collision cone
        '''

        for cone_ref, cone_properties in self.vision_cones.items():
            cone_scan = self.collision_cone_scan(cone_ref=cone_ref)

            # -> Check collision cone
            for i, val in enumerate(cone_scan):
                if val is None:
                    pass

                elif val < cone_properties["threshold"]:
                    self.__publish_collision_teamcomm(
                        cone_triggered=cone_ref)

                    if i < len(cone_scan)/2:
                        self.collision_direction = "l"
                    else:
                        self.collision_direction = "r"

                    print(f"!!!!!!! {self.robot_id} ON COLLISION COURSE ({self.collision_direction} detected in {cone_ref}) !!!!!!!")

                    return True
                
                else:
                    pass

        return False

    def __publish_collision_teamcomm(self, cone_triggered: str or None = None):
            # -> Announce end of collision on teams comms
            msg = TeamComm()
            msg.robot_id = self.robot_id
            msg.type = "Collision"
            msg.memo = json.dumps({
                "cone_triggered": cone_triggered,
                "collision_state": self.collision_state,
                "side": self.collision_direction
                })

            # -> Publish message
            self.team_comms_publisher.publish(msg=msg)
