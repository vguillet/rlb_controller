
import json
from rlb_utils.msg import TeamComm
from .robot_parameters import *

class Collision_avoidance:
    # -> Setup robot collision avoidance spec
    collision_delay_length = collision_delay_length

    def __init__(self):
        # -> Setup state tracker
        self.collision_delay = 0.
        self.collision_mode_switch = False  # Used to control collision message
        self.collision_direction = None

        # -> Setup cones dict
        self.vision_cones = vision_cones

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
            if self.collision_direction == "l":
                self.target_angular_velocity = -self.BURGER_MAX_ANG_VEL * 2/3
            else:
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
    def collision_cone_scan(self, cone_ref: str) -> list:
        '''
        Used to retreive collision cone from lazer scan
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

            # -> Check collision subcone
            for i, val in enumerate(cone_scan):
                if val is None:
                    pass

                elif val < cone_properties["threshold"]:
                    self.collision_mode_switch = True
                    self.__publish_collision_teamcomm(
                        collision_state=True, 
                        cone_triggered=cone_ref)

                    if i < len(cone_scan)/2:
                        self.collision_direction = "l"
                    else:
                        self.collision_direction = "r"

                    print(f"!!!!!!! {self.robot_id} ON COLLISION COURSE ({self.collision_direction} detected in {cone_ref}) !!!!!!!")

                    return True
                
                else:
                    pass

        if self.collision_mode_switch:
            self.collision_mode_switch = False
            self.__publish_collision_teamcomm(
                collision_state=False, 
                cone_triggered=None)

            self.collision_direction = None


        return False

    def __publish_collision_teamcomm(self, collision_state: bool, cone_triggered: str or None = None):
            # -> Announce end of collision on teams comms
            msg = TeamComm()
            msg.robot_id = self.robot_id
            msg.type = "Collision"
            msg.memo = json.dumps({
                "cone_triggered": cone_triggered
                })

            # -> Pubslish message
            self.team_comms_publisher.publish(msg=msg)
            self.team_comms_publisher.publish(msg=msg)
            self.team_comms_publisher.publish(msg=msg)
            self.team_comms_publisher.publish(msg=msg)
