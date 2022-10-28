
import os
from tabnanny import verbose
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from rlb_utils.msg import Goal, TeamComm, RLBInterrupt

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import math
import json

# from .Sequential_goto import Goto
# from .Smooth_goto import Goto
from .Hybrid_goto import Goto

from .Collision_avoidance import Collision_avoidance
from rlb_config.robot_parameters import *

# ================================================================================= Main


class Minimal_path_sequence(Node, Goto, Collision_avoidance):
    # -> Fetch turtlebot type from environment
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    # -> Setup robot properties
    BURGER_MAX_LIN_VEL = BURGER_MAX_LIN_VEL
    BURGER_MAX_ANG_VEL = BURGER_MAX_ANG_VEL

    WAFFLE_MAX_LIN_VEL = WAFFLE_MAX_LIN_VEL
    WAFFLE_MAX_ANG_VEL = WAFFLE_MAX_ANG_VEL

    LIN_VEL_STEP_SIZE = LIN_VEL_STEP_SIZE
    ANG_VEL_STEP_SIZE = ANG_VEL_STEP_SIZE

    def __init__(self):
        # -> Initialise inherited classes
        Node.__init__(self, 'main_controller')

        # -> Setup classes
        self.verbose = 0

        # -> Setup robot ID
        self.declare_parameter('robot_id', 'Turtle')
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        # -> Setup robot states
        self.kill_switch = False

        self.goal_sequence_backlog = {}
        self.goal_sequence = None
        self.goal_sequence_priority = 0

        self.prev_point = None

        self.current_angular_velocity = 0.
        self.current_linear_velocity = 0.

        self.target_angular_velocity = 0.
        self.target_linear_velocity = 0.

        # -> Create storage variables
        self.position = None
        self.orientation = None
        self.lazer_scan = None

        # ----------------------------------- Instruction publisher
        qos = QoSProfile(depth=10)
        
        self.instruction_publisher = self.create_publisher(
            msg_type=Twist,
            topic=f"/{self.robot_id}/cmd_vel",
            qos_profile=qos
            )

        # -> Setup timer for instruction publisher callback
        timer_period = 0.01  # seconds

        self.instructions_timer = self.create_timer(
            timer_period, 
            self.instruction_publisher_callback
            )

        # ----------------------------------- Team communications publisher
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL,
            )

        # self.team_comms_publisher = self.create_publisher(
        #     msg_type=TeamComm,
        #     topic="/team_comms",
        #     qos_profile=qos
        #     )

        # ----------------------------------- Team communications subscriber
        # qos = QoSProfile(
        #     reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        #     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL,
        #     )

        # self.team_comms_subscriber = self.create_subscription(
        #     msg_type=TeamComm,
        #     topic="/team_comms",
        #     callback=self.team_msg_subscriber_callback,
        #     qos_profile=qos
        #     )

        # ----------------------------------- rlb_msgs publisher
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL,
            )

        self.rlb_msgs_publisher = self.create_publisher(
            msg_type=TeamComm,
            topic="/rlb_msgs",
            qos_profile=qos
            )

        # -> Push initialization msg
        msg = TeamComm()
        msg.source = self.robot_id
        msg.source_type = "robot"
        msg.type = "Initial"

        self.rlb_msgs_publisher.publish(msg=msg)

        # ----------------------------------- Interrupts subscriber
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL,
            )

        self.team_comms_subscriber = self.create_subscription(
            msg_type=RLBInterrupt,
            topic=f"/{self.robot_id}/control/interrupt",
            callback=self.interrupt_callback,
            qos_profile=qos
            )

        # ----------------------------------- Goal subscription
        qos = QoSProfile(depth=10)

        self.goal_subscription = self.create_subscription(
            msg_type=Goal,
            topic=goals_topic,
            callback=self.goal_subscriber_callback,
            qos_profile=qos
            )

        # ----------------------------------- Odom subscription
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
            )

        self.odom_subscription = self.create_subscription(
            msg_type=PoseStamped,
            topic=f"/{self.robot_id}/pose",
            callback=self.odom_subscriber_callback,
            qos_profile=qos
            )

        # ----------------------------------- Projected odom publisher
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
            )

        self.projected_odom_publisher = self.create_publisher(
            msg_type=PoseStamped,
            topic=f"/{self.robot_id}/pose_projected",
            qos_profile=qos
            )


        # ----------------------------------- Laser scan subscription
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
            )

        self.lazer_scan_subscription = self.create_subscription(
            msg_type=LaserScan,
            topic=f"/{self.robot_id}/scan",
            callback=self.lazer_scan_subscriber_callback,
            qos_profile=qos
        )

        # ----------------------------------- Status printer
        timer_period = 1.  # seconds

        self.status_timer = self.create_timer(
            timer_period, 
            self.state_callback
            )

        Goto.__init__(self)
        Collision_avoidance.__init__(self)

    # ================================================= Callbacks definition
    def state_callback(self):
        if self.verbose > 0:
            print("\n")
            print(f"--> position: {self.position}")
            print(f"--> orientation: {self.orientation}")
        
        if self.goal_sequence is not None:
            if self.verbose > 0:
                print(f"--> Goal: {self.goal}")
                print(f"--> distance_to_goal: {round(self.distance_to_goal, 3)}")

            angle_diff = self.goal_angle - self.orientation

            if abs(angle_diff) > 180:
                new_angle_diff = 180 - (abs(angle_diff) - 180)
                
                if angle_diff < 0:
                    angle_diff = new_angle_diff
                
                else:
                    angle_diff = -new_angle_diff

            angle_diff_percent = abs(angle_diff/180)

            if self.verbose > 0:
                print(f"--> Angle difference: {round(angle_diff)} degrees ({round(angle_diff_percent*100, 2)}%)")
                print(f"--> Success angle range: {self.success_angle_range}")

    def interrupt_callback(self, msg):
        self.get_logger().warn(
            "!!!!!!!!!!!! Interrupt received !!!!!!!!!!!! "
            f"\n - Interrupt type: {msg.type}"
            f"\n - Memo: {msg.memo}"
        )

        if msg.type == "KILL":
            self.kill_switch = True
            self.stop_robot()

        elif msg.type == "CLEAR_GOAL":
            self.goal_sequence = None
            self.stop_robot()

        elif msg.type == "CLEAR_BACKLOG":
            self.goal_sequence_backlog = {}
        
        elif msg.type == "STOP_GOAL_AND_CLEAR_BACKLOG":
            self.goal_sequence_backlog = {}
            self.goal_sequence = None
            self.stop_robot()

        elif msg.type == "RESET":
            self.kill_switch = False

    # ---------------------------------- Publishers
    def instruction_publisher_callback(self):
        if self.position is None or self.orientation is None:
            self.get_logger().warn(f"!!!!!!!!!!!!!!!!!!!!!!!!! {self.robot_id} missing sensor data !!!!!!!!!!!!!!!!!!!!!!!!!")
            self.stop_robot()

            if self.collision_avoidance_mode:
                self.determine_collision_avoidance_instruction()

            return

        elif self.kill_switch:
            self.get_logger().warn("Kill switch engaged, reset using a RESET interrupt to resume")
            return

        elif self.goal_sequence is None:
            # -> Get new goal sequence
            self.get_new_goal_sequence()

            if self.goal_sequence is not None:
                if self.verbose > 0:
                    print("=================================================================")
                    print(f"     New goal sequence selected: {self.goal_sequence}")
                    print(f"     New goal sequence priority: {self.goal_sequence_priority}")
                    print(f"     First goal: {self.goal}    (distance: {round(self.distance_to_goal, 3)})")
                    print("=================================================================")
                
            else:
                self.stop_robot()
                
            return

        # -> Check goal sequence state
        if self.check_goal_sequence():
            self.goal_sequence = None
            if self.verbose > 0:
                print("++++++++++++++++++++++++++++++++++++ Goal Completed ++++++++++++++++++++++++++++++++++++")
            return

        # -> Check subgoal state, remove subgoal reached
        elif self.check_subgoal_state():
            return
        
        else:
            # -> Apply collision avoidance protocol if collision avoidance mode is enabled
            if self.collision_avoidance_mode:
                self.determine_collision_avoidance_instruction()

            # -> Apply goto protocol
            else:
                self.determine_goto_instruction()

            # -> Create instruction
            self.current_linear_velocity = self.__make_simple_profile(
                output=self.current_linear_velocity,
                input=self.target_linear_velocity,
                slop=self.LIN_VEL_STEP_SIZE / 2.0
                )

            self.current_angular_velocity = self.__make_simple_profile(
                output=self.current_angular_velocity,
                input=self.target_angular_velocity,
                slop=(self.ANG_VEL_STEP_SIZE / 2.0)
                )

            # -> Construct message
            twist = Twist()

            # -> Set linear velocity
            twist.linear.x = self.current_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            # -> Set angular velocity
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.current_angular_velocity

            # -> Publish instruction msg to robot
            self.instruction_publisher.publish(msg=twist)

            # -> Publish msg to console (ROS print)
            if self.verbose in [2, 3]:
                self.get_logger().info(f"Publishing: " +
                                    f"\n       x: {twist.linear.x}" +
                                    f"\n       y: {twist.linear.y}" +
                                    f"\n       z: {twist.linear.z}" 
                                    f"\n       ___________"
                                    f"\n       u: {twist.angular.x}" +
                                    f"\n       v: {twist.angular.y}" +
                                    f"\n       w: {twist.angular.z}")

    def projected_pose_publisher_callback(self):
        if self.projected_pose is not None:
            # -> Construct projected pose
            msg = PoseStamped()

            msg.pose.position.x = self.projected_pose[0]
            msg.pose.position.y = self.projected_pose[1]

            yaw_angle = math.tan(
                self.current_direct_path[1][1]-self.current_direct_path[1][0]/
                (self.current_direct_path[0][1]-self.current_direct_path[0][0] or 0.00000000001) * 180/math.pi
                )

            qx, qy, qz, qw = self.__get_quaternion_from_euler(
                roll=0,
                pitch=0,
                yaw=yaw_angle
            )

            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw

            self.projected_odom_publisher.publish(msg)

    # ---------------------------------- Subscribers
    # def team_msg_subscriber_callback(self, msg):
    #     pass

    def lazer_scan_subscriber_callback(self, msg):
        scan = list(msg.ranges)
        
        range_min = msg.range_min
        range_max = msg.range_max

        if range_min < lazer_scan_range_min:
            range_min = lazer_scan_range_min

        # -> Clean up ranges
        for i, range_measure in enumerate(scan):
            if range_measure < range_min or range_measure > range_max:
                scan[i] = None

        self.lazer_scan = scan

    def odom_subscriber_callback(self, msg):
        self.position = [msg.pose.position.x, msg.pose.position.y]
        self.orientation = self.__euler_from_quaternion(
            quat=msg.pose.orientation
        )[-1]
        
        # -> Publish cooresponding projected pose
        self.projected_pose_publisher_callback()
     
    def goal_subscriber_callback(self, msg):
        if self.verbose > 0:
            print(f"++++++++++++++++++++++++++++++ Goal sequence {msg.goal_sequence_id} (for {msg.robot_id}) received by {self.robot_id} ++++++++++++++++++++++++++++++")
        
        # -> If message is addressed to robot
        if msg.robot_id == self.robot_id:
            goal_sequence = {
                "ID":msg.goal_sequence_id,
                "sequence": []
                }

            # -> Populate sequence
            for point in msg.sequence:
                goal_sequence["sequence"].append([point.x, point.y, point.z])

            # -> Log task to goal_backlog according to priority
            if int(msg.priority) not in self.goal_sequence_backlog.keys():
                self.goal_sequence_backlog[int(msg.priority)] = []

            self.goal_sequence_backlog[int(msg.priority)].append(goal_sequence)
    
    # ================================================= Goal properties
    @property
    def goal(self):
        try:
            return self.goal_sequence[0]
        except:
            return None

    @property
    def path_vector(self) -> list:
        if self.goal is None:
            return [0, 0]
        
        else:
            path_vector = [
                self.goal[0] - self.position[0],
                self.goal[1] - self.position[1],
            ]

            return path_vector

    @property
    def current_direct_path(self):
        try:
            if self.goal_sequence is None:
                return None
            if self.prev_point is not None:
                return (self.prev_point, self.goal_sequence[0][0:2])
            else:
                return (self.position, self.goal_sequence[0][0:2])
        except:
            return None

    @property
    def projected_pose(self):
        if self.current_direct_path is None:
            return None

        p1 = np.array(self.current_direct_path[0])
        p2 = np.array(self.current_direct_path[1])
        p3 = np.array(self.position)

        # -> Calc. distance between p1 and p2
        l2 = np.sum((p1-p2)**2)
 
        t = np.sum((p3 - p1) * (p2 - p1)) / l2

        return p1 + t * (p2 - p1)

    @property
    def distance_to_goal(self):
        return math.sqrt(self.path_vector[0]**2 + self.path_vector[1]**2)

    @property
    def goal_angle(self):
        angle = math.atan2(self.path_vector[1], self.path_vector[0]) * 180/math.pi
        
        # -> Convert to 0<->180/0<->-180
        if angle > 180:
            return -360 + angle
        else:
            return angle
    
    # ================================================= Goal management
    
    def __publish_goal_teamcomm(self):
        msg = TeamComm()
        msg.source = self.robot_id
        msg.source_type = "robot"
        msg.type = "Goal_announcement"

        # -> Construct memo
        sequence = []
        for point in self.goal_sequence:
            sequence.append((point[0], point[1], point[2]))

        goal = {
        "id": self.goal_id,
        "sequence": sequence
            }
        msg.memo = json.dumps(goal)

        # -> Publish message
        self.rlb_msgs_publisher.publish(msg=msg)

    def check_subgoal_state(self):
        # -> Remove sub-goal if reached
        if self.distance_to_goal < self.success_distance_range:
            self.prev_point = self.goal_sequence[0][0:2]
            
            if self.verbose > 0:
                print(f"-------------------------------------> Subgoal {self.goal_sequence[0]} completed")
            
            self.goal_sequence.pop(0)

            # -> Announce new subgoal on teams comms
            self.__publish_goal_teamcomm()
        
            if len(self.goal_sequence) != 0:
                if self.verbose > 0:
                    print(f"                                       New subgoal: {self.goal_sequence[0]}   (distance: {round(self.distance_to_goal, 3)})")

            if self.verbose > 0:
                print(f"                                       Goal sequence left: {self.goal_sequence}")
            return True

        else:
            return False

    def check_goal_sequence(self):
        return len(self.goal_sequence) == 0

    def get_new_goal_sequence(self):
        # -> Determine highest priority sequence
        selected_goal_sequence_priority = -1

        for goal_priority, goal_sequence_lst in self.goal_sequence_backlog.items():
            if len(goal_sequence_lst) != 0 and goal_priority > selected_goal_sequence_priority:
                selected_goal_sequence_priority = goal_priority

        # -> Retrieve cooresponding goal sequence from goal_backlog
        if selected_goal_sequence_priority != -1:
            goal_sequence = self.goal_sequence_backlog[selected_goal_sequence_priority].pop(0)


            self.goal_sequence = goal_sequence["sequence"]
            self.goal_id = goal_sequence["ID"]
            self.goal_sequence_priority = selected_goal_sequence_priority

        # -> Announce new goal on teams comms
            self.__publish_goal_teamcomm()
        
    
    # ================================================= Utils
    @staticmethod
    def __euler_from_quaternion(quat):  
        """  
        Convert quaternion (w in last place) to euler roll, pitch, yaw (rad).  
        quat = [x, y, z, w]    
        
        """    
        x = quat.x  
        y = quat.y  
        z = quat.z  
        w = quat.w  

        sinr_cosp = 2 * (w * x + y * z)  
        cosr_cosp = 1 - 2 * (x * x + y * y)  
        roll = np.arctan2(sinr_cosp, cosr_cosp) * 180/math.pi
    
        sinp = 2 * (w * y - z * x)  
        pitch = np.arcsin(sinp) * 180/math.pi
    
        siny_cosp = 2 * (w * z + x * y)  
        cosy_cosp = 1 - 2 * (y * y + z * z)  
        yaw = np.arctan2(siny_cosp, cosy_cosp) * 180/math.pi

        # if yaw < 0:
        #     yaw = (180 - abs(yaw)) + 180
    
        return [roll, pitch, yaw]

    @staticmethod
    def __get_quaternion_from_euler(roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return qx, qy, qz, qw

    @staticmethod
    def __make_simple_profile(output, input, slop):
        if input > output:
            output = min(input, output + slop)
        elif input < output:
            output = max(input, output - slop)
        else:
            output = input

        return output

    def stop_robot(self):
        # -> Construct message
        twist = Twist()

        # -> Set linear velocity
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        # -> Set angular velocity
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        # -> Publish instruction msg to robot
        self.instruction_publisher.publish(msg=twist)
        self.instruction_publisher.publish(msg=twist)
        self.instruction_publisher.publish(msg=twist)
        self.instruction_publisher.publish(msg=twist)
        self.instruction_publisher.publish(msg=twist)


def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    path_sequence = Minimal_path_sequence()

    rclpy.spin(path_sequence)

    path_sequence.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()