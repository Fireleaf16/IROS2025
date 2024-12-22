#!/usr/bin/env python
"""
This script is used for mapping the HTC vive controller pose to the robot arm pose. 
The mapping is done by compensating the difference between the global and trac_ik coordinate systems.
When reaching the obstacle, the arm moving will be divided into xyz, amd depedning on the arm status, the arm will stop in different directions while the other directions can still move.

Author: Yichen Xie
"""
import sys

import rospy
import numpy as np
import transformations
import tf
import copy
from numpy import linalg as LA


from geometry_msgs.msg import Pose, PoseStamped,Point


from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped, Pose
from std_msgs.msg import Float64, Bool

class VivePoseMapping:
    def __init__(
        self,
        controller_side='right',
        tracking_mode='press',
        headset_mode='table',
    ):

        self.initialized = False

        if controller_side not in ['right', 'left']:
            raise ValueError(
                'controller_side should be either "right" or "left".'
            )

        if tracking_mode not in ['hold', 'press']:
            raise ValueError(
                'tracking_mode should be either "hold" or "press".'
            )


        # # Public constants:
        self.CONTROLLER_SIDE = controller_side
        self.TRACKING_MODE = tracking_mode
        self.HEADSET_MODE = headset_mode

        self.gripper_val = 0
        
        self.rec_100 = 0
        self.flagg = 0

        self.vive_stop = 0


        self.vive_buttons = [0,0,0,0]
        self.vive_axes = [0,0,0]

        self.trigger_press = False

        self.activate_button = 0

        self.base_link = "torso_lift_link"
        # self.base_link = "base_footprint"

        self.ee_link = "arm_" + self.CONTROLLER_SIDE + "_tool_link"



        # # Private variables:
        self.__vive_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }

        self.__tracking_state_machine_state = 0
        # self.__control_mode = 'position'
        self.__control_mode = 'full'

        self.__listener = tf.TransformListener()

        # # Public variables:
        self.pose_tracking = False

        # Last commanded Relaxed IK pose is required to compensate controller
        # input.
        self.last_ee_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }
        self.last_commanded_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }
        # This difference is calculated each time the tracking is started and
        # subracted from future inputs during current tracking to compensate for
        # linear misalignment between the global and relaxed_ik coordinate
        # systems.
        self.vive_pose_difference = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }
        ## arm flags
        self.extend = False
        self.right_shrink = False
        self.left_shrink = False
        self.right_forward = False
        self.left_forward = False

        self.rightup = False
        self.rightdown = False
        self.leftup = False
        self.leftdown = False

        self.right_cross = False
        self.left_cross = False

        self.collsion = False

        self.current_arm_length = 0.0
        # # ROS node:

        # # Topic publisher:
        self.__compensate_pose_pub = rospy.Publisher(f'/{self.CONTROLLER_SIDE}/compensate_pose',PoseStamped,queue_size=1,)
        self.robot_activation = rospy.Publisher('/'+controller_side+'/robot_activation', Float64, queue_size=1)
        self.rotation_activation = rospy.Publisher('/rotation_activation', Float64, queue_size=1)

        # # Topic subscriber:
        rospy.Subscriber('/arm_' + self.CONTROLLER_SIDE + '_target_pose', Pose, self.__commanded_pose_callback)

        if self.CONTROLLER_SIDE == "right":
            rospy.Subscriber('/Right_Hand',TransformStamped,self.__input_pose_callback,)
            rospy.Subscriber('/Right_Buttons', Joy, self.callback_vive_b)

        elif self.CONTROLLER_SIDE == "left":
            rospy.Subscriber('/Left_Hand',TransformStamped,self.__input_pose_callback,)
            rospy.Subscriber('/Left_Buttons', Joy, self.callback_vive_b)


        # arm_status subscriber
        rospy.Subscriber('/'+controller_side+'/arm_extend', Bool, self.__arm_extend_callback)
        rospy.Subscriber('/'+controller_side+'/arm_length', Float64, self.__arm_length_callback)
        rospy.Subscriber('/right/arm_shrink', Bool, self.__arm_right_shrink_callback)
        rospy.Subscriber('/left/arm_shrink', Bool, self.__arm_left_shrink_callback)
        rospy.Subscriber('/'+controller_side+'/arm_forward', Bool, self.__arm_forward_callback)

        rospy.Subscriber('/right/arm_updown', Float64, self.__right_updown_callback)
        rospy.Subscriber('/left/arm_updown', Float64, self.__left_updown_callback)

        rospy.Subscriber('/right/arm_cross', Bool, self.__right_arm_cross_callback)
        rospy.Subscriber('/left/arm_cross', Bool, self.__left_arm_cross_callback)
        rospy.Subscriber('/'+self.CONTROLLER_SIDE+'/target_collision', Point, self.__collsion_callback)




        # rospy.Subscriber(
        #     f'/{self.ROBOT_NAME}/relaxed_ik/commanded_pose_gcs',
        #     Pose,
        #     self.__commanded_pose_callback,
        # )

    # # Service handlers:

    # # Topic callbacks:


    ## -------------- Arm_status Callback -------------- ##
    def __arm_extend_callback(self, msg):
        self.extend = msg.data

    def __arm_forward_callback(self, msg):
        if self.CONTROLLER_SIDE == "right":
            self.right_forward = msg.data
        else:
            self.left_forward = msg.data


    def __arm_right_shrink_callback(self, msg):
        self.right_shrink = msg.data

    def __arm_left_shrink_callback(self, msg):
        self.left_shrink = msg.data

    def __arm_length_callback(self, msg):
        self.current_arm_length = msg.data



    def __right_updown_callback(self, msg):
        if msg.data > 0:
            self.rightup = True
        else:
            if msg.data < 0:
                self.rightdown = True
            else:
                self.rightup = False
                self.rightdown = False
    
    def __left_updown_callback(self, msg):
        if msg.data > 0:
            self.leftup = True
        else:
            if msg.data < 0:
                self.leftdown = True
            else:
                self.leftup = False
                self.leftdown = False


    def __right_arm_cross_callback(self, msg):
        self.right_cross = msg.data
        

    def __left_arm_cross_callback(self, msg):
        self.left_cross = msg.data
        

    def __collsion_callback(self, msg):
        self.collsion = [msg.x, msg.y, msg.z] 
        

    def callback_vive_b(self, msg):
        
        self.vive_buttons = msg.buttons
        self.vive_axes = msg.axes
        #rospy.logwarn("vive_buttons: %s", self.vive_buttons)
        self.gripper_val = self.vive_axes[2]

        self.trigger_press = False

        self.activate_button = 0
        
        if self.gripper_val == 1:  # Trigger button to hold the gripper state
            self.rec_100 += 1
            self.trigger_press = True
            # vive_menu += 1
            # rospy.sleep(0.5)

        if self.vive_buttons[2] == 1:  # Side button to start control
            self.flagg = 1
            # self.activate_button = 1

            # rospy.sleep(0.5)
            # print("started")

        if self.vive_buttons[0] == 1:
            # self.vive_menu += 1
            self.__tracking_state_machine_state = 3
            self.pose_tracking = False


            # print("home", vive_menu)
            # rospy.sleep(0.5)

        if self.vive_buttons[3] == 1:  # Side button as the stop button
            # if vive_menu % 2 == 0 and vive_menu != 0:
            self.vive_stop += 1
            # print("pause", self.vive_stop)
            self.activate_button = 1
            rospy.sleep(0.5)


    def __input_pose_callback(self, msg): #come from hand topic of the vive controller
        self.__vive_pose['position'][0] = -msg.transform.translation.x
        self.__vive_pose['position'][1] = -msg.transform.translation.y
        self.__vive_pose['position'][2] = msg.transform.translation.z - 0.96 + 0.25

        self.__vive_pose['orientation'][0] = msg.transform.rotation.x
        self.__vive_pose['orientation'][1] = msg.transform.rotation.y
        self.__vive_pose['orientation'][2] = msg.transform.rotation.z
        self.__vive_pose['orientation'][3] = msg.transform.rotation.w


    def __commanded_pose_callback(self, message): #come from target pose topic of the arm in tiago_arm_position_control
        self.last_commanded_pose['position'][0] = message.position.x
        self.last_commanded_pose['position'][1] = message.position.y
        self.last_commanded_pose['position'][2] = message.position.z

        self.last_commanded_pose['orientation'][0] = message.orientation.x
        self.last_commanded_pose['orientation'][1] = message.orientation.y
        self.last_commanded_pose['orientation'][2] = message.orientation.z
        self.last_commanded_pose['orientation'][3] = message.orientation.w

    # # Private methods:
    def __update_ee_transformation(self):
        """
            use tf transform listener to get ee link pose relative to base link
        """
        try:
            (trans, rot) = self.__listener.lookupTransform(self.base_link, self.ee_link, rospy.Time(0))
            self.last_commanded_pose['position'][0] = trans[0]
            self.last_commanded_pose['position'][1] = trans[1]
            self.last_commanded_pose['position'][2] = trans[2]
            self.last_commanded_pose['orientation'][0] = rot[0]
            self.last_commanded_pose['orientation'][1] = rot[1]
            self.last_commanded_pose['orientation'][2] = rot[2]
            self.last_commanded_pose['orientation'][3] = rot[3]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass


    def __tracking_state_machine(self, button):
        # State 0: Grip button was pressed.
        if (self.__tracking_state_machine_state == 0 and button):
            self.__tracking_state_machine_state = 1

            if self.TRACKING_MODE == 'hold':

                self.__calculate_compensation()
                self.pose_tracking = True

        # State 1: Grip button was released. Tracking is activated.
        elif (self.__tracking_state_machine_state == 1 and not button):
            if self.TRACKING_MODE == 'press':
                self.__tracking_state_machine_state = 2
                self.__calculate_compensation()
                self.pose_tracking = True

            elif self.TRACKING_MODE == 'hold':
                self.__tracking_state_machine_state = 0
                self.pose_tracking = False

        # State 2: Grip button was pressed. Tracking is deactivated.
        elif (self.__tracking_state_machine_state == 2 and button):
            self.__tracking_state_machine_state = 3

            self.pose_tracking = False

        # State 3: Grip button was released.
        elif (self.__tracking_state_machine_state == 3 and not button):
            self.__tracking_state_machine_state = 0

        self.robot_activation.publish(self.__tracking_state_machine_state)


    def __calculate_compensation(self):
        """
        Calculates the compensation for coordinate systems misalignment.
        """
        # self.last_vive_pose = copy.deepcopy(self.__vive_pose)
        if not self.initialized:
            self.__update_ee_transformation()   
            self.initialized = True

        self.vive_pose_difference['position'] = (
            self.__vive_pose['position'] #get from hand topic
            - self.last_commanded_pose['position'] #get from target pose topic
            
        )
       
        self.vive_pose_difference['orientation'] = (
            transformations.quaternion_multiply(
                self.last_commanded_pose['orientation'],
                transformations.quaternion_inverse(
                    self.__vive_pose['orientation']
                ),
            )
        )
        # rospy.logwarn("vive_pose: %s, last commanded: %s", self.__vive_pose, self.last_commanded_pose)

    # # Public methods:
    def main_loop(self):     
        self.__tracking_state_machine(self.activate_button) #(self.vive_buttons[2])

        if self.pose_tracking:
            self.publish_compensate_pose()

    def publish_compensate_pose(self):
        compensated_input_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }


        compensated_input_pose['position'] = (
            self.__vive_pose['position'] #comes from hand topic
            - self.vive_pose_difference['position'] 
            
        )
        rospy.logwarn("vive pose: %s", self.__vive_pose['position'])
        rospy.logwarn("vive pose difference: %s", self.vive_pose_difference['position'])

        # Use oculus orientation.
        if self.__control_mode == 'full':
            compensated_input_pose['orientation'] = (
                transformations.quaternion_multiply(
                    self.vive_pose_difference['orientation'],
                    self.__vive_pose['orientation'],
                )
            )


        # Orientation only
        if self.__control_mode == 'orientation':

            compensated_input_pose['position'] = (
                self.last_ee_pose['position'] #useless in real test as assuming no movement in xyz
            )
            compensated_input_pose['orientation'] = (
                transformations.quaternion_multiply(
                    self.vive_pose_difference['orientation'],
                    self.__vive_pose['orientation'],
                )
            )


        diff = LA.norm(np.array(compensated_input_pose['position']) - np.array(self.last_commanded_pose['position']))
        distance = np.array(compensated_input_pose['position']) - np.array(self.last_commanded_pose['position'])
        if diff > 0.1:
            rospy.logwarn("calulate compensate: %s, %s, %s", compensated_input_pose['position'], self.last_commanded_pose['position'], diff)
            self.__calculate_compensation()

        else:
            ## arm_status check
            arm_status_flag = True
            #rospy.logwarn("calulate distance: %s", distance)
            # Handling forward movements
            if self.left_forward or self.right_forward or (self.collsion[0] >1.1 and self.collsion[2] < 0.72):
                if distance[0] < 0:
                    arm_status_flag = True
                else:
                    compensated_input_pose['position'][0] = self.last_commanded_pose['position'][0]
                    #rospy.logwarn("Desired position exceeds max forward distance. Keeping current position.")

            elif self.right_shrink or self.left_shrink:
                if distance[0] > 0:
                    arm_status_flag = True
                else:
                    compensated_input_pose['position'][0] = self.last_commanded_pose['position'][0]
                    #rospy.logwarn("Desired position exceeds max shrink distance. Keeping current position.")

            # Handling right side extension
            elif (self.CONTROLLER_SIDE == "right" and self.extend) or self.left_cross:
                
                if distance[1] > 0:
                    arm_status_flag = True
                else:
                    compensated_input_pose['position'][1] = self.last_commanded_pose['position'][1]
                    #rospy.logwarn("distance: %s", distance)
                    #rospy.logwarn("Desired position exceeds max right distance. Keeping current position.")

            # Handling left side extension
            elif (self.CONTROLLER_SIDE == "left" and self.extend) or self.right_cross:
                if distance[1] < 0:
                    arm_status_flag = True
                else:
                    compensated_input_pose['position'][1] = self.last_commanded_pose['position'][1]
                    #rospy.logwarn("distance: %s", distance)
                    #rospy.logwarn("Desired position exceeds max left distance. Keeping current position.")
            
            # Handling updown


            elif (self.rightdown or self.leftdown or 
                (self.collsion[0] > 1.1 and self.collsion[2] > 0.72 and self.collsion[2] < 0.78) or 
                (self.collsion[0] < -0.7 and self.collsion[2] < 0.7 and self.collsion[1] < -0.45) or 
                (self.collsion[0] < -0.75 and self.collsion[2] < 0.49 and self.collsion[1] > -0.45)):
                if distance[2] > 0:
                    arm_status_flag = True
                else:
                    compensated_input_pose['position'][2] = self.last_commanded_pose['position'][2]
                    #rospy.logwarn("Desired position exceeds max down distance. Keeping current position.")


            # Default action if no specific condition is met
            else:
                arm_status_flag = True 
            ## arm_status check end

            #rospy.logwarn("arm_status_flag: %s", arm_status_flag)


            if arm_status_flag:
                overshooting = False
                #rospy.logwarn("flag true")
                
                
                if not overshooting:
                    pose_stamped = PoseStamped()
                    pose_stamped.header.stamp = rospy.Time.now()
                    pose_stamped.header.frame_id = self.base_link

                    pose_message = Pose()
                        
                    pose_message.position.x = compensated_input_pose['position'][0]
                    pose_message.position.y = compensated_input_pose['position'][1]
                    pose_message.position.z = compensated_input_pose['position'][2]
                    
                    pose_message.orientation.x = compensated_input_pose['orientation'][0]
                    pose_message.orientation.y = compensated_input_pose['orientation'][1]
                    pose_message.orientation.z = compensated_input_pose['orientation'][2]
                    pose_message.orientation.w = compensated_input_pose['orientation'][3]

                    pose_stamped.pose = pose_message

                    self.__compensate_pose_pub.publish(pose_stamped) #publish to arm_position_control which is the new target pose
                else:
                    rospy.logwarn("Overshooting. Keeping current position.")
        # rospy.logwarn("vive pose difference: %s", self.vive_pose_difference)
        # rospy.logwarn("vive last pose: %s", self.last_vive_pose)

def node_shutdown():
    """
    
    """

    print('\nvive_pose_mapping has been shutdown\n')


def main():
    """

    """

    # # ROS node:
    rospy.init_node('vive_pose_mapping')
    rospy.on_shutdown(node_shutdown)

    args = rospy.myargv(argv=sys.argv)
    controller_side = args[1]
    tracking_mode = args[2]

    vive_pose_mapping = VivePoseMapping(
        controller_side=controller_side,
        tracking_mode=tracking_mode,
        headset_mode='table',
    )

    while not rospy.is_shutdown():
        vive_pose_mapping.main_loop()


if __name__ == '__main__':
    main()
