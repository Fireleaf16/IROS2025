#!/usr/bin/env python
"""
This script is used for mapping the HTC vive controller pose to the robot arm


Author: Juo-Tung Chen
"""
import sys

import rospy
import numpy as np
import transformations
import tf
import copy
from numpy import linalg as LA


from geometry_msgs.msg import Pose, PoseStamped


from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped, Pose
from std_msgs.msg import Float64, Bool

class VivePoseMapping:
    """
    
    """

    def __init__(
        self,
        controller_side='right',
        tracking_mode='press',
        headset_mode='table',
    ):
        """
        
        """
        self.initialized = False

        if controller_side not in ['right', 'left']:
            raise ValueError(
                'controller_side should be either "right" or "left".'
            )

        if tracking_mode not in ['hold', 'press']:
            raise ValueError(
                'tracking_mode should be either "hold" or "press".'
            )

        # # Private constants:

        # # Public constants:
        self.CONTROLLER_SIDE = controller_side
        self.TRACKING_MODE = tracking_mode
        self.HEADSET_MODE = headset_mode

        self.gripper_val = 0
        
        self.rec_100 = 0
        self.flagg = 0
        self.vive_stop = 0
        self.marker = 0
        self.vive_menu = 0

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
        self.__gripper_state_machine_state = 0
        self.__mode_state_machine_state = 0
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

        # # ROS node:

        # # Service provider:

        # # Service subscriber:
        # self.__gripper_force_grasping = rospy.ServiceProxy(
        #     f'/{self.ROBOT_NAME}/gripper/force_grasping',
        #     GripperForceGrasping,
        # )
        # self.__gripper_position = rospy.ServiceProxy(
        #     f'/{self.ROBOT_NAME}/gripper/position',
        #     GripperPosition,
        # )

        # # Topic publisher:
        self.__compensate_pose_pub = rospy.Publisher(
            f'/{self.CONTROLLER_SIDE}/compensate_pose',
            PoseStamped,
            queue_size=1,
        )
        self.__ee_pose_pub = rospy.Publisher(
            f'/{self.CONTROLLER_SIDE}/ee_pose',
            Pose,
            queue_size=1,
        )
        self.robot_activation = rospy.Publisher('/'+controller_side+'/robot_activation', Float64, queue_size=1)
        self.rotation_activation = rospy.Publisher('/rotation_activation', Float64, queue_size=1)

        # # Topic subscriber:
        rospy.Subscriber('/arm_' + self.CONTROLLER_SIDE + '_target_pose', Pose, self.__commanded_pose_callback)

        if self.CONTROLLER_SIDE == "right":
            rospy.Subscriber(
                '/Right_Hand',
                TransformStamped,
                self.__input_pose_callback,
            )

            rospy.Subscriber(
                '/Right_Buttons', 
                Joy, 
                self.callback_vive_b)

        elif self.CONTROLLER_SIDE == "left":
            rospy.Subscriber(
                '/Left_Hand',
                TransformStamped,
                self.__input_pose_callback,
            )

            rospy.Subscriber(
                '/Left_Buttons', 
                Joy, 
                self.callback_vive_b)

        # rospy.Subscriber(
        #     f'/{self.ROBOT_NAME}/relaxed_ik/commanded_pose_gcs',
        #     Pose,
        #     self.__commanded_pose_callback,
        # )

    # # Service handlers:

    # # Topic callbacks:

    def callback_vive_b(self, msg):
        
        self.vive_buttons = msg.buttons
        self.vive_axes = msg.axes
        
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



    def __input_pose_callback(self, msg):
        """

        """

        negation = 1

        # if self.HEADSET_MODE == 'head':
        #     negation = -1
        # if not (msg.transform.translation.x == 0.0 and msg.transform.translation.y == 0.0 and msg.transform.translation.z == 0.0 
        # and msg.transform.rotation.x == 0.0 and msg.transform.rotation.y == 0.0 and msg.transform.rotation.z == 0.0 and msg.transform.rotation.w == 1.0):
        self.__vive_pose['position'][0] = -msg.transform.translation.x
        self.__vive_pose['position'][1] = -msg.transform.translation.y
        self.__vive_pose['position'][2] = msg.transform.translation.z - 0.96 + 0.25

        self.__vive_pose['orientation'][0] = msg.transform.rotation.x
        self.__vive_pose['orientation'][1] = msg.transform.rotation.y
        self.__vive_pose['orientation'][2] = msg.transform.rotation.z
        self.__vive_pose['orientation'][3] = msg.transform.rotation.w


    def __commanded_pose_callback(self, message):
        """
        
        """

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
            # self.last_commanded_pose['orientation'] = (
            #     transformations.quaternion_multiply(
            #         self.__vive_pose['orientation'], rot))
            # rospy.logwarn("last_commanded_pose: %s", self.last_commanded_pose)
            # pose_message = self.__compose_pose_message(self.last_ee_pose)
            
            # self.__ee_pose_pub.publish(pose_message)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def __compose_pose_message(self, pose):
        pose_message = Pose()
        pose_message.position.x = pose['position'][0] 
        pose_message.position.y = pose['position'][1] 
        pose_message.position.z = pose['position'][2] 
        pose_message.orientation.x = pose['orientation'][0] 
        pose_message.orientation.y = pose['orientation'][1] 
        pose_message.orientation.z = pose['orientation'][2] 
        pose_message.orientation.w = pose['orientation'][3] 
        return pose_message

    def __tracking_state_machine(self, button):
        """
        
        """

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

    def __mode_state_machine(self, button):
        """
        
        """

        # State 0: Button was pressed.
        if (self.__mode_state_machine_state == 0 and button):
            self.__mode_state_machine_state = 1

            self.__control_mode = 'full'
            self.__calculate_compensation()

        # State 1: Button was released.
        elif (self.__mode_state_machine_state == 1 and not button):
            self.__mode_state_machine_state = 3

        # State 2: Button was pressed.
        elif (self.__mode_state_machine_state == 3 and button):
            self.__mode_state_machine_state = 4

            self.__control_mode = 'position'

        # State 3: Button was released.
        elif (self.__mode_state_machine_state == 4 and not button):
            self.__mode_state_machine_state = 0

        self.rotation_activation.publish(self.__mode_state_machine_state)



    def __calculate_compensation(self):
        """Calculates the compensation for coordinate systems misalignment.
        
        """
        # self.last_vive_pose = copy.deepcopy(self.__vive_pose)
        if not self.initialized:
            self.__update_ee_transformation()   
            self.initialized = True

        self.vive_pose_difference['position'] = (
            self.__vive_pose['position']
            - self.last_commanded_pose['position']
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
        """
        
        """
        
        self.__tracking_state_machine(self.activate_button) #(self.vive_buttons[2])

        if self.pose_tracking:
            self.publish_compensate_pose()

        # self.__gripper_state_machine(self.trigger_press)
        # self.__mode_state_machine(self.vive_buttons[0])

    def publish_compensate_pose(self):
        """
        
        """
        # self.vive_pose_difference['position'] = (
        #     self.__vive_pose['position']
        #     - self.last_vive_pose['position']
        # )
        # self.vive_pose_difference['orientation'] = (
        #     transformations.quaternion_multiply(
        #         self.last_vive_pose['orientation'],
        #         transformations.quaternion_inverse(
        #             self.__vive_pose['orientation']
        #         ),
        #     )
        # )
        compensated_input_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }


        compensated_input_pose['position'] = (
            self.__vive_pose['position']
            - self.vive_pose_difference['position']
        )
        
        # Use fixed orientation.
        # compensated_input_pose['orientation'] = (
        #     self.last_ee_pose['orientation']
        # )

        # Use oculus orientation.
        if self.__control_mode == 'full':
            compensated_input_pose['orientation'] = (
                transformations.quaternion_multiply(
                    self.vive_pose_difference['orientation'],
                    self.__vive_pose['orientation'],
                )
            )
            
            # Real-time tracking
            # compensated_input_pose['orientation'] = (
            #         self.__vive_pose['orientation']
                
            # )


        # Orientation only
        if self.__control_mode == 'orientation':

            compensated_input_pose['position'] = (
                self.last_ee_pose['position']
            )
            compensated_input_pose['orientation'] = (
                transformations.quaternion_multiply(
                    self.vive_pose_difference['orientation'],
                    self.__vive_pose['orientation'],
                )
            )


        diff = LA.norm(np.array(compensated_input_pose['position']) - np.array(self.last_commanded_pose['position']))
        if diff > 0.15:
            rospy.logwarn("calulate compensate: %s, %s, %s", compensated_input_pose['position'], self.last_commanded_pose['position'], diff)
            self.__calculate_compensation()

        else:
            
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

            self.__compensate_pose_pub.publish(pose_stamped)
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

    # controller_side = rospy.get_param(
    #     param_name=f'{rospy.get_name()}/controller_side',
    #     default='right',
    # )

    # tracking_mode = rospy.get_param(
    #     param_name=f'{rospy.get_name()}/tracking_mode',
    #     default='press',
    # )

    vive_pose_mapping = VivePoseMapping(
        controller_side=controller_side,
        tracking_mode=tracking_mode,
        headset_mode='table',
    )
    # print('\nVive Pose mapping is ready.\n')

    while not rospy.is_shutdown():
        vive_pose_mapping.main_loop()


if __name__ == '__main__':
    main()