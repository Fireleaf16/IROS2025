#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This script is used for teleoperating the Tiago++ robots torso, head, and mobile base using HTC vive controllers.

Controller mapping:

    /Head_Motion (HTC vive headset) -> robot head
    /Right_Buttons trackpad -> x, y motion of the mobile base
    /Right_Buttons menu button -> home right arm
    /Right_Buttons squeeze button -> activate right arm
    /Left_Buttons trackpad left / right -> yaw rotation of the mobile base
    /Left_Buttons trackpad up / down -> lift / descend torso by 5 cm
    /Left_Buttons menu button -> home left arm
    /Left_Buttons squeeze button -> activate left arm


Author: Yichen Xie
"""

import sys
import rospy
import math
import numpy as np
import csv
import datetime
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix
import actionlib
import os
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped, Pose
from sensor_msgs.msg import Joy, JointState, LaserScan
from std_msgs.msg import Float64, Bool, String
from std_msgs.msg import Float32MultiArray

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

from pal_startup_msgs.srv import StartupStop, StartupStopRequest, StartupStart, StartupStartRequest
from teleop_tools_msgs.msg import IncrementAction as TTIA
from teleop_tools_msgs.msg import IncrementGoal as TTIG

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None 
        self.dt = 0.0

    def compute_control(self, target, current):
        """
        Compute the control output using PID.
        Args:
            target (float): Target value.
            current (float): Current value.

        Returns:
            float: Control output.
        """
        current_time = rospy.Time.now()
        if self.last_time is None:
            self.last_time = current_time
            self.dt = 0.0  # No elapsed time for the first computation
        else:
            self.dt = (current_time - self.last_time).to_sec()
        
        self.last_time = current_time
        #self.dt = 1
        

        error = target - current
        rospy.loginfo(f"PID Compute: Target={target}, Current={current}, Error={error}, dt={self.dt}")

        if abs(error) < 0.01:
            rospy.loginfo("PID Error is within threshold. No control output.")
            return None

        # Update integral and derivative terms
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt if self.dt > 0 else 0.0

        # Compute the control output
        control = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Log PID components
        #rospy.loginfo(f"PID Details: Kp*Error={self.kp * error}, Ki*Integral={self.ki * self.integral}, Kd*Derivative={self.kd * derivative}")
        #rospy.loginfo(f"PID Control Output: {control}")

        # Update previous error
        self.prev_error = error

        return control
    
    def rot_compute_control(self, target, current):
        """
        Compute the control output using PID.
        Args:
            target (float): Target value.
            current (float): Current value.

        Returns:
            float: Control output.
        """
        current_time = rospy.Time.now()
        if self.last_time is None:
            self.last_time = current_time
            self.dt = 0.0  # No elapsed time for the first computation
        else:
            self.dt = (current_time - self.last_time).to_sec()
        
        self.last_time = current_time
        #self.dt = 1
        

        error = target - current
        #rospy.loginfo(f"PID Rotate Compute: Target={target}, Current={current}, Error={error}, dt={self.dt}")
        if abs(error) < 0.01:
            rospy.loginfo("PID Error is within threshold. No control output.")
            return None

        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt if self.dt > 0 else 0.0

        control = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        return control


class JoystickTeleop():
    def __init__(self, sim=True):
        ## -------------- PID Controllers -------------- ##
        self.pid_x = PIDController(kp=0.01, ki=0.01, kd=0.01)
        self.pid_y = PIDController(kp=0.01, ki=0.01, kd=0.01)
        self.pid_rotation = PIDController(kp=0.01, ki=0.01, kd=0.01)

        self.target_xy = [0.0, 0.0]  # Target position in XY
        self.current_xy = [0.0, 0.0]  # Current position in XY
        self.target_orientation = 0.0  # Target orientation (yaw)
        self.current_orientation = 0.0  # Current orientation (yaw)
        self.adjust_orientation_flag = False
        self.adjust_xy_flag = False
        self.target_frame = None
        self.arm_base_link = "torso_lift_link"
        self.arm_ee_link = "arm_right_tool_link"
        self.arm_fixed_link = "arm_right_1_link"
        self.odom = "odom"
        self.goal_pose_rotate = None
        self.goal_pose = None
        self.rotation_quaternion = np.array([-0.7071, 0.0, 0.0, 0.7071])
        ## -------------- Variables -------------- ##
        self.sim = sim


        ## ------- Limits ------- ##
        self.base_speed = [0.03, 0.1]
        self.rotate_speed = [0.11, 0.31]
        self.torso_limit = 0.07
        self.head_joint_limits = [[-1.24, 1.24], [-0.98, 0.72]]

        ### ------- Head------- ###
        self.alpha = 1.0
        self.initialized = False
        self.filtered_orientation = [0.0, 0.0]  # Initial filtered orientation
        self.prev_head_pos = [0.0, 0.0]
        self.head_orientation = [0.0, 0.0]

        ### ------- Torso ------- ###
        self.torso_thres = 0.12
        self.torso_joint = 0.05

        ### ------- Base ------- ###
        self._hz = rospy.get_param('~hz', 10)
        self._forward_rate = self.base_speed[1]
        self._angular = 0
        self._linear = 0
        self._linear_y = 0
        self.angular_rate = 0.5
        self.lidar_warning_range = 1.5
        self.collision_threshold = 0.5
        self.base_domain = True
        ## -------------- Flags -------------- ##
        self.activated = False
        self.gripper_pressed = False
        self.torso_pressed = False
        self.right_extended = False
        self.left_extended = False
        self.right_move_collision = False
        self.left_move_collision = False
        self.right_above_desk = False
        self.left_above_desk = False

        ## -------------- TF Listener -------------- ##
        self.listener = tf.TransformListener()
        self.tf_br = tf.TransformBroadcaster()

        ## -------------- ROS Publishers -------------- ##
        if sim:
            self._pub_cmd = rospy.Publisher('key_vel', Twist, queue_size=10)

        else:
            self._pub_cmd = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
            self._pub_move_base = rospy.Publisher('/move_base_target', Float32MultiArray, queue_size=1)

        self.right_adjust_z_complete_pub = rospy.Publisher('/right/adjust_z_flag', Bool, queue_size=1)
        self.right_adjust_xy_complete_pub = rospy.Publisher('/right/adjust_xy_flag', Bool, queue_size=1)
        self.right_adjust_rotate_complete_pub = rospy.Publisher('/right/adjust_rotate_flag', Bool, queue_size=1)

        ## -------------- ROS Action Clients ------------ ##
        self.torso_client = actionlib.SimpleActionClient('/torso_controller/increment', TTIA)
        self.head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=2)

        

        rospy.sleep(0.1)

        ## -------------- ROS Subscribers -------------- ##

        # rospy.Subscriber('/Right_Hand', TransformStamped, self.__right_input_pose_callback)
        rospy.Subscriber('/Right_Buttons', Joy, self.__right_input_buttons_callback)
        # rospy.Subscriber('/Left_Hand', TransformStamped, self.__left_input_pose_callback)
        rospy.Subscriber('/Left_Buttons', Joy, self.__left_input_buttons_callback)
        rospy.Subscriber('/Head_Motion', PoseStamped, self.__head_motion_callback)
        rospy.Subscriber('/joint_states', JointState, self.__joint_states_callback)
        #rospy.Subscriber('/left/arm_extend', Bool, self.__left_arm_extend_callback)
        #rospy.Subscriber('/right/arm_extend', Bool, self.__right_arm_extend_callback)
        rospy.Subscriber("/scan", LaserScan, self.__scan_callback)


        rospy.Subscriber('right/move_collision', Bool, self.__right_collision_callback)
        rospy.Subscriber('left/move_collision', Bool, self.__left_collision_callback)
        rospy.Subscriber('right/base_fast_domain', Bool, self.__right_base_fast_domain_callback)

        rospy.Subscriber('/right/current_above_desk', Float64, self.__right_current_above_task_callback)
        rospy.Subscriber('/left/current_above_desk', Float64, self.__left_current_above_task_callback)


        rospy.Subscriber('/right/adjust_z', Float64, self.__right_adjust_z_callback)
        rospy.Subscriber('/right/adjust_xy', Float32MultiArray, self.__right_adjust_xy_callback)
        rospy.Subscriber('/right/adjust_rotation', Float64, self.__right_adjust_orientation_callback)

        rospy.Subscriber('/right/target_frame', String, self.__target_frame_callback)




        rospy.sleep(0.1)


        ## -------------- Shut Down Head Manager --------------

        if not sim:

            rospy.wait_for_service('/pal_startup_control/stop')
            try:
                stop_service = rospy.ServiceProxy('/pal_startup_control/stop', StartupStop)
                request = StartupStopRequest()
                request.app = 'head_manager'
                response = stop_service(request)

            except rospy.ServiceException as e:
                print("Service call failed:", e)


        rospy.logwarn("\n\n------------vive teleop ready------------\n\n")
    ## -------------- PID Control -------------- ##
    def _pid_control(self, target_xy, current_xy):
        """
        PID control for XY movement.
        """
        control_x = self.pid_x.compute_control(target_xy[0], current_xy[0])
        control_y = self.pid_y.compute_control(target_xy[1], current_xy[1])
        rospy.loginfo(f"PID Control: Target XY={target_xy}, Current XY={current_xy}")
        if control_y == None and control_x == None:
            self.adjust_xy_flag = False
            self.current_xy = [0.0, 0.0]
            self.target_xy = [0.0, 0.0]
            self.pid_x.prev_error = 0.0
            self.pid_y.prev_error = 0.0
            self.pid_x.integral = 0.0
            self.pid_y.integral = 0.0
            self.last_time = None
            self.right_adjust_xy_complete_pub.publish(Bool(data=True))
            rospy.loginfo("PID XY Control: Target reached. Stopping control.")
            return
        
        if control_x is not None:
            control_x = float(control_x)
        else:
            control_x = 0.0

        if control_y is not None:
            control_y = float(control_y)
        else:
            control_y = 0.0

        try:
            (trans, rot) = self.listener.lookupTransform(self.odom, self.arm_base_link, rospy.Time(0))
            self.base_to_odom = [trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Unable to get base pose from tf. Check if the transform is available.")
        rot = [rot[0], rot[1], rot[2], rot[3]]
        rotation_matrix = quaternion_matrix(rot)[:3, :3]
        # Velocity in odom frame
        v_odom = np.array([control_x, control_y, 0.0])  # Include z=0 for 3D transformation

        # Transform velocity to base_frame
        v_base = np.dot(rotation_matrix.T, v_odom)  # Use transpose for inverse rotation

        # Extract planar velocities in base_frame
        control_x_base = v_base[0]
        control_y_base = v_base[1]



        twist = Twist()
        twist.linear.x = control_x_base 
        twist.linear.y = control_y_base
        twist.angular.z = 0.0  # No rotation during XY control
        self._pub_cmd.publish(twist)
        try:
            (trans, rot) = self.listener.lookupTransform(self.odom, self.arm_base_link, rospy.Time(0))
            self.base_pose = [trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Unable to get base pose from tf. Check if the transform is available.")
        self.current_xy = self.base_pose[:2]
        

        rospy.loginfo(f"PID XY Control: X={control_x_base }, Y={control_y_base}")

    def _pid_rotation_control(self, target_orientation, current_orientation):
        """
        PID control for rotation (yaw).
        """
        control_rotation = self.pid_rotation.rot_compute_control(target_orientation, current_orientation)
        rospy.loginfo(f"PID Rotation Control: Target Orientation={target_orientation}, Current Orientation={current_orientation}")
        if control_rotation is None:
            self.last_time = None
            self.adjust_orientation_flag = False
            self.current_orientation = 0.0
            self.pid_rotation.prev_error = 0.0
            self.pid_rotation.integral = 0.0
            self.target_orientation = 0.0
            self.right_adjust_rotate_complete_pub.publish(Bool(data=True))
            rospy.loginfo("PID Rotation Control: Target reached. Stopping control.")
            return

        if control_rotation is not None:
            control_rotation = float(control_rotation)
        else:
            control_rotation = 0.0

        twist = Twist()
        twist.linear.x = 0.0  # No linear movement during rotation control
        twist.linear.y = 0.0
        twist.angular.z = control_rotation
        self._pub_cmd.publish(twist)
        try:
            (trans, rot) = self.listener.lookupTransform(self.odom, self.arm_base_link, rospy.Time(0))
            self.base_pose = [trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Unable to get base pose from tf. Check if the transform is available.")
        current_orientation = self.base_pose[3:]
        self.current_orientation = euler_from_quaternion(current_orientation)[2]

        rospy.loginfo(f"PID Rotation Control: Z={control_rotation}")

    def quaternion_multiply_rviz(self, q1, q2):
            """Multiply two quaternions in (x, y, z, w) order (RViz convention)."""
            x1, y1, z1, w1 = q1
            x2, y2, z2, w2 = q2
            
            w_r = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
            x_r = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
            y_r = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
            z_r = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
            
            return np.array([x_r, y_r, z_r, w_r])

    ## -------------- Callback Functions -------------- ## 
    def __target_frame_callback(self, msg):
        rospy.loginfo(f"Received target frame: {msg.data}")
        self.target_frame = msg.data

    def __right_adjust_xy_callback(self, msg):
        rospy.loginfo(f"Received XY adjustment command: {msg.data}")
        self.adjust_xy_flag = True
        try:
            (trans, rot) = self.listener.lookupTransform(self.odom, self.target_frame, rospy.Time(0))
            self.goal_pose = [trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Unable to get goal pose from tf. Check if the transform is available.")
            return
        self.target_xy = [self.goal_pose[0], self.goal_pose[1]]
        rospy.loginfo(f"Target XY: {self.target_xy}")
        try:
            (trans, rot) = self.listener.lookupTransform(self.odom, self.arm_base_link, rospy.Time(0))
            self.base_to_odom = [trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Unable to get base pose from tf. Check if the transform is available.")
        rot = [rot[0], rot[1], rot[2], rot[3]]
        rospy.loginfo(f"Base to Odom: {self.base_to_odom}")
        rotation_matrix = quaternion_matrix(rot)[:3, :3]
        rospy.loginfo(f"Rotation Matrix: {rotation_matrix}")
        adjust_in_base_frame = np.array([-0.8, 0.25, 0.0])  # Include z=0 for 3D transformation
        adjust_in_odom_frame = np.dot(rotation_matrix, adjust_in_base_frame)
        self.target_xy[0] = self.target_xy[0] + adjust_in_odom_frame[0]
        self.target_xy[1] = self.target_xy[1] + adjust_in_odom_frame[1]
        rospy.loginfo(f"Adjusted Target XY: {self.target_xy}")

    def __right_adjust_orientation_callback(self, msg):
        rospy.loginfo(f"Received orientation adjustment command: {msg.data}")
        self.adjust_orientation_flag = True
        try:
            (trans, rot) = self.listener.lookupTransform(self.odom, self.target_frame, rospy.Time(0))
            self.goal_pose_rotate = [trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Unable to get goal pose from tf. Check if the transform is available.")
            return
        target_orientation = self.goal_pose_rotate[3:]
        target_orientation = self.quaternion_multiply_rviz(target_orientation, self.rotation_quaternion)
        self.target_orientation = euler_from_quaternion(target_orientation)[2]

    def __right_current_above_task_callback(self, msg):
        self.right_above_desk = msg.data
    
    def __left_current_above_task_callback(self, msg):
        self.left_above_desk = msg.data

    def __right_collision_callback(self, msg):
        if msg.data == True:
            self.right_move_collision = True
        else:
            self.right_move_collision = False
    
    def __left_collision_callback(self, msg):
        if msg.data == True:
            self.left_move_collision = True
        else:
            self.left_move_collision = False

    def __right_base_fast_domain_callback(self, msg):
        if msg.data:
            self._forward_rate = self.base_speed[1]
            self.base_domain = True
        else:
            self._forward_rate = self.base_speed[0]
            self.base_domain = False

    def __scan_callback(self, data):
        # Store the received scan data
        self.scan_data = data
        if self.scan_data is not None:
            # Convert scan data to numpy array
            self.ranges = np.array(self.scan_data.ranges)

    def __left_input_buttons_callback(self, msg):
        """
            determine if it is squeezing or button press 

        """
        # if msg.buttons[0] == 1:
        #     self._torso_increment(0.05)  # Increment for torso up

        if (msg.buttons[2] == 1.0) and (not msg.axes[0] == 0.0 or not msg.axes[1] == 0.0):
            self._base_control(msg)
        elif self.adjust_xy_flag:
            self._pid_control(self.target_xy, self.current_xy)
        else:
            self._linear = 0.0
            self._linear_y = 0.0
            array_data = Float32MultiArray()
            array_data.data = [0, 0]
            self._pub_move_base.publish(array_data)
            # self._publish()


    def __right_input_buttons_callback(self, msg):
        """
            control mobile base using the trackpad on the left controller

        """
        if (msg.buttons[2] == 1.0) and (abs(msg.axes[0]) > 0.1 or abs(msg.axes[1]) > 0.1):
            if(abs(msg.axes[0]) > abs(msg.axes[1]) + 0.2):
                self._base_rotate(msg)

            elif(abs(msg.axes[1]) > abs(msg.axes[0]) + 0.2):
                if msg.buttons[2] == 1.0 and not (self.right_above_desk == 2.0 or self.left_above_desk == 2.0):
                    self.torso_pressed = True
                    if self.right_above_desk or self.left_above_desk:
                        if np.sign(msg.axes[1]) >0:
                            self._torso_increment(np.sign(msg.axes[1]) * 0.005)  # Increment for torso up
                    else:
                        self._torso_increment(np.sign(msg.axes[1]) * 0.005) 
                    # rospy.sleep(0.2)
                # if msg.buttons[2] == 0.0:
        elif self.adjust_orientation_flag:
            self._pid_rotation_control(self.target_orientation, self.current_orientation)
        else:
            self.torso_pressed = False

            self._angular = 0.0
        ## menu button pressed -> home 
        # if msg.buttons[0] == 1:
        #     self,home_robot_arms()
            # self._torso_increment(-0.05)  # Increment for torso up




    def __joint_states_callback(self, msg):

        self.torso_joint = msg.position[20]
        '''
        if self.torso_joint >= self.torso_thres:
            self._forward_rate = self.base_speed[0]
            #rospy.logwarn("torso extended, base slow down")
        else:
            self._forward_rate = self.base_speed[1]
        '''
        

    '''
    def __right_arm_extend_callback(self, msg):
        if msg.data == True and self._forward_rate == self.base_speed[1]:
            self._forward_rate = self.base_speed[0]
            self.right_extended = True
            rospy.logwarn("right arm extended, base slow down")

        if msg.data == False:
            self.right_extended = False

    def __left_arm_extend_callback(self, msg):
        if msg.data == True and self._forward_rate == self.base_speed[1]:
            self._forward_rate = self.base_speed[0]
            self.left_extended = True
            rospy.logwarn("right arm extended, base slow down")
        if msg.data == False:
            self.left_extended = False
    '''
    def __right_adjust_z_callback(self, msg):
        rospy.loginfo(f"Received Z adjustment command: {msg.data}")
        
        # Attempt to increment the torso and check if it was successful
        if self._torso_increment(msg.data):
            rospy.loginfo("Torso adjustment completed. Publishing success flag.")
            self.right_adjust_z_complete_pub.publish(Bool(data=True))  # Publish completion flag
        else:
            rospy.logwarn("Torso adjustment failed or timed out.")
            self.right_adjust_z_complete_pub.publish(Bool(data=False))  # Publish failure flag




    def __head_motion_callback(self, msg):
        """
            control mobile base using the trackpad on the left controller

        """

        if not self.initialized:
            self.initialized = True
            self.filtered_orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]


        # Apply low-pass filter for head motion
        filtered_orientation_x = self.alpha * msg.pose.orientation.x + (1 - self.alpha) * self.filtered_orientation[0]
        filtered_orientation_y = self.alpha * msg.pose.orientation.y + (1 - self.alpha) * self.filtered_orientation[1]
        filtered_orientation_z = self.alpha * msg.pose.orientation.z + (1 - self.alpha) * self.filtered_orientation[2]
        filtered_orientation_w = self.alpha * msg.pose.orientation.w + (1 - self.alpha) * self.filtered_orientation[3]

        roll, pitch, yaw = euler_from_quaternion([filtered_orientation_x, filtered_orientation_y, filtered_orientation_z, filtered_orientation_w])
        # roll, pitch, yaw = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])


        self.head_orientation = [round(self.bound(yaw, self.head_joint_limits[0][0], self.head_joint_limits[0][1]), 2),
                            round(self.bound(-pitch-0.52, self.head_joint_limits[1][0], self.head_joint_limits[1][1]), 2)]

        self.filtered_orientation = [filtered_orientation_x, filtered_orientation_y, filtered_orientation_z, filtered_orientation_w]
        # rospy.logwarn("head orientation = %s, %s",round(math.degrees(head_orientation[0])), round(math.degrees(head_orientation[1])))

        # self._send_head_goal(head_orientation)
        


    ## -------------- Helper Functions -------------- ## 

    def bound(self, low, high, value):
         return max(low, min(high, value))

    def _base_control(self, joy_msg):
        linear = joy_msg.axes[1]
        linear_y = -joy_msg.axes[0]
        angle_dir = math.atan2(linear_y, linear)
        linear = np.cos(angle_dir) * self._forward_rate
        linear_y = np.sin(angle_dir) * self._forward_rate
        for i in range(0, len(self.ranges), 3):
            distance = self.ranges[i]
            if distance < self.lidar_warning_range:
                # print(self.scan_data.angle_min)
                angle =  self.scan_data.angle_increment * i
                x = distance * np.sin(angle)
                y = distance * np.cos(angle)
                # print(x, y)
                if distance < self.collision_threshold:
                    if abs(x) > abs(y):
                        if linear_y * x < 0:
                            linear_y = 0.0
                    elif abs(y) >= abs(x):
                        if linear * y < 0:
                            linear = 0.0
                    # rospy.logwarn("collision detected: %s, (x, y) = %s, %s", distance, x, y)
                    # break
        array_data = Float32MultiArray()
        array_data.data = [linear, linear_y]
        self._pub_move_base.publish(array_data)
        if (not self.right_move_collision) and (not self.left_move_collision):
            self._linear = linear
            self._linear_y = linear_y
        else:
            self._linear = 0.0
            self._linear_y = 0.0


    def _base_rotate(self, msg):
        if self.base_domain:
            angular = -np.sign(msg.axes[0])*self.rotate_speed[1]
        else:
            angular = -np.sign(msg.axes[0])*self.rotate_speed[0]
        self._angular = angular
    


    def _send_head_goal(self, head_position):

        if not ((abs(self.prev_head_pos[0] - head_position[0]) <= 0.01) and (abs(self.prev_head_pos[1] - head_position[1]) <= 0.01)):

            # Create a trajectory point with the adjusted head position
            point = JointTrajectoryPoint()
            point.positions = head_position
            point.velocities = [abs(self.prev_head_pos[0] - head_position[0]), abs(self.prev_head_pos[1] - head_position[1])]  # Set desired joint velocities
            point.time_from_start = rospy.Duration(0.8)  # A small duration for smooth motion

            # Create and publish the head pose
            head_goal = JointTrajectory()
            head_goal.joint_names = ["head_1_joint", "head_2_joint"]
            head_goal.points = [point]
            head_goal.header.stamp = rospy.Time.now()
            self.head_pub.publish(head_goal)

        else:
            pass

        self.prev_head_pos = head_position

    def _torso_increment(self, increment_value):
        goal = TTIG()
        goal.increment_by = [increment_value]
        if self.torso_joint >= self.torso_limit:

            self.torso_client.send_goal(goal)
        else:
            rospy.logwarn("Torso: lower limit")
            if increment_value > 0.0:
                self.torso_client.send_goal(goal)

        finished_before_timeout = self.torso_client.wait_for_result(rospy.Duration(10))  # Timeout after 10 seconds

        if not finished_before_timeout:
            rospy.logwarn("Torso increment action did not complete before timeout.")
            return False  # Indicate that the goal did not complete successfully

        rospy.loginfo("Torso increment action completed successfully.")
        return True

    def _get_twist(self, linear, linear_y, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.linear.y = linear_y
        twist.angular.z = angular
        return twist

    def _publish(self):

        twist = self._get_twist(self._linear, self._linear_y, self._angular)
        self._pub_cmd.publish(twist)
        self._send_head_goal(self.head_orientation)


    def run(self):
        # rate = rospy.Rate(self._hz)
        self._running = True

        while self._running and not rospy.is_shutdown():
            # rospy.logwarn("%s", self.__get_arm_left_transformation())
            if self.base_domain:
                self._forward_rate = self.base_speed[1]
                #rospy.logwarn("base speed up")
            
            if self.adjust_orientation_flag:
                self._pid_rotation_control(self.target_orientation, self.current_orientation)
            elif self.adjust_xy_flag:
                self._pid_control(self.target_xy, self.current_xy)
            else:
                self._publish()

            rospy.sleep(0.01)
            # rate.sleep()



## -------------- Node Destructor -------------- ## 

def node_shutdown():
    rospy.wait_for_service('/pal_startup_control/start')
    try:
        start_service = rospy.ServiceProxy('/pal_startup_control/start', StartupStart)
        request = StartupStartRequest()
        request.app = 'head_manager'
        response = start_service(request)
        # rospy.logwarn("%s", request)
        # rospy.logwarn("%s", response)

        print('\nhead_manager started back up\n')
    except rospy.ServiceException as e:
        print('\nFailed to start head_manager\n')



def node_shutdown_sim():
    print('\nvive_teleop has been shutdown\n')


## -------------- Main Function -------------- ## 

def main():
    try:
        rospy.init_node('vive_teleop')

        args = rospy.myargv(argv=sys.argv)
        sim = args[1]

        if sim == "true":
            sim_param = True
            rospy.on_shutdown(node_shutdown_sim)
        elif sim == "false":
            sim_param = False
            rospy.on_shutdown(node_shutdown)

        app = JoystickTeleop(sim=sim_param)
        app.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

