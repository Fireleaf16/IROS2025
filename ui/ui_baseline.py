import threading
import cv2
import os
import numpy as np
import rospy
import tf2_ros
import tf.transformations
import math
import cv2.aruco as aruco
from sensor_msgs.msg import Joy, Image, JointState, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped, Pose, Twist
from std_msgs.msg import String, Float64, Bool, Int32
from tf.transformations import euler_from_quaternion
import argparse
import time
import datetime



# Parse command-line arguments
parser = argparse.ArgumentParser(description='Run the script with optional timer.')
parser.add_argument('--timer', action='store_true', help='Include to run with a timer')
args = parser.parse_args()

class videoThread(threading.Thread):
    def __init__(self, threadID, name, ip_addr):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.rgb_img = np.zeros((480, 640, 3), np.uint8) # (960, 540, 3)




        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.torso_state = 0.0

        self.head_current_state = 0.0

        self.right_act = False
        self.left_act = False
        self.right_shrink_warnings = False
        self.left_shrink_warnings = False
        self.torso_pressed = False
        self.right_status_text = "Pause"
        self.right_status_color = (100, 100, 255)
        self.left_status_text = "Pause"
        self.left_status_color = (100, 100, 255)
        # Initialize alpha value
        self.alpha = 0.5
        self.prev_blended = None

        self.torso_height_bar_length = 15  # Length of the bar
        self.torso_height_bar_thickness = 15  # Thickness of the bar
        self.bar_color = (0, 255, 0)  # Color of the bar
        self.torso_height = 0.0
        self.min_torso_height = 0.05  # Minimum torso height
        self.max_torso_height = 0.35  # Maximum torso height

        self.object_frames = [
            {"frame": "shelf_left_3", "length": 0.35, "heights": [0.02, 0.37, 0.72, 1.07,1.42]},
            {"frame": "front_desk", "length": 1.4, "heights": [0.7]},
            {"frame": "front_desk2", "length": 1.4, "heights": [0.7]},
            {"frame": "back_desk", "length": 1.0,"heights": [0.2]},
            {"frame": "right_back_desk", "length": 0.5,"heights": [0.62]}
        ]
        self.object_facing_detect = 0.78


        # Initialize warning state
        self.warning_color = (0, 0, 255)  # Red color for warning
        self.warning_color_mid = (0, 255, 255)  # Yellow color for mid warning
        self.warning_thickness = 5  # Thickness of the warning lines
        self.warning_thickness_mid = 2  # Thickness of the warning lines
        self.warning_flash_interval = 20  # Flashing interval (in frames)
        # Number of sections
        self.num_sections = 8
        self.warnings = [False] * self.num_sections
        self.warnings_mid = [False] * self.num_sections

        # Section angles (in radians)
        self.section_angles = np.linspace(-np.pi, np.pi, self.num_sections + 1)
        # Counter for flashing effect
        self.flash_counter = 0

        # LiDAR data
        self.scan_data = None
        self.point_cloud = None
        self.point_cloud_lock = threading.Lock()

        # Initialize LiDAR parameters
        self.lidar_max_range = 5.0  # Maximum range of the LiDAR in meters
        self.collision_threshold = 0.55  # Threshold distance for collision detection in meters
        self.collision_threshold_mid = 1.0  # Threshold distance for collision detection in meters
        self.ranges = np.array([])

        self.distances = [np.array([]) for _ in range(self.num_sections)]
        self.lidar_warning_range = 1.5  # Range of the LiDAR warning in pixels
        self.lidar_warning_scale = 40

        self.left_arm_warnings = False
        self.right_arm_warnings = False

        self.leftup = False
        self.leftdown = False

        self.rightup = False
        self.rightdown = False

        self.left_forward = False
        self.right_forward = False

        self.right_home = False
        self.left_home = False


        self.right_cross = False
        self.left_cross = False

        self.left_text_pos = [130, 330]
        self.right_text_pos = [420, 330]
        self.started = False
        self.init_time = time.time()
        self.time_limit = 300
        self.time_warning = 120
        self.time_color = (51, 255, 255)
        self.time_up = False
        self.timer_pos = (210, 100)

        self.left_target_collsion = False
        self.right_target_collsion = False

        # self.cap_rs2 = cv2.VideoCapture(
        #     'v4l2src device=/dev/video0 ! videoscale ! video/x-raw,width=640,height=480,framerate=30/1 ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
        # self.cap_rs3 = cv2.VideoCapture(
        #     'v4l2src device=/dev/video10 ! videoscale ! video/x-raw,width=1600,height=900,framerate=30/1 ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)

        
        self.out_send = cv2.VideoWriter(
           'appsrc! videoconvert ! video/x-raw,format=YUY2 ! jpegenc! rtpjpegpay ! udpsink host=192.168.0.173 port=2337 sync=false',
            cv2.CAP_GSTREAMER, 0, 30, (640, 480)) # (960, 540)
        
        # self.out_send = cv2.VideoWriter(
        #     'appsrc ! videoconvert ! video/x-raw,format=I420 ! x264enc tune=zerolatency bitrate=500 speed-preset=ultrafast ! rtph264pay ! udpsink host=192.168.0.173 port=2336 sync=false',
        #     cv2.CAP_GSTREAMER, 0, 30, (640, 480))

        
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use lowercase 'mp4v' instead of 'MP4V'

        self.out_save = None
        
        self.running = True

        self.right_elbow_collsion = False
        self.left_elbow_collsion = False
        self.right_move_collision = False
        self.left_move_collision = False
        self.right_above_desk = 0.0
        self.left_above_desk = 0.0
        self.base_domain = False

        rospy.init_node('user_interface')
        # KINOVA eye-in-hand camera ########################
        rospy.Subscriber('/xtion/rgb/image_raw', Image, self.callback_image)
        # control state ########################
        #rospy.Subscriber('/robot_activation', Float64, self.__tracking_button_callback, queue_size=1)
        rospy.Subscriber('/right/robot_activation', Float64, self.__right_activation_callback, queue_size=1)
        rospy.Subscriber('/left/robot_activation', Float64, self.__left_activation_callback, queue_size=1)
        # controller feedback
        #rospy.Subscriber('/Right_Hand', TransformStamped, self.__input_pose_callback)
        rospy.Subscriber('/joint_states', JointState, self.__joint_states_callback)
        rospy.Subscriber("/scan", LaserScan, self.__scan_callback)

        rospy.Subscriber('/left/arm_extend', Bool, self.__left_arm_extend_callback)
        rospy.Subscriber('/right/arm_extend', Bool, self.__right_arm_extend_callback)

        rospy.Subscriber('/left/arm_shrink', Bool, self.__left_arm_shrink_callback)
        rospy.Subscriber('/right/arm_shrink', Bool, self.__right_arm_shrink_callback)

        rospy.Subscriber('/right/arm_forward', Bool, self.__arm_right_forward_callback)
        rospy.Subscriber('/left/arm_forward', Bool, self.__arm_left_forward_callback)

        rospy.Subscriber('/right/arm_updown', Float64, self.__right_updown_callback)
        rospy.Subscriber('/left/arm_updown', Float64, self.__left_updown_callback)

        rospy.Subscriber('/right/arm_cross', Bool, self.__right_arm_cross_callback)
        rospy.Subscriber('/left/arm_cross', Bool, self.__left_arm_cross_callback)

        rospy.Subscriber('/Right_Buttons', Joy, self.__right_input_buttons_callback)
        rospy.Subscriber('/Left_Buttons', Joy, self.__left_input_buttons_callback)
        rospy.Subscriber('/Task_Num', Int32, self.__task_num_callback)
        rospy.Subscriber('/Control_method', Int32, self.__control_method_callback)
        rospy.Subscriber('/Test_number', Int32, self.__test_num_callback)


        rospy.Subscriber('/Timestamp', String, self.__timestamp_callback)

        rospy.Subscriber('/right/home', Bool, self.__right_home_callback)
        rospy.Subscriber('/left/home', Bool, self.__left_home_callback)



        rospy.Subscriber('/mobile_base_controller/cmd_vel', Twist, self.__cmd_vel_callback)
        rospy.Subscriber('/head_current_state', Float64, self.__head_callback)

        rospy.Subscriber('/right/target_collision', Bool, self.__right_collsion_callback)
        rospy.Subscriber('/left/target_collision', Bool, self.__left_collsion_callback)

        rospy.Subscriber('right/elbow_collision', Bool, self.__right_elbow_collsion_callback)
        rospy.Subscriber('left/elbow_collision', Bool, self.__left_elbow_collsion_callback)
        rospy.Subscriber('right/base_fast_domain', Bool, self.__right_base_fast_domain_callback)

        rospy.Subscriber('right/move_collision', Bool, self.__right_collision_callback)
        rospy.Subscriber('left/move_collision', Bool, self.__left_collision_callback)

        rospy.Subscriber('/right/current_above_desk', Float64, self.__right_current_above_task_callback)
        rospy.Subscriber('/left/current_above_desk', Float64, self.__left_current_above_task_callback)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    ## ------------------- Callback functions ------------------- ##
    def callback_image(self, data):
        # global rgb_img
        self.rgb_img = CvBridge().imgmsg_to_cv2(data, "bgr8")

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
            self.base_domain = True
        else:
            self.base_domain = False

    def __right_home_callback(self, msg):
        if msg.data:
            self.right_home = True
        else:
            self.right_home = False

    def __left_home_callback(self, msg):
        if msg.data:
            self.left_home = True
        else:
            self.left_home = False

    def __scan_callback(self, data):
        # Store the received scan data
        self.scan_data = data
        if self.scan_data is not None:
            # Convert scan data to numpy array
            self.ranges = np.array(self.scan_data.ranges)

    def __head_callback(self, msg):
        self.head_current_state = msg.data

    def __joint_states_callback(self, msg):

        self.torso_height = msg.position[20]

    def __cmd_vel_callback(self, msg):
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        self.angular_z = msg.angular.z
        #rospy.loginfo(f"Received cmd_vel: linear_x={self.linear_x}, linear_y={self.linear_y}, angular_z={self.angular_z}")

    def __torso_states_callback(self, msg):
        self.torso_state = msg.data

    ## -------------------elbow Callback functions ------------------- ##
    def __right_elbow_collsion_callback(self, msg):
        if msg.data:
            self.right_elbow_collsion = True
        else:
            self.right_elbow_collsion = False
    
    def __left_elbow_collsion_callback(self, msg): 
        if msg.data:
            self.left_elbow_collsion = True
        else:
            self.left_elbow_collsion = False
    ##------------------- Arm feedback -------------------##
    def __right_collsion_callback(self, msg):
        if msg.data:
            self.right_target_collsion = True
        else:
            self.right_target_collsion = False
    
    def __left_collsion_callback(self, msg):
        if msg.data:
            self.left_target_collsion = True
        else:
            self.left_target_collsion = False

    def __right_arm_cross_callback(self, msg):
        if msg.data:
            self.right_cross = True
        else:
            self.right_cross = False

    def __left_arm_cross_callback(self, msg):
        if msg.data:
            self.left_cross = True
        else:
            self.left_cross = False

    def __arm_left_forward_callback(self, msg):
        if msg.data == True:
            self.left_forward = True
        else:
            self.left_forward = False

    def __arm_right_forward_callback(self, msg):
        if msg.data == True:
            self.right_forward = True
        else:
            self.right_forward = False


    def __left_arm_extend_callback(self, msg):
        if msg.data:
            self.left_arm_warnings = True
        else:
            self.left_arm_warnings = False

    def __right_arm_extend_callback(self, msg):
        if msg.data:
            self.right_arm_warnings = True
        else:
            self.right_arm_warnings = False

    def __left_arm_shrink_callback(self, msg):
        if msg.data:
            self.left_shrink_warnings = True
        else:
            self.left_shrink_warnings = False
    
    def __right_arm_shrink_callback(self, msg):
        if msg.data:
            self.right_shrink_warnings = True
        else:
            self.right_shrink_warnings = False

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


    def __right_activation_callback(self, msg):
        if msg.data == 2.0 and not self.right_act:
            self.right_act = True
            self.right_status_text = "Controlling"
            self.right_status_color = (0, 255, 0)

        if msg.data == 0.0 and self.right_act:
            self.right_act = False
            self.right_status_text = "Pause"
            self.right_status_color = (100, 100, 255)

    def __left_activation_callback(self, msg):
        if msg.data == 2.0 and not self.left_act:
            self.left_act = True
            self.left_status_text = "Controlling"
            self.left_status_color = (0, 255, 0)
        if msg.data == 0.0 and self.left_act:
            self.left_act = False
            self.left_status_text = "Pause"
            self.left_status_color = (100, 100, 255)

    def __right_input_buttons_callback(self, msg):
        for button_state in msg.buttons:
            if button_state == 1 and not self.started:  # 1 is typically the state for a button press
                self.started = True
                self.init_time = time.time()
                break  # No need to check the rest of the buttons

        if (msg.buttons[2] == 1.0) and (abs(msg.axes[0]) > 0.1 or abs(msg.axes[1]) > 0.1):
            if(abs(msg.axes[1]) > abs(msg.axes[0]) + 0.2):
                if msg.buttons[2] == 1.0 and not self.torso_pressed:
                    self.torso_pressed = True
                    self.torso_state = np.sign(msg.axes[1])  # Increment for torso up
                    # rospy.sleep(0.2)
                # if msg.buttons[2] == 0.0:
        else:
            self.torso_pressed = False
            self.torso_state = 0.0
    
    def __left_input_buttons_callback(self, msg):
        for button_state in msg.buttons:
            if button_state == 1 and not self.started:  # 1 is typically the state for a button press
                self.started = True
                self.init_time = time.time()

                break  # No need to check the rest of the buttons
    
    ###------------------- object detection -------------------###

    def get_yaw_from_quaternion(self,quat):
        # Convert quaternion to Euler angles and extract the yaw
        euler = tf.transformations.euler_from_quaternion(quat)
        return euler[2]  # Yaw

    def angle_between_vectors(self, v1, v2):
        unit_v1 = v1 / np.linalg.norm(v1)
        unit_v2 = v2 / np.linalg.norm(v2)
        dot_product = np.dot(unit_v1, unit_v2)
        return np.arccos(dot_product)           


    def __get_transformation(self, parent, child):
        try:
            trans = self.tfBuffer.lookup_transform(parent, child, rospy.Time(0))
            pose_message = Pose()
            pose_message.position.x = trans.transform.translation.x
            pose_message.position.y = trans.transform.translation.y
            pose_message.position.z = trans.transform.translation.z

            pose_message.orientation.x = trans.transform.rotation.x
            pose_message.orientation.y = trans.transform.rotation.y
            pose_message.orientation.z = trans.transform.rotation.z
            pose_message.orientation.w = trans.transform.rotation.w

            return pose_message
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def find_facing_object(self, robot_pose):
        # Robot's position in the world frame
        robot_position = np.array([robot_pose.position.x, robot_pose.position.y])

        # Robot's orientation in quaternion (x, y, z, w)
        robot_orientation = [
            robot_pose.orientation.x,
            robot_pose.orientation.y,
            robot_pose.orientation.z,
            robot_pose.orientation.w
        ]

        # Convert the quaternion to yaw (rotation around the z-axis)
        yaw = self.get_yaw_from_quaternion(robot_orientation)

        # Robot's direction vector based on its yaw
        robot_direction = np.array([np.cos(yaw), np.sin(yaw)])

        min_angle = np.deg2rad(30)
        facing_object = None

        for obj in self.object_frames:
            obj_frame = obj["frame"]
            #rospy.logwarn("Object frame: %s", obj_frame)
            obj_length = obj["length"]
            obj_heights = obj["heights"]
            obj_pose = self.__get_transformation('odom', obj_frame)
            #rospy.loginfo("Object pose: %s", obj_pose)
            if obj_pose is not None:
                # Object's position in the world frame
                obj_position = np.array([obj_pose.position.x, obj_pose.position.y])

                # Vector from the robot to the object
                obj_vector = obj_position - robot_position

                # Calculate the angle between the robot's direction vector and the object vector
                angle = self.angle_between_vectors(robot_direction, obj_vector)
                #rospy.loginfo("Angle between robot and object: %f", angle)
                # Find the object with the smallest angle to the robot's direction
                if angle < min_angle:
                    min_angle = angle
                    facing_object = {"frame": obj_frame, "length": obj_length,  "heights": obj_heights,"position": obj_position, "angle": angle}

        return facing_object

    def get_frame_angle(self, base_frame, target_frame):
        try: 
            trans = self.tfBuffer.lookup_transform(base_frame, target_frame, rospy.Time())
            x, y, z, w = trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w
            roll, pitch, yaw = euler_from_quaternion([x, y, z, w])
            return yaw
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF lookup failed for frame: {}".format(target_frame))
            return None

    def get_frame_coordinates(self, base_frame, target_frame):
        try:
            trans = self.tfBuffer.lookup_transform(base_frame, target_frame, rospy.Time())
            x, y, z= self.convert_3d_to_2d(trans.transform.translation)
            return (int(x), int(y), int(z))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF lookup failed for frame: {}".format(target_frame))
            return None

    def convert_3d_to_2d(self, translation):
        # Placeholder function to convert 3D coordinates to 2D screen coordinates
        # Implement the conversion based on camera parameters or testing
        x = translation.x * 45  
        y = translation.y * 50
        z = translation.z * 35  
        return x, y, z


    def __task_num_callback(self, msg):
        rospy.logwarn(msg)

        self.task_num = msg.data

    def __control_method_callback(self, msg):
        rospy.logwarn(msg)

        self.control_method = msg.data
    
    def __test_num_callback(self, msg):
        rospy.logwarn(msg)

        self.test_num = msg.data

    def __timestamp_callback(self, msg):
        rospy.logwarn(msg)
        if self.control_method == 0:
            filename = "original"
        elif self.control_method == 1:
            filename = "egocentric"
        elif self.control_method == 2:
            filename = "combined"
        self.vid_name = os.path.expanduser(f"~/ICRA2025/teleop_data/task{self.task_num}/{filename}/test{self.test_num}/ui.mp4")
        self.__make_sure_path_exists(self.vid_name)
        self.out_save = cv2.VideoWriter(self.vid_name, self.fourcc, 30, (640,480))

    def __make_sure_path_exists(self, path):
        directory = os.path.dirname(path)
        if not os.path.exists(directory):
            os.makedirs(directory)


    def draw_warnings(self, frame):
        if self.time_up and self.flash_counter < self.warning_flash_interval:
            cv2.putText(frame, 'Time\'s Up', (150, 200), cv2.FONT_HERSHEY_SIMPLEX, 4, (20, 20, 230), 7, cv2.LINE_AA)

        ## Arm extension warnings

        
        if self.right_home and self.flash_counter < self.warning_flash_interval:
            cv2.putText(frame, 'Home', (self.right_text_pos[0], self.right_text_pos[1]-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

        if self.left_home  and self.flash_counter < self.warning_flash_interval:
            cv2.putText(frame, 'Home', (self.left_text_pos[0], self.left_text_pos[1]-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)


        ## LiDAR warnings
        for i in range(0, len(self.ranges), 3):
            distance = self.ranges[i]
            if distance < self.lidar_warning_range:
                # print(self.scan_data.angle_min)
                angle =  self.scan_data.angle_increment * i
                x = int(distance * 40 * np.sin(angle)) - 8
                y = int(distance * 50 * np.cos(angle))
                # print(x, y)
                if distance < self.collision_threshold:
                    self.warning_color = (0, 0, 255)
                else:
                    self.warning_color = (0, 255, 0)
                # Draw the dot
                cv2.circle(frame, (360+x, 375+y), 2, self.warning_color, 2)


    def draw_rotation_arrow(self, image, center, radius, start_angle, end_angle, color, thickness, flag):
        # Draw the arc
        cv2.ellipse(image, center, (radius, radius), 0, start_angle, end_angle, color, thickness)

        # Calculate the end point of the arc to position the arrow
        end_x = int(center[0] + radius * np.cos(np.radians(end_angle)))
        end_y = int(center[1] - radius * np.sin(np.radians(end_angle)))

        # Calculate the start point of the arrow
        start_x = int(center[0] + radius * np.cos(np.radians(end_angle)))  # Small angle to offset arrow start
        start_y = int(center[1] - radius * np.sin(np.radians(end_angle)))

        # Draw the arrowhead
        if flag == 0:
            cv2.arrowedLine(image, (start_x-5, start_y), (end_x, end_y), color, thickness, tipLength=0.3)
            cv2.arrowedLine(image, (start_x+3, start_y-5), (end_x, end_y), color, thickness, tipLength=0.3)
        else:
            cv2.arrowedLine(image, (start_x+5, start_y), (end_x, end_y), color, thickness, tipLength=0.3)
            cv2.arrowedLine(image, (start_x-3, start_y-5), (end_x, end_y), color, thickness, tipLength=0.3)

    def draw_dashed_line(self, img, pt1, pt2, color, thickness=2, dash_length=1, gap_length=1):
        """
        Draws a dashed line on an image.
        
        :param img: The image on which to draw the line.
        :param pt1: The starting point of the line (x1, y1).
        :param pt2: The ending point of the line (x2, y2).
        :param color: The color of the line.
        :param thickness: The thickness of the line.
        :param dash_length: The length of each dash.
        :param gap_length: The length of the gap between dashes.
        """
        # Calculate the total length of the line
        total_length = int(np.linalg.norm(np.array(pt2) - np.array(pt1)))
        
        # Calculate the number of dashes and gaps
        dashes = int(total_length / (dash_length + gap_length))
        
        # Calculate the vector along the line
        line_vector = np.array(pt2) - np.array(pt1)
        line_vector = line_vector / np.linalg.norm(line_vector)
        
        # Draw each dash
        for i in range(dashes):
            start_point = np.array(pt1) + i * (dash_length + gap_length) * line_vector
            end_point = start_point + dash_length * line_vector
            start_point = tuple(start_point.astype(int))
            end_point = tuple(end_point.astype(int))
            cv2.line(img, start_point, end_point, color, thickness)

    def format_time(self, seconds):
        """
            Function to format seconds into HH:MM:SS
        """
        m, s = divmod(seconds, 60)
        # h, m = divmod(m, 60)
        return "{:02}:{:02}".format(int(m), int(s))
    
    def run(self):
        
        while not rospy.is_shutdown():
            frame_rs1 = self.rgb_img
            robot_pose = self.__get_transformation('odom', 'torso_lift_link')
            facing_object = None

            ##------------- draw lidar warnings -------------##
            # cv2.putText(frame_rs1, 'Base', (332, 377), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (127,127,127), 1, cv2.LINE_AA)
            self.draw_warnings(frame_rs1)
            
            if robot_pose:
                facing_object = self.find_facing_object(robot_pose)
                if facing_object:
                    #rospy.loginfo("Robot is facing: %s", facing_object)
                    a = 1
                else:
                    #rospy.loginfo("No object in front of the robot within 30 degrees.")
                    facing_object = None
            else:
                rospy.logwarn("Robot pose not available.")
            
            robot_base_length = 0.5
            robot_base_pixel_length = 15
            robot_torso_length = 0.46
            robot_torso_pixel_length = 16

            boundary_pixel =  25
            detect_angle = np.deg2rad(30)
            # Right or left detect check:
            if facing_object:
                side_or = self.__get_transformation('torso_lift_link',facing_object["frame"])
                side = np.array([side_or.position.x, side_or.position.y])
                left_boundary = 253 - boundary_pixel
                right_boundary = 253 + boundary_pixel
                if side[0]<1.0:
                    if side[1]<0:
                        center_point_pixel = int(247+15/2+boundary_pixel*facing_object["angle"]/detect_angle)
                    else:
                        center_point_pixel = int(247+15/2-boundary_pixel*facing_object["angle"]/detect_angle)

                    startpoint = max(int(center_point_pixel - facing_object["length"]/2/robot_base_length*robot_base_pixel_length),int(left_boundary))
                    endpoint = min(int(center_point_pixel + facing_object["length"]/2/robot_base_length*robot_base_pixel_length),int(right_boundary))
                    if facing_object["frame"] == "shelf_left_3":
                        for i in range (0,5):
                            shelf_buttom = int(410-facing_object["heights"][0]/robot_torso_length*robot_torso_pixel_length)
                            shelf_top = int(410-facing_object["heights"][4]/robot_torso_length*robot_torso_pixel_length)
                            height = int(410-facing_object["heights"][i]/robot_torso_length*robot_torso_pixel_length)
                            #rospy.logwarn(f"Startpoint: {startpoint}, Endpoint: {endpoint}")
                            #rospy.logwarn(f"Height: {height}")
                            cv2.line(frame_rs1, (startpoint, height), (endpoint, height), (0, 255, 255), 2)
                            #self.draw_dashed_line(frame_rs1, (left_boundary, height), (startpoint, height), (150, 150, 255), thickness=1, dash_length=3, gap_length=3)
                            #self.draw_dashed_line(frame_rs1, (endpoint, height), (right_boundary+5, height), (150, 150, 255), thickness=1, dash_length=3, gap_length=3)
                        if int(center_point_pixel - facing_object["length"]/2/robot_base_length*robot_base_pixel_length) >= int(left_boundary):
                            cv2.line(frame_rs1, (startpoint, shelf_buttom), (startpoint, shelf_top), (0, 255, 255), 2)
                        if int(center_point_pixel + facing_object["length"]/2/robot_base_length*robot_base_pixel_length) <= int(right_boundary):
                            cv2.line(frame_rs1, (endpoint, shelf_buttom), (endpoint, shelf_top), (0, 255, 255), 2)
                    else:
                        height = int(405-facing_object["heights"][0]/robot_torso_length*robot_torso_pixel_length)
                        #rospy.logwarn(f"Startpoint: {startpoint}, Endpoint: {endpoint}")
                        #rospy.logwarn(f"Height: {height}")
                        cv2.line(frame_rs1, (startpoint-15, height), (endpoint+15, height), (0, 255, 255), 2)
                        #self.draw_dashed_line(frame_rs1, (left_boundary, height), (startpoint, height), (150, 150, 255), thickness=1, dash_length=3, gap_length=3)
                        #self.draw_dashed_line(frame_rs1, (endpoint, height), (right_boundary+5, height), (150, 150, 255), thickness=1, dash_length=3, gap_length=3)

            # specify eye-in-hand camera
            #frame_rs1 = self.rgb_img

            ##------------- display control status -------------##
            cv2.putText(frame_rs1, self.right_status_text, (self.right_text_pos[0], self.right_text_pos[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.right_status_color, 2, cv2.LINE_AA)
            cv2.putText(frame_rs1, self.left_status_text, (self.left_text_pos[0], self.left_text_pos[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.left_status_color, 2, cv2.LINE_AA)
            
            ##------------- draw torso status -------------##
            bar_position_y = 365  # Vertical position of the bar
            bar_position_x = 245  # Vertical position of the bar
            base_torso_x = 250
            base_torso_y = 405
            #cv2.rectangle(frame_rs1, (bar_position_x, bar_position_y), (bar_position_x + self.torso_height_bar_thickness, bar_position_y + self.torso_height_bar_length), self.bar_color, 2)
            
            #shoulder line
            line_position_y = int(bar_position_y+self.torso_height_bar_length - ((self.torso_height - self.min_torso_height) / (self.max_torso_height - self.min_torso_height)) * self.torso_height_bar_length)
            
            # buttom line
            buttom_y = line_position_y +18 #shoulder to moveable white buttom height
            
            #cv2.line(frame_rs1, (base_torso_x, base_torso_y), (base_torso_x + self.torso_height_bar_thickness-5, base_torso_y), (0, 0, 0), 3) # base_line
            #cv2.line(frame_rs1, (bar_position_x, buttom_y), (bar_position_x + self.torso_height_bar_thickness-5, buttom_y), (0, 0, 0), 3) #buttom line


            #cv2.line(frame_rs1, (bar_position_x, line_position_y), (bar_position_x + self.torso_height_bar_thickness, line_position_y), (255, 255, 255), 3) # shoulder line
            #rospy.logwarn(f"Torso status drawn at bar_position_x: {bar_position_x}, line_position_y: {line_position_y}")
            

 
            # boundary base fast domain
            base_coords = self.get_frame_coordinates('torso_lift_link','odom')
            yaw = self.get_frame_angle('torso_lift_link','odom')
            #rospy.logwarn(f"Base coords: {base_coords}, Yaw: {yaw}")
            if base_coords and yaw is not None:
                # Rectangle parameters
                left = 0.19
                right = 0.3
                front = 0.42
                back = 0.22
                center_x = 350 - base_coords[1]
                center_y = 375 - base_coords[0]
                scale = 50

                # Original corner points of the rectangle before rotation
                corners = np.array([
                    [center_x - left * scale, center_y - front * scale],  # Top-left
                    [center_x + right * scale, center_y - front * scale],  # Top-right
                    [center_x + right * scale, center_y + back * scale],  # Bottom-right
                    [center_x - left * scale, center_y + back * scale]   # Bottom-left
                ])

                # Rotation matrix for the yaw angle
                rotation_matrix = cv2.getRotationMatrix2D((center_x, center_y), np.degrees(yaw), 1)

                # Apply rotation to the corners
                rotated_corners = cv2.transform(np.array([corners]), rotation_matrix)[0]

                # Draw the rotated rectangle
                cv2.polylines(frame_rs1, [np.int32(rotated_corners)], isClosed=True, color=(160, 16, 160), thickness=2)

                
            ##------------- draw base-------------##
            if not self.base_domain:
                cv2.rectangle(frame_rs1, (340, 360), (360, 390), (178, 178, 178), 5)
                cv2.circle(frame_rs1, (350, 375), 2, (178, 178, 178), -1)
            else:
                cv2.rectangle(frame_rs1, (340, 360), (360, 390), (0,255,0), 5)
                cv2.circle(frame_rs1, (350, 375), 2, (0,255,0), -1)


            green = (0,255,0)
            red = (0,0,255)
            blue = (255,0,0)
            orange = (0,165,255)


            ##------------- arm position -------------##
            center_coordinates = (350, 375)
            right_shoulder_coordinates_top = (360,375)
            left_shoulder_coordinates_top = (340,375)
            current_bar = (bar_position_x+7,line_position_y)


            left_shoulder_coordinates_front = (bar_position_x,line_position_y+3)
            right_shoulder_coordinates_front = (bar_position_x+self.torso_height_bar_thickness,line_position_y+3)



            # Get frame coordinates
            arm_right_4_link_coords = self.get_frame_coordinates('torso_lift_link','arm_right_4_link')
            arm_right_tool_link_coords = self.get_frame_coordinates('torso_lift_link','gripper_right_grasping_frame')
            
            arm_left_4_link_coords = self.get_frame_coordinates('torso_lift_link','arm_left_4_link')
            arm_left_tool_link_coords = self.get_frame_coordinates('torso_lift_link','gripper_left_grasping_frame')



            # Draw frames and connecting line if coordinates are available
            if arm_right_4_link_coords and arm_right_tool_link_coords:
                arm_right_4_link_coords_top = (center_coordinates[0]-arm_right_4_link_coords[1],center_coordinates[1]-arm_right_4_link_coords[0])
                arm_right_tool_link_coords_top = (center_coordinates[0]-arm_right_tool_link_coords[1],center_coordinates[1]-arm_right_tool_link_coords[0])

                arm_right_4_link_coords_front =  (current_bar[0]-arm_right_4_link_coords[1],current_bar[1]-arm_right_4_link_coords[2])
                arm_right_tool_link_coords_front =  (current_bar[0]-arm_right_tool_link_coords[1],current_bar[1]-arm_right_tool_link_coords[2])
                


                # Draw connecting line
                cv2.line(frame_rs1, right_shoulder_coordinates_top, arm_right_4_link_coords_top, (255, 0, 0), 2)  # Blue line
                cv2.line(frame_rs1, arm_right_4_link_coords_top, arm_right_tool_link_coords_top, (255, 0, 0), 2)  # Blue line


                if self.right_target_collsion or self.right_move_collision or self.right_above_desk:
                    cv2.circle(frame_rs1, arm_right_tool_link_coords_front, 5, red, -1)
                else:
                    cv2.circle(frame_rs1, arm_right_tool_link_coords_front, 5, orange, -1)  # orange for arm_tool_link

                cv2.line(frame_rs1, right_shoulder_coordinates_front, arm_right_4_link_coords_front, (255, 0, 0), 2)  # Blue line
                cv2.line(frame_rs1, arm_right_4_link_coords_front, arm_right_tool_link_coords_front, (255, 0, 0), 2)  # Blue line


                # Draw circles at the frame positions
                if self.right_elbow_collsion:
                    cv2.circle(frame_rs1, arm_right_4_link_coords_top, 5, red, -1)
                else:
                    cv2.circle(frame_rs1, arm_right_4_link_coords_top, 5, blue, -1)  # blue for arm_4_link
                if self.right_target_collsion or self.right_move_collision or self.right_above_desk:
                    cv2.circle(frame_rs1, arm_right_tool_link_coords_top, 5, red, -1)
                else:
                    cv2.circle(frame_rs1, arm_right_tool_link_coords_top, 5, orange, -1)  # orange for arm_tool_link

                
                if self.right_elbow_collsion:
                    cv2.circle(frame_rs1, arm_right_4_link_coords_front, 5, red, -1)
                else:
                    cv2.circle(frame_rs1, arm_right_4_link_coords_front, 5, blue, -1)  # blue for arm_4_link
                cv2.circle(frame_rs1, right_shoulder_coordinates_front,3,blue,-1)
            
            if arm_left_4_link_coords and arm_left_tool_link_coords:
                arm_left_4_link_coords_top = (center_coordinates[0]-arm_left_4_link_coords[1],center_coordinates[1]-arm_left_4_link_coords[0])
                arm_left_tool_link_coords_top = (center_coordinates[0]-arm_left_tool_link_coords[1],center_coordinates[1]-arm_left_tool_link_coords[0])

                arm_left_4_link_coords_front =  (current_bar[0]-arm_left_4_link_coords[1],current_bar[1]-arm_left_4_link_coords[2])
                arm_left_tool_link_coords_front =  (current_bar[0]-arm_left_tool_link_coords[1],current_bar[1]-arm_left_tool_link_coords[2])

               
                # Draw connecting line
                cv2.line(frame_rs1, left_shoulder_coordinates_top, arm_left_4_link_coords_top, (255, 0, 0), 2)  # Blue line
                cv2.line(frame_rs1, arm_left_4_link_coords_top, arm_left_tool_link_coords_top, (255, 0, 0), 2)  # Blue line

                if self.left_target_collsion or self.left_move_collision or self.left_above_desk:
                    cv2.circle(frame_rs1, arm_left_tool_link_coords_front, 5, red, -1)
                else:
                    cv2.circle(frame_rs1, arm_left_tool_link_coords_front, 5,orange, -1)  # orange for arm_tool_link
                cv2.line(frame_rs1, arm_left_4_link_coords_front, arm_left_tool_link_coords_front, (255, 0, 0), 2)  # Blue line
                cv2.line(frame_rs1, left_shoulder_coordinates_front, arm_left_4_link_coords_front, (255, 0, 0), 2)  # Blue line


                # Draw circles at the frame positions
                if self.left_elbow_collsion:
                    cv2.circle(frame_rs1, arm_left_4_link_coords_top, 5, red, -1)
                else:
                    cv2.circle(frame_rs1, arm_left_4_link_coords_top, 5, blue, -1)  # blue for arm_4_link
                if self.left_target_collsion or self.left_move_collision or self.left_above_desk:
                    cv2.circle(frame_rs1, arm_left_tool_link_coords_top, 5, red, -1)
                else:
                    cv2.circle(frame_rs1, arm_left_tool_link_coords_top, 5, orange, -1)  # orange for arm_tool_link
                cv2.circle(frame_rs1, left_shoulder_coordinates_front,3,blue,-1)
                
                if self.left_elbow_collsion:
                    cv2.circle(frame_rs1, arm_left_4_link_coords_front, 5, red, -1)
                else:
                    cv2.circle(frame_rs1, arm_left_4_link_coords_front, 5, blue, -1)  # blue for arm_4_link

            # draw fixed shoulder
            cv2.circle(frame_rs1, right_shoulder_coordinates_top, 3, blue, -1)  #blue for arm_right_shouler
            cv2.circle(frame_rs1, left_shoulder_coordinates_top, 3, blue, -1)  # blue for arm_left_shouler




            #draw base and torso
            if self.right_above_desk or self.left_above_desk:
                cv2.rectangle(frame_rs1, (base_torso_x, base_torso_y), (bar_position_x + self.torso_height_bar_thickness-5, buttom_y), red, 2)
            else:
                cv2.rectangle(frame_rs1, (base_torso_x, base_torso_y), (bar_position_x + self.torso_height_bar_thickness-5, buttom_y), (0,50,100), 2)
    
            cv2.rectangle(frame_rs1, (bar_position_x, buttom_y), (bar_position_x + self.torso_height_bar_thickness, line_position_y), (178, 178, 178), 3)

            if self.torso_height > 0.345:
                cv2.line(frame_rs1, (bar_position_x, line_position_y), (bar_position_x + self.torso_height_bar_thickness, line_position_y), red, 3)
            if self.torso_height < 0.075:
                cv2.line(frame_rs1, (bar_position_x, buttom_y), (bar_position_x + self.torso_height_bar_thickness, buttom_y), (0, 0, 255), 3)
            if not self.base_domain:
                cv2.rectangle(frame_rs1, (bar_position_x, 410), (bar_position_x + self.torso_height_bar_thickness, base_torso_y), (178, 178, 178), 2)
            else:
                cv2.rectangle(frame_rs1, (bar_position_x, 410), (bar_position_x + self.torso_height_bar_thickness, base_torso_y), green, 2)

            #draw speed and rotation
            if self.linear_x != 0 or self.linear_y != 0:
                angle = math.atan2(self.linear_y, self.linear_x)
                #rospy.loginfo(f"angle: {angle}")
                start_point = (350, 375)
                # Define the length of the arrow
                arrow_length = 20  # You can adjust the length as needed

                # Calculate the end point based on the angle
                end_point = (int(start_point[0] - arrow_length * math.sin(angle)),int(start_point[1] - arrow_length * math.cos(angle)))
                
                cv2.arrowedLine(frame_rs1, start_point, end_point, red, thickness = 2, tipLength=0.3)

            
            
            if self.torso_state < 0 and self.torso_height > 0.075:
                start_point = (253, line_position_y +15)  # Starting point of the arrow
                end_point = (253,line_position_y +35)  # Ending point of the arrow
                color = red
                thickness = 2  # Thickness of the arrow
                cv2.arrowedLine(frame_rs1, start_point, end_point, color, thickness, tipLength=0.3)
            
            if self.torso_state > 0 and self.torso_height < 0.345:
                start_point = (253,line_position_y +5)  # Starting point of the arrow
                end_point = (253,line_position_y -15)  # Ending point of the arrow
                color = red
                thickness = 2  # Thickness of the arrow
                cv2.arrowedLine(frame_rs1, start_point, end_point, color, thickness, tipLength=0.3)

            ##------------- draw rotation arrow -------------##
            if self.angular_z > 0:
                center = (350, 375)
                radius = 10
                start_angle = 360
                end_angle = 180
                color = red
                thickness = 2
                flag = 1
                # Draw the rotation arrow
                self.draw_rotation_arrow(frame_rs1, center, radius, start_angle, end_angle, color, thickness, flag)
            if self.angular_z < 0:
                center = (350, 375)
                radius = 10
                start_angle = 180
                end_angle = 360
                color = red
                thickness = 2
                flag = 0
                # Draw the rotation arrow
                self.draw_rotation_arrow(frame_rs1, center, radius, start_angle, end_angle, color, thickness, flag)


            ##------------- display time -------------##
            if args.timer:
                if self.started:
                    elapsed = time.time() - self.init_time
                    # Calculate the remaining time
                    remaining = self.time_limit - elapsed
                    if remaining <= 0:
                        remaining = 0
                        self.time_up = True
                    else:
                        if remaining < self.time_warning:
                            self.time_color = (20, 20, 230)
                        formatted_time = self.format_time(remaining)
                        time_text = f"Time: {formatted_time}"
                        cv2.putText(frame_rs1, time_text, self.timer_pos, cv2.FONT_HERSHEY_SIMPLEX, 1, self.time_color, 2, cv2.LINE_AA)
            else:
                pass
                # if self.started:
                #     elapsed = time.time() - self.init_time
                #     formatted_time = self.format_time(elapsed)
                #     time_text = f"Time: {formatted_time}"
                #     cv2.putText(frame_rs1, time_text, self.timer_pos, cv2.FONT_HERSHEY_SIMPLEX, 1, self.time_color, 2, cv2.LINE_AA)

            # display user interface       
            # frame_rs2_small = cv2.resize(frame_rs2, (200, 112), interpolation = cv2.INTER_AREA)

            # Replace the region of frame_rs1 with the blended image
            #frame_rs1[50:162, 150:350] = frame_rs2_small

            # Send out the visual feedback to unity
            
            self.out_send.write(frame_rs1)
            
            
            if self.out_save is not None:
                self.out_save.write(frame_rs1)
    
            cv2.namedWindow('Main View', cv2.WINDOW_GUI_NORMAL)
            cv2.resizeWindow('Main View', 640, 480)
            cv2.imshow('Main View', frame_rs1)
            time.sleep(0.03)

            # cv2.namedWindow('Eye-in-hand View', cv2.WINDOW_GUI_NORMAL)
            # cv2.resizeWindow('Eye-in-hand View', 960, 540)
            # cv2.imshow('Eye-in-hand View', frame_rs2)

            # subscibe keyborad input
            key = cv2.waitKey(50)
            if key & 0xFF == ord('q'):
                break
            
        self.running = False
        # self.cap_rs1.release()
        # self.cap_rs2.release()
        self.out_send.release()
        if self.out_save is not None:

            self.out_save.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    ip_addr = '130.215.181.94'

    v_thread = videoThread(0, 'videoT', ip_addr)
    v_thread.start()
