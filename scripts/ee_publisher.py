
"""
This script is used for publishing the end-effector tf transforms.

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

import rospy
import math
import numpy as np
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from numpy import linalg as LA
import tf

import sys

class TiagoArmEEChecker():
    def __init__(self,
                 controller_side='right'):
        self.controller_side = controller_side  
        rospy.init_node('check_'+controller_side+'ee_loc')
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.arm_extend_pub = rospy.Publisher('/'+ controller_side +'/arm_extend', Bool, queue_size=1)

    def run(self):

        while not rospy.is_shutdown():
            try:

                trans = self.tfBuffer.lookup_transform('arm_'+self.controller_side+'_1_link', 'arm_'+self.controller_side+'_tool_link', rospy.Time())

                translation = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])

                # Calculate the norm (magnitude) of the translation vector
                ee_norm = LA.norm(translation)

                ext_msg = Bool()
                ext_msg.data = ee_norm > 0.7
                # if ee_norm > 0.7:
                    # rospy.logwarn("norm: %s", ee_norm)
                self.arm_extend_pub.publish(ext_msg)
                rospy.sleep(0.01)


            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # rate.sleep()
                continue
        

## -------------- Main Function -------------- ## 
def main():
    try:
        args = rospy.myargv(argv=sys.argv)
        controller_side = args[1]

        # rospy.init_node('tiago_arm'+controller_side+'_position_control')
        rospy.init_node('check_'+controller_side+'ee_loc')
        app = TiagoArmEEChecker(controller_side=controller_side)
        app.run()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
