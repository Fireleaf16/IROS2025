#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def publish_static_transforms():
    rospy.init_node('static_tf_broadcaster')

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Right arm transform
    right_transform = TransformStamped()
    right_transform.header.stamp = rospy.Time.now()
    right_transform.header.frame_id = "arm_right_tool_link"
    right_transform.child_frame_id = "arm_right_toolcenter_frame"
    right_transform.transform.translation.x = 0.2
    right_transform.transform.translation.y = 0.0
    right_transform.transform.translation.z = 0.0
    right_transform.transform.rotation.x = 0.0
    right_transform.transform.rotation.y = 0.0
    right_transform.transform.rotation.z = 0.0
    right_transform.transform.rotation.w = 1.0

    # Left arm transform
    left_transform = TransformStamped()
    left_transform.header.stamp = rospy.Time.now()
    left_transform.header.frame_id = "arm_left_tool_link"
    left_transform.child_frame_id = "arm_left_toolcenter_frame"
    left_transform.transform.translation.x = 0.2
    left_transform.transform.translation.y = 0.0
    left_transform.transform.translation.z = 0.0
    left_transform.transform.rotation.x = 0.0
    left_transform.transform.rotation.y = 0.0
    left_transform.transform.rotation.z = 0.0
    left_transform.transform.rotation.w = 1.0

    # Broadcast both transforms
    broadcaster.sendTransform([right_transform, left_transform])
    rospy.loginfo("Broadcasting static transforms for both arms")
    rospy.spin()

if __name__ == "__main__":
    publish_static_transforms()
