#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

def publish_transform():
    rospy.init_node('my_transform_publisher')
    broadcaster = tf2_ros.TransformationBroadcaster()

    while not rospy.is_shutdown():
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'laser_frame'
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0