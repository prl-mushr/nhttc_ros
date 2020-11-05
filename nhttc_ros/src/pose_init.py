#!/usr/bin/env python

import rospy
import tf
import tf.transformations
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
import math as m
import time

rospy.init_node("pose_initializer")

def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))

pub1 = rospy.Publisher("/car1/initialpose", PoseWithCovarianceStamped, queue_size=1)
pub2 = rospy.Publisher("/car2/initialpose", PoseWithCovarianceStamped, queue_size=1)
pub3 = rospy.Publisher("/car3/initialpose", PoseWithCovarianceStamped, queue_size=1)
pub4 = rospy.Publisher("/car4/initialpose", PoseWithCovarianceStamped, queue_size=1)

time.sleep(0.1)
now = rospy.Time.now()
cur_pose = PoseWithCovarianceStamped()
cur_pose.header.frame_id = "/map"
cur_pose.header.stamp = now

cur_pose.pose.pose.position.x = 10
cur_pose.pose.pose.position.y = 10
cur_pose.pose.pose.position.z = 0.0
rot = -m.pi
cur_pose.pose.pose.orientation = angle_to_quaternion(rot)
pub1.publish(cur_pose)

cur_pose.pose.pose.position.x = 0
cur_pose.pose.pose.position.y = 10
cur_pose.pose.pose.position.z = 0.0
rot = -m.pi/2
cur_pose.pose.pose.orientation = angle_to_quaternion(rot)
pub2.publish(cur_pose)

cur_pose.pose.pose.position.x = 10
cur_pose.pose.pose.position.y = 0
cur_pose.pose.pose.position.z = 0.0
rot = m.pi/2
cur_pose.pose.pose.orientation = angle_to_quaternion(rot)
pub3.publish(cur_pose)

cur_pose.pose.pose.position.x = 0
cur_pose.pose.pose.position.y = 0
cur_pose.pose.pose.position.z = 0.0
rot = m.pi/4
cur_pose.pose.pose.orientation = angle_to_quaternion(rot)
pub4.publish(cur_pose)