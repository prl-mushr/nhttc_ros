#!/usr/bin/env python
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
import math as m
import time

rospy.init_node("pose_initializer")

count = 0


def angle_to_quaternion(angle):
    """
    Convert an angle in radians into a quaternion _message_.

    Params:
        angle in radians
    Returns:
        quaternion (unit quaternion)
    """
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))


def pose_callback(data, args):
    """
    function for counting the number of cars.

    Params:
        data: car pose
        args: tag/index of the car (0,1,...)
    Returns:
        None
    """
    global count
    if(count < args):
        count = args


pub = []
goal_pub = []
sub = []

for i in range(8):
    subscriber = rospy.Subscriber("/car" + str(i + 1) + "/car_pose", PoseStamped, pose_callback, (i + 1))
    publisher = rospy.Publisher("/car" + str(i + 1) + "/initialpose", PoseWithCovarianceStamped, queue_size=1)
    goal_publisher = rospy.Publisher("/car" + str(i + 1) + "/move_base_simple/goal", PoseStamped, queue_size=1)
    sub.append(subscriber)
    pub.append(publisher)
    goal_pub.append(goal_publisher)

time.sleep(3)  # give some time for the code to initialize.

now = rospy.Time.now()
cur_pose = PoseWithCovarianceStamped()
cur_pose.header.frame_id = "/map"
cur_pose.header.stamp = now

R = 2.5

for i in range(count):
    fraction = float(i) / float(count)
    angle = 2 * m.pi * fraction
    cur_pose.pose.pose.position.x = R * m.cos(angle) + R
    cur_pose.pose.pose.position.y = 0.8 * R * m.sin(angle) + 0.6 * R
    cur_pose.pose.pose.position.z = 0.0
    if(count == 1):
        rot = 0
    else:
        rot = angle - m.pi
    # wrap around
    if(rot > 2 * m.pi):
        rot -= 2 * m.pi
    if(rot < -2 * m.pi):
        rot += 2 * m.pi
    cur_pose.pose.pose.orientation = angle_to_quaternion(rot)
    pub[i].publish(cur_pose)
