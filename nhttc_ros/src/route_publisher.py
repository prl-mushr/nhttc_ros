#!/usr/bin/env python

import rospy
import tf
import tf.transformations
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseArray, Pose
import math as m
import time


CARS = 1
FRAME = "/map"

rospy.init_node("route_publisher")

count = 0

def prepare_route():
	# now = rospy.Time.now()
	route = PoseArray()
	route.header.frame_id = FRAME
	route.header.stamp = rospy.Time.now()
	for i in range(10):
		p = Pose()
		p.position.x, p.position.y, p.position.z = i, i, 0
		p.orientation = angle_to_quaternion(m.pi/2)
		route.poses.append(p)
	return route


def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))

# list of publishers and subscribers
pubs = []
# this is basically initializing all the subscribers for counting the number of cars and publishers for initiailizing pose and goal points.
for i in range(CARS):
	publisher = rospy.Publisher("/car" + str(i+1) + "/waypoints", PoseArray, queue_size=1)
	pubs.append(publisher)

time.sleep(3) # give some time for the code to initialize.

print("Route Publisher Running")

while not rospy.is_shutdown():
	for pub in pubs:
		pub.publish(prepare_route())
	time.sleep(3)
# # initial pose setting -> not required in real; comment if not required.
# now = rospy.Time.now()
# cur_pose = PoseWithCovarianceStamped()
# cur_pose.header.frame_id = FRAME
# cur_pose.header.stamp = now

# # sets the cars on the circumference of a circle with radius = R. the positions are equi-distant (3 cars at 120 degrees, 4 at 90 and so on)
# R = 5*m.sqrt(2)
# print(count)
# for i in range(count):
# 	fraction = float(i)/float(count)
# 	angle = 2*m.pi*fraction
# 	cur_pose.pose.pose.position.x = R*m.cos(angle)
# 	cur_pose.pose.pose.position.y = R*m.sin(angle)
# 	cur_pose.pose.pose.position.z = 0.0
# 	rot = angle - m.pi
# 	#wrap around
# 	if(rot>2*m.pi):
# 		rot -= 2*m.pi
# 	if(rot< -2*m.pi):
# 		rot += 2*m.pi
# 	cur_pose.pose.pose.orientation = angle_to_quaternion(rot)
# 	pub[i].publish(cur_pose)


# # goal setting: sets the position of the goal points. goal points are diametrically opposite to the car's starting position.
# now = rospy.Time.now()
# goal_pose = PoseStamped()
# goal_pose.header.frame_id = FRAME
# goal_pose.header.stamp = now
# for i in range(count):
# 	fraction = float(i)/float(count)
# 	angle = m.pi + 2*m.pi*fraction
# 	goal_pose.pose.position.x = R*m.cos(angle)
# 	goal_pose.pose.position.y = R*m.sin(angle)
# 	goal_pose.pose.position.z = 0.0
# 	rot = angle
# 	#wrap around
# 	if(rot>2*m.pi):
# 		rot -= 2*m.pi
# 	if(rot< -2*m.pi):
# 		rot += 2*m.pi
# 	goal_pose.pose.orientation = angle_to_quaternion(rot)
# 	goal_pub[i].publish(goal_pose)
