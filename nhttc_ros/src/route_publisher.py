#!/usr/bin/env python
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseArray, Pose
import math as m
import time
import argparse
import numpy as np

rospy.init_node("route_publisher")

count = 0

def pose_callback(data,args):
	global count
	if(count<args):
		count = args

def prepare_route(args,tag):
	# now = rospy.Time.now()
	global count
	R = 3.5
	fraction = float(tag)/float(count)
	angle = 2*m.pi*fraction
	X0 = m.cos(angle)*R
	Y0 = m.sin(angle)*R

	route = PoseArray()
	route.header.frame_id = "/map"
	route.header.stamp = rospy.Time.now()
	if(args.style == "straight"):
		Xf,Yf = m.cos(angle+m.pi)*R,m.sin(angle+m.pi)*R
		dX = (Xf - X0)*0.1
		dY = (Yf - Y0)*0.1
		print(dX,dY)
		for i in range(10):
			p = Pose()
			p.position.x, p.position.y, p.position.z = X0 + float(i)*dX, Y0 + float(i)*dY, 0
			p.orientation = angle_to_quaternion(m.pi/2)
			route.poses.append(p)

	if(args.style == "circle"):
		R = 2.5
		for i in range(10):
			fraction = float(i)/10
			angle = 2*m.pi*fraction
			p = Pose()
			if(tag%2):
				p.position.x, p.position.y, p.position.z = R*m.cos(-m.pi/2 + angle), R*m.sin(- m.pi/2 + angle)+R, 0
			else:
				p.position.x, p.position.y, p.position.z = R*m.cos(-m.pi/2 -angle), R*m.sin(-m.pi/2 - angle)+R, 0
			p.orientation = angle_to_quaternion(angle)
			route.poses.append(p)

	if(args.style == "fly-by"):
		dX = (0 - X0)/7.0
		dY = 0.5
		if(tag == 0):
			for i in range(10):
				p = Pose()
				if(i<5):
					p.position.x, p.position.y, p.position.z = X0+dX*i, Y0+0, 0
				else:
					p.position.x, p.position.y, p.position.z = 0, Y0+dY*(i-4), 0
				p.orientation = angle_to_quaternion(m.pi/2)
				route.poses.append(p)
		else:
			for i in range(10):
				p = Pose()
				p.position.x, p.position.y, p.position.z = X0+dX*i, Y0+0, 0
				p.orientation = angle_to_quaternion(m.pi/2)
				route.poses.append(p)
	if(args.style == "halt"):
		dX = (0 - X0)/7.0
		dY = 0.5
		if(tag == 0):
			for i in range(14):
				p = Pose()
				if(i<10):
					p.position.x, p.position.y, p.position.z = X0+dX*i, Y0+0, 0
				else:
					p.position.x, p.position.y, p.position.z = 2*dX, Y0+dY*(i-10), 0
				p.orientation = angle_to_quaternion(m.pi/2)
				route.poses.append(p)
		else:
			for i in range(6):
				p = Pose()
				p.position.x, p.position.y, p.position.z = X0+dX*i, Y0+0, 0
				p.orientation = angle_to_quaternion(m.pi/2)
				route.poses.append(p)
	return route

def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))

if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='mushr')
	parser.add_argument('--style', type=str, default="fly-by", help="path style")
	args = parser.parse_args()
	sub = []
	pub = []
	for i in range(8):
		subscriber = rospy.Subscriber("/car" + str(i+1) + "/car_pose", PoseStamped, pose_callback,(i+1))
		publisher = rospy.Publisher("/car" + str(i+1) + "/waypoints", PoseArray, queue_size=1)
		sub.append(subscriber)
		pub.append(publisher)
	print("Route Publisher Running",count)
	time.sleep(1)
	for i in range(count):
		pub[i].publish(prepare_route(args,i))
