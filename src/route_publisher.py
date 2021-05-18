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
from shapely.geometry import Polygon
import matplotlib.pyplot as plt

rospy.init_node("route_publisher")

count = 0

actual = []
polygon = []
min_dist = 10

def pose_callback(data,args):
	global count
	global actual
	global min_dist
	if(count<args):
		count = args
	if(args == 1):
		x = data.pose.position.x
		y = data.pose.position.y
		if(len(actual)>0):
			last_xy = actual[-1]
			if(m.fabs(x - last_xy[0]) > 0.05 or m.fabs(y - last_xy[1])>0.05):
				actual.append(np.array([x,y]))
				# print(x,y)
		else:
			actual.append([x,y])
	if(args==2):
		x = data.pose.position.x
		y = data.pose.position.y
		if(len(actual)==0):
			return
		xy = actual[-1]
		distance = m.sqrt((x-xy[0])**2 + (y-xy[1])**2)
		if(distance < min_dist):
			min_dist = distance

# first is for default
# second for cgr comparison
# third is for max_ttc
# fourth is for SolT
# fifth is for time.

def prepare_route(args,tag):
	# now = rospy.Time.now()
	global count
	global polygon
	R = 2.5
	fraction = float(tag)/float(count)
	angle = 2*m.pi*fraction
	X0 = m.cos(angle)*R + R
	Y0 = m.sin(angle)*R + 0.6*R

	route = PoseArray()
	route.header.frame_id = "/map"
	route.header.stamp = rospy.Time.now()

	if(args.style == "fly-by"):
		dX = 0.25
		dY = 0.25
		if(tag == 0):
			for i in range(20):
				p = Pose()
				if(i<=10):
					p.position.x, p.position.y, p.position.z = X0-dX*i, Y0+0, 0.001
				else:
					p.position.x, p.position.y, p.position.z = X0-dX*10, Y0+dY*(i-10), 0.001
				p.orientation = angle_to_quaternion(m.pi/2)
				polygon.append([p.position.x,p.position.y])
				route.poses.append(p)
		else:
			for i in range(7):
				p = Pose()
				p.position.x, p.position.y, p.position.z = X0+dX*i, Y0+0, 0.001
				p.orientation = angle_to_quaternion(m.pi/2)
				route.poses.append(p)
	if(args.style == "halt"):
		dX = (0 - X0)/7.0
		dY = 0.5
		if(tag == 0):
			for i in range(14):
				p = Pose()
				if(i<10):
					p.position.x, p.position.y, p.position.z = X0+dX*i, Y0+0, 0.001
				else:
					p.position.x, p.position.y, p.position.z = 2*dX, Y0+dY*(i-10), 0.001
				p.orientation = angle_to_quaternion(m.pi/2)
				route.poses.append(p)
		else:
			for i in range(6):
				p = Pose()
				p.position.x, p.position.y, p.position.z = X0+dX*i, Y0+0, 0.001
				p.orientation = angle_to_quaternion(m.pi/2)
				route.poses.append(p)
	if(args.style == "proving a point"):
		dX = 0
		dY = 0.5
		for i in range(7):
			p = Pose()
			p.position.x, p.position.y, p.position.z = X0, Y0 + dY*i, 0.001
			p.orientation = angle_to_quaternion(m.pi/2)
			route.poses.append(p)

	return route

def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))

if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='mushr')
	parser.add_argument('--style', type=str, default="proving a point", help="path style")
	args = parser.parse_args()
	sub = []
	pub = []
	for i in range(8):
		subscriber = rospy.Subscriber("/car" + str(i+1) + "/car_pose", PoseStamped, pose_callback,(i+1))
		publisher = rospy.Publisher("/car" + str(i+1) + "/waypoints", PoseArray, queue_size=1)
		sub.append(subscriber)
		pub.append(publisher)
	time.sleep(1)
	print("Route Publisher Running",count)
	count = 2
	for i in range(count):
		pub[i].publish(prepare_route(args,i))
	## uncomment the following lines for experiment data collection.
	# start_time = time.time()
	# dist = 10
	# while(dist>0.5):
	# 	try:
	# 		cur_pos = actual[-1]
	# 		final_pos = polygon[-1]
	# 		dist = m.sqrt((cur_pos[0]-final_pos[0])**2 + (cur_pos[1]-final_pos[1])**2)
	# 	except:
	# 		print("something bwoke uwu",len(polygon))
	# end_time = time.time()
	# exp_dist = float(len(polygon))*0.5
	# exp_time = exp_dist/0.4 # 0.3 m/s max speed
	# act_time = end_time - start_time
	# time_ratio = act_time/exp_time

	# act_dist = 0.5 # prepend by 0.5 to account for the 0.5 m cutoff.
	# prev_xy = None
	# polygon.reverse()
	# last_point = polygon[0]

	# for xy in actual:
	# 	polygon.append([xy[0],xy[1]])
	# 	if(prev_xy is not None):
	# 		act_dist += m.sqrt((xy[0] - prev_xy[0])**2 + (xy[1] - prev_xy[1])**2)
	# 	prev_xy = xy
	# print(act_dist,exp_dist)
	# path_ratio = act_dist/exp_dist
	# polygon.append([last_point[0],last_point[1]])
	# area = Polygon(polygon).area
	# average_cte = area/exp_dist
	# polygon = np.array(polygon)
	# disp_act = np.array(actual)
	# # plt.plot(polygon[:,0],polygon[:,1])
	# # plt.plot(disp_act[:,0],disp_act[:,1],color='r')
	# # plt.show()
	# print("path_ratio: {} | time_ratio: {} | average_cte: {} | minimum_dist: {}".format( round(1/path_ratio,3), round(1/time_ratio,3), round(average_cte,3), round(min_dist,3) ) )
