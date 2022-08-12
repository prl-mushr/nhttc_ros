#!/usr/bin/env python
import roslaunch
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray
import os
import subprocess
import numpy as np
import time 
from bs4 import BeautifulSoup as bs
import matplotlib.pyplot as plt

rospy.init_node('bag_rerun', anonymous=True)

sys_type = ["full_sys", "full_sys_wo_ded", "locl_sys", "globl_sys", "idot_sys"]

min_dist = 2
def pose_callback(msg,i):
    global car_poses
    global collision
    global min_dist
    car_poses[i,0] = msg.pose.position.x
    car_poses[i,1] = msg.pose.position.y
    for j in range(4):
        dist = np.linalg.norm(car_poses[i,:] - car_poses[j,:])
        if(dist != 0):
            if(dist < 0.4 and j != i):
                collision = True
            if(min_dist > dist and j!=i):
                min_dist = dist  # keep track of min distance between any 2 cars

plan_pub = False
def goal_callback(msg):
    global plan_pub
    plan_pub = True  # plan has been published

car_poses = np.zeros((4,2))
finished = np.zeros(4)  # storing goal-reach status of each car
deadlock = np.zeros(4)
time_error = np.zeros(4)
cte = np.zeros(4)
collision = False
def fin_callback(msg, i):  # callback for the same
    global finished
    global deadlock
    global cte
    global time_error
    finished[i] = msg.pose.position.z  # nhttc sets it to 1 when it has reached goal
    if(msg.pose.orientation.x and deadlock[i] == 0): # did we face a deadlock?
        deadlock[i] = 1
    if(finished[i]):
        cte[i] = msg.pose.orientation.y
    time_error[i] = msg.pose.orientation.z  # keep updating time error until timeout

subs = []
subs_ = []
goal_sub = None
def sub_unsub(sub, goal_listen):
    global subs, subs_,goal_sub
    if(sub and not goal_listen):
        for i in range(4):
            subscriber = rospy.Subscriber("/car" + str(i + 1) + "/cur_goal", PoseStamped, fin_callback, i)  # subscriber
            subscriber_ = rospy.Subscriber("/car" + str(i + 1) + "/car_pose", PoseStamped, pose_callback, i)  # subscribe to car-pose to check collisions
            subs.append(subscriber)
            subs_.append(subscriber_)
    elif goal_listen:
        goal_sub = rospy.Subscriber("/car1/waypoints", PoseArray, goal_callback)  # subscribe to car-pose to check collisions for first car (plans pubbed simultaneously)
    else:
        for i in range(4):
            subs[i].unregister()
            subs_[i].unregister()
        goal_sub.unregister()

# for test_sys in sys_type:
for _ in range(3,5):
    test_sys = sys_type[_]
    N = 100
    timeout = 60 # 60 second time out
    success = np.zeros(N)
    recovery = np.zeros((N,2))
    CTE_list = np.zeros(N)
    min_dist_list = np.zeros(N)
    time_list = np.zeros(N)
    collision_log = np.zeros(N)
    deadlock_log = np.zeros(N)
    
    for i in range(N):
        print("iteration " + str(i) + "starting")

        plan_pub = False
        sub_unsub(sub=False, goal_listen = True)
        pp = subprocess.Popen(['rosbag', 'play', '-q', '-r 100', 'clcbs_data_test_'+str(i)+'.bag'], cwd = "/home/stark/catkin_mushr/src/nhttc_ros/bags/output_bags/"+test_sys)
        wait_start = time.time()
        while(not plan_pub):
            collision = False
            min_dist = 2
            if(time.time() - wait_start > 15):
                pp = subprocess.Popen(['rosbag', 'play', '-q', '-r 10', 'clcbs_data_test_'+str(i)+'.bag'], cwd = "/home/stark/catkin_mushr/src/nhttc_ros/bags/output_bags/"+test_sys)
                wait_start = time.time()
        sub_unsub(sub=True, goal_listen = False)

        start_time = time.time()  # note the start time
        finished = np.zeros(4)  # reset the finished status
        deadlock = np.zeros(4)
        cte = np.zeros(4)
        time_error = np.zeros(4)
        collision = False
        min_dist = 2
        while(finished.all() != 1 and time.time() - start_time < timeout):
            rospy.sleep(1) # check once per second if all cars have reached their goal or if we have time-out
        if(finished.all() != 1 or collision):  # happens if the cars don't finish within some stipulated time, usually due to deadlock
            success[i] = 0 # failure
        else:
            success[i] = (not collision) # success if not collision

        print("success, collision: ", success[i], collision)
        time_list[i] = time_error.mean()
        CTE_list[i] = cte.mean()
        min_dist_list[i] = min_dist
        deadlock_log[i] = deadlock.any() == 1
        if(deadlock.all()):
            recovery[i,0] = success[i]
            recovery[i,1] = 1  # indicate that it was indeed a deadlock
        collision_log[i] = collision
        sub_unsub(sub=False, goal_listen = False)  # unsub from topics
        time.sleep(5)
    deadlock_recovery_rate = recovery[np.where(recovery[:,1]),0].mean()*100
    print(recovery)
    print("deadlock recovery rate: ", deadlock_recovery_rate, "%")
    print("CTE per run in meters: ", CTE_list);
    print("collision_log: ", collision_log)
    print("time_error:", time_list)
    L = N//10
    # prob_dist = np.zeros((L, 6))
    raw_data = []
    raw_data.append(success)
    raw_data.append(collision_log)
    raw_data.append(CTE_list)
    raw_data.append(time_list)
    raw_data.append(min_dist_list)
    raw_data.append(deadlock_log)

    np.save("/home/stark/catkin_mushr/src/nhttc_ros/bags/output_bags/"+test_sys+"/output_raw_new.npy", raw_data)