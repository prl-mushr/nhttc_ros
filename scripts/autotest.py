#!/usr/bin/env python
import roslaunch
import rospy
from geometry_msgs.msg import PoseStamped
import os
import subprocess
import numpy as np
import time 
from bs4 import BeautifulSoup as bs
import matplotlib.pyplot as plt

rospy.init_node('autotest', anonymous=True)


def adjust_launch_file(filename, i):
    with open(filename, 'r') as f:
        data = f.read()
    bs_data = bs(data, 'xml')
    bag_name = bs_data.find("arg", {"name":"bag_name"})
    rec_name = bs_data.find("arg", {"name":"record_name"})
    bag_name["default"] = "clcbs_data_2"  # can put custom name according to "i" here
    rec_name["default"] = "clcbs_data_2_test"+str(i+1)

    output = bs_data.prettify()  # prettify doesn't actually make it prettier. 
    with open(filename, 'w') as f:
        f.write(output)

def adjust_rosparams(i, N):
    ## do something using i if you want.
    rospy.set_param('sim', True)
    rospy.set_param('carrot_goal_ratio', 0.5)
    rospy.set_param('max_ttc', 1.5)  # settingthis to 0 should make the nhttc system behave like a oblivious tracker in theory
    rospy.set_param('solver_time', 20)
    rospy.set_param('obey_time', True)  # set this to false if testing nhttc standalone
    rospy.set_param('allow_reverse', False)
    rospy.set_param('safety_radius', 0.1)
    rospy.set_param('adaptive_lookahead', False)
    rospy.set_param('push_configuration', True)
    rospy.set_param('push_limit_radius', 1.82)
    rospy.set_param('delivery_tolerance', 0.1)
    rospy.set_param('speed_lim', 0.4)
    rospy.set_param('add_noise', 0)
    rospy.set_param("deadlock_solve", False)  # setting this to false when not using deadlock solvers
    rospy.set_param("time_error_thresh", 0.1)
    rospy.set_param("nhttc_standalone", False)  # do not set to true if max_ttc is also true
    rospy.set_param("pure_pursuit", True)

## this helps us track whether the launch file has been completely shut down yet or not
process_generate_running = True
class ProcessListener(roslaunch.pmon.ProcessListener):
    global process_generate_running

    def process_died(self, name, exit_code):
        global process_generate_running  # set this variable to false when process has died cleanly.
        if(name[:11] != "rosbag_play" and name[:13] != "rosbag_record"):  # prevent these guys from killing the whole process
            process_generate_running = False
            rospy.logwarn("%s died with code %s", name, exit_code)


min_dist = 2
def pose_callback(msg,i):
    global car_poses
    global collision
    global min_dist
    car_poses[i,0] = msg.pose.position.x
    car_poses[i,1] = msg.pose.position.y
    for j in range(4):
        dist = np.linalg.norm(car_poses[i,:] - car_poses[j,:])
        if(dist < 0.45 and j != i):
            collision = True
        if(min_dist > dist and j!=i):
            min_dist = dist  # keep track of min distance between any 2 cars

car_poses = np.zeros((4,2))
finished = np.zeros(4)  # storing goal-reach status of each car
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
        time_error[i] = msg.pose.orientation.z

for i in range(4):
    subscriber = rospy.Subscriber("/car" + str(i + 1) + "/cur_goal", PoseStamped, fin_callback, i)  # subscriber
    subscriber_ = rospy.Subscriber("/car" + str(i + 1) + "/car_pose", PoseStamped, pose_callback, i)  # subscribe to car-pose to check collisions

N = 100
timeout = 60 # 60 second time out
success = np.zeros(N)
recovery = np.zeros((N,2))
CTE_list = np.zeros(N)
min_dist_list = np.zeros(N)
time_list = np.zeros(N)
collision_log = np.zeros(N)
launchfile = "/home/stark/catkin_mushr/src/nhttc_ros/launch/autotest.launch"
param_file = "/home/stark/catkin_mushr/src/nhttc_ros/config/autotest.yaml"
for i in range(N):
    print("iteration " + str(i) + "starting")

    adjust_launch_file(launchfile, i)
    adjust_rosparams(i,N)

    process_generate_running = True
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, 
                                              [launchfile],
                                              process_listeners=[ProcessListener()])
    launch.start()  # launch the damn thing
    time.sleep(10)  # time it takes for the whole thing to boot into existence
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

    time_list[i] = time_error.mean()
    CTE_list[i] = cte.mean()
    min_dist_list[i] = min_dist
    # if( i >= N//2):
    #     # cases where we are using a recovery method
    if(deadlock.all()):
        recovery[i,0] = success[i]
        recovery[i,1] = 1  # indicate that it was indeed a deadlock
    collision_log[i] = collision
    launch.shutdown()
    while(process_generate_running):
        rospy.sleep(1)
    print("waiting 5 seconds for clean exit")
    time.sleep(5)

print("success rate without deadlock solving (measure of deadlock prevention): ",np.mean(success)*100.0, "%")
# print("success rate with deadlock solving (measure of deadlock cure): ", np.mean(success[N//2:])*100.0,"%")

deadlock_recovery_rate = recovery[np.where(recovery[:,1]),0].mean()*100
print(recovery)
print("deadlock recovery rate: ", deadlock_recovery_rate, "%")
print("CTE per run in meters: ", CTE_list);
print("collision_log: ", collision_log)
# print("time_error:", time_list)
L = N//10
prob_dist = np.zeros((L, 6))
for i in range(L):
    prob_dist[i, 0] = success[L*i:L*(i+1)].mean()
    prob_dist[i, 1] = collision_log[L*i:L*(i+1)].mean()
    prob_dist[i, 2] = recovery[L*i:L*(i+1),1].mean()
    prob_dist[i, 3] = CTE_list[L*i:L*(i+1)].mean()
    prob_dist[i, 4] = time_list[L*i:L*(i+1)].mean()
    prob_dist[i, 5] = min_dist_list[L*i:L*(i+1)].mean()
np.save("/home/stark/catkin_mushr/src/nhttc_ros/bags/output.npy", prob_dist)
prob_dist = np.load("/home/stark/catkin_mushr/src/nhttc_ros/bags/output.npy", allow_pickle=True)
print(prob_dist)
X = np.arange(0, 100, 10)
plt.suptitle("performance with ideal parameters")
plt.xlabel("number of test cases")
plt.ylabel("value")
plt.plot(X, prob_dist[:,0], label="success frequency")
plt.scatter(X, prob_dist[:,0])
plt.plot(X, prob_dist[:,5], label="min_dist")
plt.scatter(X, prob_dist[:,5])
# plt.plot(X, prob_dist[:,2], label="deadlock frequency")
# plt.scatter(X, prob_dist[:,2])
plt.plot(X, prob_dist[:,3], label="final error")
plt.scatter(X, np.abs(prob_dist[:,3]))
# plt.plot(X, prob_dist[:,4], label="average time_error")
# plt.scatter(X, prob_dist[:,4])
plt.legend()
plt.show()