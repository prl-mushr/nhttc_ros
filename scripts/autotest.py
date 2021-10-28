#!/usr/bin/env python
import roslaunch
import rospy
from geometry_msgs.msg import PoseStamped
import os
import subprocess
import numpy as np
import time 
from bs4 import BeautifulSoup as bs

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
    rospy.set_param('carrot_goal_ratio', 0.2)
    rospy.set_param('max_ttc', 0.5)
    rospy.set_param('solver_time', 20)
    rospy.set_param('obey_time', True)
    rospy.set_param('allow_reverse', False)
    rospy.set_param('safety_radius', 0.1)
    rospy.set_param('adaptive_lookahead', False)
    rospy.set_param('push_configuration', True)
    rospy.set_param('push_limit_radius', 1.82)
    rospy.set_param('delivery_tolerance', 0.1)
    rospy.set_param('speed_lim', 0.4)
    rospy.set_param('add_noise', 0)
    if(i < N//2):
        rospy.set_param("deadlock_solve", False)
    else:
        rospy.set_param("deadlock_solve", True)

## this helps us track whether the launch file has been completely shut down yet or not
process_generate_running = True
class ProcessListener(roslaunch.pmon.ProcessListener):
    global process_generate_running

    def process_died(self, name, exit_code):
        global process_generate_running  # set this variable to false when process has died cleanly.
        if(name[:11] != "rosbag_play" and name[:13] != "rosbag_record"):  # prevent these guys from killing the whole process
            process_generate_running = False
            rospy.logwarn("%s died with code %s", name, exit_code)

def pose_callback(msg,i):
    global car_poses
    global collision
    car_poses[i,0] = msg.pose.position.x
    car_poses[i,1] = msg.pose.position.y
    for j in range(4):
        dist = np.linalg.norm(car_poses[i,:] - car_poses[j,:])
        if(dist < 0.3 and j != i):
            collision = True
            break

car_poses = np.zeros((4,2))
finished = np.zeros(4)  # storing goal-reach status of each car
def fin_callback(msg, i):  # callback for the same
    global finished
    global deadlock
    global cte
    finished[i] = msg.pose.position.z  # nhttc sets it to 1 when it has reached goal
    if(msg.pose.orientation.x and deadlock[i] == 0): # did we face a deadlock?
        deadlock[i] = 1
    if(finished[i]):
        cte[i] = msg.pose.orientation.y
for i in range(4):
    subscriber = rospy.Subscriber("/car" + str(i + 1) + "/cur_goal", PoseStamped, fin_callback, i)  # subscriber
    subscriber_ = rospy.Subscriber("/car" + str(i + 1) + "/car_pose", PoseStamped, pose_callback, i)  # subscribe to car-pose to check collisions

N = 20
timeout = 70  # 60 second time out
success = np.zeros(N)
recovery = np.zeros((N//2,2))
CTE_list = np.zeros(N)
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
    collision = False
    while(finished.all() != 1 and time.time() - start_time < timeout):
        rospy.sleep(1) # check once per second if all cars have reached their goal or if we have time-out
    if(finished.all() != 1 or collision):  # happens if the cars don't finish within some stipulated time, usually due to deadlock
        success[i] = 0 # failure
        CTE_list[i] = -1  # invalid
    else:
        success[i] = (not collision) # success if not collision
        CTE_list[i] = cte.mean()
    if( i >= N//2):
        # cases where we are using a recovery method
        if(deadlock.all()):
            recovery[i-N//2,0] = success[i]
            recovery[i-N//2,1] = 1  # indicate that it was indeed a deadlock
    collision_log[i] = collision
    launch.shutdown()
    while(process_generate_running):
        rospy.sleep(1)
    print("waiting 5 seconds for clean exit")
    time.sleep(5)

print("success rate without deadlock solving (measure of deadlock prevention): ",np.mean(success[:N//2])*100.0, "%")
print("success rate with deadlock solving (measure of deadlock cure): ", np.mean(success[N//2:])*100.0,"%")

deadlock_recovery_rate = recovery[np.where(recovery[:,1]),0].mean()*100
print(recovery)
print("deadlock recovery rate: ", deadlock_recovery_rate, "%")
print("CTE per run in meters: ", CTE_list);
print("collision_log: ", collision_log)