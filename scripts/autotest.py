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

def adjust_rosparams(i):
    ## do something using i if you want.
    rospy.set_param('sim', True)
    rospy.set_param('carrot_goal_ratio', 0.2)
    rospy.set_param('max_ttc', 2.0)
    rospy.set_param('solver_time', 20)
    rospy.set_param('obey_time', True)
    rospy.set_param('allow_reverse', False)
    rospy.set_param('safety_radius', 0.1)
    rospy.set_param('adaptive_lookahead', False)
    rospy.set_param('push_configuration', True)
    rospy.set_param('push_limit_radius', 1.82)
    rospy.set_param('delivery_tolerance', 0.1)
    rospy.set_param('speed_lim', 0.4)

## this helps us track whether the launch file has been completely shut down yet or not
process_generate_running = True
class ProcessListener(roslaunch.pmon.ProcessListener):
    global process_generate_running

    def process_died(self, name, exit_code):
        global process_generate_running  # set this variable to false when process has died cleanly.
        if(name[:11] != "rosbag_play" and name[:13] != "rosbag_record"):
            process_generate_running = False
            rospy.logwarn("%s died with code %s", name, exit_code)

finished = np.zeros(4)  # storing goal-reach status of each car
def fin_callback(msg, i):  # callback for the same
    global finished
    finished[i] = msg.pose.position.z  # nhttc sets it to 1 when it has reached goal

for i in range(4):
    subscriber = rospy.Subscriber("/car" + str(i + 1) + "/cur_goal", PoseStamped, fin_callback, i)  # subscriber

N = 2
success = np.zeros(N)
launchfile = "/home/stark/catkin_mushr/src/nhttc_ros/launch/autotest.launch"
param_file = "/home/stark/catkin_mushr/src/nhttc_ros/config/autotest.yaml"
for i in range(N):
    print("iteration " + str(i) + "starting")

    adjust_launch_file(launchfile, i)
    adjust_rosparams(i)

    process_generate_running = True
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, 
                                              [launchfile],
                                              process_listeners=[ProcessListener()])
    launch.start()  # launch the damn thing

    start_time = time.time()  # note the start time
    finished = np.zeros(4)  # reset the finished status
    while(finished.all() != 1 and time.time() - start_time < 50):
        rospy.sleep(1) # check once per second if all cars have reached their goal or if we have time-out
    if(finished.all() != 1):  # happens if the cars don't finish within some stipulated time, usually due to deadlock
        success[i] = 0 # failure
    else:
        success[i] = 1 # success
    launch.shutdown()
    while(process_generate_running):
        rospy.sleep(1)
    print("waiting 5 seconds for clean exit")
    time.sleep(5)

print(np.mean(success)*100.0)
