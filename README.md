# nhttc_ros
(Assuming catkin_ws exists)
downloading all the files:
```
cd catkin_ws/src
git clone --branch devel https://github.com/naughtyStark/nhttc_ros.git
cd nhttc_ros
git submodule init
git submodule update --force --recursive --init --remote

```
please ensure that you have xterm installed! (used for debugging).
```
sudo apt install xterm
```
rviz setup:
start roscore
```
roscore
```
open rviz 
```
rosrun rviz rviz
```
on the top left, click on File->open. Then navigate to nhttc_ros/nhttc_ros/rviz and select nhttc.rviz 
press Ctrl+S before exiting

Compilation:
```
cd catkin_ws
catkin_make
```
Making all python nodes executable:
```
chmod 755 [filename].py
```
Specifically, 
```
chmod 755 nhttc_ros/nhttc_ros/src/nhttc_pose_init.py
```
Running:
```
roslaunch nhttc_ros multi_teleop.launch
```

To run on real car:

1. On lab computer, set its ROS_IP to its own IP address (use `ip a` to check and `export ROS_IP=XXX.XXX.XXX.XXX`). Then run roscore.

2. On lab computer, run roscore.

3. Initialize mocap pose stream with `roslaunch mushr_mocap car_pose_publisher.launch`. There are some context dependent arguments that need to be set, which are the car names, so be mindful of that.

4. SSH into the car (the rest of the steps are done ssh-ed into the car)

5. Set the car's ROS_IP to its own IP, and also set its ROS_MASTER_URI to whatever is listed as the ROS_MASTER_URI on the lab computer where you ran roscore (include the full string, for example 'http://192.168.1.205:11311')

6. Rostopic list on the car to see if it is able to read the car_pose topics published from master, and try to rostopic echo them as well.

7. If everything is connected correctly, run `roslaunch nhttc_ros real.launch` on the car.

