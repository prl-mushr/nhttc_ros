# nhttc_ros
(Assuming catkin_ws exists)
downloading all the files:
```
cd catkin_ws/src
git clone https://github.com/naughtyStark/nhttc_ros.git
cd nhttc_ros
git submodule init
git submodule update --recursive
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
