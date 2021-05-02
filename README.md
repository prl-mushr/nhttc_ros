# nhttc_ros
(Assuming catkin_ws exists)
cloning the repo files:
```
cd catkin_ws/src
git clone --branch devel https://github.com/naughtyStark/nhttc_ros.git
cd nhttc_ros
git submodule init
git submodule update --force --recursive --init --remote
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
# API

The nhttc_ros wrapper has the following parameters:
#### car_name (string):
The name assigned to the car (can be any string but prefer car{index number})

#### solver_time (int):
The maximum time in milliseconds for which the solver is allowed to run

#### max_ttc (float):
The maximum time-to-collision used by the solver: any agents that have a time to collision larger than this will not be considered in the cost function

#### carrot_goal_ratio (float):
The ratio of the carrot-goal/lookahead distance to the turning radius. value of 1 means that the lookahead distance is the same as the turning radius.

#### obey_time" value(boolean):
Set to true if the car is supposed to adhere to the time coordinate of the waypoints (as in, to arrive at a waypoint at a given time and not before/after).

#### allow_reverse (boolean):
Set to true if the car is allowed to go in reverse.

### Publishers
Topic | Type | Description
------|------|------------
`/car_name/mux/ackermann_cmd_mux/input/navigation` | [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html)| steering and speed control of car corresponding to car_name.
`/car_name/cur_goal` | [geometry_msgs/PoseStamp](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)| topic on which current waypoint is published.
`/car_name/time_goal` | [geometry_msgs/PoseStamp](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)| waypoint being used for timing purposes.

### Subscribers
Topic | Type | Description
------|------|------------
`/car1/car_pose` | [geometry_msgs/PoseStamp](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)| position of car 1
...
`/car{n}/car_pose` | [geometry_msgs/PoseStamp](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)| position of car {n}
`/car_name/waypoints` | [geometry_msgs/PoseArray](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.htmll)| waypoint array corresponding to car_name. The z axis coordinate represents the time difference between 2 waypoints.

Note that the z axis data in `/car/waypoints` topic represents the time difference between two consecutive waypoints in a unitless fashion, and the number should be pre-multiplied by 0.001 before publishing (so that the visualization on rviz does not look elevated). 1 unit of time here is equal to (distance between two waypoints/expected speed of the agent). If the car moves at 0.5 m/s and the distance between two waypoints is 1 meter, then 1 unit of time would equal to 2 seconds. 