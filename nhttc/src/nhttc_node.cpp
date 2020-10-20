#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "tf2/LinearMath/Quaternion.h" //keeping these because they may be useful in the future?
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <string>

class nhttc
{
public:
  ros::Subscriber sub_pose;
  ros::Subscriber sub_other_pose[4];
  ros::Subscriber sub_goal;
  ros::Publisher pub_cmd;

  std::string self_name;
  std::ostringstream s;

  float cur_pose[3];
  float other_pose[3];
  float goal_pose[3];
  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    float rpy[3];
    rpy_from_quat(rpy,msg);
    cur_pose[0] = msg->pose.position.x;
    cur_pose[1] = msg->pose.position.y;
    cur_pose[2] = rpy[2]; 
    ROS_INFO("self: %f",cur_pose[0]);
  }

  void OtherPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    float rpy[3];
    rpy_from_quat(rpy,msg);
    other_pose[0] = msg->pose.position.x;
    other_pose[1] = msg->pose.position.y;
    other_pose[2] = rpy[2]; 
    ROS_INFO("Other: %f",other_pose[0]);
  }

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    float rpy[3];
    rpy_from_quat(rpy,msg);
    goal_pose[0] = msg->pose.position.x;
    goal_pose[1] = msg->pose.position.y;
    goal_pose[2] = rpy[2]; // goal yaw. if mike is short for micheal, is yaw short for yee-haw? I would very much like it to be so. 
  }

  void send_commands(float speed, float steer) // speed is in m/s. steering is in radians
  {
    ackermann_msgs::AckermannDriveStamped output_msg;
    output_msg.header.stamp = ros::Time::now();
    output_msg.header.frame_id = "";
    output_msg.drive.steering_angle = steer;
    output_msg.drive.speed = speed;
    pub_cmd.publish(output_msg);
  }
  nhttc(ros::NodeHandle &nh)
  {
    nh.getParam("car_name",self_name);
    sub_pose = nh.subscribe("/" + self_name +"/car_pose",10,&nhttc::PoseCallback,this);
    //redneck solution for multi-agent setting
    sub_other_pose[0] = nh.subscribe("/car1/car_pose",10,&nhttc::OtherPoseCallback,this);
    sub_other_pose[1] = nh.subscribe("/car2/car_pose",10,&nhttc::OtherPoseCallback,this);
    sub_other_pose[2] = nh.subscribe("/car3/car_pose",10,&nhttc::OtherPoseCallback,this);
    sub_other_pose[3] = nh.subscribe("/car4/car_pose",10,&nhttc::OtherPoseCallback,this);

    sub_goal = nh.subscribe("/move_base_simple/goal",10,&nhttc::GoalCallback,this);
    pub_cmd = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/"+ self_name +"/mux/ackermann_cmd_mux/input/navigation",10);
  }

  void rpy_from_quat(float rpy[3],const geometry_msgs::PoseStamped::ConstPtr& msg) //not the best way to do it but I was getting errors when I tried to pass the pose.orientation cuz I don't understand the data type
  { // you'd think that with ROS being such a widely used back-end there would be a function to convert quaternion to euler but nooooooooo (they do provide euler to quaternion tho) TODO: remove this comment for being "unacademic"
    float q[4];
    q[0] = msg->pose.orientation.x;
    q[1] = msg->pose.orientation.y;
    q[2] = msg->pose.orientation.z;
    q[3] = msg->pose.orientation.w;
    rpy[2] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),1 - 2 * (q[1] * q[1] + q[2] * q[2]));
    rpy[0] = asinf(2.0f * (q[0] * q[2] - q[3] * q[1]));
    rpy[1] = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
  }

  void plan()
  {
    /*
    ...
    code for planning to be inserted here
    Note: current pose is stored in cur_pose (cur_pose[0] = x, cur_pose[1] = y, cur_pose[2] = heading angle in radians)
    goal pose is given by goal_pose (0->x, 1->y, 2-> theta)
    ...
    */
    //send speed and steering commands. Speed is in m/s, steering is in radians
    float speed = 0.1; //speed in m/s
    float steering_angle = 0.1; //steering angle in radians. +ve is left. -ve is right 
    send_commands(speed,steering_angle); //just sending out anything for now;
    return;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc,argv,"location_monitor");
  ros::NodeHandle nh("~");
  nhttc local_planner(nh);
  ros::Rate r(50);
  while(ros::ok)
  {
    ros::spinOnce(); //this line wasted 2 hours of my time.
    local_planner.plan();
    r.sleep();
  }
  return 0;
}
