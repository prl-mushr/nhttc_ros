#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "tf2/LinearMath/Quaternion.h" //keeping these because they may be useful in the future?
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"



class nhttc
{
  ros::NodeHandle nh;
  ros::Subscriber sub_pose;
  ros::Subscriber sub_goal;
  ros::Publisher pub_cmd;

  float cur_pose[3];
  float goal_pose[3];
  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    float rpy[3];
    rpy_from_quat(rpy,msg);
    cur_pose[0] = msg->pose.position.x;
    cur_pose[1] = msg->pose.position.y;
    cur_pose[2] = rpy[2]; 
  }

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    float rpy[3];
    rpy_from_quat(rpy,msg);
    goal_pose[0] = msg->pose.position.x;
    goal_pose[1] = msg->pose.position.y;
    goal_pose[2] = rpy[2]; // goal yaw
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
public:
  nhttc()
  {
    sub_pose = nh.subscribe("/car/car_pose",10,&nhttc::PoseCallback,this);
    sub_goal = nh.subscribe("/move_base_simple/goal",10,&nhttc::GoalCallback,this);
    pub_cmd = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/car/mux/ackermann_cmd_mux/input/navigation",10);
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
  nhttc local_planner;
  
  ros::Rate r(50);
  while(ros::ok)
  {
    local_planner.plan();
    r.sleep();
  }
  return 0;
}