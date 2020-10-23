#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "tf2/LinearMath/Quaternion.h" //keeping these because they may be useful in the future?
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <string>
#include <sstream>

class nhttc_ros
{
public:
  ros::Subscriber sub_pose,sub_twist;
  ros::Subscriber sub_other_pose[4],sub_other_odom[4];
  ros::Subscriber sub_goal;
  ros::Publisher pub_cmd;

  std::string self_name;
  std::string other_name;
  char index;
  std::ostringstream s;

  float cur_state[7];
  float other_state[4][7];
  float goal_pose[3];

  float get_velocity_bf(float vx,float vy,float theta) //get body frame velocity.
  {
    return vx*cosf(theta) + vy*sinf(theta); //ROS uses the ENU reference frame, so this theta is wrt to the x axis. 
  }

  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    float rpy[3];
    rpy_from_quat(rpy,msg);
    cur_state[0] = msg->pose.position.x;
    cur_state[1] = msg->pose.position.y;
    cur_state[2] = rpy[2]; 
  }
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    cur_state[3] = msg->twist.twist.linear.x;
    cur_state[4] = msg->twist.twist.linear.y;
    cur_state[5] = msg->twist.twist.angular.z;
    cur_state[6] = get_velocity_bf(cur_state[3],cur_state[4],cur_state[2]);
  }

  void OtherPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int i)
  {
    float rpy[3];
    rpy_from_quat(rpy,msg);
    other_state[i][0] = msg->pose.position.x;
    other_state[i][1] = msg->pose.position.y;
    other_state[i][2] = rpy[2]; 
  }

  void OtherOdomCallback(const nav_msgs::Odometry::ConstPtr& msg, int i)
  {
    other_state[i][3] = msg->twist.twist.linear.x;
    other_state[i][4] = msg->twist.twist.linear.y;
    other_state[i][5] = msg->twist.twist.angular.z;
    other_state[i][6] = get_velocity_bf(other_state[i][3],other_state[i][4],other_state[i][2]);

  }

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    float rpy[3];
    rpy_from_quat(rpy,msg);
    goal_pose[0] = msg->pose.position.x;
    goal_pose[1] = msg->pose.position.y;
    goal_pose[2] = rpy[2]; // goal yaw. if mike is short for micheal, is yaw short for yee-haw? I would very much like it to be so. TODO: remove this comment for being "unacademic" 
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
  nhttc_ros(ros::NodeHandle &nh)
  {
    nh.getParam("car_name",self_name);
    sub_pose = nh.subscribe("/" + self_name +"/car_pose",10,&nhttc_ros::PoseCallback,this);
    sub_twist = nh.subscribe("/"+ self_name +"/vesc/odom",10,&nhttc_ros::OdomCallback,this);
    // solution for multi-agent setting
    // insert hackerman_meme.jpg here
    for(int i=0;i<4;i++)
    { //you'd think that with ROS being such a widely used backend that it would be simple to convert an integer to a string but nooooooo I have to make it a stringstream, get the .str() of it, then get the .c_str() of that to make it work
      s.str("");
      s<<"/car";
      s<<i+1;
      s<<"/car_pose";
      sub_other_pose[i] = nh.subscribe<geometry_msgs::PoseStamped>((s.str()).c_str(),10,boost::bind(&nhttc_ros::OtherPoseCallback,this,_1,i));
      s.str("");
      s<<"/car";
      s<<i+1;
      s<<"/vesc/odom";
      sub_other_odom[i] = nh.subscribe<nav_msgs::Odometry>((s.str()).c_str(),10,boost::bind(&nhttc_ros::OtherOdomCallback,this,_1,i));
    }
    sub_goal = nh.subscribe("/move_base_simple/goal",10,&nhttc_ros::GoalCallback,this);
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
    
    state:
    state[0] = x;
    state[1] = y;
    state[2] = yaw/heading angle; (theta)
    state[3] = velocity in x direction
    state[4] = velocity in y direction
    state[5] = rate of rotation about z axis.
    state[6] = body frame speed (forward is +ve, backward is -ve)
    ...
    */

    /* Preferred interface: 
    nhttc_agent.set_params(param); // params include time allowed for planning, time-step, etc. More on this later as I build it out.

    nhttc_agent.set_obstacles(list_of_obstacle_states); // void nhttc_class::set_obstacles(float agent_states[4][6])
      This list could be built with a global list enhanced with additional obstacles as necessary. Most of our programming will likely go into
      generating and modifying the obstacles going into the planner.
      Individual Obstacle requirements: 
        1. Type (Velocity, MuSHRCar, PixelBlock, etc.) 
        2. type-specific State 
        3. type-specific controls

    nhttc_agent.set_goals(goals); 
    The goals are either for the agent AND all the obstacles, or just for the agents. I think its just for the current agent but I'll have
    to double check. This is where the carrot goal will come in, i.e. the input from the global planner.


    nhttc_agent.set_ego(cur_state); // void nhttc_class:set_ego(float self_state[6])
      Agent State Requirements:
        1. Type (can just be initialized on creation so you don't constantly pass it in)
        2. type-specific State (X, Y, Heading Angle) (I have to double check if its with respect to the global or car frame)
        3. type-specific controls (Speed, Steering angle)

    nhttc_agent.get_controls(return_parameter);
      This will direcly return the new control actions very easily

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
  nhttc_ros local_planner(nh);
  ros::Rate r(50);
  while(ros::ok)
  {
    ros::spinOnce(); //this line wasted 2 hours of my time.
    local_planner.plan();
    r.sleep();
  }
  return 0;
}
