#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <string>
#include <sstream>
#include "nhttc_interface.h"

class nhttc_ros
{
public:
  ros::Subscriber sub_other_pose[16],sub_other_control[16];
  ros::Subscriber sub_goal;
  ros::Publisher pub_cmd;

  std::string self_name;
  std::string other_name;
  char index;
  std::ostringstream s;

  Eigen::Vector2f goal;
  int own_index;
  int solver_time;
  int num_agents_max;
  bool simulation;
  bool goal_received;
  SGDOptParams global_params;
  std::vector<Agent> agents; //all agents
  int count;

  std::vector<TTCObstacle*> obstacles = BuildObstacleList(agents);

  void agent_setup(int i)
  {
    Eigen::Vector2f goal(0.0,0.0);
    Eigen::VectorXf pos = Eigen::VectorXf::Zero(3);
    agents.emplace_back(GetAgentParts(6, pos, true, goal), global_params);
  }

  void OtherPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int i)
  {
    if(count<i)
    {
      int last_count = count+1;
      count = i;
      for(int i=last_count; i <= count;i++)
      {
        agent_setup(i);
      }
    }
    float rpy[3];
    Eigen::VectorXf x_o = Eigen::VectorXf::Zero(3);
    rpy_from_quat(rpy,msg);
    x_o[0] = msg->pose.position.x;
    x_o[1] = msg->pose.position.y;
    x_o[2] = rpy[2];
    agents[i].SetEgo(x_o);
  }

  void OtherControlCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg, int i)
  {
    // for safety in case a car is initialized after all others have been init.
    if(count<i)
    {
      int last_count = count+1;
      count = i;
      for(int i=last_count; i <= count;i++)
      {
        agent_setup(i);
      }
    }
    Eigen::VectorXf controls = Eigen::VectorXf::Zero(2);
    controls[0] = msg->drive.speed;
    controls[1] = msg->drive.steering_angle;
    if(i!=own_index)
    {
      agents[i].SetControls(controls);
    }
  }

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    goal[0] = msg->pose.position.x;
    goal[1] = msg->pose.position.y;
    agents[own_index].UpdateGoal(goal);
    goal_received = true;
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
    goal_received = false; // start with the assumption that your life has no goal.
    nh.getParam("car_name",self_name);
    if(not nh.getParam("solver_time",solver_time))
    {
      solver_time = 10; // 10 ms solver time for each agent.
    }
    if(not nh.getParam("sim",simulation))
    {
      simulation = true;
    }
    if(not nh.getParam("max_agents",num_agents_max))
    {
      num_agents_max = 8;
    }

    ConstructGlobalParams(&global_params);
    count = -1; //number of agents. start from -1.
    // solution for multi-agent setting
    // set 8 -> make a param.
    for(int i=0;i<num_agents_max;i++) //limit for i can be more but not less than the total no. of cars.
    { //you'd think that with ROS being such a widely used backend that it would be simple to convert an integer to a string but nooooooo I have to make it a stringstream, get the .str() of it, then get the .c_str() of that to make it work
      s.str("");
      s<<"/car";
      s<<i+1;
      if(s.str().c_str()=="/"+self_name)
      {
        own_index = i;
      } // Add param for sim/real car_pose | mocap_pose
      if(simulation)
      {
        s<<"/car_pose";
      }
      else
      {
        s<<"/mocap_pose";
      }
      sub_other_pose[i] = nh.subscribe<geometry_msgs::PoseStamped>((s.str()).c_str(),10,boost::bind(&nhttc_ros::OtherPoseCallback,this,_1,i));
      s.str("");
      s<<"/car";
      s<<i+1;
      s<<"/mux/ackermann_cmd_mux/input/navigation";
      sub_other_control[i] = nh.subscribe<ackermann_msgs::AckermannDriveStamped>((s.str()).c_str(),10,boost::bind(&nhttc_ros::OtherControlCallback,this,_1,i));
    }
    sub_goal = nh.subscribe("/"+self_name+"/move_base_simple/goal",10,&nhttc_ros::GoalCallback,this);
    pub_cmd = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/"+ self_name +"/mux/ackermann_cmd_mux/input/navigation",10);
    ROS_INFO("node started");

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
    // create obstacle list.
    obstacles = BuildObstacleList(agents);

    agents[own_index].SetPlanTime(solver_time); //20 ms planning window
    agents[own_index].SetObstacles(obstacles, size_t(own_index));

    Eigen::VectorXf controls = Eigen::VectorXf::Zero(2); //controls are 0,0 by default.
    float dist = (agents[own_index].prob->params.x_0.head(2) - agents[own_index].goal).norm(); //distance from goal wp.
    if(dist>0.2 and goal_received) // 20 cm tolerance to goal
    {
      controls = agents[own_index].UpdateControls();
    }
    if(dist<0.2 or !(goal_received))
    {
      controls[0] = 0;
      controls[1] = 0;
      // do something to announce that I have reached
    }

    float speed = controls[0]; //speed in m/s
    float steering_angle = controls[1]; //steering angle in radians. +ve is left. -ve is right 
    send_commands(speed,steering_angle); //just sending out anything for now;
    return;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc,argv,"nhttc_local_planner");
  ros::NodeHandle nh("~");
  nhttc_ros local_planner(nh);
  ros::Rate r(50);
  for(int i = 0; i < 200;i++) //wait for 5 seconds: this can be removed for the real world; it corresponds to the time taken by the rest of the stuff to init
  {
    ros::spinOnce(); // spin but don't plan anything.
    r.sleep();
  }
  while(ros::ok)
  {
    ros::spinOnce(); //this line wasted 2 hours of my time.
    local_planner.plan();
    // r.sleep();
  }
  return 0;
}
