#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <string>
#include <sstream>
#include "nhttc_interface.h"

class nhttc_ros
{
public:
  ros::Subscriber sub_other_pose[16],sub_other_control[16];
  ros::Subscriber sub_goal,sub_wp;
  ros::Publisher pub_cmd, viz_pub;

  int car_count;
  std::string self_name;
  std::string other_name1, other_name2, other_name3, other_name4, other_name5, other_name6, other_name7;

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

  std::vector<Eigen::Vector2f> waypoints;
  int current_wp_index, max_index;

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

  void WPCallBack(const geometry_msgs::PoseArray::ConstPtr& msg)
  {
    Eigen::Vector2f wp;
    int num = msg->poses.size();//sizeof(msg->poses)/sizeof(msg->poses[0]);
    for(int i =0;i<num;i++)
    {
      wp[0] = msg->poses[i].position.x;
      wp[1] = msg->poses[i].position.y;
      waypoints.push_back(wp);
    }
    goal_received = true;
    goal = waypoints[current_wp_index]; 
    agents[own_index].UpdateGoal(goal); // set the goal 
    max_index = num;
    return;
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

  void viz_publish()
  {
    geometry_msgs::PoseStamped output_msg;
    output_msg.header.stamp = ros::Time::now();
    output_msg.header.frame_id = "/map";
    output_msg.pose.position.x = agents[own_index].goal[0];
    output_msg.pose.position.y = agents[own_index].goal[1];
    viz_pub.publish(output_msg);
  }
  nhttc_ros(ros::NodeHandle &nh)
  {
    goal_received = false; // start with the assumption that your life has no goal.
    current_wp_index = 0;
    max_index = 0;

    // Get car name information from launch file
    nh.getParam("car_count", car_count);
    nh.getParam("other_name1", other_name1);
    nh.getParam("other_name2", other_name2);
    nh.getParam("other_name3", other_name3);
    nh.getParam("other_name4", other_name4);
    nh.getParam("other_name5", other_name5);
    nh.getParam("other_name6", other_name6);
    nh.getParam("other_name7", other_name7);
    
    if(not nh.getParam("solver_time", solver_time))
    {
      solver_time = 10; // 10 ms solver time for each agent.
    }
    if(not nh.getParam("sim", simulation))
    {
      simulation = true;
    }
    if(not nh.getParam("max_agents", num_agents_max))
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
      //s<<i+1;
      if(s.str().c_str()=="/"+self_name)
      {
        own_index = i;
      } // Add param for sim/real car_pose | mocap_pose
      s<<"/car_pose";
      ROS_INFO(s.str().c_str());
      sub_other_pose[i] = nh.subscribe<geometry_msgs::PoseStamped>((s.str()).c_str(),10,boost::bind(&nhttc_ros::OtherPoseCallback,this,_1,i));
      s.str("");
      s<<"/car";
      //s<<i+1;
      s<<"/mux/ackermann_cmd_mux/input/navigation";
      ROS_INFO(s.str().c_str());
      sub_other_control[i] = nh.subscribe<ackermann_msgs::AckermannDriveStamped>((s.str()).c_str(),10,boost::bind(&nhttc_ros::OtherControlCallback,this,_1,i));
    }
    sub_goal = nh.subscribe("/"+self_name+"/move_base_simple/goal",10,&nhttc_ros::GoalCallback,this);
    sub_wp = nh.subscribe("/"+self_name+"/waypoints",10,&nhttc_ros::WPCallBack,this);
    pub_cmd = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/"+ self_name +"/mux/ackermann_cmd_mux/input/navigation",10);
    viz_pub = nh.advertise<geometry_msgs::PoseStamped>("/"+self_name+"/cur_goal",10);
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
    if (agents.size() <= 0)
    {
      ROS_INFO("WARNING: EMPTY AGENT ARRAY");
    }
    
    // create obstacle list.
    obstacles = BuildObstacleList(agents);
    agents[own_index].SetPlanTime(solver_time); //20 ms planning window
    agents[own_index].SetObstacles(obstacles, size_t(own_index));

    Eigen::VectorXf controls = Eigen::VectorXf::Zero(2); //controls are 0,0 by default.
    if(goal_received)
    {
      float dist = (agents[own_index].prob->params.x_0.head(2) - agents[own_index].goal).norm(); //distance from goal wp.

      if(dist>0.2 + agents[own_index].prob->params.radius) // 20 cm tolerance to goal
      {
        controls = agents[own_index].UpdateControls();
      }
      if(dist<1.5 + agents[own_index].prob->params.radius)
      {
        if(current_wp_index < max_index-1)
        {
          agents[own_index].goal = waypoints[++current_wp_index];
          viz_publish();
        }
        if(dist<0.05)
        {
          controls[0] = 0;
          controls[1] = 0;
          goal_received = false;
        }
      }
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
    r.sleep();
  }
  return 0;
}
