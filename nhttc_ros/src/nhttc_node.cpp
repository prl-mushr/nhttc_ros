#include "ros/ros.h"
#include "ros/master.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <string>
#include <sstream>
#include "nhttc_interface.h"
#include <boost/algorithm/string.hpp>
class nhttc_ros
{
public:
  ros::Subscriber sub_other_pose[16],sub_other_control[16];
  ros::Subscriber sub_goal,sub_wp;
  ros::Publisher pub_cmd, viz_pub;

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
  float cutoff_dist;
  float steer_limit;
  float wheelbase;
  SGDOptParams global_params;
  std::vector<Agent> agents; //all agents
  int count;

  std::vector<TTCObstacle*> obstacles = BuildObstacleList(agents);

  std::vector<Eigen::Vector2f> waypoints;
  int current_wp_index, max_index;

  ros::master::V_TopicInfo master_topics;

  void agent_setup(int i)
  {
    Eigen::Vector2f goal(0.0,0.0);
    Eigen::VectorXf pos = Eigen::VectorXf::Zero(3);
    agents.emplace_back(GetAgentParts(6, pos, true, goal), global_params);
  }

  void OtherPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int i)
  {
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
    waypoints.clear(); // reset
    current_wp_index = 0;
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
    ros::Rate r(1);
    for(int i=0;i<5;i++)
    {
      r.sleep();
    }
    ros::master::getTopics(master_topics);

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) 
    {
      const ros::master::TopicInfo& info = *it;
      std::vector<std::string> strs;
      if(info.datatype == "geometry_msgs/PoseStamped")
      {
        boost::split(strs, info.name, boost::is_any_of("/"));
        if(strs[2]=="car_pose")
        {
          count++;
          if(strs[1] == self_name)
          {
            own_index = count;
          }
          agent_setup(count);

          sub_other_pose[count] = nh.subscribe<geometry_msgs::PoseStamped>(info.name,10,boost::bind(&nhttc_ros::OtherPoseCallback,this,_1,count));
          s.str("");
          s<<"/";
          s<<strs[1];
          s<<"/mux/ackermann_cmd_mux/input/navigation";
          sub_other_control[count] = nh.subscribe<ackermann_msgs::AckermannDriveStamped>((s.str()).c_str(),10,boost::bind(&nhttc_ros::OtherControlCallback,this,_1,count));
        }
      }
    }
    sub_wp = nh.subscribe("/"+self_name+"/waypoints",10,&nhttc_ros::WPCallBack,this);
    pub_cmd = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/"+ self_name +"/mux/ackermann_cmd_mux/input/navigation",10);
    viz_pub = nh.advertise<geometry_msgs::PoseStamped>("/"+self_name+"/cur_goal",10);
    
    ROS_INFO("node started");
  }

  void setup()
  {
    steer_limit = 0.1*M_PI; // max steering angle ~18 degrees. :(. I wanted to tokyo drift with the MuSHR car. 
    agents[own_index].prob->params.steer_limit = steer_limit;
    wheelbase = agents[own_index].prob->params.wheelbase;
    agents[own_index].prob->params.u_lb = Eigen::Vector2f(-0.3f, -0.1f * M_PI);
    agents[own_index].prob->params.u_ub = Eigen::Vector2f(0.3f,0.1f * M_PI);
    agents[own_index].prob->params.max_ttc = 6;
    ROS_INFO("%f, %f",steer_limit,wheelbase);
    fabs(steer_limit) == 0 ? cutoff_dist = 1.0 : cutoff_dist = wheelbase/tanf(fabs(steer_limit)); //CHANGED
    cutoff_dist += agents[own_index].prob->params.radius;
    ROS_INFO("%f",cutoff_dist);
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
    if(goal_received)
    {  
      float dist = (agents[own_index].prob->params.x_0.head(2) - agents[own_index].goal).norm(); //distance from goal wp.
      if(dist > cutoff_dist) // 20 cm tolerance to goal
      {
        controls = agents[own_index].UpdateControls();
      }
      if(dist < cutoff_dist)
      {
        if(current_wp_index < max_index-1)
        {
          agents[own_index].goal = waypoints[++current_wp_index];
          viz_publish();
        }
        if(dist < cutoff_dist*0.25 and current_wp_index >= max_index-1)
        {
          ROS_INFO("reached");
          controls[0] = 0;
          controls[1] = 0;
          goal_received = false;
        }
        else
        {
          controls = agents[own_index].UpdateControls(); // in case it is the final waypoint
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
  for(int i = 0; i < 50;i++) //wait for 5 seconds: this can be removed for the real world; it corresponds to the time taken by the rest of the stuff to init
  {
    ros::spinOnce(); // spin but don't plan anything.
    r.sleep();
  }
  local_planner.setup();
  while(ros::ok)
  {
    ros::spinOnce(); //this line wasted 2 hours of my time.
    local_planner.plan();
    r.sleep();
  }
  return 0;
}
