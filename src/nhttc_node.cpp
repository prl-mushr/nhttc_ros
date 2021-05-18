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

/**
 * nhttc_ros class.
 *
 * This class provides a ros interface to the nhttc backend
 */
class nhttc_ros
{
public:
  ros::Subscriber sub_other_pose[16],sub_other_control[16];
  ros::Subscriber sub_goal,sub_wp;
  ros::Publisher pub_cmd, viz_pub, time_pub;

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
  float turning_radius;
  float carrot_goal_ratio;
  float max_ttc;
  SGDOptParams global_params;
  std::vector<Agent> agents; //all agents
  int count;
  float speed_lim = 0.4f;
  float cur_time_stamp;
  int time_index;
  bool obey_time;
  bool allow_reverse;
  bool adaptive_lookahead;

  std::vector<TTCObstacle*> obstacles = BuildObstacleList(agents);

  std::vector<Eigen::Vector2f> waypoints;
  std::vector<float> time_stamps;
  std::vector<std::string> name_list;
  int current_wp_index, max_index;

  ros::master::V_TopicInfo master_topics;

  ros::Time begin;
  /**
   * setting up agents
   *
   * This function initializes the nhttc agents with a preset starting pose and goal
   *
   * @param takes the index value corresponding to that agent
   */
  void agent_setup(int i)
  {
    Eigen::Vector2f goal(0.0,0.0);
    Eigen::VectorXf pos = Eigen::VectorXf::Zero(3);
    agents.emplace_back(GetAgentParts(6, pos, true, goal), global_params);
  }

  /**
   * Quaternion to euler angle converter
   *
   * Converts quaternion representation to euler angle representation. rpy = [roll, pitch, yaw]. rpy is in radians, following the ENU reference frame
   * @params euler angle array (float), pose message (geometry_msgs::PoseStamped)
   */
  void rpy_from_quat(float rpy[3],const geometry_msgs::PoseStamped::ConstPtr& msg) 
  { 
    float q[4];
    q[0] = msg->pose.orientation.x;
    q[1] = msg->pose.orientation.y;
    q[2] = msg->pose.orientation.z;
    q[3] = msg->pose.orientation.w;
    rpy[2] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),1 - 2 * (q[1] * q[1] + q[2] * q[2]));
    rpy[0] = asinf(2.0f * (q[0] * q[2] - q[3] * q[1]));
    rpy[1] = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
  }

  /**
   * Pose callback
   *
   * This function listens to the pose-publishers of all the agents
   *
   * @param takes the pose msg (geometry_msgs::PoseStamped) and the index value corresponding to the agent.
   */
  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int i)
  {
    float rpy[3];
    Eigen::VectorXf x_o = Eigen::VectorXf::Zero(3);
    rpy_from_quat(rpy,msg);
    x_o[0] = msg->pose.position.x;
    x_o[1] = msg->pose.position.y;
    x_o[2] = rpy[2];
    agents[i].SetEgo(x_o);
  }

  /**
   * Control callback
   *
   * This function listens to the control-publishers of all the agents. This is required by the nhttc as it takes into account the current state and actions of the other agents
   *
   * @param takes the control msg (ackermann_msgs::AckermannDriveStamped) and the index value corresponding to the agent.
   */
  void ControlCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg, int i)
  {
    Eigen::VectorXf controls = Eigen::VectorXf::Zero(2);
    controls[0] = msg->drive.speed;
    controls[1] = msg->drive.steering_angle;
    if(i!=own_index)
    {
      agents[i].SetControls(controls);
    }
  }

  /**
   * Waypoint callback
   *
   * This function listens to the waypoint messages (geometry_msgs::PoseArray) published by the global planner.
   *
   * @param takes the waypoint message.
   */
  void WPCallBack(const geometry_msgs::PoseArray::ConstPtr& msg)
  {
    Eigen::Vector2f wp,wp_last;
    int num = msg->poses.size();
    waypoints.clear(); // reset
    float time_stamp=0;
    time_stamps.clear();
    current_wp_index = 0;
    wp[0] = msg->poses[1].position.x - msg->poses[0].position.x;
    wp[1] = msg->poses[1].position.y - msg->poses[0].position.y;
    float time_step = wp.norm()/speed_lim;
    for(int i =0;i<num;i++)
    {
      wp[0] = msg->poses[i].position.x;
      wp[1] = msg->poses[i].position.y;
      time_stamps.push_back(time_stamp);
      time_stamp += (msg->poses[i].position.z*1000)*time_step;
      waypoints.push_back(wp);
    }
    goal_received = true;
    goal = waypoints[current_wp_index]; 
    agents[own_index].UpdateGoal(goal); // set the goal 
    max_index = num;
    cur_time_stamp = time_stamps[1];
    time_index = 1; //reset
    begin = ros::Time::now();// + ros::Duration(0.02); // add 0.02 seconds corresponding to the 50 hz update rate.
  }

  /**
   * Single-goal callback
   *
   * Used for a single-goal point mission.
   *
   * @param goal point message (geometry_msgs::PoseStamped)
   */
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    goal[0] = msg->pose.position.x;
    goal[1] = msg->pose.position.y;
    agents[own_index].UpdateGoal(goal);
    goal_received = true;
  }

  /**
   * Send throttle/steering commands
   *
   * sends commands to the vesc/mux/input/navigation topic
   *
   * @param speed (float, m/s) steering angle (float, radius)
   */
  void send_commands(float speed, float steer)
  {
    ackermann_msgs::AckermannDriveStamped output_msg;
    output_msg.header.stamp = ros::Time::now();
    output_msg.header.frame_id = "";
    output_msg.drive.steering_angle = steer;
    output_msg.drive.speed = speed;
    pub_cmd.publish(output_msg);
  }
  /**
   * Publish position of current goal-point for visualization
   *
   * Publishes the current nhttc-goal to the /car_/cur_goal topic. Used for visualizing where the car is currently trying to go.
   */
  void viz_publish()
  {
    geometry_msgs::PoseStamped output_msg;
    output_msg.header.stamp = ros::Time::now();
    output_msg.header.frame_id = "/map";
    output_msg.pose.position.x = agents[own_index].goal[0];
    output_msg.pose.position.y = agents[own_index].goal[1];
    viz_pub.publish(output_msg);
  }

  /**
   * Publish position of current time-point for visualization
   *
   * Publishes the current time-point to the /car_/time_goal topic. Used for visualizing the point with which the car is trying to
   * synchronize it's timing.
   */
  void time_viz_publish()
  {
    geometry_msgs::PoseStamped output_msg;
    output_msg.header.stamp = ros::Time::now();
    output_msg.header.frame_id = "/map";
    output_msg.pose.position.x = waypoints[time_index][0];
    output_msg.pose.position.y = waypoints[time_index][1];
    time_pub.publish(output_msg);
  }

  /**
   * Check for new agents
   *
   * updates the agent array by finding published topics with topic type PoseStamped and unique car_name.
   * @params: ros::NodeHandle
   * @returns: None
   */
  void check_new_agents(ros::NodeHandle &nh)
  {
    ros::master::getTopics(master_topics);

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) 
    {
      const ros::master::TopicInfo& info = *it;
      std::vector<std::string> strs;
      if(info.datatype == "geometry_msgs/PoseStamped")
      {
        boost::split(strs, info.name, boost::is_any_of("/"));
        if(!strs.empty())
        {
          if(strs[2]=="car_pose" or strs[2]=="mocap_pose")
          {
            bool common = false;
            for(int i=0;i<=name_list.size() and name_list.size()!=0 ;i++)
            {
              if(strs[1]==name_list[i])
              {
                common = true;
              }
            }
            if(not common)
            {
              ROS_INFO("car found:%s",strs[1].c_str());
              name_list.push_back(strs[1]);
              count++;
              if(strs[1] == self_name)
              {
                own_index = count;
              }
              agent_setup(count);

              sub_other_pose[count] = nh.subscribe<geometry_msgs::PoseStamped>(info.name,10,boost::bind(&nhttc_ros::PoseCallback,this,_1,count));
              s.str("");
              s<<"/";
              s<<strs[1];
              s<<"/mux/ackermann_cmd_mux/input/navigation";
              sub_other_control[count] = nh.subscribe<ackermann_msgs::AckermannDriveStamped>((s.str()).c_str(),10,boost::bind(&nhttc_ros::ControlCallback,this,_1,count));
            }
          }
        }
      }
    }
  }

  /**
   * nhttc ros constructor
   *
   * Sets up the publishers, subscribers and the whole shebang
   */
  nhttc_ros(ros::NodeHandle &nh)
  {
    own_index = -1;
    goal_received = false; // start with the assumption that the car has no goal
    current_wp_index = 0; // set current waypoint index to 0
    max_index = 0; //set max_waypoint index to 0
    // get all the params
    nh.getParam("car_name",self_name);
    if(not nh.getParam("solver_time",solver_time))
    {
      solver_time = 10; // 10 ms solver time for each agent.
    }
    if(not nh.getParam("sim",simulation))
    {
      simulation = true; // run in simulation by default.
    }
    if(not nh.getParam("max_agents",num_agents_max))
    {
      num_agents_max = 8; // default maximum number of agents
    }
    if(not nh.getParam("carrot_goal_ratio",carrot_goal_ratio))
    {
      carrot_goal_ratio = 1.0f; //default distance to the ever-changing goal
    }
    if(not nh.getParam("max_ttc",max_ttc))
    {
      max_ttc = 6.0f; // default ttc 
    }
    if(not nh.getParam("obey_time", obey_time))
    {
      obey_time = false;// false by default
    }
    if(not nh.getParam("allow_reverse", allow_reverse))
    {
      allow_reverse = true;// true by default (default behavior is to not have any constraints on the nav engine)
    }
    if(not nh.getParam("adaptive_lookahead",adaptive_lookahead))
    {
      adaptive_lookahead = false;
    }

    ConstructGlobalParams(&global_params);
    count = -1; //number of agents. start from -1.
    // sleep rate. Wait for all the agents to be set up before you start counting.
    /*
     * The following code-block listens to all the published topics and filters out those which end with /car_pose or /mocap_pose.
     * When it finds the corresponding topic, it creates the subscribers/publishers for that agent and increases the count for the number of agents.
     */
    ros::Rate r(1);
    for(int i=0;i<5;i++)
    {
      check_new_agents(nh);
      r.sleep();
    } // We do this once at the beginning in order to find the car belonging to our own nav controller
    // set up all the publishers/subscribers
    sub_wp = nh.subscribe("/"+self_name+"/waypoints",10,&nhttc_ros::WPCallBack,this);
    pub_cmd = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/"+ self_name +"/mux/ackermann_cmd_mux/input/navigation",10);
    viz_pub = nh.advertise<geometry_msgs::PoseStamped>("/"+self_name+"/cur_goal",10);
    time_pub = nh.advertise<geometry_msgs::PoseStamped>("/"+self_name+"/time_goal",10);
    ROS_INFO("node started"); // tell the world that the node has initialized.
  }

  /**
   * Parameter setting for the ego-agent.
   *
   * Sets up the tuning parameters for the ego-agent. These parameters are taken from the launch file. The tuning parameters include 
   * 1) The carrot-goal ratio: (lookahead distance)/(turning radius of the car)
   * 2) max_ttc: maximum time-to-collision
   */
  void setup()
  {
    steer_limit = 0.1*M_PI; // max steering angle ~18 degrees. :(. I wanted to drift with the MuSHR car. 
    wheelbase = agents[own_index].prob->params.wheelbase;
    agents[own_index].prob->params.steer_limit = steer_limit;
    agents[own_index].prob->params.vel_limit = speed_lim;
    agents[own_index].prob->params.u_lb = allow_reverse ? Eigen::Vector2f(-speed_lim, -steer_limit) : Eigen::Vector2f(0, -steer_limit);
    agents[own_index].prob->params.u_ub = Eigen::Vector2f(speed_lim,steer_limit);
    agents[own_index].prob->params.max_ttc = max_ttc;
    ROS_INFO("max_ttc: %f, carrot_goal_ratio: %f",max_ttc, carrot_goal_ratio);
    turning_radius = wheelbase/tanf(fabs(steer_limit));
    fabs(steer_limit) == 0 ? cutoff_dist = 1.0 : cutoff_dist = carrot_goal_ratio*turning_radius; 
    cutoff_dist += agents[own_index].prob->params.radius;
    ROS_INFO("carrot_goal_dist: %f",cutoff_dist);
    ROS_INFO("obey_time:%d",int(obey_time));
    ROS_INFO("adaptive_lookahead, %d", int(adaptive_lookahead));
  }

  /**
  * do_skip_wp
  * 
  * checks if the current waypoint should be skipped or not based on whether it sits within the turning circle
  */
  bool do_skip_waypoint(Eigen::Vector2f wp_vec, float th)
  {
    if(!adaptive_lookahead)
    {
      return false;
    }
    Eigen::Vector2f turning_center_right,turning_center_left;
    float turning_radius = wheelbase/tanf(fabs(steer_limit));
    // X is forward, Y is left
    turning_center_left = turning_radius*Eigen::Vector2f(cosf(M_PI*0.5 + th), sinf(M_PI*0.5 - th)); // rotate the turning center into world frame
    turning_center_right = turning_radius*Eigen::Vector2f(cosf(M_PI*0.5 - th), sinf(-M_PI*0.5 + th));
    // find the distance between the current waypoint and the turning centers in the car's body frame.
    turning_center_left -= wp_vec;
    turning_center_right -= wp_vec;
    if(turning_center_right.norm() < turning_radius - 0.1125 or turning_center_left.norm() < turning_radius - 0.1125)
    {
      return true;
    }
    return false;
  } 


  /**
   * local planner
   *
   * Calls the nhttc solver and sends the output of the solver to the ego-agent. Note that the solver solves only for one agent but it does need
   * to know what the other agents are doing. 
   */
  void plan()
  {
    // create obstacle list.
    if(agents.size()>0)
    {
      obstacles = BuildObstacleList(agents);
      agents[own_index].SetPlanTime(solver_time); //20 ms planning window TODO: see if this only needs to be done once
      agents[own_index].SetObstacles(obstacles, size_t(own_index)); // set the obstacles 

      Eigen::VectorXf controls = Eigen::VectorXf::Zero(2); //controls are 0,0 by default.
      Eigen::VectorXf agent_state = agents[own_index].prob->params.x_0; //get agent's current state
      //if we have a goal
      if(goal_received)
      { 
        // Find distance to current nhttc waypoint  
        Eigen::Vector2f wp_vec = (agents[own_index].goal - agent_state.head(2)); // vector joining agent to waypoint
        Eigen::Vector2f head_vec = Eigen::Vector2f(cosf(agent_state[2]),sinf(agent_state[2])); // heading vector 
        float dist = wp_vec.norm(); //distance from goal wp taking into account the aspect angle. If the point is perpendicular to my direction of motion, I have probably passed it.
        // now search for the nearest waypoint thats still ahead of me.
        // note, this depends on previously calculated heading vector. This evaluation works similarly to the above calc.
        Eigen::Vector2f tp_vec = (waypoints[time_index] - agent_state.head(2)); // tp: time point
        float multiplier = tp_vec.dot(head_vec)>0? 1.0f : -1.0f;
        float time_point_dist = multiplier*tp_vec.norm();
        if(time_point_dist < agents[own_index].prob->params.safety_radius)
        {
          time_index++;
          time_viz_publish();
          // re-evaluate the time_point_distance
          tp_vec = (waypoints[time_index] - agent_state.head(2));
          multiplier = tp_vec.dot(head_vec)>0? 1.0f : -1.0f;
          time_point_dist = multiplier*tp_vec.norm();
        }

        if(obey_time)
        {
          float current_time = float((ros::Time::now() - begin).toSec()); //the current time relative to the time at which the plans were published.
          cur_time_stamp = time_stamps[time_index];  // time stamp by which the car must arrive at the time-waypoint. The time waypoint is simply the next closest waypoint in the plan.
          float delta_time = cur_time_stamp - current_time; //time left to reach the time-waypoint
          float virtual_dist = delta_time*speed_lim; // distance from the time-waypoint at which the car should be if it had to travel at the default speed
          float dist_error = time_point_dist - virtual_dist; // error between the expected distance-to-go and the actual distance-to-go
          // I can detect a stall from the dist_error*10. 
          float delta_speed = std::min(std::max(dist_error*10,-speed_lim),speed_lim*2); // a Proportional controller for changing the speed with some min-max limits
          agents[own_index].prob->params.vel_limit = speed_lim + delta_speed; //set the new speed limit 
          agents[own_index].prob->params.u_lb = allow_reverse ? Eigen::Vector2f(-(speed_lim + delta_speed), -steer_limit) : Eigen::Vector2f(0, -steer_limit); // set the upper and lower bound corresponding to this limit
          agents[own_index].prob->params.u_ub = Eigen::Vector2f(speed_lim + delta_speed,steer_limit);
        }

        if(do_skip_waypoint(wp_vec,agent_state[2]) and current_wp_index < max_index - 1) //second condition is to ensure it doesn't finess the last waypoint
        {
          dist = cutoff_dist - 0.1;  // set the distance to be just below cutoff. Safe as long as dist is used for decision making only
        }
        if(dist > cutoff_dist) // cutoff distance is the tolerance to goal. When goal-distance is less than cutoff, the goal is assumed to be reached
        {
          controls = agents[own_index].UpdateControls();
        }
        if(dist < cutoff_dist)
        {
          if(current_wp_index < max_index-1)
          {
            agents[own_index].goal = waypoints[++current_wp_index];
            viz_publish(); // publish new goal point 
          }
          if(dist < wheelbase and current_wp_index >= max_index-1) //condition for having reached the final waypoint. the cutoff distance for the final waypoint is less than the standard cutoff. This can later be extended to "must-pass" waypoints.
          {
            controls[0] = 0;
            controls[1] = 0;
            goal_received = false;
          }
          else
          {
            controls = agents[own_index].UpdateControls(); // in case it is the final waypoint, keep going until dist-to-go is less than wheelbase
          }
        }
      }
      float speed = controls[0]; //speed in m/s
      float steering_angle = controls[1]; //steering angle in radians. +ve is left. -ve is right 
      send_commands(speed,steering_angle); //just sending out anything for now;
      return;
    }
  }
};

/**
 * main function
 *
 * Sets up the nhttc object and initializes everything.
 */
int main(int argc, char** argv)
{
  ros::init(argc,argv,"nhttc_local_planner");
  ros::NodeHandle nh("~");
  nhttc_ros local_planner(nh);
  ros::Rate r(40);
  bool init = false; //flag for node initialization (indicates if the car's car_pose topic has been found)

  while(ros::ok)
  {
    ros::spinOnce(); 
    // local_planner.check_new_agents(nh);
    if(local_planner.own_index != -1 and !init) //if car had not been initialized before and has now found the car_pose topic
    {
      init = true; // set init to true
      local_planner.setup(); // set up the agent
    }
    if(init) //if init
    {
      local_planner.plan();
    }
    r.sleep();
  }
  return 0;
}
