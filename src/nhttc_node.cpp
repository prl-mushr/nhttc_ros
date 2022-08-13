#include "ros/ros.h"
#include "ros/master.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <visualization_msgs/Marker.h>
#include <string>
#include <sstream>
#include "nhttc_interface.h"
#include <boost/algorithm/string.hpp>
#include <random>
#include <chrono>

/**
 * nhttc_ros class.
 *
 * This class provides a ros interface to the nhttc backend
 */
class nhttc_ros
{
public:
  ros::Subscriber sub_other_pose[16],sub_other_control[16], sub_other_WP[16];
  ros::Subscriber sub_goal,sub_wp, sub_marker;
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
  float safety_radius;
  float delivery_tolerance;
  int time_index;
  bool obey_time;
  bool allow_reverse;
  bool adaptive_lookahead;
  bool push_reconfigure;
  bool reconfigure_index_found = false;
  bool push_configuration;
  bool destination_reached = false;
  bool deadlock_solve = false;
  bool deadlock_flag = false;
  bool nhttc_standalone = false;
  bool pure_pursuit = false;
  bool nhttc_speed_only = false;
  float add_noise = 0;
  Eigen::Vector2f mode_switch_pos;
  int reconfigure_index;
  float push_limit_radius;
  float last_time_error = 0, time_error_rate=0;
  float cte = 0, cte_final, max_time_error_rate = 0; // diagnostics variables
  float time_error_thresh;
  float time_ratio = 0;
  std::vector<TTCObstacle*> obstacles = BuildObstacleList(agents);

  std::vector<Eigen::Vector2f> waypoints;
  std::vector<Eigen::VectorXf> waypoints_gen[16];
  std::vector<float> time_stamps;
  std::vector<std::string> name_list;
  int current_wp_index, max_index, max_index_gen[16];
  int priority_count;

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
    if(simulation)
    {
      float dist = agents[i].prob->params.radius;
      x_o[0] -= dist*cosf(x_o[2]);
      x_o[1] -= dist*sinf(x_o[2]);
      if(add_noise != 0)
      {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator (seed);
        std::normal_distribution<double> distribution (0.0,add_noise);
        float disp[2];
        disp[0] = float(distribution(generator));
        disp[1] = float(distribution(generator));
        x_o[0] += disp[0];
        x_o[1] += disp[1];
      }

    }
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
   * Marker callback. Useful to know when to switch between pushing and normal operation.
   *
   * This function listens to the marker messags published by the planner to get the location of the point where we must switch configuration
   * @param takes marker message.
  */
  void MCallback(const visualization_msgs::Marker::ConstPtr& msg)
  {
    if(msg->type == visualization_msgs::Marker::CUBE)
    {
      mode_switch_pos[0] = msg->pose.position.x;
      mode_switch_pos[1] = msg->pose.position.y;
    }
  }

  /**
   * function for calculating the index at which we must reconfigure the controller for normal behaviour with reverse allowed.
  */
  void get_reconfigure_index()
  {
    float min_dist = 10, dist;
    int reconfig_ind = 0;
    for(int i = 0; i < max_index; i++)
    {
      dist = (waypoints[i] - mode_switch_pos).norm();
      if(dist < min_dist)
      {
        min_dist = dist;
        reconfig_ind = i;
      }
    }
    reconfigure_index = reconfig_ind;
    if(nhttc_standalone)
    {
      reconfigure_index = 0; //reconfig_ind; // local to global transfer. I use a local variable to avoid confusion/entanglement.
      waypoints.clear();
      Eigen::Vector2f wp,dummy;
      dummy = waypoints[max_index - 1];
      wp = mode_switch_pos;
      waypoints.push_back(wp);
      waypoints.push_back(dummy);
      current_wp_index = 0;
      goal = waypoints[0];
      goal_received = true;
      agents[own_index].UpdateGoal(goal); // set the goal 
      max_index = 2;
      cur_time_stamp = time_stamps[1];
      time_index = 0; //reset
      begin = ros::Time::now();// + ros::Duration(0.02); // add 0.02 seconds corresponding to the 50 hz update rate.
    }
  }
  /**
  function to reset parameters to normal mode (non-pushing). Do not use unless both pushing and non-pushing modes are needed.
  */
  void update_param_normal()
  {
    agents[own_index].prob->params.safety_radius = safety_radius; // reduce the safety radius
    steer_limit = 0.1*M_PI;
    agents[own_index].prob->params.steer_limit = steer_limit;
    agents[own_index].prob->params.u_lb = allow_reverse ? Eigen::Vector2f(-speed_lim, -steer_limit) : Eigen::Vector2f(0, -steer_limit);
    agents[own_index].prob->params.u_ub = Eigen::Vector2f(speed_lim, steer_limit);
    ROS_INFO("parameters updated!");
  }

  /**
   * Waypoint callback
   *
   * This function listens to the waypoint messages (geometry_msgs::PoseArray) published by the global planner.
   *
   * @param takes the waypoint message.
   */
  void WPCallBack_other(const geometry_msgs::PoseArray::ConstPtr& msg, int j)
  {
    Eigen::VectorXf wp = Eigen::VectorXf::Zero(4);
    int num = msg->poses.size();
    float q[4];
    float time_stamp=0;
    if(j  == own_index)
    {
      time_stamps.clear();
      waypoints.clear(); // reset
    }
    wp[0] = msg->poses[1].position.x - msg->poses[0].position.x;
    wp[1] = msg->poses[1].position.y - msg->poses[0].position.y;
    float time_step = (wp.head(2)).norm()/speed_lim;
    for(int i =0;i<num;i++)
    {
      wp[0] = msg->poses[i].position.x;
      wp[1] = msg->poses[i].position.y;
      q[0] = msg->poses[i].orientation.x;
      q[1] = msg->poses[i].orientation.y;
      q[2] = msg->poses[i].orientation.z;
      q[3] = msg->poses[i].orientation.w;
      wp[2] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),1 - 2 * (q[1] * q[1] + q[2] * q[2]));
      wp[3] = time_stamp;
      waypoints_gen[j].push_back(wp);
      if(j == own_index)
      {
        waypoints.push_back(wp.head(2)); // so that we can maintain backwards compatibility
        time_stamps.push_back(time_stamp); // so that we can maintain backwards compatibility
      }
      time_stamp += (msg->poses[i].position.z*1000)*time_step;
    }
    max_index_gen[j] = num; // save the generalized max index number.
    if(j == own_index)
    {
      current_wp_index = 0;
      goal = waypoints[0];
      goal_received = true;
      agents[own_index].UpdateGoal(goal); // set the goal 
      max_index = num;
      cur_time_stamp = time_stamps[1];
      time_index = 1; //reset
      begin = ros::Time::now();// + ros::Duration(0.02); // add 0.02 seconds corresponding to the 50 hz update rate.
    }
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
    output_msg.pose.position.z = float(destination_reached); // 1 if goal reached
    output_msg.pose.orientation.x = float(deadlock_flag);
    output_msg.pose.orientation.y = cte_final;
    output_msg.pose.orientation.z = time_ratio;
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
              s.str("");
              s<<"/";
              s<<strs[1];
              s<<"/waypoints";
              sub_other_WP[count] = nh.subscribe<geometry_msgs::PoseArray>((s.str()).c_str(), 10, boost::bind(&nhttc_ros::WPCallBack_other,this,_1,count));
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
    if(not nh.getParam("/solver_time",solver_time))
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
    if(not nh.getParam("/carrot_goal_ratio",carrot_goal_ratio))
    {
      carrot_goal_ratio = 1.0f; //default distance to the ever-changing goal
    }
    if(not nh.getParam("/max_ttc",max_ttc))
    {
      max_ttc = 6.0f; // default ttc 
    }
    if(not nh.getParam("/obey_time", obey_time))
    {
      obey_time = false;// false by default
    }
    if(not nh.getParam("/allow_reverse", allow_reverse))
    {
      allow_reverse = true;// true by default (default behavior is to not have any constraints on the nav engine)
    }
    if(not nh.getParam("/adaptive_lookahead",adaptive_lookahead))
    {
      adaptive_lookahead = false;
    }
    if(not nh.getParam("/safety_radius", safety_radius))
    {
      safety_radius = 0.2f; // 0.2 m safety radius by default for pushing
    }
    if(not nh.getParam("/push_configuration", push_configuration))
    {
      push_configuration = false;
    }
    push_reconfigure = push_configuration; // if push configuration is true, then push-reconfiguration will be needed.
    if(not nh.getParam("/push_limit_radius", push_limit_radius))
    {
      push_limit_radius = 1.82f;
    }
    if(not nh.getParam("/delivery_tolerance", delivery_tolerance))
    {
      delivery_tolerance = 0.05; // 5 cm default delivery tolerance
    }
    if(not nh.getParam("/speed_lim", speed_lim))
    {
      speed_lim = 0.4;
    }
    if(not nh.getParam("/deadlock_solve", deadlock_solve))
    {
      deadlock_solve = false; // false by default
    }
    if(not nh.getParam("/add_noise", add_noise))
    {
      add_noise = 0;
    }
    if(not nh.getParam("/time_error_thresh", time_error_thresh))
    {
      time_error_thresh = 0.1f;
    }
    if(not nh.getParam("/nhttc_standalone", nhttc_standalone))
    {
      nhttc_standalone = false;
    }
    if(not nh.getParam("/pure_pursuit", pure_pursuit))
    {
      pure_pursuit = false;
    }
    if(not nh.getParam("/nhttc_speed_only", nhttc_speed_only))
    {
      nhttc_speed_only = false;
    }
    speed_lim = std::min(speed_lim, 1.0f); // gotta limit speed to 1 m/s. Lets not push our luck to far!
    delivery_tolerance = delivery_tolerance > 0.01 ? delivery_tolerance : 0.01;

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
    sub_marker = nh.subscribe("/"+self_name+"/marker",10,&nhttc_ros::MCallback,this);
    // sub_wp = nh.subscribe("/"+self_name+"/waypoints",10,&nhttc_ros::WPCallBack,this);
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
    float steer_limit_default = steer_limit;
    wheelbase = agents[own_index].prob->params.wheelbase;
    if(push_configuration) // basically check if pushing will happen.
    {
      steer_limit = atanf(wheelbase/push_limit_radius);
    }
    float car_radius = 0.25f;

    for(int i = 0; i <=count; i++)
    {
      agents[i].prob->params.radius = car_radius;
      agents[i].prob->params.safety_radius = safety_radius;
      agents[i].prob->params.steer_limit = steer_limit;
      agents[i].prob->params.vel_limit = speed_lim;
      agents[i].prob->params.u_lb = allow_reverse && !(push_reconfigure) ? Eigen::Vector2f(-speed_lim, -steer_limit) : Eigen::Vector2f(0, -steer_limit);
      agents[i].prob->params.u_ub = Eigen::Vector2f(speed_lim,steer_limit);
      agents[i].prob->params.max_ttc = max_ttc;
    }
    turning_radius = wheelbase/tanf(fabs(steer_limit_default));
    fabs(steer_limit) == 0 ? cutoff_dist = 1.0 : cutoff_dist = carrot_goal_ratio*turning_radius; 
    cutoff_dist += agents[own_index].prob->params.radius;
    // print parameters so that the user can confirm them before each run:
    ROS_INFO("running in simulation ? %d", int(simulation));
    ROS_INFO("carrot_goal_ratio: %f",carrot_goal_ratio);
    ROS_INFO("lookahead distance:%f", cutoff_dist);
    ROS_INFO("max_ttc: %f", max_ttc);
    ROS_INFO("solver_time: %d", solver_time);
    ROS_INFO("obey_time:%d", int(obey_time));
    ROS_INFO("allow_reverse: %d", int(allow_reverse));
    ROS_INFO("safety_radius: %f", safety_radius);
    ROS_INFO("adaptive_lookahead, %d", int(adaptive_lookahead));
    ROS_INFO("push_configuration, %d", int(push_configuration));
    ROS_INFO("Max steering_angle, %f", steer_limit*57.3);
    ROS_INFO("delivery_tolerance, %f cm", delivery_tolerance*100);
    ROS_INFO("speed_limit: %f m/s", speed_lim);
    ROS_INFO("deadlock_solve: %d", int(deadlock_solve));
    ROS_INFO("nhttc_speed_only: %d", int(nhttc_speed_only));
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

  bool dtheta_dv(Eigen::VectorXf own_state, Eigen::VectorXf other_state, float Vel[2], float dt, float &d_theta_dv)
  {
    float theta[2];
    float dS[2];
    float numerator, denominator;
    float d_theta;
    dS[0] = Vel[0]*dt;
    dS[1] = Vel[1]*dt;
    numerator = other_state[1] + dS[1]*sinf(other_state[2]) - (own_state[1] + dS[0]*sinf(own_state[2]));
    denominator = other_state[0] + dS[1]*cosf(other_state[2]) - (own_state[0] + dS[0]*cosf(own_state[2]));
    theta[0] = atan2f(numerator,denominator);

    dS[0] = (Vel[0] + 1)*dt; // small incremeent in speed
    numerator = other_state[1] + dS[1]*sinf(other_state[2]) - (own_state[1] + dS[0]*sinf(own_state[2]));
    denominator = other_state[0] + dS[1]*cosf(other_state[2]) - (own_state[0] + dS[0]*cosf(own_state[2]));
    theta[1] = atan2f(numerator,denominator);
    d_theta = theta[1] - theta[0];
    if(d_theta == 0)
      return false;
    d_theta_dv = d_theta; // derivative of theta wrt velocity.
    // ROS_INFO("d_theta: %f", d_theta_dv);
    return true;

  }

  /*
  pure_pursuit controller for setting baseline
  */
  void pure_pursuit_controller(float &speed, float &steering)
  {
    if(!pure_pursuit or !goal_received)
    {
      return;
    }
    Eigen::VectorXf agent_state = agents[own_index].prob->params.x_0; //get agent's current state
    Eigen::Vector2f head_vec = Eigen::Vector2f(cosf(agent_state[2]),sinf(agent_state[2])); // heading vector
    Eigen::Vector2f wp_vec = (agents[own_index].goal - agent_state.head(2));
    // pure pursuit controller. Ref: https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
    float x = wp_vec[1]*head_vec[0] - wp_vec[0]*head_vec[1];
    float l2 = wp_vec.norm();
    l2 *= l2;  // square up mfs
    float k = 2 * x / l2;  // curvature
    steering = atanf(wheelbase*k);

    if(steering > steer_limit)
    {
      steering = steer_limit;
    }
    if(steering < -steer_limit)
    {
      steering = -steer_limit;
    }

    if(destination_reached)
    {
      speed = 0;
    }

    if(not nhttc_speed_only)  // don't modify speed if nhttc is being used for speed calc.
    {  
      speed = agents[own_index].prob->params.vel_limit;
      // we use the nhttc speed unless stuck 
      ROS_INFO("loop start");
      float delta_t = 0.025; // because 4 Hz update rate.
      float theta[16], lambda_ref[16], lambda[16], lambda_err[16]; // Define variables where they are used because we live in 2022 and this is C++ not C. Would have created a {} in C
      float PD[16], delta_v[16];
      float gain = 1;

      Eigen::VectorXf state_target[16], state[16];
      
      state_target[own_index] = waypoints_gen[own_index][time_index];
      state[own_index] = agents[own_index].prob->params.x_0.head(2);
      for(int i = 0;i <= count; i++)
      {
        if(i == own_index)
          continue; // don't measure lambda against yourself, obviously.
        state_target[i] = waypoints_gen[i][0]; // first point of that agent.
        state_target[own_index] = waypoints_gen[own_index][0];
        float theta_start = atan2f(state_target[i][1] - state_target[own_index][1], state_target[i][0] - state_target[own_index][0]);
        // int other_index;
        // for(int iter =0; iter < max_index_gen[i]; iter++)
        // {
        //   if(fabs(cur_time_stamp - waypoints_gen[i][iter][3]) < 0.2) // roughly the same time stamp
        //   {
        //     other_index = iter;
        //     break;
        //   }
        // }
        // ROS_INFO("other index: %d", other_index);
        state_target[i] = waypoints_gen[i][ std::min(max_index_gen[i] -1, time_index) ]; // next waypoint for the other agent.
        state_target[own_index] = waypoints_gen[own_index][time_index];

        float theta_now = atan2f(state_target[i][1] - state_target[own_index][1], state_target[i][0] - state_target[own_index][0]);
        state[i] = agents[i].prob->params.x_0.head(2);

        theta[i] = atan2f( state[i][1] - state[own_index][1], state[i][0] - state[own_index][0]);
        lambda_ref[i] = theta_now - theta_start; // this is the expected winding number at next waypoint, also known as the target winding number
        lambda[i] = theta[i] - theta_start;
        lambda_err[i] = lambda_ref[i] - lambda[i];
        if(lambda_err[i] > M_PI)
          lambda_err[i] -= M_PI;
        else if(lambda_err[i] < -M_PI)
          lambda_err[i] += M_PI;
        float vel[] = {speed, agents[i].prob->params.u_curr[0] };
        if(dtheta_dv(agents[own_index].prob->params.x_0, agents[i].prob->params.x_0, vel, delta_t, PD[i])) // find dtheta/dv around the speed calc'd by timing logic
        {
          delta_v[i] = delta_t* gain * lambda_err[i] / PD[i];
        }
        else
        {
          delta_v[i] = 0;
        }
      }
      float smallest_delta = delta_v[(own_index+1)%count];
      for(int i = 0; i<=count; i++)
      {
        if(i == own_index)
          continue;
        // speed += delta_v[i]; // take average of all for now.
        if(smallest_delta > delta_v[i])
        {
          smallest_delta = delta_v[i];
        }
      }
      ROS_INFO("smallest_delta: %f", smallest_delta);
      smallest_delta = std::min(std::max(smallest_delta,-0.4f),0.4f);
      // speed = std::max(speed + smallest_delta, 0.0f);
    }

  } 

  /**
   * deadlock detection function
   *
   * detect deadlock on the basis of 3 conditions existing simultaneously:
   * condition 1: being closer than 1.5 meters to at least 2 cars
   * condition 2: time_error (time lag from expected plan) greater than 3 seconds
   * condition 3: time_error_rate (rate of change of time_error) greater than 1 second per second. 
  */
  bool deadlock_detected(float dist_error)
  {
    static ros::Time time_since_last_movement;
    static double time_since_stuck;
    int stuck_count = 0;
    priority_count = 0;
    if(goal_received and not destination_reached)
    {
      for(int i = 0; i <= count; i++)
      {
        Eigen::VectorXf agent_state = agents[own_index].prob->params.x_0; //get agent's current state
        Eigen::VectorXf other_state = agents[i].prob->params.x_0; //get agent's current state
        Eigen::Vector2f C = (agents[i].prob->params.x_0.head(2) - agents[own_index].prob->params.x_0.head(2));
        Eigen::Vector2f head_vec = Eigen::Vector2f(cosf(agent_state[2]),sinf(agent_state[2])); // heading vector
        Eigen::Vector2f head_vec_other = Eigen::Vector2f(cosf(other_state[2]),sinf(other_state[2])); // heading vector
        float dist_thresh = 2*(agents[own_index].prob->params.safety_radius + agents[own_index].prob->params.radius);
        float separation = C.norm();
        C = C/separation; // unit vector
        float theta_thresh = atan2f(dist_thresh, separation);
        float theta = acos(head_vec.dot(C));
        float theta_other = acos(head_vec_other.dot(-C));
        float speed_own = agents[own_index].prob->params.u_curr[0];
        float speed_oth = agents[i].prob->params.u_curr[0];
        if(i != own_index and speed_own < speed_lim*0.2 and speed_oth < 0.2*speed_lim and separation < 1.5)
        { //if some other agent is close to me and both of us are moving slow as hell we probably stuck
          stuck_count++;
          if(theta > theta_thresh)
          {
            priority_count = 1; // move bitch.
          }
          else if(theta < theta_thresh and theta_other < theta_thresh)
          {
            priority_count = int(own_index > i);
          }
        }
      }
      if(stuck_count == 0)
      {
        time_since_last_movement = ros::Time::now();  //reset timer
      }
      time_since_stuck = (ros::Time::now() - time_since_last_movement).toSec();
      if(time_since_stuck > time_error_thresh)
      {

        return true;
      }
    }
    return false;
  }

  /*
  * function that tries to solve the deadlock
  * this function should only be called after the parameters have been set by the main code because it will reset them. 
  * So it is necessary that the parameters set here be updated every control cycle in the main loop.
  *
  */
  void solve_deadlock()
  {
    if(not deadlock_solve)
    {
      return;
    }
    agents[own_index].prob->params.steer_limit = 0.1*M_PI; // very large swing
    agents[own_index].prob->params.u_lb = allow_reverse ? Eigen::Vector2f(-speed_lim, -0.1*M_PI) : Eigen::Vector2f(0, -0.1*M_PI);
    agents[own_index].prob->params.u_ub = Eigen::Vector2f(speed_lim, 0.1*M_PI);

    for(int i = 0; i <= count; i++)
    {
      if(i != own_index)
      {
        agents[i].prob->params.u_curr[0] = std::max(0.2f,agents[i].prob->params.u_curr[0]); // max of true and hallucinated speed.
      }
    }
    obstacles = BuildObstacleList(agents);
    agents[own_index].SetObstacles(obstacles, size_t(own_index)); // set the obstacles
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
      Eigen::Vector2f head_vec = Eigen::Vector2f(cosf(agent_state[2]),sinf(agent_state[2])); // heading vector
      //if we have a goal
      if(goal_received)
      { 
        if(not reconfigure_index_found)
        {
          get_reconfigure_index();
          ROS_INFO("reconfigure_index found at %d",reconfigure_index);
          reconfigure_index_found = true; // Do this calc once.
          update_param_normal();
        }
        if(nhttc_standalone and push_configuration and current_wp_index == 1)
        {
          agents[own_index].prob->params.safety_radius = safety_radius; // reduce the safety radius
          steer_limit = atanf(wheelbase/push_limit_radius);
          agents[own_index].prob->params.steer_limit = steer_limit;
          agents[own_index].prob->params.u_lb = allow_reverse ? Eigen::Vector2f(-speed_lim, -steer_limit) : Eigen::Vector2f(0, -steer_limit);
          agents[own_index].prob->params.u_ub = Eigen::Vector2f(speed_lim, steer_limit);
          ROS_INFO("parameters updated!");
        }
        // else
        // {
        //   if(current_wp_index == reconfigure_index+1 and push_configuration)
        //   {
        //     push_configuration = false;
        //     // update_param_normal();
        //     // add our block as a static agent to avoid. This is not ideal, should be updated to get the block's actual location.
        //     agent_setup(count+1);
        //     Eigen::VectorXf block_x_o = Eigen::VectorXf::Zero(3);
        //     Eigen::VectorXf block_controls = Eigen::VectorXf::Zero(2);
        //     block_x_o[0] = waypoints[reconfigure_index][0];
        //     block_x_o[1] = waypoints[reconfigure_index][1];
        //     block_x_o[2] = 0;
        //     agents[count+1].prob->params.safety_radius = 0;
        //     agents[count+1].prob->params.radius = 0.1;
        //     agents[count+1].prob->params.max_ttc = max_ttc;
        //     agents[count+1].SetEgo(block_x_o);
        //     agents[count+1].SetControls(block_controls);
        //   }
        // }
                // Find distance to current nhttc waypoint

        agents[own_index].SetEgo(agent_state); // This is such a bad way of doing things. No semaphore locks. What if the callback changes the value right after this line? #POSSIBLE BUG

        Eigen::Vector2f wp_vec = (agents[own_index].goal - agent_state.head(2)); // vector joining agent to waypoint
        cte = fabs(wp_vec[0]*head_vec[1] - wp_vec[1]*head_vec[0]);  //cross track error for measurement purposes.
        if(simulation)
        {
          cte_final = (waypoints[max_index-1] - (agent_state.head(2) + agents[own_index].prob->params.radius*Eigen::Vector2f(cosf(agent_state[2]), sinf(agent_state[2])) ) ).norm();  // final cross track error ##METRIC
        }
        else
        {
          cte_final = (waypoints[max_index-1] - (agent_state.head(2))).norm();
        }
        float multiplier = fabs(wp_vec.dot(head_vec));
        float dist = multiplier; //distance from goal wp.
        if(nhttc_standalone)
        {
          dist = wp_vec.norm();
        }
        // now search for the nearest waypoint thats still ahead of me.
        // note, this depends on previously calculated heading vector. This evaluation works similarly to the above calc.
        Eigen::Vector2f tp_vec = (waypoints[time_index] - agent_state.head(2)); // tp: time point
        multiplier = fabs(tp_vec.dot(head_vec));
        float time_point_dist = multiplier; // removed the multiplier here, because in if the waypoints are intentionally behind the car, this would just skip them. we don't want that.
        if(time_point_dist < 0.1)  // there should be an additional condition that prevents time index from being less than waypoint index
        {
          time_index = std::min(max_index - 1, time_index + 1);
          time_viz_publish();
          // re-evaluate the time_point_distance
          tp_vec = (waypoints[time_index] - agent_state.head(2));
          multiplier = fabs(tp_vec.dot(head_vec));
          time_point_dist = multiplier;
        }

        float current_time = float((ros::Time::now() - begin).toSec()); //the current time relative to the time at which the plans were published.
        cur_time_stamp = time_stamps[time_index];  // time stamp by which the car must arrive at the time-waypoint. The time waypoint is simply the next closest waypoint in the plan.

        if(obey_time)
        {
          float delta_time = cur_time_stamp - current_time; //time left to reach the time-waypoint
          float virtual_dist = delta_time*speed_lim; // distance from the time-waypoint at which the car should be if it had to travel at the default speed
          float dist_error = time_point_dist - virtual_dist; // error between the expected distance-to-go and the actual distance-to-go
          // I can detect a stall from the dist_error*10. 
          float delta_speed = std::min(std::max(dist_error,-speed_lim),speed_lim*2); // a Proportional controller for changing the speed with some min-max limits
          // ROS_INFO("dist_error: %f", dist_error);
          // setting up the parameters for timing. This also resets the parameters as soon as the deadlock is gone.
          agents[own_index].prob->params.vel_limit = speed_lim + delta_speed; //set the new speed limit 
          agents[own_index].prob->params.steer_limit = steer_limit; 
          agents[own_index].prob->params.u_lb = allow_reverse ? Eigen::Vector2f(-(speed_lim + delta_speed), -steer_limit) : Eigen::Vector2f(0, -steer_limit); // set the upper and lower bound corresponding to this limit
          agents[own_index].prob->params.u_ub = Eigen::Vector2f(speed_lim + delta_speed,steer_limit);
          
          deadlock_flag = false;
          agents[own_index].prob->params.safety_radius = safety_radius;
          if(deadlock_detected(dist_error))
          {
            ROS_INFO("DEADLOCK DETECTED");
            deadlock_flag = true;
            viz_publish();
            solve_deadlock();
          }
          if(not deadlock_flag and current_wp_index > 2 and current_wp_index < max_index-1) // prevent updation if in deadlock or just started moving or about to halt
          {
            max_time_error_rate = max_time_error_rate*0.9 + (dist_error/speed_lim)*0.1; // running average of sorts. ##METRIC
          }
        }
        time_ratio = current_time/cur_time_stamp; // ratio of the actual time over expected time at any point in time #METRIC

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
            if(0 and current_wp_index == reconfigure_index and dist*wp_vec.dot(head_vec) > delivery_tolerance and push_configuration) // last condition tests whether the wp has been overshot or not
            {  
              ROS_INFO("zeroing in on delivery location! %f", dist);
              agents[own_index].prob->params.safety_radius = 0;
              controls = agents[own_index].UpdateControls();
            }
            else
            {
              current_wp_index = std::min(max_index - 1, current_wp_index + 1);
              agents[own_index].goal = waypoints[current_wp_index];
              controls = agents[own_index].UpdateControls();
              viz_publish(); // publish new goal point 
            }
          }
          else
          {
            // agents[own_index].prob->params.safety_radius = 0;
            controls = agents[own_index].UpdateControls(); // in case it is the final waypoint, keep going until dist-to-go is less than wheelbase

            if(current_wp_index >= max_index-1 and dist < agents[own_index].prob->params.radius)
            {
              controls[0] = 0;
              controls[1] = 0;
              goal_received = false;
              ROS_INFO("kill power");
              destination_reached = true;
              viz_publish();
            }
          }
        }
      }
      // controls[0] = push_configuration ? std::max(0.0f, controls[0]) : controls[0];
      float speed = controls[0]; //speed in m/s
      float steering_angle = controls[1]; //steering angle in radians. +ve is left. -ve is right
      pure_pursuit_controller(speed,steering_angle);  // call this function to make use of pure pursuit code
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
