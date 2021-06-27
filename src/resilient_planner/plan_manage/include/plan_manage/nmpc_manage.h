#ifndef _NMPC_MANAGE_H_
#define _NMPC_MANAGE_H_

//other dependences
#include <plan_manage/nmpc_utils.h>


class NMPCManage
{

public:
  NMPCManage(){}
  ~NMPCManage(){}

  enum FSM_EXEC_STATE
  {
    INIT,
    WAIT_TARGET,
    INIT_YAW,
    GEN_NEW_TRAJ,
    REPLAN_TRAJ,
    EXEC_TRAJ,
    EMERGENCY_STOP
  };

  void init(ros::NodeHandle& nh);
  std::vector<Eigen::Vector3d> kino_path_;


private:

  /* front-end map and path*/
  
  tgk_planner::OccMap::Ptr env_ptr_;

  /* fsm parameters */
  FSM_EXEC_STATE exec_state_;
  int wps_index_;

  /* odom variables */
  Eigen::VectorXd stateOdom_, stateOdomPrevious_, stateMpc_;
  double init_yaw_;
  Eigen::Vector3d end_pt_, start_acc_, end_acc_, external_acc_, last_external_acc_;
  double thrust_acc_ = 0;
  //P v euler;  // odometry state


  /* decision variables */
  bool have_traj_ = false;
  bool have_target_ = false;
  bool have_odom_ = false;
  bool have_extforce_  = false;

  bool trigger_ = false;
  bool exec_mpc_ = false;

  bool replan_force_surpass_ = false;
  bool call_escape_emergency_ = false;
  bool call_init_yaw_ = false;
  bool consider_force_ = false;

  int surpass_count_ = 0;
  int plan_fail_count_ = 0;
  /* MPC output */
  resilient_planner::NMPCSolver nmpc_solver_;

  double ext_noise_bound_;
  double odomT;
  ros::Time tOdom, tMpc;


  /* ROS  related */
  ros::Subscriber odom_sub_, waypoint_sub_, extforce_sub_;
  ros::Timer exec_timer_, safety_timer_, adjust_yaw_timer_, mpc_timer_, waypoint_timer_;
  ros::Publisher odom_vis_pub_, path_pub_, poly_pub_, pos_cmd_pub_, path_point_pub_, nmpc_point_pub_ , trajectory_pub_, goal_point_pub_;

  /* FSM related */ 
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  void execFSMCallback(const ros::TimerEvent& e);
  void checkReplanCallback(const ros::TimerEvent& e);
  void adjustYawCallback(const ros::TimerEvent& e);
  void mpcCallback(const ros::TimerEvent& e);

  /* ROS callbacks */
  void odometryCallback(const nav_msgs::Odometry& msg);
  void odometryTransCallback(const nav_msgs::Odometry& msg);
  void waypointCallback(const nav_msgs::PathConstPtr& msg);
  void extforceCallback(const geometry_msgs::WrenchStamped& msg);
  //void thrustCallback(const std_msgs::Float64& msg);
  /* visualization */
  void displayPath();  
  void displayGoalPoint();

};



#endif
