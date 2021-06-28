#include <plan_manage/nmpc_manage.h>

using namespace std;

void NMPCManage::init(ros::NodeHandle &nh)
{
  ROS_INFO_STREAM("NMPCManage::init\n");

  exec_state_ = FSM_EXEC_STATE::INIT;

  int sim_odom_type;
  nh.param("nmpc/sim_odom_type", sim_odom_type, 1);
  nh.param("nmpc/ext_noise_bound", ext_noise_bound_, 0.5);

  stateOdom_ = Eigen::VectorXd::Zero(9);
  stateOdomPrevious_ = Eigen::VectorXd::Zero(9);
  stateMpc_ = Eigen::VectorXd::Zero(9);

  /*  map intial  */
  env_ptr_.reset(new OccMap);
  env_ptr_->init(nh);

  nmpc_solver_.initROS(nh, env_ptr_);

  goal_sub_ = nh.subscribe("/goal_topic", 1, &NMPCManage::goalCallback, this);

  // publishers
  path_pub_ = nh.advertise<nav_msgs::Path>("kino_path", 1, true);
  goal_point_pub_ = nh.advertise<visualization_msgs::Marker>("goal_point", 2);
  // subscribers

  if (sim_odom_type == 1)
  { // use the imu
    odom_sub_ = nh.subscribe("/odom_world", 1, &NMPCManage::odometryCallback, this);
  }
  else
  { // use rotorS(velocity is in the body frame)
    odom_sub_ = nh.subscribe("/odom_world", 1, &NMPCManage::odometryTransCallback, this);
  }

  extforce_sub_ = nh.subscribe("/forces", 1, &NMPCManage::extforceCallback, this);

  // /forces_estimation/thrust
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &NMPCManage::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &NMPCManage::checkReplanCallback, this);
  mpc_timer_ = nh.createTimer(ros::Duration(0.05), &NMPCManage::mpcCallback, this); // 50ms
}


void NMPCManage::mpcCallback(const ros::TimerEvent &e)
{
  if (!exec_mpc_)  return;

  int status = nmpc_solver_.solveNMPC(stateOdom_, external_acc_);
  //  1 --- success
  //  0 --- at global end
  // -1 --- reaching global end process but not finished
  // -2 --- need replan
  // -3 --- odom far away from predict state
  switch (status)
  {
    case 1:
    {
      ROS_INFO_STREAM(" [NMPCSolver] Success ! ");
      break;
    }
    case 0:
    {
      ROS_INFO_STREAM(" [NMPCSolver] Reach the global end ! ");
      exec_mpc_ = false;
      have_target_ = false;
      ros::Duration(0.1).sleep();
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }
    case -1:
    {
      ROS_INFO_STREAM(" [NMPCSolver] Start to reach the global end ! ");
      ros::Duration(0.1).sleep();
      break;
    }
    case -2:
    {
      ROS_INFO_STREAM(" [NMPCSolver] MPC back-end needs replan ! ");
      exec_mpc_ = false;
      changeFSMExecState(REPLAN_TRAJ, "FSM");
      break;
    }
    case -3:
    {
      ROS_INFO_STREAM(" [NMPCSolver] Odom far away from predict state ! ");
      exec_mpc_ = false;
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

  }
}

static string state_str[8] = {"INIT", "WAIT_TARGET", "INIT_YAW", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ"};

void NMPCManage::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
{
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void NMPCManage::execFSMCallback(const ros::TimerEvent &e)
{
  // trigger is for the goal
  // have goal is for the planning
  // if the goal changes, then we should set trigger as true and update new goal

  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100)
  {
    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
    if (!have_odom_)
      cout << "no odom." << endl;
    if (!have_target_)
      cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state_)
  {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (have_target_)
      {
        changeFSMExecState(WAIT_TARGET, "FSM");
      }
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
      {
        consider_force_ = false;
        return;
      }
      else
      { 
        changeFSMExecState(INIT_YAW, "FSM");
        call_init_yaw_ = true;
      }
      break;
    }

    case INIT_YAW:
    {
      if (call_init_yaw_)
      {
        Eigen::Vector3d dir = end_pt_ - stateOdom_.segment(0, 3);
        init_yaw_ = atan2(dir(1), dir(0));

        if (abs(stateOdom_(8) - init_yaw_) >= 0.8)
        {
          nmpc_solver_.callInitYaw(stateOdom_, init_yaw_);
        }
        call_init_yaw_ = false;
        break;
      }

      if (abs(stateOdom_(8) - init_yaw_) < 0.8)
      {
        consider_force_ = true;
        std::cout << "[resilient_planner] Finish init yaw ! The odom is: " << stateOdom_(8) << ", yaw is : " << init_yaw_ << std::endl;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {
      exec_mpc_ = false;

      std::cout << "[resilient_planner] Plan_fail_count_ ：" << plan_fail_count_ << std::endl;
      if (plan_fail_count_ > 3){
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        plan_fail_count_ = 0;
        break;
      }

      if (nmpc_solver_.getKinoPath(stateOdom_, end_pt_, external_acc_, false))
      {
        std::cout << "[resilient_planner] Kino plan success!" << std::endl;
        have_traj_ = true;
        trigger_ = false;
        exec_mpc_ = true;
        replan_force_surpass_ = false;
        nmpc_solver_.getKinoTraj(kino_path_);
        displayPath();
        plan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        plan_fail_count_ += 1;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      break;
    }

    case REPLAN_TRAJ:
    {
      std::cout << "[resilient_planner] Replan_fail_count_ ：" << plan_fail_count_ << std::endl;
      exec_mpc_ = false;

      if (plan_fail_count_ > 3){
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        plan_fail_count_ = 0;
        break;
      }

      if (nmpc_solver_.getKinoPath(stateOdom_, end_pt_, external_acc_, true)){
        std::cout << "[resilient_planner] Kino replan success!" << std::endl;
        have_traj_ = true;
        trigger_ = false;
        exec_mpc_ = true;
        replan_force_surpass_ = false;
        nmpc_solver_.getKinoTraj(kino_path_);
        displayPath();
        plan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        plan_fail_count_ += 1;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      // change the target
      if (trigger_ && exec_mpc_)
      {
        cout << "exec_mpc_." << exec_mpc_ << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

  }
}


void NMPCManage::displayPath()
{
  nav_msgs::Path path_msg;
  for (const auto &it : kino_path_)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = it(0);
    pose.pose.position.y = it(1);
    pose.pose.position.z = it(2);
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;

    path_msg.poses.push_back(pose);
  }

  path_msg.header.frame_id = "world";
  path_pub_.publish(path_msg);
}


void NMPCManage::checkReplanCallback(const ros::TimerEvent &e)
{
  // step one: check the target
  // 1. the local target is or not collision free
  if (have_target_)
  {
    if (!env_ptr_->checkPosSurround(end_pt_, 1.2))
    {
      /* try to find a max distance goal around */
      bool new_goal = false;
      const double dr = 0.2, dtheta = 30, dz = 0.2;
      double new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr){
        for (double theta = -90; theta <= 270; theta += dtheta){
          for (double nz = 1.0; nz <= 1.6; nz += dz){

            new_x = end_pt_(0) + r * cos(theta);
            new_y = end_pt_(1) + r * sin(theta);
            new_z = nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            if (env_ptr_->checkPosSurround(new_pt, 1.5)){
              end_pt_ = new_pt;
              have_target_ = true;
              displayGoalPoint();
              break;
            }
          }
        }
      }

      if (exec_state_ == EXEC_TRAJ){
        ROS_WARN("[checkReplan] Change goal, replan.");
        changeFSMExecState(REPLAN_TRAJ, "SAFETY");
      }else{
        have_target_ = false;
        ROS_WARN("[checkReplan] Goal near collision, stop.");
        changeFSMExecState(WAIT_TARGET, "SAFETY");
      }
    }
  }

  if (have_traj_)
  {
    for (int i = 0; i < kino_path_.size(); i+= 5)
    {  
      if (!env_ptr_->checkPosSurround(kino_path_[i], 1.1))
      {
        ROS_WARN("[checkReplan] Trajectory collides, replan.");
        changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        break;
      }
    }
  }
}

void NMPCManage::displayGoalPoint()
{
  visualization_msgs::Marker sphere;
  sphere.header.frame_id = "world";
  sphere.header.stamp = ros::Time::now();
  sphere.type = visualization_msgs::Marker::SPHERE;
  sphere.action = visualization_msgs::Marker::ADD;

  sphere.pose.orientation.w = 1.0;
  sphere.color.r = 0.0;
  sphere.color.g = 0.0;
  sphere.color.b = 0.0;
  sphere.color.a = 1.0;
  sphere.scale.x = 0.3;
  sphere.scale.y = 0.3;
  sphere.scale.z = 0.3;
  sphere.pose.position.x = end_pt_(0);
  sphere.pose.position.y = end_pt_(1);
  sphere.pose.position.z = end_pt_(2);

  goal_point_pub_.publish(sphere);
}

void NMPCManage::extforceCallback(const geometry_msgs::WrenchStamped &msg)
{
  // the sub is mass normolized value
  if (consider_force_)
  {
    double diverse = max(max(abs(msg.wrench.force.x), abs(msg.wrench.force.y)), abs(msg.wrench.force.z));
    if (diverse <= ext_noise_bound_)
    {
      external_acc_.setZero();
      last_external_acc_ << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z; // revise as zero
      surpass_count_ = 0;
    }
    else
    {
      external_acc_ << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z; // revise as zero if unstale

      Eigen::Vector3d diff = last_external_acc_ - external_acc_;
      double surpass = max(max(abs(diff(0)), abs(diff(1))), abs(diff(2)));

      if (surpass > ext_noise_bound_)
      { 
        ROS_INFO_STREAM("[resilient_planner]: External forces varies too large, need to replan ! ");
        ROS_INFO_STREAM("last_external_acc_: \n"  << last_external_acc_);
        ROS_INFO_STREAM("external_acc: \n"    << external_acc_);

        surpass_count_ += 1;

        if (surpass_count_ >= 1)
        {
          // after replan, set it false
          replan_force_surpass_ = true;
          last_external_acc_ << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z;

          if ( have_target_){
            ROS_WARN("[checkReplan] External forces varies upper bound, replan !");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }

          if (surpass > 10.0)
          {
            ROS_WARN("[extforceCallback] The external forces is too fierce ! stop !");
            have_target_ = false;
            changeFSMExecState(WAIT_TARGET, "SAFETY");
          }
        }
      }
      else
      {
        surpass_count_ = 0;
      }
    }
  }
}

// get odom for planning
void NMPCManage::odometryCallback(const nav_msgs::Odometry &msg)
{
  // in real world practice(we use the celocity in the world frame)

  odomT = (msg.header.stamp - tOdom).toSec();
  tOdom = msg.header.stamp;
  stateOdomPrevious_ = stateOdom_;

  stateOdom_(0) = msg.pose.pose.position.x;
  stateOdom_(1) = msg.pose.pose.position.y;
  stateOdom_(2) = msg.pose.pose.position.z;

  stateOdom_(3) = msg.twist.twist.linear.x;
  stateOdom_(4) = msg.twist.twist.linear.y;
  stateOdom_(5) = msg.twist.twist.linear.z;

  Eigen::Quaterniond quaternion;
  quaternion.w() = msg.pose.pose.orientation.w;
  quaternion.x() = msg.pose.pose.orientation.x;
  quaternion.y() = msg.pose.pose.orientation.y;
  quaternion.z() = msg.pose.pose.orientation.z;

  Eigen::Vector3d temp2;
  mav_msgs::getEulerAnglesFromQuaternion(quaternion, &temp2);
  stateOdom_.segment(6, 3) = temp2;

  have_odom_ = true;
}

/*
* For vio and planners in our lab systems, we directly use the global velocity in odometry
* and in Rotors Simulator the ground truth velocity is given in the body frame\
* http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
*/

void NMPCManage::odometryTransCallback(const nav_msgs::Odometry &msg)
{
  // use RotorS sim
  odomT = (msg.header.stamp - tOdom).toSec();
  tOdom = msg.header.stamp;
  stateOdomPrevious_ = stateOdom_;

  mav_msgs::EigenOdometry odometry;
  mav_msgs::eigenOdometryFromMsg(msg, &odometry);
  stateOdom_(0) = odometry.position_W(0);
  stateOdom_(1) = odometry.position_W(1);
  stateOdom_(2) = odometry.position_W(2);
  const Eigen::Matrix3d R_W_I = odometry.orientation_W_B.toRotationMatrix();
  Eigen::Vector3d velocity_W = R_W_I * odometry.velocity_B;
  stateOdom_(3) = velocity_W(0);
  stateOdom_(4) = velocity_W(1);
  stateOdom_(5) = velocity_W(2);
  Eigen::Vector3d temp2;
  mav_msgs::getEulerAnglesFromQuaternion(odometry.orientation_W_B, &temp2);
  stateOdom_.segment(6, 3) = temp2;

  have_odom_ = true;
}

// revise from fast planner
void NMPCManage::goalCallback(const geometry_msgs::PoseStamped &msg)
{

  if (msg.pose.position.z < -0.1) return;
  cout << "Triggered!" << endl;
  trigger_ = true;
  have_target_ = true;

  end_pt_ << msg.pose.position.x, msg.pose.position.y, 1.2; // fix z around 1-1.5
  ROS_INFO_STREAM("[resilient_planner] The end point is : \n"  << end_pt_);
  displayGoalPoint();

}
