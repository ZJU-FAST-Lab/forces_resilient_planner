#ifndef _NMPC_UTILS_H_ 
#define _NMPC_UTILS_H_

#include <numeric> 
#include <iomanip>
#include <algorithm>
#include <time.h>
#include <complex>
#include <unistd.h>
#include <deque>
#include <typeinfo>
#include <stdio.h>
#include <stdlib.h>
#include <tf/tf.h>

#include <Eigen/Eigenvalues>
//other dependences

/* sikang */
#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/ellipsoid_decomp.h>
/* kinodynamic a star search */
#include <path_searching/kinodynamic_astar.h>
/* ros */
#include <geometry_msgs/WrenchStamped.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <visualization_msgs/Marker.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "FORCESNLPsolver_normal.h"
#include "FORCESNLPsolver_final.h"

using namespace std;

namespace resilient_planner
{

  typedef std::deque<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > MPCDeque;

  struct FORCESParams
  {
    /* force Pro  */
    int num_pre_params = 10;
    int num_const = 30;
    int num_iter = 130; // num_pre_params_ + num_const_ * 4
    int num_var = 17;
    int planning_horizon  = 20; //horizon length

  };


  class FORCESNormal{
  public:

    FORCESNormal();
    ~FORCESNormal(){}

    FORCESNLPsolver_normal_extfunc extfunc_eval_;
    FORCESNLPsolver_normal_output output_;
    FORCESNLPsolver_normal_params params_;
    FORCESNLPsolver_normal_info info_;
    FORCESParams value_;
    void setParasNormal(double w_stage_wp,  double w_stage_input, double w_input_rate,
                        double w_terminal_wp, double w_terminal_input);
    void updateNormal(MPCDeque &mpc_output);
    int solveNormal(MPCDeque &mpc_output,  Eigen::Vector3d  &external_acc,
                                vector<Eigen::Vector3d> &ref_total_pos,
                                vector<double> &ref_total_yaw,
                                vector<Eigen::Matrix3d>  &ellipsoid_matrices,
                                vector<LinearConstraint3D> &poly_constraints, 
                                Eigen::VectorXd &poly_indices);

  };


  class FORCESFinal{
  public:

    FORCESFinal();
    ~FORCESFinal(){}

    FORCESNLPsolver_final_extfunc extfunc_eval_final_;
    FORCESNLPsolver_final_output output_final_;
    FORCESNLPsolver_final_params params_final_;
    FORCESNLPsolver_final_info info_final_;
    FORCESParams value_final_;
    void setParasFinal(double w_final_stage_wp,  double w_final_stage_input, double w_input_rate,
                       double w_final_terminal_wp, double w_final_terminal_input);
    void updateFinal(MPCDeque &mpc_output);
    int solveFinal(MPCDeque &mpc_output,  Eigen::Vector3d  &external_acc,
                                vector<Eigen::Vector3d> &ref_total_pos,
                                vector<double> &ref_total_yaw,
                                vector<Eigen::Matrix3d>  &ellipsoid_matrices,
                                vector<LinearConstraint3D> &poly_constraints, 
                                Eigen::VectorXd &poly_indices);

  };


  class NMPCSolver{
  public:
    
    NMPCSolver();
    ~NMPCSolver(){}

    enum CMD_STATUS
    {
      INIT_POSITION,
      ROTATE_YAW,
      PUB_END,
      PUB_TRAJ,
      WAIT
    };

    CMD_STATUS  cmd_status_;

    void initROS(ros::NodeHandle& nh, OccMap::Ptr& env_ptr_);

    //update euler with matrix
    Eigen::Matrix3d updateMatrix(Eigen::Vector3d& odom_euler, Eigen::Vector3d& odom_vel, double thrust_c);

    bool getKinoPath(Eigen::VectorXd& stateOdom,
                    Eigen::Vector3d end_pt, Eigen::Vector3d external_acc, bool replan=false);

    /// main function, get the corridor constraints
    int solveNMPC(Eigen::VectorXd& stateMpc, Eigen::Vector3d external_acc);
    /// get the distrubance elliposid with time 
    Eigen::Matrix3d getDistrEllipsoid(double t, Eigen::MatrixXd &Q_origin);

    /*  FORCES PRO RELATED  */
    bool switch_to_final = false;
    bool  have_mpc_traj_ = false;
      
    MPCDeque mpc_output_, pre_mpc_output_;  // always keep the size as the length of the planning horizon+1
    Eigen::Vector3d external_acc_;
    Eigen::VectorXd stateMpc_ , stateOdom_;
    
    std::vector<LinearConstraint3D> poly_constraints_;
    std::vector<double> poly_index;
    //void getGalaxyPoly();
    void getSikangPoly();
    
    
    vec_Vec3f vec_obs_; 

    // 3. the external force surpass the noise bound
    int forces_num_ ;
    int traj_id = 0;

    void calculate_yaw(Eigen::Vector3d pos,  Eigen::Vector3d pos_next);
    void getCurTraj(int index);
    int getSikangConst(Eigen::Matrix3d E);

    vec_E<Polyhedron3D> polys;
    void getKinoTraj(std::vector<Eigen::Vector3d>& kino_path_); 
    bool pub_end_ = false;


    /* ego size */
    Eigen::Matrix3d ego_size_;
    /* drag_coefficient */
    Eigen::Matrix3d drag_coefficient_matrix_;

    void callInitYaw(Eigen::VectorXd odom, double init_yaw);
    Eigen::VectorXd realOdom_;
    double init_yaw_, init_yaw_dot_;

  private:
    
    // for solver
    FORCESNormal nmpc_forces_solver_normal_;
    FORCESFinal nmpc_forces_solver_final_;

    // for gazebo simulation
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;

    /* paramaters  */
    double g, mass;
    double epsilon = 0.06;
    double Ts_ = 0.05;  //sampling time
    int planning_horizon_  = 20; //horizon length
    double real_thrust_c_ = 7.3;

    Eigen::Vector3d w_, end_pt_, cmd_end_pt_, end_v_, start_pt_, start_v_, start_a_;
    
    /* state matrix*/
    Eigen::Matrix<double, 9, 9> At_, Phi_;
    Eigen::Matrix<double, 9, 4> Bt_;
    Eigen::Matrix<double, 9, 3> Dt_;
    Eigen::Matrix<double, 4, 9> Kt_;

    /* dim */
    const static int nx = 9;
    const static int nu = 4;
    const static int nw = 3;
    const static int var = 17;

    /* ROS related */
    ros::Subscriber cloud_sub_, extforce_sub_ ;
    ros::Publisher ellipsoid_pub_, poly_pub_, ref_marker_pub_, nmpc_marker_pub_;
    ros::Time pre_mpc_start_time_ , mpc_start_time_, kino_start_time_, change_yaw_time_;
    ros::Publisher pos_cmd_pub_, traj_cmd_pub_, cmd_vis_pub_;
    ros::Timer cmd_timer_;

    /* reference kinodynamic a star path */
    std::unique_ptr<KinodynamicAstar> kino_path_finder_;
    Eigen::Vector3d ref_pos_;
    double ref_yaw_;
    double max_tau_;
    std::vector<Eigen::Vector3d> kino_path_;  // for collision check
    double kino_size_;

    std::vector<Eigen::Vector3d> ref_total_pos_;
    std::vector<double> ref_total_yaw_;


    bool finish_mpc_cmd_ = false;
    bool initialized_output_  = false;
    bool initialized_odom_    = false;
    bool have_init_ = false;
    bool trigger_command_ = false;
    bool kino_replan_ = false;

    int exit_code = -1;
      /*note 
      exit_code = 1 Optimal solution is found 
                  0 Timeout
                  6 NaN or INF
                  7 Infeasible
                  100 license error
      */
    int fail_count_ = 0;
    int replan_count_ = 0;

    /* to compute the yaw */
    Eigen::Vector3d forward_pos_;
    double last_yaw_, last_yaw_dot_;

    Eigen::MatrixXd solveLyapunovEq(Eigen::MatrixXd& eigMat_A, Eigen::MatrixXd& eigMat_Q);
    /// compute the minkowski approximate sum as a new ellipsoid 
    Eigen::Matrix3d minkowskiSumEllipsoid(Eigen::Matrix3d Q1, Eigen::Matrix3d Q2);
    Eigen::Matrix3d eulerToRot(Eigen::Vector3d& odom_euler);

    void initMPCOutput();
    void updateFORCESResults();
    void setFORCESParams(Eigen::VectorXd &poly_indices);

    /*  visulizations   */
    //vec_E<Ellipsoid3D> Es_;
    std::vector<Eigen::Matrix3d> ellipsoid_matrices_; 
    void displayEllipsoids();
    void displayRefPoints();
    void displayPoly();
    void displayNMPCPoints();

    /* ROS callbacks */
    void cloudCallback(const sensor_msgs::PointCloud2& msg);
    void cmdTrajCallback(const ros::TimerEvent& e);

    void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id, const Eigen::Vector4d& color);

  };

}

#endif
