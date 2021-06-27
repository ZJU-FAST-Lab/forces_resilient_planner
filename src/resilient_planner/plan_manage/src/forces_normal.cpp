#include <plan_manage/nmpc_utils.h>
#include "FORCESNLPsolver_normal_casadi2forces.c"
#include "FORCESNLPsolver_normal_casadi.c"

using namespace std;

/* CasADi - FORCES interface */
extern void FORCESNLPsolver_normal_casadi2forces(FORCESNLPsolver_normal_float *x,       /* primal var_nums                                         */
                                                 FORCESNLPsolver_normal_float *y,       /* eq. constraint multiplers                           */
                                                 FORCESNLPsolver_normal_float *l,       /* ineq. constraint multipliers                        */
                                                 FORCESNLPsolver_normal_float *p,       /* parameters                                          */
                                                 FORCESNLPsolver_normal_float *f,       /* objective function (scalar)                         */
                                                 FORCESNLPsolver_normal_float *nabla_f, /* gradient of objective function                      */
                                                 FORCESNLPsolver_normal_float *c,       /* dynamics                                            */
                                                 FORCESNLPsolver_normal_float *nabla_c, /* Jacobian of the dynamics (column major)             */
                                                 FORCESNLPsolver_normal_float *h,       /* inequality constraints                              */
                                                 FORCESNLPsolver_normal_float *nabla_h, /* Jacobian of inequality constraints (column major)   */
                                                 FORCESNLPsolver_normal_float *hess,    /* Hessian (column major)                              */
                                                 solver_int32_default stage,            /* stage number (0 indexed)                            */
                                                 solver_int32_default iteration,        /* iteration number of solver                          */
                                                 solver_int32_default threadID /* Id of caller thread 								   */);

namespace resilient_planner
{

  FORCESNormal::FORCESNormal()
  {
    /* FORCES PRO interface */
    extfunc_eval_ = &FORCESNLPsolver_normal_casadi2forces;
    params_.num_of_threads = 1;





  }


  void FORCESNormal::setParasNormal(double &w_stage_wp,  double &w_stage_input, double &w_input_rate,
                               double &w_terminal_wp, double &w_terminal_input)
  {

    /* FORCES PRO NORMAL solver */
    for (int i = 0; i < value_.planning_horizon; i++)
    {
      //index.p.weights       =   7:9 ;                           % [w_wp, w_input, w_input_rate]
      params_.all_parameters[i * value_.num_iter + 6] = w_stage_wp;    // w_wp
      params_.all_parameters[i * value_.num_iter + 7] = w_stage_input; // w_input
      params_.all_parameters[i * value_.num_iter + 8] = w_input_rate;  // w_input_rate
    }
    // revise the terminal weight
    params_.all_parameters[(value_.planning_horizon - 1) * value_.num_iter + 6] = w_terminal_wp;    // w_wp
    params_.all_parameters[(value_.planning_horizon - 1) * value_.num_iter + 7] = w_terminal_input; // w_input

  }




  int FORCESNormal::solveNormal(MPCDeque &mpc_output, Eigen::Vector3d &external_acc,
                                vector<Eigen::Vector3d> &ref_total_pos,
                                vector<double> &ref_total_yaw,
                                vector<Eigen::Matrix3d> &ellipsoid_matrices,
                                vector<LinearConstraint3D> &poly_constraints, Eigen::VectorXd &poly_indices)
  {




    params_.xinit[0] = mpc_output.at(1)(8);
    params_.xinit[1] = mpc_output.at(1)(9);
    params_.xinit[2] = mpc_output.at(1)(10);
    // state velocity
    params_.xinit[3] = mpc_output.at(1)(11);
    params_.xinit[4] = mpc_output.at(1)(12);
    params_.xinit[5] = mpc_output.at(1)(13);
    // state euler
    params_.xinit[6] = mpc_output.at(1)(14);
    params_.xinit[7] = mpc_output.at(1)(15);
    params_.xinit[8] = mpc_output.at(1)(16);

    for (int i = 0; i < value_.planning_horizon; i++)
    {
      // control input index.z.inputs        =   1:5  ;
      // control input index.z.inputs = 1:4; [rollrate_c, pitchrate_c, yawrate_c, thrust_c]
      params_.x0[i * value_.num_var + 0] = mpc_output.at(i + 1)(0);
      params_.x0[i * value_.num_var + 1] = mpc_output.at(i + 1)(1);
      params_.x0[i * value_.num_var + 2] = mpc_output.at(i + 1)(2);
      params_.x0[i * value_.num_var + 3] = mpc_output.at(i + 1)(3);
      // previous control input
      params_.x0[i * value_.num_var + 4] = mpc_output.at(i + 1)(4);
      params_.x0[i * value_.num_var + 5] = mpc_output.at(i + 1)(5);
      params_.x0[i * value_.num_var + 6] = mpc_output.at(i + 1)(6);
      params_.x0[i * value_.num_var + 7] = mpc_output.at(i + 1)(7);
      // index.z.pos
      params_.x0[i * value_.num_var + 8] = mpc_output.at(i + 1)(8);
      params_.x0[i * value_.num_var + 9] = mpc_output.at(i + 1)(9);
      params_.x0[i * value_.num_var + 10] = mpc_output.at(i + 1)(10);
      // index.z.vel          =   9:11 ;        % velocity, [vx, vy, vz]
      params_.x0[i * value_.num_var + 11] = mpc_output.at(i + 1)(11);
      params_.x0[i * value_.num_var + 12] = mpc_output.at(i + 1)(12);
      params_.x0[i * value_.num_var + 13] = mpc_output.at(i + 1)(13);
      // index.z.euler        =   12:14;  [roll, pitch, yaw]
      params_.x0[i * value_.num_var + 14] = mpc_output.at(i + 1)(14);
      params_.x0[i * value_.num_var + 15] = mpc_output.at(i + 1)(15);
      params_.x0[i * value_.num_var + 16] = mpc_output.at(i + 1)(16);

      // index.p.wayPoint      =   1:3 ;                           % [xg, yg, zg]  is the reference path point
      params_.all_parameters[i * value_.num_iter + 0] = ref_total_pos.at(i)(0);
      params_.all_parameters[i * value_.num_iter + 1] = ref_total_pos.at(i)(1);
      params_.all_parameters[i * value_.num_iter + 2] = ref_total_pos.at(i)(2);
      // index.p.extForceBias  =   4:6 ;                           % the external force
      params_.all_parameters[i * value_.num_iter + 3] = external_acc(0);
      params_.all_parameters[i * value_.num_iter + 4] = external_acc(1);
      params_.all_parameters[i * value_.num_iter + 5] = external_acc(2);
      // index.p.yaw           =   10  ;                           % the reference yaw
      params_.all_parameters[i * value_.num_iter + 9] = ref_total_yaw.at(i); // the reference yaw is calculated each time. and update

      VecDf b = (poly_constraints.at(poly_indices(i))).b();
      MatD3f A_const = (poly_constraints.at(poly_indices(i))).A();

      for (int j = 0; j < value_.num_const; ++j)
      {
        if (j < (int)b.size())
        {
          // index.p.polyConstA ;     % 3*n
          params_.all_parameters[i * value_.num_iter + value_.num_pre_params + j * 3 + 0] = A_const(j, 0);
          params_.all_parameters[i * value_.num_iter + value_.num_pre_params + j * 3 + 1] = A_const(j, 1);
          params_.all_parameters[i * value_.num_iter + value_.num_pre_params + j * 3 + 2] = A_const(j, 2);

          // index.p.polyConstb ;   % 1*n    nmax = 30;s
          //A << A_const(j, 0), A_const(j, 1), A_const(j, 2);
          Eigen::MatrixXd temp_a = ellipsoid_matrices.at(i) * (A_const.row(j)).transpose();
          //b_addition(i,j) = temp_a.norm();
          params_.all_parameters[i * value_.num_iter + value_.num_pre_params + value_.num_const * 3 + j] = b(j) - temp_a.norm();
        }
        else
        {
          // A
          params_.all_parameters[i * value_.num_iter + value_.num_pre_params + j * 3 + 0] = 0.0;
          params_.all_parameters[i * value_.num_iter + value_.num_pre_params + j * 3 + 1] = 0.0;
          params_.all_parameters[i * value_.num_iter + value_.num_pre_params + j * 3 + 2] = 0.0;
          // b
          params_.all_parameters[i * value_.num_iter + value_.num_pre_params + value_.num_const * 3 + j] = 0.0;
        }
      }
    }




      // for (int i = 0; i < value_.planning_horizon; i++)
      // {
      //   for (int j = 0; j < value_.num_iter; ++j)
      //   {
      //     std::cout << params_.all_parameters[i * value_.num_iter + j] << " ";
      //   }
      //   std::cout << "\n"   << std::endl;
      // }
      // std::cout << "============================ params.all_parameters====================================" << std::endl;
      for (int i = 0; i < value_.planning_horizon; i++)
      {
        for (int j = 0; j < value_.num_var; ++j)
        {
          std::cout << params_.x0[i * value_.num_var + j] << " ";
        }
        std::cout << "\n"
                  << std::endl;
      }
      std::cout << "===============================params.x0=================================" << std::endl;
      for (int j = 0; j < 9; ++j)
      {
        std::cout << params_.xinit[j] << " ";
      }
      std::cout << "\n"
                << std::endl;
      std::cout << "=================================params.xinit===============================" << std::endl;

      for (int j = 0; j < 20; ++j)
      {
        std::cout <<ref_total_pos.at(j)<< " ";
      }
      std::cout << "================================reference points===============================" << std::endl;



    return FORCESNLPsolver_normal_solve(&params_, &output_, &info_, NULL, extfunc_eval_);
  }

  void FORCESNormal::updateNormal(MPCDeque &mpc_output)
  {
    cout << "NMPCSolver::updateMPCOutput: the normal mode  " << endl;
    for (int j = 0; j < value_.num_var; j++)
    {
      mpc_output.at(0)(j) = output_.x01[j];
      mpc_output.at(1)(j) = output_.x02[j];
      mpc_output.at(2)(j) = output_.x03[j];
      mpc_output.at(3)(j) = output_.x04[j];
      mpc_output.at(4)(j) = output_.x05[j];
      mpc_output.at(5)(j) = output_.x06[j];
      mpc_output.at(6)(j) = output_.x07[j];
      mpc_output.at(7)(j) = output_.x08[j];
      mpc_output.at(8)(j) = output_.x09[j];
      mpc_output.at(9)(j) = output_.x10[j];
      mpc_output.at(10)(j) = output_.x11[j];
      mpc_output.at(11)(j) = output_.x12[j];
      mpc_output.at(12)(j) = output_.x13[j];
      mpc_output.at(13)(j) = output_.x14[j];
      mpc_output.at(14)(j) = output_.x15[j];
      mpc_output.at(15)(j) = output_.x16[j];
      mpc_output.at(16)(j) = output_.x17[j];
      mpc_output.at(17)(j) = output_.x18[j];
      mpc_output.at(18)(j) = output_.x19[j];
      mpc_output.at(19)(j) = output_.x20[j];
    }

  }







} // end resilient_planner
