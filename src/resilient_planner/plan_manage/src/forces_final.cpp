#include <plan_manage/nmpc_utils.h>
#include "FORCESNLPsolver_final_casadi2forces.c"
#include "FORCESNLPsolver_final_casadi.c"

using namespace std;

/* CasADi - FORCES interface */
extern void FORCESNLPsolver_final_casadi2forces(FORCESNLPsolver_final_float *x,       /* primal vars                                         */
                                                FORCESNLPsolver_final_float *y,       /* eq. constraint multiplers                           */
                                                FORCESNLPsolver_final_float *l,       /* ineq. constraint multipliers                        */
                                                FORCESNLPsolver_final_float *p,       /* parameters                                          */
                                                FORCESNLPsolver_final_float *f,       /* objective function (scalar)                         */
                                                FORCESNLPsolver_final_float *nabla_f, /* gradient of objective function                      */
                                                FORCESNLPsolver_final_float *c,       /* dynamics                                            */
                                                FORCESNLPsolver_final_float *nabla_c, /* Jacobian of the dynamics (column major)             */
                                                FORCESNLPsolver_final_float *h,       /* inequality constraints                              */
                                                FORCESNLPsolver_final_float *nabla_h, /* Jacobian of inequality constraints (column major)   */
                                                FORCESNLPsolver_final_float *hess,    /* Hessian (column major)                              */
                                                solver_int32_default stage,           /* stage number (0 indexed)                            */
                                                solver_int32_default iteration,       /* iteration number of solver                          */
                                                solver_int32_default threadID /* Id of caller thread 								   */);


namespace resilient_planner
{

  FORCESFinal::FORCESFinal()
  {
    /* FORCES PRO interface */
    extfunc_eval_final_ = &FORCESNLPsolver_final_casadi2forces;
    params_final_.num_of_threads = 1;

  }


  void FORCESFinal::setParasFinal(double w_final_stage_wp,  double w_final_stage_input, double w_input_rate,
                                  double w_final_terminal_wp, double w_final_terminal_input)
  {
    /* FORCES PRO TERMINAL solver */
    for (int i = 0; i < value_final_.planning_horizon; i++)
    {
      //index.p.weights       =   7:9 ;                           % [w_wp, w_input, w_input_rate]
      params_final_.all_parameters[i * value_final_.num_iter + 6] = w_final_stage_wp;    // w_wp
      params_final_.all_parameters[i * value_final_.num_iter + 7] = w_final_stage_input; // w_input
      params_final_.all_parameters[i * value_final_.num_iter + 8] = w_input_rate;        // w_input_rate
    }
    // revise the terminal weight
    params_final_.all_parameters[(value_final_.planning_horizon - 1) * value_final_.num_iter + 6] = w_final_terminal_wp;    // w_wp
    params_final_.all_parameters[(value_final_.planning_horizon - 1) * value_final_.num_iter + 7] = w_final_terminal_input; // w_input

  }


  int FORCESFinal::solveFinal(MPCDeque &mpc_output,  Eigen::Vector3d  &external_acc,
                              vector<Eigen::Vector3d> &ref_total_pos,
                              vector<double> &ref_total_yaw,
                              vector<Eigen::Matrix3d>  &ellipsoid_matrices,
                              vector<LinearConstraint3D> &poly_constraints, Eigen::VectorXd &poly_indices)
  {
    // state position
    params_final_.xinit[0] = mpc_output.at(1)(8);
    params_final_.xinit[1] = mpc_output.at(1)(9);
    params_final_.xinit[2] = mpc_output.at(1)(10);
    // state velocity
    params_final_.xinit[3] = mpc_output.at(1)(11);
    params_final_.xinit[4] = mpc_output.at(1)(12);
    params_final_.xinit[5] = mpc_output.at(1)(13);
    // state euler
    params_final_.xinit[6] = mpc_output.at(1)(14);
    params_final_.xinit[7] = mpc_output.at(1)(15);
    params_final_.xinit[8] = mpc_output.at(1)(16);

    for (int i = 0; i < value_final_.planning_horizon; i++)
    {
      // control input index.z.inputs
      params_final_.x0[i * value_final_.num_var + 0] = mpc_output.at(i + 1)(0);
      params_final_.x0[i * value_final_.num_var + 1] = mpc_output.at(i + 1)(1);
      params_final_.x0[i * value_final_.num_var + 2] = mpc_output.at(i + 1)(2);
      params_final_.x0[i * value_final_.num_var + 3] = mpc_output.at(i + 1)(3);
      // previous control input
      params_final_.x0[i * value_final_.num_var + 4] = mpc_output.at(i + 1)(4);
      params_final_.x0[i * value_final_.num_var + 5] = mpc_output.at(i + 1)(5);
      params_final_.x0[i * value_final_.num_var + 6] = mpc_output.at(i + 1)(6);
      params_final_.x0[i * value_final_.num_var + 7] = mpc_output.at(i + 1)(7);
      // index.z.pos
      params_final_.x0[i * value_final_.num_var + 8] = mpc_output.at(i + 1)(8);
      params_final_.x0[i * value_final_.num_var + 9] = mpc_output.at(i + 1)(9);
      params_final_.x0[i * value_final_.num_var + 10] = mpc_output.at(i + 1)(10);
      // index.z.vel
      params_final_.x0[i * value_final_.num_var + 11] = mpc_output.at(i + 1)(11);
      params_final_.x0[i * value_final_.num_var + 12] = mpc_output.at(i + 1)(12);
      params_final_.x0[i * value_final_.num_var + 13] = mpc_output.at(i + 1)(13);
      // index.z.euler
      params_final_.x0[i * value_final_.num_var + 14] = mpc_output.at(i + 1)(14);
      params_final_.x0[i * value_final_.num_var + 15] = mpc_output.at(i + 1)(15);
      params_final_.x0[i * value_final_.num_var + 16] = mpc_output.at(i + 1)(16);

      // index.p.wayPoint
      params_final_.all_parameters[i * value_final_.num_iter + 0] = ref_total_pos.at(i)(0);
      params_final_.all_parameters[i * value_final_.num_iter + 1] = ref_total_pos.at(i)(1);
      params_final_.all_parameters[i * value_final_.num_iter + 2] = ref_total_pos.at(i)(2);
      // index.p.extForceBias
      params_final_.all_parameters[i * value_final_.num_iter + 3] = external_acc(0);
      params_final_.all_parameters[i * value_final_.num_iter + 4] = external_acc(1);
      params_final_.all_parameters[i * value_final_.num_iter + 5] = external_acc(2);
      // index.p.yaw
      params_final_.all_parameters[i * value_final_.num_iter + 9] = ref_total_yaw.at(i); 
      // the reference yaw is calculated each time. and update

      VecDf b = (poly_constraints.at(poly_indices(i))).b();
      MatD3f A_const = (poly_constraints.at(poly_indices(i))).A();

      for (int j = 0; j < value_final_.num_const; ++j)
      {
        if (j < (int)b.size())
        {
          // index.p.polyConstA ;     % 3*n
          params_final_.all_parameters[i * value_final_.num_iter + value_final_.num_pre_params + j * 3 + 0] = A_const(j, 0);
          params_final_.all_parameters[i * value_final_.num_iter + value_final_.num_pre_params + j * 3 + 1] = A_const(j, 1);
          params_final_.all_parameters[i * value_final_.num_iter + value_final_.num_pre_params + j * 3 + 2] = A_const(j, 2);

          // index.p.polyConstb ;     % 1*n    nmax = 30;
          Eigen::MatrixXd temp_a = ellipsoid_matrices.at(i) * (A_const.row(j)).transpose();
          params_final_.all_parameters[i * value_final_.num_iter + value_final_.num_pre_params + value_final_.num_const * 3 + j] = b(j) - temp_a.norm();
        }
        else
        {
          // A
          params_final_.all_parameters[i * value_final_.num_iter + value_final_.num_pre_params + j * 3 + 0] = 0.0;
          params_final_.all_parameters[i * value_final_.num_iter + value_final_.num_pre_params + j * 3 + 1] = 0.0;
          params_final_.all_parameters[i * value_final_.num_iter + value_final_.num_pre_params + j * 3 + 2] = 0.0;
          // b
          params_final_.all_parameters[i * value_final_.num_iter + value_final_.num_pre_params + value_final_.num_const * 3 + j] = 0.0;
        }
      }
    }

    return FORCESNLPsolver_final_solve(&params_final_, &output_final_, &info_final_, NULL, extfunc_eval_final_);
  }

  void FORCESFinal::updateFinal(MPCDeque &mpc_output)
  {
    for (int j = 0; j < value_final_.num_var; j++)
    {
      mpc_output.at(0)(j) = output_final_.x01[j];
      mpc_output.at(1)(j) = output_final_.x02[j];
      mpc_output.at(2)(j) = output_final_.x03[j];
      mpc_output.at(3)(j) = output_final_.x04[j];
      mpc_output.at(4)(j) = output_final_.x05[j];
      mpc_output.at(5)(j) = output_final_.x06[j];
      mpc_output.at(6)(j) = output_final_.x07[j];
      mpc_output.at(7)(j) = output_final_.x08[j];
      mpc_output.at(8)(j) = output_final_.x09[j];
      mpc_output.at(9)(j) = output_final_.x10[j];
      mpc_output.at(10)(j) = output_final_.x11[j];
      mpc_output.at(11)(j) = output_final_.x12[j];
      mpc_output.at(12)(j) = output_final_.x13[j];
      mpc_output.at(13)(j) = output_final_.x14[j];
      mpc_output.at(14)(j) = output_final_.x15[j];
      mpc_output.at(15)(j) = output_final_.x16[j];
      mpc_output.at(16)(j) = output_final_.x17[j];
      mpc_output.at(17)(j) = output_final_.x18[j];
      mpc_output.at(18)(j) = output_final_.x19[j];
      mpc_output.at(19)(j) = output_final_.x20[j];
    }
  }
}  // end resilient_planner