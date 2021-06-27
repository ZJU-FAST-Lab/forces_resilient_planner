/*
 * CasADi to FORCESPRO Template - missing information to be filled in by createCasadi.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2021. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "FORCESNLPsolver_normal/include/FORCESNLPsolver_normal.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif


#include "FORCESNLPsolver_normal_casadi.h"



/* copies data from sparse matrix into a dense one */
static void sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, FORCESNLPsolver_normal_callback_float *data, FORCESNLPsolver_normal_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((FORCESNLPsolver_normal_float) data[j]);
        }
    }
}




/* CasADi to FORCESPRO interface */
extern void FORCESNLPsolver_normal_casadi2forces(FORCESNLPsolver_normal_float *x,        /* primal vars                                         */
                                 FORCESNLPsolver_normal_float *y,        /* eq. constraint multiplers                           */
                                 FORCESNLPsolver_normal_float *l,        /* ineq. constraint multipliers                        */
                                 FORCESNLPsolver_normal_float *p,        /* parameters                                          */
                                 FORCESNLPsolver_normal_float *f,        /* objective function (scalar)                         */
                                 FORCESNLPsolver_normal_float *nabla_f,  /* gradient of objective function                      */
                                 FORCESNLPsolver_normal_float *c,        /* dynamics                                            */
                                 FORCESNLPsolver_normal_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 FORCESNLPsolver_normal_float *h,        /* inequality constraints                              */
                                 FORCESNLPsolver_normal_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 FORCESNLPsolver_normal_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* CasADi input and output arrays */
    const FORCESNLPsolver_normal_callback_float *in[4];
    FORCESNLPsolver_normal_callback_float *out[7];
	

	/* Allocate working arrays for CasADi */
	FORCESNLPsolver_normal_callback_float w[167];
	
    /* temporary storage for CasADi sparse output */
    FORCESNLPsolver_normal_callback_float this_f;
    FORCESNLPsolver_normal_callback_float nabla_f_sparse[12];
    FORCESNLPsolver_normal_callback_float h_sparse[30];
    FORCESNLPsolver_normal_callback_float nabla_h_sparse[90];
    FORCESNLPsolver_normal_callback_float c_sparse[13];
    FORCESNLPsolver_normal_callback_float nabla_c_sparse[64];
            
    
    /* pointers to row and column info for 
     * column compressed format used by CasADi */
    solver_int32_default nrow, ncol;
    const solver_int32_default *colind, *row;
    
    /* set inputs for CasADi */
	in[0] = x;
	in[1] = p;
	in[2] = l;
	in[3] = y;

	if ((stage >= 0) && (stage < 1))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		FORCESNLPsolver_normal_objective_1(in, out, NULL, w, 0);
		if (nabla_f != NULL)
		{
			nrow = FORCESNLPsolver_normal_objective_1_sparsity_out(1)[0];
			ncol = FORCESNLPsolver_normal_objective_1_sparsity_out(1)[1];
			colind = FORCESNLPsolver_normal_objective_1_sparsity_out(1) + 2;
			row = FORCESNLPsolver_normal_objective_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = c_sparse;
		out[1] = nabla_c_sparse;
		FORCESNLPsolver_normal_dynamics_1(in, out, NULL, w, 0);
		if (c != NULL)
		{
			nrow = FORCESNLPsolver_normal_dynamics_1_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_normal_dynamics_1_sparsity_out(0)[1];
			colind = FORCESNLPsolver_normal_dynamics_1_sparsity_out(0) + 2;
			row = FORCESNLPsolver_normal_dynamics_1_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}

		if (nabla_c != NULL)
		{
			nrow = FORCESNLPsolver_normal_dynamics_1_sparsity_out(1)[0];
			ncol = FORCESNLPsolver_normal_dynamics_1_sparsity_out(1)[1];
			colind = FORCESNLPsolver_normal_dynamics_1_sparsity_out(1) + 2;
			row = FORCESNLPsolver_normal_dynamics_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}

		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		FORCESNLPsolver_normal_inequalities_1(in, out, NULL, w, 0);
		if (h != NULL)
		{
			nrow = FORCESNLPsolver_normal_inequalities_1_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_normal_inequalities_1_sparsity_out(0)[1];
			colind = FORCESNLPsolver_normal_inequalities_1_sparsity_out(0) + 2;
			row = FORCESNLPsolver_normal_inequalities_1_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}

		if (nabla_h != NULL)
		{
			nrow = FORCESNLPsolver_normal_inequalities_1_sparsity_out(1)[0];
			ncol = FORCESNLPsolver_normal_inequalities_1_sparsity_out(1)[1];
			colind = FORCESNLPsolver_normal_inequalities_1_sparsity_out(1) + 2;
			row = FORCESNLPsolver_normal_inequalities_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}

	}

	if ((stage >= 1) && (stage < 19))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		FORCESNLPsolver_normal_objective_2(in, out, NULL, w, 0);
		if (nabla_f != NULL)
		{
			nrow = FORCESNLPsolver_normal_objective_2_sparsity_out(1)[0];
			ncol = FORCESNLPsolver_normal_objective_2_sparsity_out(1)[1];
			colind = FORCESNLPsolver_normal_objective_2_sparsity_out(1) + 2;
			row = FORCESNLPsolver_normal_objective_2_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = c_sparse;
		out[1] = nabla_c_sparse;
		FORCESNLPsolver_normal_dynamics_2(in, out, NULL, w, 0);
		if (c != NULL)
		{
			nrow = FORCESNLPsolver_normal_dynamics_2_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_normal_dynamics_2_sparsity_out(0)[1];
			colind = FORCESNLPsolver_normal_dynamics_2_sparsity_out(0) + 2;
			row = FORCESNLPsolver_normal_dynamics_2_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}

		if (nabla_c != NULL)
		{
			nrow = FORCESNLPsolver_normal_dynamics_2_sparsity_out(1)[0];
			ncol = FORCESNLPsolver_normal_dynamics_2_sparsity_out(1)[1];
			colind = FORCESNLPsolver_normal_dynamics_2_sparsity_out(1) + 2;
			row = FORCESNLPsolver_normal_dynamics_2_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}

		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		FORCESNLPsolver_normal_inequalities_2(in, out, NULL, w, 0);
		if (h != NULL)
		{
			nrow = FORCESNLPsolver_normal_inequalities_2_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_normal_inequalities_2_sparsity_out(0)[1];
			colind = FORCESNLPsolver_normal_inequalities_2_sparsity_out(0) + 2;
			row = FORCESNLPsolver_normal_inequalities_2_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}

		if (nabla_h != NULL)
		{
			nrow = FORCESNLPsolver_normal_inequalities_2_sparsity_out(1)[0];
			ncol = FORCESNLPsolver_normal_inequalities_2_sparsity_out(1)[1];
			colind = FORCESNLPsolver_normal_inequalities_2_sparsity_out(1) + 2;
			row = FORCESNLPsolver_normal_inequalities_2_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}

	}

	if ((stage >= 19) && (stage < 20))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		FORCESNLPsolver_normal_objective_20(in, out, NULL, w, 0);
		if (nabla_f != NULL)
		{
			nrow = FORCESNLPsolver_normal_objective_20_sparsity_out(1)[0];
			ncol = FORCESNLPsolver_normal_objective_20_sparsity_out(1)[1];
			colind = FORCESNLPsolver_normal_objective_20_sparsity_out(1) + 2;
			row = FORCESNLPsolver_normal_objective_20_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		FORCESNLPsolver_normal_inequalities_20(in, out, NULL, w, 0);
		if (h != NULL)
		{
			nrow = FORCESNLPsolver_normal_inequalities_20_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_normal_inequalities_20_sparsity_out(0)[1];
			colind = FORCESNLPsolver_normal_inequalities_20_sparsity_out(0) + 2;
			row = FORCESNLPsolver_normal_inequalities_20_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}

		if (nabla_h != NULL)
		{
			nrow = FORCESNLPsolver_normal_inequalities_20_sparsity_out(1)[0];
			ncol = FORCESNLPsolver_normal_inequalities_20_sparsity_out(1)[1];
			colind = FORCESNLPsolver_normal_inequalities_20_sparsity_out(1) + 2;
			row = FORCESNLPsolver_normal_inequalities_20_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}

	}


    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((FORCESNLPsolver_normal_float) this_f);
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif
