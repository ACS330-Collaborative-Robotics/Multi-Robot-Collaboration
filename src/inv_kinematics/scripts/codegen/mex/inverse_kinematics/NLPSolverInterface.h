/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * NLPSolverInterface.h
 *
 * Code generation for function 'NLPSolverInterface'
 *
 */

#pragma once

/* Include files */
#include "inverse_kinematics_types.h"
#include "rtwtypes.h"
#include "covrt.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void NLPSolverInterface_solve(
    const emlrtStack *sp, c_robotics_core_internal_Damped *obj,
    const emxArray_real_T *seed, emxArray_real_T *xSol,
    real_T *solutionInfo_Iterations, real_T *solutionInfo_RRAttempts,
    real_T *solutionInfo_Error, real_T *solutionInfo_ExitFlag,
    char_T solutionInfo_Status_data[], int32_T solutionInfo_Status_size[2]);

/* End of code generation (NLPSolverInterface.h) */
