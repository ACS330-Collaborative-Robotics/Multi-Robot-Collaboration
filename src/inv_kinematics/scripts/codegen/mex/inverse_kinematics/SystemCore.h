/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * SystemCore.h
 *
 * Code generation for function 'SystemCore'
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
void SystemCore_step(
    const emlrtStack *sp, inverseKinematics *obj, const real_T varargin_2[16],
    const emxArray_struct_T *varargin_4, emxArray_struct_T *varargout_1,
    real_T *varargout_2_Iterations, real_T *varargout_2_NumRandomRestarts,
    real_T *varargout_2_PoseErrorNorm, real_T *varargout_2_ExitFlag,
    char_T varargout_2_Status_data[], int32_T varargout_2_Status_size[2]);

/* End of code generation (SystemCore.h) */
