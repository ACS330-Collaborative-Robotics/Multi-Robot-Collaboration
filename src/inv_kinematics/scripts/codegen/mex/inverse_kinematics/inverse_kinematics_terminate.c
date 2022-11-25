/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * inverse_kinematics_terminate.c
 *
 * Code generation for function 'inverse_kinematics_terminate'
 *
 */

/* Include files */
#include "inverse_kinematics_terminate.h"
#include "_coder_inverse_kinematics_mex.h"
#include "inverse_kinematics_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void inverse_kinematics_atexit(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  /* Free instance data */
  covrtFreeInstanceData(&emlrtCoverageInstance);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void inverse_kinematics_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (inverse_kinematics_terminate.c) */
