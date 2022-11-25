/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * inverse_kinematics_initialize.c
 *
 * Code generation for function 'inverse_kinematics_initialize'
 *
 */

/* Include files */
#include "inverse_kinematics_initialize.h"
#include "_coder_inverse_kinematics_mex.h"
#include "inverse_kinematics_data.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void inverse_kinematics_once(void);

/* Function Definitions */
static void inverse_kinematics_once(void)
{
  mex_InitInfAndNan();
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(
      &emlrtCoverageInstance,
      "/home/conor/catkin_ws/src/inv_kinematics/scripts/inverse_kinematics.m",
      0U, 1U, 2U, 0U, 0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 0U, 0U, "inverse_kinematics", 0, -1,
               1897);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 1U, 1842, -1, 1882);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 0U, 618, -1, 1825);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 0U, 0U, 1826, 1837, 1886);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 0U);
}

void inverse_kinematics_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2022b(&st);
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtLicenseCheckR2022a(&st, "EMLRT:runTime:MexFunctionNeedsLicense",
                          "robotics_system_toolbox", 2);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    inverse_kinematics_once();
  }
}

/* End of code generation (inverse_kinematics_initialize.c) */
