/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_inverse_kinematics_mex.c
 *
 * Code generation for function '_coder_inverse_kinematics_mex'
 *
 */

/* Include files */
#include "_coder_inverse_kinematics_mex.h"
#include "_coder_inverse_kinematics_api.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_initialize.h"
#include "inverse_kinematics_terminate.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void inverse_kinematics_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                    int32_T nrhs, const mxArray *prhs[6])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 6, 4,
                        18, "inverse_kinematics");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 18,
                        "inverse_kinematics");
  }
  /* Call the function. */
  inverse_kinematics_api(prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&inverse_kinematics_atexit);
  /* Module initialization. */
  inverse_kinematics_initialize();
  /* Dispatch the entry-point. */
  inverse_kinematics_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  inverse_kinematics_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "UTF-8", true);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_inverse_kinematics_mex.c) */
