/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * inverse_kinematics_internal_types.h
 *
 * Code generation for function 'inverse_kinematics'
 *
 */

#pragma once

/* Include files */
#include "inverse_kinematics_types.h"
#include "rtwtypes.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef enum_c_robotics_core_internal_NLPSol
#define enum_c_robotics_core_internal_NLPSol
enum c_robotics_core_internal_NLPSol
{
  LocalMinimumFound = 1, /* Default value */
  IterationLimitExceeded,
  TimeLimitExceeded,
  StepSizeBelowMinimum,
  ChangeInErrorBelowMinimum,
  SearchDirectionInvalid,
  HessianNotPositiveSemidefinite,
  TrustRegionRadiusBelowMinimum
};
#endif /* enum_c_robotics_core_internal_NLPSol */
#ifndef typedef_c_robotics_core_internal_NLPSol
#define typedef_c_robotics_core_internal_NLPSol
typedef enum c_robotics_core_internal_NLPSol c_robotics_core_internal_NLPSol;
#endif /* typedef_c_robotics_core_internal_NLPSol */

#ifndef typedef_b_rigidBodyTree
#define typedef_b_rigidBodyTree
typedef struct {
  boolean_T matlabCodegenIsDeleted;
  d_robotics_manip_internal_Rigid *TreeInternal;
} b_rigidBodyTree;
#endif /* typedef_b_rigidBodyTree */

#ifndef typedef_rtDesignRangeCheckInfo
#define typedef_rtDesignRangeCheckInfo
typedef struct {
  int32_T lineNo;
  int32_T colNo;
  const char_T *fName;
  const char_T *pName;
} rtDesignRangeCheckInfo;
#endif /* typedef_rtDesignRangeCheckInfo */

#ifndef typedef_rtRunTimeErrorInfo
#define typedef_rtRunTimeErrorInfo
typedef struct {
  int32_T lineNo;
  int32_T colNo;
  const char_T *fName;
  const char_T *pName;
} rtRunTimeErrorInfo;
#endif /* typedef_rtRunTimeErrorInfo */

/* End of code generation (inverse_kinematics_internal_types.h) */
