/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * RigidBodyTree.h
 *
 * Code generation for function 'RigidBodyTree'
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
d_robotics_manip_internal_Rigid *
RigidBodyTree_RigidBodyTree(const emlrtStack *sp,
                            d_robotics_manip_internal_Rigid *obj);

void RigidBodyTree_addBody(const emlrtStack *sp,
                           e_robotics_manip_internal_Rigid *obj,
                           c_robotics_manip_internal_Rigid *bodyin,
                           const emxArray_char_T *parentName,
                           d_robotics_manip_internal_Colli *iobj_0,
                           rigidBodyJoint *iobj_1,
                           c_robotics_manip_internal_Rigid *iobj_2);

e_robotics_manip_internal_Rigid *
b_RigidBodyTree_RigidBodyTree(const emlrtStack *sp,
                              e_robotics_manip_internal_Rigid *obj);

void c_RigidBodyTree_efficientFKAndJ(const emlrtStack *sp,
                                     e_robotics_manip_internal_Rigid *obj,
                                     const emxArray_real_T *qv,
                                     const emxArray_char_T *body1Name,
                                     real_T T_data[], int32_T T_size[2],
                                     emxArray_real_T *Jac);

real_T c_RigidBodyTree_findBodyIndexBy(const emlrtStack *sp,
                                       e_robotics_manip_internal_Rigid *obj,
                                       const emxArray_char_T *bodyname);

void c_RigidBodyTree_get_JointPositi(const emlrtStack *sp,
                                     e_robotics_manip_internal_Rigid *obj,
                                     emxArray_real_T *limits);

void c_RigidBodyTree_validateConfigu(const emlrtStack *sp,
                                     e_robotics_manip_internal_Rigid *obj,
                                     const emxArray_struct_T *Q,
                                     emxArray_real_T *qvec);

/* End of code generation (RigidBodyTree.h) */
