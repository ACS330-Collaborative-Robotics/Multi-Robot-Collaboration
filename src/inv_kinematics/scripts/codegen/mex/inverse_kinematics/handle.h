/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * handle.h
 *
 * Code generation for function 'handle'
 *
 */

#pragma once

/* Include files */
#include "inverse_kinematics_internal_types.h"
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
void b_handle_matlabCodegenDestructo(const emlrtStack *sp,
                                     c_robotics_manip_internal_Rigid *obj);

void c_handle_matlabCodegenDestructo(const emlrtStack *sp,
                                     b_rigidBodyTree *obj);

void d_handle_matlabCodegenDestructo(const emlrtStack *sp,
                                     inverseKinematics *obj);

void e_handle_matlabCodegenDestructo(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *obj);

void f_handle_matlabCodegenDestructo(const emlrtStack *sp, rigidBodyTree *obj);

void g_handle_matlabCodegenDestructo(const emlrtStack *sp,
                                     e_robotics_manip_internal_Rigid *obj);

void h_handle_matlabCodegenDestructo(const emlrtStack *sp,
                                     d_robotics_manip_internal_Colli *obj);

void handle_matlabCodegenDestructor(const emlrtStack *sp,
                                    d_robotics_manip_internal_Rigid *obj);

void i_handle_matlabCodegenDestructo(const emlrtStack *sp,
                                     c_robotics_manip_internal_IKExt *obj);

/* End of code generation (handle.h) */
