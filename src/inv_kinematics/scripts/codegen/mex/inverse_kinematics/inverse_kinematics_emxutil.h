/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * inverse_kinematics_emxutil.h
 *
 * Code generation for function 'inverse_kinematics_emxutil'
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
void c_emxEnsureCapacity_robotics_ma(const emlrtStack *sp,
                                     c_emxArray_robotics_manip_inter *emxArray,
                                     int32_T oldNumel,
                                     const emlrtRTEInfo *srcLocation);

void c_emxFreeMatrix_robotics_manip_(
    const emlrtStack *sp, d_robotics_manip_internal_Colli pMatrix[13]);

void c_emxFreeStruct_robotics_core_i(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *pStruct);

void c_emxFreeStruct_robotics_manip_(const emlrtStack *sp,
                                     c_robotics_manip_internal_Rigid *pStruct);

void c_emxFree_robotics_manip_intern(
    const emlrtStack *sp, c_emxArray_robotics_manip_inter **pEmxArray);

void c_emxInitMatrix_robotics_manip_(
    const emlrtStack *sp, d_robotics_manip_internal_Colli pMatrix[13],
    const emlrtRTEInfo *srcLocation);

void c_emxInitStruct_robotics_core_i(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *pStruct,
                                     const emlrtRTEInfo *srcLocation);

void c_emxInitStruct_robotics_manip_(const emlrtStack *sp,
                                     d_robotics_manip_internal_Rigid *pStruct,
                                     const emlrtRTEInfo *srcLocation);

void c_emxInit_robotics_manip_intern(
    const emlrtStack *sp, c_emxArray_robotics_manip_inter **pEmxArray,
    const emlrtRTEInfo *srcLocation);

void d_emxFreeMatrix_robotics_manip_(
    const emlrtStack *sp, c_robotics_manip_internal_Rigid pMatrix[12]);

void d_emxFreeStruct_robotics_manip_(const emlrtStack *sp,
                                     d_robotics_manip_internal_Rigid *pStruct);

void d_emxInitMatrix_robotics_manip_(
    const emlrtStack *sp, c_robotics_manip_internal_Rigid pMatrix[12],
    const emlrtRTEInfo *srcLocation);

void d_emxInitStruct_robotics_manip_(const emlrtStack *sp,
                                     c_robotics_manip_internal_IKExt *pStruct,
                                     const emlrtRTEInfo *srcLocation);

void e_emxFreeMatrix_robotics_manip_(
    const emlrtStack *sp, c_robotics_manip_internal_Rigid pMatrix[6]);

void e_emxFreeStruct_robotics_manip_(const emlrtStack *sp,
                                     d_robotics_manip_internal_Colli *pStruct);

void e_emxInitMatrix_robotics_manip_(const emlrtStack *sp,
                                     c_robotics_manip_internal_Rigid pMatrix[6],
                                     const emlrtRTEInfo *srcLocation);

void e_emxInitStruct_robotics_manip_(const emlrtStack *sp,
                                     e_robotics_manip_internal_Rigid *pStruct,
                                     const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_boolean_T(const emlrtStack *sp,
                                 emxArray_boolean_T *emxArray, int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_char_T(const emlrtStack *sp, emxArray_char_T *emxArray,
                              int32_T oldNumel,
                              const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_int32_T(const emlrtStack *sp, emxArray_int32_T *emxArray,
                               int32_T oldNumel,
                               const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_ptrdiff_t(const emlrtStack *sp,
                                 emxArray_ptrdiff_t *emxArray, int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_real_T(const emlrtStack *sp, emxArray_real_T *emxArray,
                              int32_T oldNumel,
                              const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_struct_T(const emlrtStack *sp,
                                b_emxArray_struct_T *emxArray, int32_T oldNumel,
                                const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_struct_T1(const emlrtStack *sp,
                                 emxArray_struct_T *emxArray, int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_struct_T2(const emlrtStack *sp,
                                 c_emxArray_struct_T *emxArray,
                                 int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation);

void emxExpand_struct_T(b_emxArray_struct_T *emxArray, int32_T fromIndex,
                        int32_T toIndex);

void emxExpand_struct_T1(emxArray_struct_T *emxArray, int32_T fromIndex,
                         int32_T toIndex);

void emxFreeMatrix_rigidBodyJoint(const emlrtStack *sp,
                                  rigidBodyJoint pMatrix[13]);

void emxFreeMatrix_rigidBodyJoint1(const emlrtStack *sp,
                                   rigidBodyJoint pMatrix[12]);

void emxFreeMatrix_rigidBodyJoint2(const emlrtStack *sp,
                                   rigidBodyJoint pMatrix[7]);

void emxFreeStruct_inverseKinematics(const emlrtStack *sp,
                                     inverseKinematics *pStruct);

void emxFreeStruct_rigidBodyJoint(const emlrtStack *sp,
                                  rigidBodyJoint *pStruct);

void emxFreeStruct_rigidBodyTree(const emlrtStack *sp, rigidBodyTree *pStruct);

void emxFree_boolean_T(const emlrtStack *sp, emxArray_boolean_T **pEmxArray);

void emxFree_char_T(const emlrtStack *sp, emxArray_char_T **pEmxArray);

void emxFree_int32_T(const emlrtStack *sp, emxArray_int32_T **pEmxArray);

void emxFree_ptrdiff_t(const emlrtStack *sp, emxArray_ptrdiff_t **pEmxArray);

void emxFree_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray);

void emxFree_struct_T(const emlrtStack *sp, emxArray_struct_T **pEmxArray);

void emxFree_struct_T1(const emlrtStack *sp, b_emxArray_struct_T **pEmxArray);

void emxFree_struct_T2(const emlrtStack *sp, c_emxArray_struct_T **pEmxArray);

void emxInitMatrix_rigidBodyJoint(const emlrtStack *sp,
                                  rigidBodyJoint pMatrix[13],
                                  const emlrtRTEInfo *srcLocation);

void emxInitMatrix_rigidBodyJoint1(const emlrtStack *sp,
                                   rigidBodyJoint pMatrix[12],
                                   const emlrtRTEInfo *srcLocation);

void emxInitMatrix_rigidBodyJoint2(const emlrtStack *sp,
                                   rigidBodyJoint pMatrix[7],
                                   const emlrtRTEInfo *srcLocation);

void emxInitStruct_inverseKinematics(const emlrtStack *sp,
                                     inverseKinematics *pStruct,
                                     const emlrtRTEInfo *srcLocation);

void emxInitStruct_rigidBodyJoint(const emlrtStack *sp, rigidBodyJoint *pStruct,
                                  const emlrtRTEInfo *srcLocation);

void emxInitStruct_rigidBodyTree(const emlrtStack *sp, rigidBodyTree *pStruct,
                                 const emlrtRTEInfo *srcLocation);

void emxInitStruct_struct_T(b_struct_T *pStruct);

void emxInitStruct_struct_T1(struct_T *pStruct);

void emxInit_boolean_T(const emlrtStack *sp, emxArray_boolean_T **pEmxArray,
                       const emlrtRTEInfo *srcLocation);

void emxInit_char_T(const emlrtStack *sp, emxArray_char_T **pEmxArray,
                    const emlrtRTEInfo *srcLocation);

void emxInit_int32_T(const emlrtStack *sp, emxArray_int32_T **pEmxArray,
                     int32_T numDimensions, const emlrtRTEInfo *srcLocation);

void emxInit_ptrdiff_t(const emlrtStack *sp, emxArray_ptrdiff_t **pEmxArray,
                       const emlrtRTEInfo *srcLocation);

void emxInit_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray,
                    int32_T numDimensions, const emlrtRTEInfo *srcLocation);

void emxInit_struct_T(const emlrtStack *sp, emxArray_struct_T **pEmxArray,
                      const emlrtRTEInfo *srcLocation);

void emxInit_struct_T1(const emlrtStack *sp, b_emxArray_struct_T **pEmxArray,
                       const emlrtRTEInfo *srcLocation);

void emxInit_struct_T2(const emlrtStack *sp, c_emxArray_struct_T **pEmxArray,
                       const emlrtRTEInfo *srcLocation);

void f_emxFreeMatrix_robotics_manip_(
    const emlrtStack *sp, d_robotics_manip_internal_Colli pMatrix[7]);

void f_emxFreeStruct_robotics_manip_(const emlrtStack *sp,
                                     c_robotics_manip_internal_IKExt *pStruct);

void f_emxInitMatrix_robotics_manip_(const emlrtStack *sp,
                                     d_robotics_manip_internal_Colli pMatrix[7],
                                     const emlrtRTEInfo *srcLocation);

void f_emxInitStruct_robotics_manip_(const emlrtStack *sp,
                                     c_robotics_manip_internal_Rigid *pStruct,
                                     const emlrtRTEInfo *srcLocation);

void g_emxFreeStruct_robotics_manip_(const emlrtStack *sp,
                                     e_robotics_manip_internal_Rigid *pStruct);

void g_emxInitStruct_robotics_manip_(const emlrtStack *sp,
                                     d_robotics_manip_internal_Colli *pStruct,
                                     const emlrtRTEInfo *srcLocation);

/* End of code generation (inverse_kinematics_emxutil.h) */
