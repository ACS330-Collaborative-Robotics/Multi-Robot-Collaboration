/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * inverse_kinematics_emxutil.c
 *
 * Code generation for function 'inverse_kinematics_emxutil'
 *
 */

/* Include files */
#include "inverse_kinematics_emxutil.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_types.h"
#include "rt_nonfinite.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void c_emxEnsureCapacity_robotics_ma(const emlrtStack *sp,
                                     c_emxArray_robotics_manip_inter *emxArray,
                                     int32_T oldNumel,
                                     const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel = (int32_T)emlrtSizeMulR2012b((size_t)(uint32_T)newNumel,
                                           (size_t)(uint32_T)emxArray->size[i],
                                           srcLocation, (emlrtCTX)sp);
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData =
        emlrtCallocMex((uint32_T)i, sizeof(c_robotics_manip_internal_Colli));
    if (newData == NULL) {
      emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
    }
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data,
             sizeof(c_robotics_manip_internal_Colli) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }
    emxArray->data = (c_robotics_manip_internal_Colli *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void c_emxFreeMatrix_robotics_manip_(
    const emlrtStack *sp, d_robotics_manip_internal_Colli pMatrix[13])
{
  int32_T i;
  for (i = 0; i < 13; i++) {
    e_emxFreeStruct_robotics_manip_(sp, &pMatrix[i]);
  }
}

void c_emxFreeStruct_robotics_core_i(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *pStruct)
{
  emxFree_real_T(sp, &pStruct->ConstraintMatrix);
  emxFree_real_T(sp, &pStruct->ConstraintBound);
  emxFree_real_T(sp, &pStruct->SeedInternal);
}

void c_emxFreeStruct_robotics_manip_(const emlrtStack *sp,
                                     c_robotics_manip_internal_Rigid *pStruct)
{
  emxFree_char_T(sp, &pStruct->NameInternal);
}

void c_emxFree_robotics_manip_intern(
    const emlrtStack *sp, c_emxArray_robotics_manip_inter **pEmxArray)
{
  if (*pEmxArray != (c_emxArray_robotics_manip_inter *)NULL) {
    if (((*pEmxArray)->data != (c_robotics_manip_internal_Colli *)NULL) &&
        (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }
    emlrtFreeMex((*pEmxArray)->size);
    emlrtRemoveHeapReference((emlrtCTX)sp, (void *)pEmxArray);
    emlrtFreeEmxArray(*pEmxArray);
    *pEmxArray = (c_emxArray_robotics_manip_inter *)NULL;
  }
}

void c_emxInitMatrix_robotics_manip_(
    const emlrtStack *sp, d_robotics_manip_internal_Colli pMatrix[13],
    const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  for (i = 0; i < 13; i++) {
    g_emxInitStruct_robotics_manip_(sp, &pMatrix[i], srcLocation);
  }
}

void c_emxInitStruct_robotics_core_i(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *pStruct,
                                     const emlrtRTEInfo *srcLocation)
{
  emxInit_real_T(sp, &pStruct->ConstraintMatrix, 2, srcLocation);
  emxInit_real_T(sp, &pStruct->ConstraintBound, 1, srcLocation);
  emxInit_real_T(sp, &pStruct->SeedInternal, 1, srcLocation);
}

void c_emxInitStruct_robotics_manip_(const emlrtStack *sp,
                                     d_robotics_manip_internal_Rigid *pStruct,
                                     const emlrtRTEInfo *srcLocation)
{
  f_emxInitStruct_robotics_manip_(sp, &pStruct->Base, srcLocation);
  c_emxInitMatrix_robotics_manip_(sp, pStruct->_pobj0, srcLocation);
  emxInitMatrix_rigidBodyJoint(sp, pStruct->_pobj1, srcLocation);
  d_emxInitMatrix_robotics_manip_(sp, pStruct->_pobj2, srcLocation);
}

void c_emxInit_robotics_manip_intern(
    const emlrtStack *sp, c_emxArray_robotics_manip_inter **pEmxArray,
    const emlrtRTEInfo *srcLocation)
{
  c_emxArray_robotics_manip_inter *emxArray;
  int32_T i;
  *pEmxArray = (c_emxArray_robotics_manip_inter *)emlrtMallocEmxArray(
      sizeof(c_emxArray_robotics_manip_inter));
  if ((void *)*pEmxArray == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emlrtPushHeapReferenceStackEmxArray((emlrtCTX)sp, true, (void *)pEmxArray,
                                      (void *)&c_emxFree_robotics_manip_intern,
                                      NULL, NULL, NULL);
  emxArray = *pEmxArray;
  emxArray->data = (c_robotics_manip_internal_Colli *)NULL;
  emxArray->numDimensions = 2;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T) * 2U);
  if ((void *)emxArray->size == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < 2; i++) {
    emxArray->size[i] = 0;
  }
}

void d_emxFreeMatrix_robotics_manip_(
    const emlrtStack *sp, c_robotics_manip_internal_Rigid pMatrix[12])
{
  int32_T i;
  for (i = 0; i < 12; i++) {
    c_emxFreeStruct_robotics_manip_(sp, &pMatrix[i]);
  }
}

void d_emxFreeStruct_robotics_manip_(const emlrtStack *sp,
                                     d_robotics_manip_internal_Rigid *pStruct)
{
  c_emxFreeStruct_robotics_manip_(sp, &pStruct->Base);
  c_emxFreeMatrix_robotics_manip_(sp, pStruct->_pobj0);
  emxFreeMatrix_rigidBodyJoint(sp, pStruct->_pobj1);
  d_emxFreeMatrix_robotics_manip_(sp, pStruct->_pobj2);
}

void d_emxInitMatrix_robotics_manip_(
    const emlrtStack *sp, c_robotics_manip_internal_Rigid pMatrix[12],
    const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  for (i = 0; i < 12; i++) {
    f_emxInitStruct_robotics_manip_(sp, &pMatrix[i], srcLocation);
  }
}

void d_emxInitStruct_robotics_manip_(const emlrtStack *sp,
                                     c_robotics_manip_internal_IKExt *pStruct,
                                     const emlrtRTEInfo *srcLocation)
{
  emxInit_char_T(sp, &pStruct->BodyName, srcLocation);
  emxInit_real_T(sp, &pStruct->ErrTemp, 1, srcLocation);
  emxInit_real_T(sp, &pStruct->GradTemp, 1, srcLocation);
}

void e_emxFreeMatrix_robotics_manip_(const emlrtStack *sp,
                                     c_robotics_manip_internal_Rigid pMatrix[6])
{
  int32_T i;
  for (i = 0; i < 6; i++) {
    c_emxFreeStruct_robotics_manip_(sp, &pMatrix[i]);
  }
}

void e_emxFreeStruct_robotics_manip_(const emlrtStack *sp,
                                     d_robotics_manip_internal_Colli *pStruct)
{
  c_emxFree_robotics_manip_intern(sp, &pStruct->CollisionGeometries);
}

void e_emxInitMatrix_robotics_manip_(const emlrtStack *sp,
                                     c_robotics_manip_internal_Rigid pMatrix[6],
                                     const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  for (i = 0; i < 6; i++) {
    f_emxInitStruct_robotics_manip_(sp, &pMatrix[i], srcLocation);
  }
}

void e_emxInitStruct_robotics_manip_(const emlrtStack *sp,
                                     e_robotics_manip_internal_Rigid *pStruct,
                                     const emlrtRTEInfo *srcLocation)
{
  f_emxInitStruct_robotics_manip_(sp, &pStruct->Base, srcLocation);
  e_emxInitMatrix_robotics_manip_(sp, pStruct->_pobj0, srcLocation);
  f_emxInitMatrix_robotics_manip_(sp, pStruct->_pobj1, srcLocation);
  emxInitMatrix_rigidBodyJoint2(sp, pStruct->_pobj2, srcLocation);
}

void emxEnsureCapacity_boolean_T(const emlrtStack *sp,
                                 emxArray_boolean_T *emxArray, int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel = (int32_T)emlrtSizeMulR2012b((size_t)(uint32_T)newNumel,
                                           (size_t)(uint32_T)emxArray->size[i],
                                           srcLocation, (emlrtCTX)sp);
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = emlrtCallocMex((uint32_T)i, sizeof(boolean_T));
    if (newData == NULL) {
      emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
    }
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(boolean_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }
    emxArray->data = (boolean_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void emxEnsureCapacity_char_T(const emlrtStack *sp, emxArray_char_T *emxArray,
                              int32_T oldNumel, const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel = (int32_T)emlrtSizeMulR2012b((size_t)(uint32_T)newNumel,
                                           (size_t)(uint32_T)emxArray->size[i],
                                           srcLocation, (emlrtCTX)sp);
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = emlrtCallocMex((uint32_T)i, sizeof(char_T));
    if (newData == NULL) {
      emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
    }
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }
    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void emxEnsureCapacity_int32_T(const emlrtStack *sp, emxArray_int32_T *emxArray,
                               int32_T oldNumel,
                               const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel = (int32_T)emlrtSizeMulR2012b((size_t)(uint32_T)newNumel,
                                           (size_t)(uint32_T)emxArray->size[i],
                                           srcLocation, (emlrtCTX)sp);
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = emlrtCallocMex((uint32_T)i, sizeof(int32_T));
    if (newData == NULL) {
      emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
    }
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(int32_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }
    emxArray->data = (int32_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void emxEnsureCapacity_ptrdiff_t(const emlrtStack *sp,
                                 emxArray_ptrdiff_t *emxArray, int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel = (int32_T)emlrtSizeMulR2012b((size_t)(uint32_T)newNumel,
                                           (size_t)(uint32_T)emxArray->size[i],
                                           srcLocation, (emlrtCTX)sp);
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = emlrtCallocMex((uint32_T)i, sizeof(ptrdiff_t));
    if (newData == NULL) {
      emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
    }
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(ptrdiff_t) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }
    emxArray->data = (ptrdiff_t *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void emxEnsureCapacity_real_T(const emlrtStack *sp, emxArray_real_T *emxArray,
                              int32_T oldNumel, const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel = (int32_T)emlrtSizeMulR2012b((size_t)(uint32_T)newNumel,
                                           (size_t)(uint32_T)emxArray->size[i],
                                           srcLocation, (emlrtCTX)sp);
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = emlrtCallocMex((uint32_T)i, sizeof(real_T));
    if (newData == NULL) {
      emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
    }
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }
    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void emxEnsureCapacity_struct_T(const emlrtStack *sp,
                                b_emxArray_struct_T *emxArray, int32_T oldNumel,
                                const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel = (int32_T)emlrtSizeMulR2012b((size_t)(uint32_T)newNumel,
                                           (size_t)(uint32_T)emxArray->size[i],
                                           srcLocation, (emlrtCTX)sp);
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = emlrtCallocMex((uint32_T)i, sizeof(b_struct_T));
    if (newData == NULL) {
      emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
    }
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(b_struct_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }
    emxArray->data = (b_struct_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
  if (oldNumel > newNumel) {
    emxExpand_struct_T(emxArray, oldNumel, newNumel);
  }
}

void emxEnsureCapacity_struct_T1(const emlrtStack *sp,
                                 emxArray_struct_T *emxArray, int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel = (int32_T)emlrtSizeMulR2012b((size_t)(uint32_T)newNumel,
                                           (size_t)(uint32_T)emxArray->size[i],
                                           srcLocation, (emlrtCTX)sp);
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = emlrtCallocMex((uint32_T)i, sizeof(struct_T));
    if (newData == NULL) {
      emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
    }
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(struct_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }
    emxArray->data = (struct_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
  if (oldNumel > newNumel) {
    emxExpand_struct_T1(emxArray, oldNumel, newNumel);
  }
}

void emxEnsureCapacity_struct_T2(const emlrtStack *sp,
                                 c_emxArray_struct_T *emxArray,
                                 int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel = (int32_T)emlrtSizeMulR2012b((size_t)(uint32_T)newNumel,
                                           (size_t)(uint32_T)emxArray->size[i],
                                           srcLocation, (emlrtCTX)sp);
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = emlrtCallocMex((uint32_T)i, sizeof(d_struct_T));
    if (newData == NULL) {
      emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
    }
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(d_struct_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }
    emxArray->data = (d_struct_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void emxExpand_struct_T(b_emxArray_struct_T *emxArray, int32_T fromIndex,
                        int32_T toIndex)
{
  int32_T i;
  for (i = fromIndex; i < toIndex; i++) {
    emxInitStruct_struct_T(&emxArray->data[i]);
  }
}

void emxExpand_struct_T1(emxArray_struct_T *emxArray, int32_T fromIndex,
                         int32_T toIndex)
{
  int32_T i;
  for (i = fromIndex; i < toIndex; i++) {
    emxInitStruct_struct_T1(&emxArray->data[i]);
  }
}

void emxFreeMatrix_rigidBodyJoint(const emlrtStack *sp,
                                  rigidBodyJoint pMatrix[13])
{
  int32_T i;
  for (i = 0; i < 13; i++) {
    emxFreeStruct_rigidBodyJoint(sp, &pMatrix[i]);
  }
}

void emxFreeMatrix_rigidBodyJoint1(const emlrtStack *sp,
                                   rigidBodyJoint pMatrix[12])
{
  int32_T i;
  for (i = 0; i < 12; i++) {
    emxFreeStruct_rigidBodyJoint(sp, &pMatrix[i]);
  }
}

void emxFreeMatrix_rigidBodyJoint2(const emlrtStack *sp,
                                   rigidBodyJoint pMatrix[7])
{
  int32_T i;
  for (i = 0; i < 7; i++) {
    emxFreeStruct_rigidBodyJoint(sp, &pMatrix[i]);
  }
}

void emxFreeStruct_inverseKinematics(const emlrtStack *sp,
                                     inverseKinematics *pStruct)
{
  emxFree_real_T(sp, &pStruct->Limits);
  f_emxFreeStruct_robotics_manip_(sp, &pStruct->_pobj0);
  emxFreeMatrix_rigidBodyJoint1(sp, pStruct->_pobj1);
  e_emxFreeMatrix_robotics_manip_(sp, pStruct->_pobj2);
  c_emxFreeMatrix_robotics_manip_(sp, pStruct->_pobj3);
  g_emxFreeStruct_robotics_manip_(sp, &pStruct->_pobj4);
  emxFreeStruct_rigidBodyTree(sp, &pStruct->_pobj5);
  c_emxFreeStruct_robotics_core_i(sp, &pStruct->_pobj6);
}

void emxFreeStruct_rigidBodyJoint(const emlrtStack *sp, rigidBodyJoint *pStruct)
{
  emxFree_char_T(sp, &pStruct->Type);
  emxFree_real_T(sp, &pStruct->MotionSubspace);
  emxFree_char_T(sp, &pStruct->NameInternal);
  emxFree_real_T(sp, &pStruct->PositionLimitsInternal);
  emxFree_real_T(sp, &pStruct->HomePositionInternal);
}

void emxFreeStruct_rigidBodyTree(const emlrtStack *sp, rigidBodyTree *pStruct)
{
  e_emxFreeStruct_robotics_manip_(sp, &pStruct->_pobj0);
  g_emxFreeStruct_robotics_manip_(sp, &pStruct->_pobj1);
}

void emxFree_boolean_T(const emlrtStack *sp, emxArray_boolean_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_boolean_T *)NULL) {
    if (((*pEmxArray)->data != (boolean_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }
    emlrtFreeMex((*pEmxArray)->size);
    emlrtRemoveHeapReference((emlrtCTX)sp, (void *)pEmxArray);
    emlrtFreeEmxArray(*pEmxArray);
    *pEmxArray = (emxArray_boolean_T *)NULL;
  }
}

void emxFree_char_T(const emlrtStack *sp, emxArray_char_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }
    emlrtFreeMex((*pEmxArray)->size);
    emlrtRemoveHeapReference((emlrtCTX)sp, (void *)pEmxArray);
    emlrtFreeEmxArray(*pEmxArray);
    *pEmxArray = (emxArray_char_T *)NULL;
  }
}

void emxFree_int32_T(const emlrtStack *sp, emxArray_int32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int32_T *)NULL) {
    if (((*pEmxArray)->data != (int32_T *)NULL) && (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }
    emlrtFreeMex((*pEmxArray)->size);
    emlrtRemoveHeapReference((emlrtCTX)sp, (void *)pEmxArray);
    emlrtFreeEmxArray(*pEmxArray);
    *pEmxArray = (emxArray_int32_T *)NULL;
  }
}

void emxFree_ptrdiff_t(const emlrtStack *sp, emxArray_ptrdiff_t **pEmxArray)
{
  if (*pEmxArray != (emxArray_ptrdiff_t *)NULL) {
    if (((*pEmxArray)->data != (ptrdiff_t *)NULL) &&
        (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }
    emlrtFreeMex((*pEmxArray)->size);
    emlrtRemoveHeapReference((emlrtCTX)sp, (void *)pEmxArray);
    emlrtFreeEmxArray(*pEmxArray);
    *pEmxArray = (emxArray_ptrdiff_t *)NULL;
  }
}

void emxFree_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }
    emlrtFreeMex((*pEmxArray)->size);
    emlrtRemoveHeapReference((emlrtCTX)sp, (void *)pEmxArray);
    emlrtFreeEmxArray(*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

void emxFree_struct_T(const emlrtStack *sp, emxArray_struct_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_struct_T *)NULL) {
    if (((*pEmxArray)->data != (struct_T *)NULL) && (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }
    emlrtFreeMex((*pEmxArray)->size);
    emlrtRemoveHeapReference((emlrtCTX)sp, (void *)pEmxArray);
    emlrtFreeEmxArray(*pEmxArray);
    *pEmxArray = (emxArray_struct_T *)NULL;
  }
}

void emxFree_struct_T1(const emlrtStack *sp, b_emxArray_struct_T **pEmxArray)
{
  if (*pEmxArray != (b_emxArray_struct_T *)NULL) {
    if (((*pEmxArray)->data != (b_struct_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }
    emlrtFreeMex((*pEmxArray)->size);
    emlrtRemoveHeapReference((emlrtCTX)sp, (void *)pEmxArray);
    emlrtFreeEmxArray(*pEmxArray);
    *pEmxArray = (b_emxArray_struct_T *)NULL;
  }
}

void emxFree_struct_T2(const emlrtStack *sp, c_emxArray_struct_T **pEmxArray)
{
  if (*pEmxArray != (c_emxArray_struct_T *)NULL) {
    if (((*pEmxArray)->data != (d_struct_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }
    emlrtFreeMex((*pEmxArray)->size);
    emlrtRemoveHeapReference((emlrtCTX)sp, (void *)pEmxArray);
    emlrtFreeEmxArray(*pEmxArray);
    *pEmxArray = (c_emxArray_struct_T *)NULL;
  }
}

void emxInitMatrix_rigidBodyJoint(const emlrtStack *sp,
                                  rigidBodyJoint pMatrix[13],
                                  const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  for (i = 0; i < 13; i++) {
    emxInitStruct_rigidBodyJoint(sp, &pMatrix[i], srcLocation);
  }
}

void emxInitMatrix_rigidBodyJoint1(const emlrtStack *sp,
                                   rigidBodyJoint pMatrix[12],
                                   const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  for (i = 0; i < 12; i++) {
    emxInitStruct_rigidBodyJoint(sp, &pMatrix[i], srcLocation);
  }
}

void emxInitMatrix_rigidBodyJoint2(const emlrtStack *sp,
                                   rigidBodyJoint pMatrix[7],
                                   const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  for (i = 0; i < 7; i++) {
    emxInitStruct_rigidBodyJoint(sp, &pMatrix[i], srcLocation);
  }
}

void emxInitStruct_inverseKinematics(const emlrtStack *sp,
                                     inverseKinematics *pStruct,
                                     const emlrtRTEInfo *srcLocation)
{
  emxInit_real_T(sp, &pStruct->Limits, 2, srcLocation);
  d_emxInitStruct_robotics_manip_(sp, &pStruct->_pobj0, srcLocation);
  emxInitMatrix_rigidBodyJoint1(sp, pStruct->_pobj1, srcLocation);
  e_emxInitMatrix_robotics_manip_(sp, pStruct->_pobj2, srcLocation);
  c_emxInitMatrix_robotics_manip_(sp, pStruct->_pobj3, srcLocation);
  e_emxInitStruct_robotics_manip_(sp, &pStruct->_pobj4, srcLocation);
  emxInitStruct_rigidBodyTree(sp, &pStruct->_pobj5, srcLocation);
  c_emxInitStruct_robotics_core_i(sp, &pStruct->_pobj6, srcLocation);
}

void emxInitStruct_rigidBodyJoint(const emlrtStack *sp, rigidBodyJoint *pStruct,
                                  const emlrtRTEInfo *srcLocation)
{
  emxInit_char_T(sp, &pStruct->Type, srcLocation);
  emxInit_real_T(sp, &pStruct->MotionSubspace, 2, srcLocation);
  emxInit_char_T(sp, &pStruct->NameInternal, srcLocation);
  emxInit_real_T(sp, &pStruct->PositionLimitsInternal, 2, srcLocation);
  emxInit_real_T(sp, &pStruct->HomePositionInternal, 1, srcLocation);
}

void emxInitStruct_rigidBodyTree(const emlrtStack *sp, rigidBodyTree *pStruct,
                                 const emlrtRTEInfo *srcLocation)
{
  g_emxInitStruct_robotics_manip_(sp, &pStruct->_pobj0, srcLocation);
  e_emxInitStruct_robotics_manip_(sp, &pStruct->_pobj1, srcLocation);
}

void emxInitStruct_struct_T(b_struct_T *pStruct)
{
  pStruct->JointName.size[0] = 0;
  pStruct->JointName.size[1] = 0;
  pStruct->JointPosition.size[0] = 0;
  pStruct->JointPosition.size[1] = 0;
}

void emxInitStruct_struct_T1(struct_T *pStruct)
{
  pStruct->JointName.size[0] = 0;
  pStruct->JointName.size[1] = 0;
  pStruct->JointPosition.size[0] = 0;
  pStruct->JointPosition.size[1] = 0;
}

void emxInit_boolean_T(const emlrtStack *sp, emxArray_boolean_T **pEmxArray,
                       const emlrtRTEInfo *srcLocation)
{
  emxArray_boolean_T *emxArray;
  *pEmxArray =
      (emxArray_boolean_T *)emlrtMallocEmxArray(sizeof(emxArray_boolean_T));
  if ((void *)*pEmxArray == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emlrtPushHeapReferenceStackEmxArray((emlrtCTX)sp, true, (void *)pEmxArray,
                                      (void *)&emxFree_boolean_T, NULL, NULL,
                                      NULL);
  emxArray = *pEmxArray;
  emxArray->data = (boolean_T *)NULL;
  emxArray->numDimensions = 1;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T));
  if ((void *)emxArray->size == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  emxArray->size[0] = 0;
}

void emxInit_char_T(const emlrtStack *sp, emxArray_char_T **pEmxArray,
                    const emlrtRTEInfo *srcLocation)
{
  emxArray_char_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_char_T *)emlrtMallocEmxArray(sizeof(emxArray_char_T));
  if ((void *)*pEmxArray == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emlrtPushHeapReferenceStackEmxArray((emlrtCTX)sp, true, (void *)pEmxArray,
                                      (void *)&emxFree_char_T, NULL, NULL,
                                      NULL);
  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = 2;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T) * 2U);
  if ((void *)emxArray->size == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < 2; i++) {
    emxArray->size[i] = 0;
  }
}

void emxInit_int32_T(const emlrtStack *sp, emxArray_int32_T **pEmxArray,
                     int32_T numDimensions, const emlrtRTEInfo *srcLocation)
{
  emxArray_int32_T *emxArray;
  int32_T i;
  *pEmxArray =
      (emxArray_int32_T *)emlrtMallocEmxArray(sizeof(emxArray_int32_T));
  if ((void *)*pEmxArray == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emlrtPushHeapReferenceStackEmxArray((emlrtCTX)sp, true, (void *)pEmxArray,
                                      (void *)&emxFree_int32_T, NULL, NULL,
                                      NULL);
  emxArray = *pEmxArray;
  emxArray->data = (int32_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size =
      (int32_T *)emlrtMallocMex(sizeof(int32_T) * (uint32_T)numDimensions);
  if ((void *)emxArray->size == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void emxInit_ptrdiff_t(const emlrtStack *sp, emxArray_ptrdiff_t **pEmxArray,
                       const emlrtRTEInfo *srcLocation)
{
  emxArray_ptrdiff_t *emxArray;
  *pEmxArray =
      (emxArray_ptrdiff_t *)emlrtMallocEmxArray(sizeof(emxArray_ptrdiff_t));
  if ((void *)*pEmxArray == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emlrtPushHeapReferenceStackEmxArray((emlrtCTX)sp, true, (void *)pEmxArray,
                                      (void *)&emxFree_ptrdiff_t, NULL, NULL,
                                      NULL);
  emxArray = *pEmxArray;
  emxArray->data = (ptrdiff_t *)NULL;
  emxArray->numDimensions = 1;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T));
  if ((void *)emxArray->size == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  emxArray->size[0] = 0;
}

void emxInit_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray,
                    int32_T numDimensions, const emlrtRTEInfo *srcLocation)
{
  emxArray_real_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real_T *)emlrtMallocEmxArray(sizeof(emxArray_real_T));
  if ((void *)*pEmxArray == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emlrtPushHeapReferenceStackEmxArray((emlrtCTX)sp, true, (void *)pEmxArray,
                                      (void *)&emxFree_real_T, NULL, NULL,
                                      NULL);
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size =
      (int32_T *)emlrtMallocMex(sizeof(int32_T) * (uint32_T)numDimensions);
  if ((void *)emxArray->size == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void emxInit_struct_T(const emlrtStack *sp, emxArray_struct_T **pEmxArray,
                      const emlrtRTEInfo *srcLocation)
{
  emxArray_struct_T *emxArray;
  int32_T i;
  *pEmxArray =
      (emxArray_struct_T *)emlrtMallocEmxArray(sizeof(emxArray_struct_T));
  if ((void *)*pEmxArray == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emlrtPushHeapReferenceStackEmxArray((emlrtCTX)sp, true, (void *)pEmxArray,
                                      (void *)&emxFree_struct_T, NULL, NULL,
                                      NULL);
  emxArray = *pEmxArray;
  emxArray->data = (struct_T *)NULL;
  emxArray->numDimensions = 2;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T) * 2U);
  if ((void *)emxArray->size == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < 2; i++) {
    emxArray->size[i] = 0;
  }
}

void emxInit_struct_T1(const emlrtStack *sp, b_emxArray_struct_T **pEmxArray,
                       const emlrtRTEInfo *srcLocation)
{
  b_emxArray_struct_T *emxArray;
  int32_T i;
  *pEmxArray =
      (b_emxArray_struct_T *)emlrtMallocEmxArray(sizeof(b_emxArray_struct_T));
  if ((void *)*pEmxArray == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emlrtPushHeapReferenceStackEmxArray((emlrtCTX)sp, true, (void *)pEmxArray,
                                      (void *)&emxFree_struct_T1, NULL, NULL,
                                      NULL);
  emxArray = *pEmxArray;
  emxArray->data = (b_struct_T *)NULL;
  emxArray->numDimensions = 2;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T) * 2U);
  if ((void *)emxArray->size == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < 2; i++) {
    emxArray->size[i] = 0;
  }
}

void emxInit_struct_T2(const emlrtStack *sp, c_emxArray_struct_T **pEmxArray,
                       const emlrtRTEInfo *srcLocation)
{
  c_emxArray_struct_T *emxArray;
  int32_T i;
  *pEmxArray =
      (c_emxArray_struct_T *)emlrtMallocEmxArray(sizeof(c_emxArray_struct_T));
  if ((void *)*pEmxArray == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emlrtPushHeapReferenceStackEmxArray((emlrtCTX)sp, true, (void *)pEmxArray,
                                      (void *)&emxFree_struct_T2, NULL, NULL,
                                      NULL);
  emxArray = *pEmxArray;
  emxArray->data = (d_struct_T *)NULL;
  emxArray->numDimensions = 2;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T) * 2U);
  if ((void *)emxArray->size == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < 2; i++) {
    emxArray->size[i] = 0;
  }
}

void f_emxFreeMatrix_robotics_manip_(const emlrtStack *sp,
                                     d_robotics_manip_internal_Colli pMatrix[7])
{
  int32_T i;
  for (i = 0; i < 7; i++) {
    e_emxFreeStruct_robotics_manip_(sp, &pMatrix[i]);
  }
}

void f_emxFreeStruct_robotics_manip_(const emlrtStack *sp,
                                     c_robotics_manip_internal_IKExt *pStruct)
{
  emxFree_char_T(sp, &pStruct->BodyName);
  emxFree_real_T(sp, &pStruct->ErrTemp);
  emxFree_real_T(sp, &pStruct->GradTemp);
}

void f_emxInitMatrix_robotics_manip_(const emlrtStack *sp,
                                     d_robotics_manip_internal_Colli pMatrix[7],
                                     const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  for (i = 0; i < 7; i++) {
    g_emxInitStruct_robotics_manip_(sp, &pMatrix[i], srcLocation);
  }
}

void f_emxInitStruct_robotics_manip_(const emlrtStack *sp,
                                     c_robotics_manip_internal_Rigid *pStruct,
                                     const emlrtRTEInfo *srcLocation)
{
  emxInit_char_T(sp, &pStruct->NameInternal, srcLocation);
}

void g_emxFreeStruct_robotics_manip_(const emlrtStack *sp,
                                     e_robotics_manip_internal_Rigid *pStruct)
{
  c_emxFreeStruct_robotics_manip_(sp, &pStruct->Base);
  e_emxFreeMatrix_robotics_manip_(sp, pStruct->_pobj0);
  f_emxFreeMatrix_robotics_manip_(sp, pStruct->_pobj1);
  emxFreeMatrix_rigidBodyJoint2(sp, pStruct->_pobj2);
}

void g_emxInitStruct_robotics_manip_(const emlrtStack *sp,
                                     d_robotics_manip_internal_Colli *pStruct,
                                     const emlrtRTEInfo *srcLocation)
{
  c_emxInit_robotics_manip_intern(sp, &pStruct->CollisionGeometries,
                                  srcLocation);
}

/* End of code generation (inverse_kinematics_emxutil.c) */
