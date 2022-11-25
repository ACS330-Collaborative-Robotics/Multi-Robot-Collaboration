/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * handle.c
 *
 * Code generation for function 'handle'
 *
 */

/* Include files */
#include "handle.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_internal_types.h"
#include "inverse_kinematics_types.h"
#include "rt_nonfinite.h"
#include "collisioncodegen_api.hpp"

/* Variable Definitions */
static emlrtRSInfo hq_emlrtRSI = {
    22,                                            /* lineNo */
    "matlabCodegenHandle/matlabCodegenDestructor", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/eml/eml/+coder/+internal/"
    "matlabCodegenHandle.m" /* pathName */
};

static emlrtRSInfo iq_emlrtRSI = {
    244,                   /* lineNo */
    "CollisionSet/delete", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/CollisionSet.m" /* pathName */
};

static emlrtRTEInfo vb_emlrtRTEI = {
    243,                   /* lineNo */
    25,                    /* colNo */
    "CollisionSet/delete", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/CollisionSet.m" /* pName */
};

static emlrtBCInfo le_emlrtBCI = {
    -1,                    /* iFirst */
    -1,                    /* iLast */
    244,                   /* lineNo */
    79,                    /* colNo */
    "",                    /* aName */
    "CollisionSet/delete", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/CollisionSet.m", /* pName */
    0                           /* checkKind */
};

static emlrtBCInfo me_emlrtBCI = {
    -1,                    /* iFirst */
    -1,                    /* iLast */
    244,                   /* lineNo */
    45,                    /* colNo */
    "",                    /* aName */
    "CollisionSet/delete", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/CollisionSet.m", /* pName */
    0                           /* checkKind */
};

/* Function Definitions */
void b_handle_matlabCodegenDestructo(const emlrtStack *sp,
                                     c_robotics_manip_internal_Rigid *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

void c_handle_matlabCodegenDestructo(const emlrtStack *sp, b_rigidBodyTree *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

void d_handle_matlabCodegenDestructo(const emlrtStack *sp,
                                     inverseKinematics *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    if (obj->isInitialized == 1) {
      obj->isInitialized = 2;
    }
  }
}

void e_handle_matlabCodegenDestructo(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

void f_handle_matlabCodegenDestructo(const emlrtStack *sp, rigidBodyTree *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

void g_handle_matlabCodegenDestructo(const emlrtStack *sp,
                                     e_robotics_manip_internal_Rigid *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

void h_handle_matlabCodegenDestructo(const emlrtStack *sp,
                                     d_robotics_manip_internal_Colli *obj)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T b_i;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if (!obj->matlabCodegenIsDeleted) {
    real_T d;
    int32_T i;
    obj->matlabCodegenIsDeleted = true;
    st.site = &hq_emlrtRSI;
    d = obj->Size;
    i = (int32_T)d;
    emlrtForLoopVectorCheckR2021a(1.0, 1.0, d, mxDOUBLE_CLASS, (int32_T)d,
                                  &vb_emlrtRTEI, &st);
    for (b_i = 0; b_i < i; b_i++) {
      int32_T i1;
      b_st.site = &iq_emlrtRSI;
      i1 = obj->CollisionGeometries->size[1] - 1;
      if (b_i > i1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, i1, &le_emlrtBCI, &b_st);
      }
      collisioncodegen_destructGeometry(
          &obj->CollisionGeometries->data[b_i].CollisionPrimitive);
      i1 = obj->CollisionGeometries->size[1] - 1;
      if (b_i > i1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, i1, &me_emlrtBCI, &st);
      }
    }
  }
}

void handle_matlabCodegenDestructor(const emlrtStack *sp,
                                    d_robotics_manip_internal_Rigid *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

void i_handle_matlabCodegenDestructo(const emlrtStack *sp,
                                     c_robotics_manip_internal_IKExt *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

/* End of code generation (handle.c) */
