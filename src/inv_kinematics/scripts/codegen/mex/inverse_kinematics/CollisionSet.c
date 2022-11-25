/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * CollisionSet.c
 *
 * Code generation for function 'CollisionSet'
 *
 */

/* Include files */
#include "CollisionSet.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_emxutil.h"
#include "inverse_kinematics_types.h"
#include "rt_nonfinite.h"
#include "collisioncodegen_api.hpp"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo wc_emlrtRSI = {
    231,                 /* lineNo */
    "CollisionSet/copy", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/CollisionSet.m" /* pathName */
};

static emlrtRSInfo xc_emlrtRSI = {
    235,                 /* lineNo */
    "CollisionSet/copy", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/CollisionSet.m" /* pathName */
};

static emlrtRTEInfo b_emlrtRTEI = {
    52,                          /* lineNo */
    25,                          /* colNo */
    "CollisionSet/CollisionSet", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/CollisionSet.m" /* pName */
};

static emlrtBCInfo g_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    53,                          /* lineNo */
    45,                          /* colNo */
    "",                          /* aName */
    "CollisionSet/CollisionSet", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/CollisionSet.m", /* pName */
    0                           /* checkKind */
};

static emlrtDCInfo emlrtDCI = {
    33,                          /* lineNo */
    61,                          /* colNo */
    "CollisionSet/CollisionSet", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/CollisionSet.m", /* pName */
    1                           /* checkKind */
};

static emlrtDCInfo b_emlrtDCI = {
    33,                          /* lineNo */
    61,                          /* colNo */
    "CollisionSet/CollisionSet", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/CollisionSet.m", /* pName */
    4                           /* checkKind */
};

static emlrtRTEInfo i_emlrtRTEI = {
    234,                 /* lineNo */
    21,                  /* colNo */
    "CollisionSet/copy", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/CollisionSet.m" /* pName */
};

static emlrtBCInfo k_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    235,                 /* lineNo */
    44,                  /* colNo */
    "",                  /* aName */
    "CollisionSet/copy", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/CollisionSet.m", /* pName */
    0                           /* checkKind */
};

static emlrtBCInfo l_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    235,                 /* lineNo */
    78,                  /* colNo */
    "",                  /* aName */
    "CollisionSet/copy", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/CollisionSet.m", /* pName */
    0                           /* checkKind */
};

static emlrtRTEInfo vc_emlrtRTEI = {
    33,             /* lineNo */
    61,             /* colNo */
    "CollisionSet", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/CollisionSet.m" /* pName */
};

/* Function Definitions */
d_robotics_manip_internal_Colli *
CollisionSet_CollisionSet(const emlrtStack *sp,
                          d_robotics_manip_internal_Colli *obj,
                          real_T maxElements)
{
  void *c_defaultCollisionObj_GeometryI;
  c_robotics_manip_internal_Colli expl_temp;
  d_robotics_manip_internal_Colli *b_obj;
  real_T d;
  int32_T b_i;
  int32_T i;
  int32_T i1;
  b_obj = obj;
  b_obj->Size = 0.0;
  b_obj->MaxElements = maxElements;
  if (!(b_obj->MaxElements >= 0.0)) {
    emlrtNonNegativeCheckR2012b(b_obj->MaxElements, &b_emlrtDCI,
                                (emlrtConstCTX)sp);
  }
  d = b_obj->MaxElements;
  if (d != (int32_T)muDoubleScalarFloor(d)) {
    emlrtIntegerCheckR2012b(d, &emlrtDCI, (emlrtConstCTX)sp);
  }
  i = b_obj->CollisionGeometries->size[0] * b_obj->CollisionGeometries->size[1];
  b_obj->CollisionGeometries->size[0] = 1;
  b_obj->CollisionGeometries->size[1] = (int32_T)d;
  c_emxEnsureCapacity_robotics_ma(sp, b_obj->CollisionGeometries, i,
                                  &vc_emlrtRTEI);
  c_defaultCollisionObj_GeometryI = NULL;
  d = b_obj->MaxElements;
  i = (int32_T)d;
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, d, mxDOUBLE_CLASS, (int32_T)d,
                                &b_emlrtRTEI, (emlrtConstCTX)sp);
  if ((int32_T)d - 1 >= 0) {
    expl_temp.CollisionPrimitive = c_defaultCollisionObj_GeometryI;
    for (i1 = 0; i1 < 16; i1++) {
      b_i = iv[i1];
      expl_temp.LocalPose[i1] = b_i;
      expl_temp.WorldPose[i1] = b_i;
    }
  }
  for (b_i = 0; b_i < i; b_i++) {
    i1 = b_obj->CollisionGeometries->size[1];
    if (b_i > i1 - 1) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, i1 - 1, &g_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    b_obj->CollisionGeometries->data[b_i] = expl_temp;
  }
  b_obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

d_robotics_manip_internal_Colli *
CollisionSet_copy(const emlrtStack *sp,
                  const d_robotics_manip_internal_Colli *obj,
                  d_robotics_manip_internal_Colli *iobj_0)
{
  c_robotics_manip_internal_Colli expl_temp;
  d_robotics_manip_internal_Colli *newObj;
  emlrtStack st;
  real_T newObj_LocalPose[16];
  real_T newObj_WorldPose[16];
  real_T d;
  int32_T b_i;
  int32_T i;
  int32_T i1;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &wc_emlrtRSI;
  newObj = CollisionSet_CollisionSet(&st, iobj_0, obj->MaxElements);
  newObj->Size = obj->Size;
  d = obj->Size;
  i = (int32_T)d;
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, d, mxDOUBLE_CLASS, (int32_T)d,
                                &i_emlrtRTEI, (emlrtConstCTX)sp);
  for (b_i = 0; b_i < i; b_i++) {
    void *primitive_GeometryInternal;
    st.site = &xc_emlrtRSI;
    i1 = obj->CollisionGeometries->size[1] - 1;
    if (b_i > i1) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, i1, &l_emlrtBCI, &st);
    }
    primitive_GeometryInternal = collisioncodegen_copyGeometry(
        obj->CollisionGeometries->data[b_i].CollisionPrimitive);
    for (i1 = 0; i1 < 16; i1++) {
      newObj_LocalPose[i1] = obj->CollisionGeometries->data[b_i].LocalPose[i1];
    }
    for (i1 = 0; i1 < 16; i1++) {
      newObj_WorldPose[i1] = obj->CollisionGeometries->data[b_i].WorldPose[i1];
    }
    i1 = newObj->CollisionGeometries->size[1];
    expl_temp.CollisionPrimitive = primitive_GeometryInternal;
    memcpy(&expl_temp.LocalPose[0], &newObj_LocalPose[0], 16U * sizeof(real_T));
    memcpy(&expl_temp.WorldPose[0], &newObj_WorldPose[0], 16U * sizeof(real_T));
    if (b_i > i1 - 1) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, i1 - 1, &k_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    newObj->CollisionGeometries->data[b_i] = expl_temp;
  }
  return newObj;
}

/* End of code generation (CollisionSet.c) */
