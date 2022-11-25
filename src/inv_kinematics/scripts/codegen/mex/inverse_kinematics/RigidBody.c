/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * RigidBody.c
 *
 * Code generation for function 'RigidBody'
 *
 */

/* Include files */
#include "RigidBody.h"
#include "CollisionSet.h"
#include "inverse_kinematics_data.h"
#include "inverse_kinematics_emxutil.h"
#include "inverse_kinematics_types.h"
#include "rigidBodyJoint.h"
#include "rt_nonfinite.h"
#include "validatestring.h"
#include "warning.h"

/* Variable Definitions */
static emlrtRSInfo sd_emlrtRSI = {
    189,              /* lineNo */
    "RigidBody/copy", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pathName */
};

static emlrtRSInfo td_emlrtRSI = {
    190,              /* lineNo */
    "RigidBody/copy", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pathName */
};

static emlrtRSInfo ud_emlrtRSI = {
    200,              /* lineNo */
    "RigidBody/copy", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pathName */
};

static emlrtRSInfo vd_emlrtRSI = {
    88,                    /* lineNo */
    "RigidBody/RigidBody", /* fcnName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pathName */
};

static emlrtRSInfo
    wd_emlrtRSI =
        {
            215,                   /* lineNo */
            "rigidBodyJoint/copy", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

static emlrtRSInfo
    xd_emlrtRSI =
        {
            359,                           /* lineNo */
            "rigidBodyJoint/copyInternal", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

static emlrtRSInfo
    yd_emlrtRSI =
        {
            361,                           /* lineNo */
            "rigidBodyJoint/copyInternal", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

static emlrtRSInfo
    ae_emlrtRSI =
        {
            363,                           /* lineNo */
            "rigidBodyJoint/copyInternal", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

static emlrtRSInfo
    be_emlrtRSI =
        {
            364,                           /* lineNo */
            "rigidBodyJoint/copyInternal", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

static emlrtRSInfo
    ce_emlrtRSI =
        {
            155,                             /* lineNo */
            "rigidBodyJoint/rigidBodyJoint", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

static emlrtRSInfo
    le_emlrtRSI =
        {
            227,                       /* lineNo */
            "rigidBodyJoint/set.Name", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

static emlrtRSInfo
    me_emlrtRSI =
        {
            231,                       /* lineNo */
            "rigidBodyJoint/set.Name", /* fcnName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pathName */
};

static emlrtRTEInfo gd_emlrtRTEI = {
    189,         /* lineNo */
    57,          /* colNo */
    "RigidBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pName */
};

static emlrtRTEInfo hd_emlrtRTEI = {
    95,          /* lineNo */
    25,          /* colNo */
    "RigidBody", /* fName */
    "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/+robotics/+manip/"
    "+internal/RigidBody.m" /* pName */
};

static emlrtRTEInfo
    id_emlrtRTEI =
        {
            358,              /* lineNo */
            13,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtRTEInfo
    jd_emlrtRTEI =
        {
            359,              /* lineNo */
            13,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtRTEInfo
    kd_emlrtRTEI =
        {
            367,              /* lineNo */
            13,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtRTEInfo
    ld_emlrtRTEI =
        {
            367,              /* lineNo */
            47,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtRTEInfo
    md_emlrtRTEI =
        {
            368,              /* lineNo */
            44,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtRTEInfo
    nd_emlrtRTEI =
        {
            368,              /* lineNo */
            13,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtRTEInfo
    od_emlrtRTEI =
        {
            229,              /* lineNo */
            17,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtRTEInfo
    pd_emlrtRTEI =
        {
            370,              /* lineNo */
            13,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

static emlrtRTEInfo
    qd_emlrtRTEI =
        {
            370,              /* lineNo */
            39,               /* colNo */
            "rigidBodyJoint", /* fName */
            "/usr/local/MATLAB/R2022b/toolbox/robotics/robotmanip/"
            "rigidBodyJoint.m" /* pName */
};

/* Function Definitions */
c_robotics_manip_internal_Rigid *
RigidBody_RigidBody(const emlrtStack *sp, c_robotics_manip_internal_Rigid *obj,
                    d_robotics_manip_internal_Colli *iobj_0,
                    rigidBodyJoint *iobj_1)
{
  static const char_T jname[14] = {'d', 'u', 'm', 'm', 'y', 'b', 'o',
                                   'd', 'y', '1', '_', 'j', 'n', 't'};
  static const char_T bname[10] = {'d', 'u', 'm', 'm', 'y',
                                   'b', 'o', 'd', 'y', '1'};
  static const char_T b_cv1[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  static const int8_T b_iv[6] = {0, 0, 1, 0, 0, 0};
  static const int8_T iv1[6] = {0, 0, 0, 0, 0, 1};
  c_robotics_manip_internal_Rigid *b_obj;
  emlrtStack st;
  emxArray_char_T *switch_expression;
  real_T poslim_data[12];
  int32_T exitg1;
  int32_T i;
  int32_T loop_ub;
  char_T *switch_expression_data;
  int8_T msubspace_data[36];
  int8_T b_I[9];
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  b_obj = obj;
  i = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
  b_obj->NameInternal->size[0] = 1;
  b_obj->NameInternal->size[1] = 10;
  emxEnsureCapacity_char_T(sp, b_obj->NameInternal, i, &uc_emlrtRTEI);
  for (i = 0; i < 10; i++) {
    b_obj->NameInternal->data[i] = bname[i];
  }
  st.site = &o_emlrtRSI;
  iobj_1->InTree = false;
  for (i = 0; i < 16; i++) {
    iobj_1->JointToParentTransform[i] = iv[i];
  }
  for (i = 0; i < 16; i++) {
    iobj_1->ChildToJointTransform[i] = iv[i];
  }
  i = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  emxEnsureCapacity_char_T(&st, iobj_1->NameInternal, i, &jc_emlrtRTEI);
  for (i = 0; i < 14; i++) {
    iobj_1->NameInternal->data[i] = jname[i];
  }
  i = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  emxEnsureCapacity_char_T(&st, iobj_1->Type, i, &kc_emlrtRTEI);
  for (i = 0; i < 5; i++) {
    iobj_1->Type->data[i] = b_cv[i];
  }
  emxInit_char_T(&st, &switch_expression, &lc_emlrtRTEI);
  i = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  emxEnsureCapacity_char_T(&st, switch_expression, i, &lc_emlrtRTEI);
  switch_expression_data = switch_expression->data;
  loop_ub = iobj_1->Type->size[1];
  for (i = 0; i < loop_ub; i++) {
    switch_expression_data[i] = iobj_1->Type->data[i];
  }
  b_bool = false;
  if (switch_expression->size[1] == 8) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 8) {
        if (switch_expression_data[loop_ub] != b_cv1[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    loop_ub = 0;
  } else {
    b_bool = false;
    if (switch_expression->size[1] == 9) {
      loop_ub = 0;
      do {
        exitg1 = 0;
        if (loop_ub < 9) {
          if (switch_expression_data[loop_ub] != cv[loop_ub]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      loop_ub = 1;
    } else {
      loop_ub = -1;
    }
  }
  emxFree_char_T(&st, &switch_expression);
  switch (loop_ub) {
  case 0:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv[i];
    }
    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  case 1:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = iv1[i];
    }
    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  default:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0;
    }
    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }
  i = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->MotionSubspace, i, &mc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    iobj_1->MotionSubspace->data[i] = msubspace_data[i];
  }
  i = iobj_1->PositionLimitsInternal->size[0] *
      iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(&st, iobj_1->PositionLimitsInternal, i,
                           &nc_emlrtRTEI);
  for (i = 0; i < 2; i++) {
    iobj_1->PositionLimitsInternal->data[i] = poslim_data[i];
  }
  i = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->HomePositionInternal, i, &oc_emlrtRTEI);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  b_obj->JointInternal = iobj_1;
  b_obj->Index = -1.0;
  b_obj->ParentIndex = -1.0;
  b_obj->MassInternal = 1.0;
  b_obj->CenterOfMassInternal[0] = 0.0;
  b_obj->CenterOfMassInternal[1] = 0.0;
  b_obj->CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    b_I[i] = 0;
  }
  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (i = 0; i < 9; i++) {
    b_obj->InertiaInternal[i] = b_I[i];
  }
  for (i = 0; i < 36; i++) {
    msubspace_data[i] = 0;
  }
  for (loop_ub = 0; loop_ub < 6; loop_ub++) {
    msubspace_data[loop_ub + 6 * loop_ub] = 1;
  }
  for (i = 0; i < 36; i++) {
    b_obj->SpatialInertia[i] = msubspace_data[i];
  }
  st.site = &p_emlrtRSI;
  b_obj->CollisionsInternal = CollisionSet_CollisionSet(&st, iobj_0, 0.0);
  b_obj->matlabCodegenIsDeleted = false;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return b_obj;
}

c_robotics_manip_internal_Rigid *
RigidBody_copy(const emlrtStack *sp, c_robotics_manip_internal_Rigid *obj,
               d_robotics_manip_internal_Colli *iobj_0, rigidBodyJoint *iobj_1,
               c_robotics_manip_internal_Rigid *iobj_2)
{
  static const char_T b_cv[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const int8_T b_iv[6] = {0, 0, 1, 0, 0, 0};
  static const int8_T iv1[6] = {0, 0, 0, 0, 0, 1};
  c_robotics_manip_internal_Rigid *newbody;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  emxArray_char_T *jname;
  emxArray_char_T *jtype;
  emxArray_real_T *c_obj;
  rigidBodyJoint *b_obj;
  real_T f_obj[36];
  real_T d_obj[16];
  real_T poslim_data[12];
  real_T e_obj[9];
  real_T obj_idx_0;
  real_T obj_idx_1;
  real_T obj_idx_2;
  real_T *obj_data;
  int32_T exitg1;
  int32_T i;
  int32_T loop_ub;
  char_T jointtype_data[9];
  char_T *jname_data;
  char_T *jtype_data;
  int8_T msubspace_data[36];
  int8_T b_I[9];
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &sd_emlrtRSI;
  emxInit_char_T(&st, &jtype, &id_emlrtRTEI);
  i = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = obj->NameInternal->size[1];
  emxEnsureCapacity_char_T(&st, jtype, i, &gd_emlrtRTEI);
  jtype_data = jtype->data;
  loop_ub = obj->NameInternal->size[1];
  for (i = 0; i < loop_ub; i++) {
    jtype_data[i] = obj->NameInternal->data[i];
  }
  b_st.site = &n_emlrtRSI;
  c_st.site = &g_emlrtRSI;
  newbody = iobj_2;
  b_st.site = &vd_emlrtRSI;
  c_st.site = &y_emlrtRSI;
  if (jtype->size[1] == 0) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &c_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonempty",
        "MATLAB:rigidBody:expectedNonempty", 3, 4, 5, "bname");
  }
  i = iobj_2->NameInternal->size[0] * iobj_2->NameInternal->size[1];
  iobj_2->NameInternal->size[0] = 1;
  iobj_2->NameInternal->size[1] = jtype->size[1];
  emxEnsureCapacity_char_T(&st, iobj_2->NameInternal, i, &uc_emlrtRTEI);
  loop_ub = jtype->size[1];
  for (i = 0; i < loop_ub; i++) {
    iobj_2->NameInternal->data[i] = jtype_data[i];
  }
  emxInit_char_T(&st, &jname, &jd_emlrtRTEI);
  i = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = jtype->size[1] + 4;
  emxEnsureCapacity_char_T(&st, jname, i, &hd_emlrtRTEI);
  jname_data = jname->data;
  loop_ub = jtype->size[1];
  for (i = 0; i < loop_ub; i++) {
    jname_data[i] = jtype_data[i];
  }
  jname_data[jtype->size[1]] = '_';
  jname_data[jtype->size[1] + 1] = 'j';
  jname_data[jtype->size[1] + 2] = 'n';
  jname_data[jtype->size[1] + 3] = 't';
  b_st.site = &o_emlrtRSI;
  iobj_2->JointInternal =
      rigidBodyJoint_rigidBodyJoint(&b_st, &iobj_1[0], jname);
  iobj_2->Index = -1.0;
  iobj_2->ParentIndex = -1.0;
  iobj_2->MassInternal = 1.0;
  iobj_2->CenterOfMassInternal[0] = 0.0;
  iobj_2->CenterOfMassInternal[1] = 0.0;
  iobj_2->CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    b_I[i] = 0;
  }
  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (i = 0; i < 9; i++) {
    iobj_2->InertiaInternal[i] = b_I[i];
  }
  for (i = 0; i < 36; i++) {
    msubspace_data[i] = 0;
  }
  for (loop_ub = 0; loop_ub < 6; loop_ub++) {
    msubspace_data[loop_ub + 6 * loop_ub] = 1;
  }
  for (i = 0; i < 36; i++) {
    iobj_2->SpatialInertia[i] = msubspace_data[i];
  }
  b_st.site = &p_emlrtRSI;
  iobj_2->CollisionsInternal =
      CollisionSet_CollisionSet(&b_st, &iobj_0[0], 0.0);
  iobj_2->matlabCodegenIsDeleted = false;
  st.site = &td_emlrtRSI;
  b_obj = obj->JointInternal;
  b_st.site = &wd_emlrtRSI;
  i = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = b_obj->Type->size[1];
  emxEnsureCapacity_char_T(&b_st, jtype, i, &id_emlrtRTEI);
  jtype_data = jtype->data;
  loop_ub = b_obj->Type->size[1];
  for (i = 0; i < loop_ub; i++) {
    jtype_data[i] = b_obj->Type->data[i];
  }
  c_st.site = &xd_emlrtRSI;
  i = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = b_obj->NameInternal->size[1];
  emxEnsureCapacity_char_T(&c_st, jname, i, &jd_emlrtRTEI);
  jname_data = jname->data;
  loop_ub = b_obj->NameInternal->size[1];
  for (i = 0; i < loop_ub; i++) {
    jname_data[i] = b_obj->NameInternal->data[i];
  }
  c_st.site = &yd_emlrtRSI;
  iobj_1[1].InTree = false;
  for (i = 0; i < 16; i++) {
    iobj_1[1].JointToParentTransform[i] = iv[i];
  }
  for (i = 0; i < 16; i++) {
    iobj_1[1].ChildToJointTransform[i] = iv[i];
  }
  d_st.site = &x_emlrtRSI;
  e_st.site = &y_emlrtRSI;
  if (jname->size[1] == 0) {
    emlrtErrorWithMessageIdR2018a(
        &e_st, &c_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonempty",
        "MATLAB:rigidBodyJoint:expectedNonempty", 3, 4, 5, "jname");
  }
  d_st.site = &q_emlrtRSI;
  i = iobj_1[1].NameInternal->size[0] * iobj_1[1].NameInternal->size[1];
  iobj_1[1].NameInternal->size[0] = 1;
  iobj_1[1].NameInternal->size[1] = jname->size[1];
  emxEnsureCapacity_char_T(&c_st, iobj_1[1].NameInternal, i, &jc_emlrtRTEI);
  loop_ub = jname->size[1];
  for (i = 0; i < loop_ub; i++) {
    iobj_1[1].NameInternal->data[i] = jname_data[i];
  }
  int32_T jointtype_size[2];
  emxFree_char_T(&c_st, &jname);
  d_st.site = &ce_emlrtRSI;
  validatestring(&d_st, jtype, jointtype_data, jointtype_size);
  i = iobj_1[1].Type->size[0] * iobj_1[1].Type->size[1];
  iobj_1[1].Type->size[0] = 1;
  iobj_1[1].Type->size[1] = jointtype_size[1];
  emxEnsureCapacity_char_T(&c_st, iobj_1[1].Type, i, &kc_emlrtRTEI);
  loop_ub = jointtype_size[1];
  for (i = 0; i < loop_ub; i++) {
    iobj_1[1].Type->data[i] = jointtype_data[i];
  }
  i = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = iobj_1[1].Type->size[1];
  emxEnsureCapacity_char_T(&c_st, jtype, i, &lc_emlrtRTEI);
  jtype_data = jtype->data;
  loop_ub = iobj_1[1].Type->size[1];
  for (i = 0; i < loop_ub; i++) {
    jtype_data[i] = iobj_1[1].Type->data[i];
  }
  b_bool = false;
  if (jtype->size[1] == 8) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 8) {
        if (jtype_data[loop_ub] != b_cv[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    loop_ub = 0;
  } else {
    b_bool = false;
    if (jtype->size[1] == 9) {
      loop_ub = 0;
      do {
        exitg1 = 0;
        if (loop_ub < 9) {
          if (jtype_data[loop_ub] != cv[loop_ub]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      loop_ub = 1;
    } else {
      loop_ub = -1;
    }
  }
  switch (loop_ub) {
  case 0:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv[i];
    }
    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1[1].VelocityNumber = 1.0;
    iobj_1[1].PositionNumber = 1.0;
    iobj_1[1].JointAxisInternal[0] = 0.0;
    iobj_1[1].JointAxisInternal[1] = 0.0;
    iobj_1[1].JointAxisInternal[2] = 1.0;
    break;
  case 1:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = iv1[i];
    }
    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1[1].VelocityNumber = 1.0;
    iobj_1[1].PositionNumber = 1.0;
    iobj_1[1].JointAxisInternal[0] = 0.0;
    iobj_1[1].JointAxisInternal[1] = 0.0;
    iobj_1[1].JointAxisInternal[2] = 1.0;
    break;
  default:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0;
    }
    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1[1].VelocityNumber = 0.0;
    iobj_1[1].PositionNumber = 0.0;
    iobj_1[1].JointAxisInternal[0] = 0.0;
    iobj_1[1].JointAxisInternal[1] = 0.0;
    iobj_1[1].JointAxisInternal[2] = 0.0;
    break;
  }
  i = iobj_1[1].MotionSubspace->size[0] * iobj_1[1].MotionSubspace->size[1];
  iobj_1[1].MotionSubspace->size[0] = 6;
  iobj_1[1].MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(&c_st, iobj_1[1].MotionSubspace, i, &mc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    iobj_1[1].MotionSubspace->data[i] = msubspace_data[i];
  }
  i = iobj_1[1].PositionLimitsInternal->size[0] *
      iobj_1[1].PositionLimitsInternal->size[1];
  iobj_1[1].PositionLimitsInternal->size[0] = 1;
  iobj_1[1].PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(&c_st, iobj_1[1].PositionLimitsInternal, i,
                           &nc_emlrtRTEI);
  for (i = 0; i < 2; i++) {
    iobj_1[1].PositionLimitsInternal->data[i] = poslim_data[i];
  }
  i = iobj_1[1].HomePositionInternal->size[0];
  iobj_1[1].HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(&c_st, iobj_1[1].HomePositionInternal, i,
                           &oc_emlrtRTEI);
  iobj_1[1].HomePositionInternal->data[0] = 0.0;
  c_st.site = &ae_emlrtRSI;
  loop_ub = b_obj->NameInternal->size[1];
  if (loop_ub != 0) {
    c_st.site = &be_emlrtRSI;
    i = jtype->size[0] * jtype->size[1];
    jtype->size[0] = 1;
    jtype->size[1] = b_obj->NameInternal->size[1];
    emxEnsureCapacity_char_T(&c_st, jtype, i, &cd_emlrtRTEI);
    jtype_data = jtype->data;
    loop_ub = b_obj->NameInternal->size[1];
    for (i = 0; i < loop_ub; i++) {
      jtype_data[i] = b_obj->NameInternal->data[i];
    }
    c_st.site = &be_emlrtRSI;
    d_st.site = &le_emlrtRSI;
    e_st.site = &y_emlrtRSI;
    if (jtype->size[1] == 0) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &c_emlrtRTEI,
          "Coder:toolbox:ValidateattributesexpectedNonempty",
          "MATLAB:rigidBodyJoint:expectedNonempty", 3, 4, 4, "Name");
    }
    if (!iobj_1[1].InTree) {
      i = iobj_1[1].NameInternal->size[0] * iobj_1[1].NameInternal->size[1];
      iobj_1[1].NameInternal->size[0] = 1;
      iobj_1[1].NameInternal->size[1] = jtype->size[1];
      emxEnsureCapacity_char_T(&c_st, iobj_1[1].NameInternal, i, &od_emlrtRTEI);
      loop_ub = jtype->size[1];
      for (i = 0; i < loop_ub; i++) {
        iobj_1[1].NameInternal->data[i] = jtype_data[i];
      }
    } else {
      d_st.site = &me_emlrtRSI;
      e_st.site = &ne_emlrtRSI;
      warning(&e_st);
    }
  }
  emxFree_char_T(&b_st, &jtype);
  loop_ub = b_obj->PositionLimitsInternal->size[0] * 2;
  i = iobj_1[1].PositionLimitsInternal->size[0] *
      iobj_1[1].PositionLimitsInternal->size[1];
  iobj_1[1].PositionLimitsInternal->size[0] =
      b_obj->PositionLimitsInternal->size[0];
  iobj_1[1].PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(&b_st, iobj_1[1].PositionLimitsInternal, i,
                           &kd_emlrtRTEI);
  emxInit_real_T(&b_st, &c_obj, 1, &ld_emlrtRTEI);
  i = c_obj->size[0];
  c_obj->size[0] = loop_ub;
  emxEnsureCapacity_real_T(&b_st, c_obj, i, &ld_emlrtRTEI);
  obj_data = c_obj->data;
  for (i = 0; i < loop_ub; i++) {
    obj_data[i] = b_obj->PositionLimitsInternal->data[i];
  }
  loop_ub = c_obj->size[0];
  for (i = 0; i < loop_ub; i++) {
    iobj_1[1].PositionLimitsInternal->data[i] = obj_data[i];
  }
  i = c_obj->size[0];
  c_obj->size[0] = b_obj->HomePositionInternal->size[0];
  emxEnsureCapacity_real_T(&b_st, c_obj, i, &md_emlrtRTEI);
  obj_data = c_obj->data;
  loop_ub = b_obj->HomePositionInternal->size[0];
  for (i = 0; i < loop_ub; i++) {
    obj_data[i] = b_obj->HomePositionInternal->data[i];
  }
  i = iobj_1[1].HomePositionInternal->size[0];
  iobj_1[1].HomePositionInternal->size[0] = c_obj->size[0];
  emxEnsureCapacity_real_T(&b_st, iobj_1[1].HomePositionInternal, i,
                           &nd_emlrtRTEI);
  loop_ub = c_obj->size[0];
  for (i = 0; i < loop_ub; i++) {
    iobj_1[1].HomePositionInternal->data[i] = obj_data[i];
  }
  obj_idx_0 = b_obj->JointAxisInternal[0];
  obj_idx_1 = b_obj->JointAxisInternal[1];
  obj_idx_2 = b_obj->JointAxisInternal[2];
  iobj_1[1].JointAxisInternal[0] = obj_idx_0;
  iobj_1[1].JointAxisInternal[1] = obj_idx_1;
  iobj_1[1].JointAxisInternal[2] = obj_idx_2;
  loop_ub = 6 * b_obj->MotionSubspace->size[1];
  i = iobj_1[1].MotionSubspace->size[0] * iobj_1[1].MotionSubspace->size[1];
  iobj_1[1].MotionSubspace->size[0] = 6;
  iobj_1[1].MotionSubspace->size[1] = b_obj->MotionSubspace->size[1];
  emxEnsureCapacity_real_T(&b_st, iobj_1[1].MotionSubspace, i, &pd_emlrtRTEI);
  i = c_obj->size[0];
  c_obj->size[0] = loop_ub;
  emxEnsureCapacity_real_T(&b_st, c_obj, i, &qd_emlrtRTEI);
  obj_data = c_obj->data;
  for (i = 0; i < loop_ub; i++) {
    obj_data[i] = b_obj->MotionSubspace->data[i];
  }
  loop_ub = c_obj->size[0];
  for (i = 0; i < loop_ub; i++) {
    iobj_1[1].MotionSubspace->data[i] = obj_data[i];
  }
  emxFree_real_T(&b_st, &c_obj);
  for (i = 0; i < 16; i++) {
    d_obj[i] = b_obj->JointToParentTransform[i];
  }
  for (i = 0; i < 16; i++) {
    iobj_1[1].JointToParentTransform[i] = d_obj[i];
  }
  for (i = 0; i < 16; i++) {
    d_obj[i] = b_obj->ChildToJointTransform[i];
  }
  for (i = 0; i < 16; i++) {
    iobj_1[1].ChildToJointTransform[i] = d_obj[i];
  }
  iobj_2->JointInternal = &iobj_1[1];
  iobj_2->MassInternal = obj->MassInternal;
  obj_idx_0 = obj->CenterOfMassInternal[0];
  obj_idx_1 = obj->CenterOfMassInternal[1];
  obj_idx_2 = obj->CenterOfMassInternal[2];
  iobj_2->CenterOfMassInternal[0] = obj_idx_0;
  iobj_2->CenterOfMassInternal[1] = obj_idx_1;
  iobj_2->CenterOfMassInternal[2] = obj_idx_2;
  for (i = 0; i < 9; i++) {
    e_obj[i] = obj->InertiaInternal[i];
  }
  for (i = 0; i < 9; i++) {
    iobj_2->InertiaInternal[i] = e_obj[i];
  }
  for (i = 0; i < 36; i++) {
    f_obj[i] = obj->SpatialInertia[i];
  }
  for (i = 0; i < 36; i++) {
    iobj_2->SpatialInertia[i] = f_obj[i];
  }
  st.site = &ud_emlrtRSI;
  iobj_2->CollisionsInternal =
      CollisionSet_copy(&st, obj->CollisionsInternal, &iobj_0[1]);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return newbody;
}

c_robotics_manip_internal_Rigid *b_RigidBody_RigidBody(
    const emlrtStack *sp, c_robotics_manip_internal_Rigid *obj,
    d_robotics_manip_internal_Colli *iobj_0, rigidBodyJoint *iobj_1)
{
  static const char_T jname[14] = {'d', 'u', 'm', 'm', 'y', 'b', 'o',
                                   'd', 'y', '2', '_', 'j', 'n', 't'};
  static const char_T bname[10] = {'d', 'u', 'm', 'm', 'y',
                                   'b', 'o', 'd', 'y', '2'};
  static const char_T b_cv1[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  static const int8_T b_iv[6] = {0, 0, 1, 0, 0, 0};
  static const int8_T iv1[6] = {0, 0, 0, 0, 0, 1};
  c_robotics_manip_internal_Rigid *b_obj;
  emlrtStack st;
  emxArray_char_T *switch_expression;
  real_T poslim_data[12];
  int32_T exitg1;
  int32_T i;
  int32_T loop_ub;
  char_T *switch_expression_data;
  int8_T msubspace_data[36];
  int8_T b_I[9];
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  b_obj = obj;
  i = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
  b_obj->NameInternal->size[0] = 1;
  b_obj->NameInternal->size[1] = 10;
  emxEnsureCapacity_char_T(sp, b_obj->NameInternal, i, &uc_emlrtRTEI);
  for (i = 0; i < 10; i++) {
    b_obj->NameInternal->data[i] = bname[i];
  }
  st.site = &o_emlrtRSI;
  iobj_1->InTree = false;
  for (i = 0; i < 16; i++) {
    iobj_1->JointToParentTransform[i] = iv[i];
  }
  for (i = 0; i < 16; i++) {
    iobj_1->ChildToJointTransform[i] = iv[i];
  }
  i = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  emxEnsureCapacity_char_T(&st, iobj_1->NameInternal, i, &jc_emlrtRTEI);
  for (i = 0; i < 14; i++) {
    iobj_1->NameInternal->data[i] = jname[i];
  }
  i = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  emxEnsureCapacity_char_T(&st, iobj_1->Type, i, &kc_emlrtRTEI);
  for (i = 0; i < 5; i++) {
    iobj_1->Type->data[i] = b_cv[i];
  }
  emxInit_char_T(&st, &switch_expression, &lc_emlrtRTEI);
  i = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  emxEnsureCapacity_char_T(&st, switch_expression, i, &lc_emlrtRTEI);
  switch_expression_data = switch_expression->data;
  loop_ub = iobj_1->Type->size[1];
  for (i = 0; i < loop_ub; i++) {
    switch_expression_data[i] = iobj_1->Type->data[i];
  }
  b_bool = false;
  if (switch_expression->size[1] == 8) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 8) {
        if (switch_expression_data[loop_ub] != b_cv1[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    loop_ub = 0;
  } else {
    b_bool = false;
    if (switch_expression->size[1] == 9) {
      loop_ub = 0;
      do {
        exitg1 = 0;
        if (loop_ub < 9) {
          if (switch_expression_data[loop_ub] != cv[loop_ub]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      loop_ub = 1;
    } else {
      loop_ub = -1;
    }
  }
  emxFree_char_T(&st, &switch_expression);
  switch (loop_ub) {
  case 0:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv[i];
    }
    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  case 1:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = iv1[i];
    }
    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  default:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0;
    }
    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }
  i = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->MotionSubspace, i, &mc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    iobj_1->MotionSubspace->data[i] = msubspace_data[i];
  }
  i = iobj_1->PositionLimitsInternal->size[0] *
      iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(&st, iobj_1->PositionLimitsInternal, i,
                           &nc_emlrtRTEI);
  for (i = 0; i < 2; i++) {
    iobj_1->PositionLimitsInternal->data[i] = poslim_data[i];
  }
  i = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->HomePositionInternal, i, &oc_emlrtRTEI);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  b_obj->JointInternal = iobj_1;
  b_obj->Index = -1.0;
  b_obj->ParentIndex = -1.0;
  b_obj->MassInternal = 1.0;
  b_obj->CenterOfMassInternal[0] = 0.0;
  b_obj->CenterOfMassInternal[1] = 0.0;
  b_obj->CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    b_I[i] = 0;
  }
  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (i = 0; i < 9; i++) {
    b_obj->InertiaInternal[i] = b_I[i];
  }
  for (i = 0; i < 36; i++) {
    msubspace_data[i] = 0;
  }
  for (loop_ub = 0; loop_ub < 6; loop_ub++) {
    msubspace_data[loop_ub + 6 * loop_ub] = 1;
  }
  for (i = 0; i < 36; i++) {
    b_obj->SpatialInertia[i] = msubspace_data[i];
  }
  st.site = &p_emlrtRSI;
  b_obj->CollisionsInternal = CollisionSet_CollisionSet(&st, iobj_0, 0.0);
  b_obj->matlabCodegenIsDeleted = false;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return b_obj;
}

c_robotics_manip_internal_Rigid *c_RigidBody_RigidBody(
    const emlrtStack *sp, c_robotics_manip_internal_Rigid *obj,
    d_robotics_manip_internal_Colli *iobj_0, rigidBodyJoint *iobj_1)
{
  static const char_T jname[14] = {'d', 'u', 'm', 'm', 'y', 'b', 'o',
                                   'd', 'y', '3', '_', 'j', 'n', 't'};
  static const char_T bname[10] = {'d', 'u', 'm', 'm', 'y',
                                   'b', 'o', 'd', 'y', '3'};
  static const char_T b_cv1[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  static const int8_T b_iv[6] = {0, 0, 1, 0, 0, 0};
  static const int8_T iv1[6] = {0, 0, 0, 0, 0, 1};
  c_robotics_manip_internal_Rigid *b_obj;
  emlrtStack st;
  emxArray_char_T *switch_expression;
  real_T poslim_data[12];
  int32_T exitg1;
  int32_T i;
  int32_T loop_ub;
  char_T *switch_expression_data;
  int8_T msubspace_data[36];
  int8_T b_I[9];
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  b_obj = obj;
  i = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
  b_obj->NameInternal->size[0] = 1;
  b_obj->NameInternal->size[1] = 10;
  emxEnsureCapacity_char_T(sp, b_obj->NameInternal, i, &uc_emlrtRTEI);
  for (i = 0; i < 10; i++) {
    b_obj->NameInternal->data[i] = bname[i];
  }
  st.site = &o_emlrtRSI;
  iobj_1->InTree = false;
  for (i = 0; i < 16; i++) {
    iobj_1->JointToParentTransform[i] = iv[i];
  }
  for (i = 0; i < 16; i++) {
    iobj_1->ChildToJointTransform[i] = iv[i];
  }
  i = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  emxEnsureCapacity_char_T(&st, iobj_1->NameInternal, i, &jc_emlrtRTEI);
  for (i = 0; i < 14; i++) {
    iobj_1->NameInternal->data[i] = jname[i];
  }
  i = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  emxEnsureCapacity_char_T(&st, iobj_1->Type, i, &kc_emlrtRTEI);
  for (i = 0; i < 5; i++) {
    iobj_1->Type->data[i] = b_cv[i];
  }
  emxInit_char_T(&st, &switch_expression, &lc_emlrtRTEI);
  i = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  emxEnsureCapacity_char_T(&st, switch_expression, i, &lc_emlrtRTEI);
  switch_expression_data = switch_expression->data;
  loop_ub = iobj_1->Type->size[1];
  for (i = 0; i < loop_ub; i++) {
    switch_expression_data[i] = iobj_1->Type->data[i];
  }
  b_bool = false;
  if (switch_expression->size[1] == 8) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 8) {
        if (switch_expression_data[loop_ub] != b_cv1[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    loop_ub = 0;
  } else {
    b_bool = false;
    if (switch_expression->size[1] == 9) {
      loop_ub = 0;
      do {
        exitg1 = 0;
        if (loop_ub < 9) {
          if (switch_expression_data[loop_ub] != cv[loop_ub]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      loop_ub = 1;
    } else {
      loop_ub = -1;
    }
  }
  emxFree_char_T(&st, &switch_expression);
  switch (loop_ub) {
  case 0:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv[i];
    }
    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  case 1:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = iv1[i];
    }
    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  default:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0;
    }
    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }
  i = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->MotionSubspace, i, &mc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    iobj_1->MotionSubspace->data[i] = msubspace_data[i];
  }
  i = iobj_1->PositionLimitsInternal->size[0] *
      iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(&st, iobj_1->PositionLimitsInternal, i,
                           &nc_emlrtRTEI);
  for (i = 0; i < 2; i++) {
    iobj_1->PositionLimitsInternal->data[i] = poslim_data[i];
  }
  i = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->HomePositionInternal, i, &oc_emlrtRTEI);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  b_obj->JointInternal = iobj_1;
  b_obj->Index = -1.0;
  b_obj->ParentIndex = -1.0;
  b_obj->MassInternal = 1.0;
  b_obj->CenterOfMassInternal[0] = 0.0;
  b_obj->CenterOfMassInternal[1] = 0.0;
  b_obj->CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    b_I[i] = 0;
  }
  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (i = 0; i < 9; i++) {
    b_obj->InertiaInternal[i] = b_I[i];
  }
  for (i = 0; i < 36; i++) {
    msubspace_data[i] = 0;
  }
  for (loop_ub = 0; loop_ub < 6; loop_ub++) {
    msubspace_data[loop_ub + 6 * loop_ub] = 1;
  }
  for (i = 0; i < 36; i++) {
    b_obj->SpatialInertia[i] = msubspace_data[i];
  }
  st.site = &p_emlrtRSI;
  b_obj->CollisionsInternal = CollisionSet_CollisionSet(&st, iobj_0, 0.0);
  b_obj->matlabCodegenIsDeleted = false;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return b_obj;
}

c_robotics_manip_internal_Rigid *d_RigidBody_RigidBody(
    const emlrtStack *sp, c_robotics_manip_internal_Rigid *obj,
    d_robotics_manip_internal_Colli *iobj_0, rigidBodyJoint *iobj_1)
{
  static const char_T jname[14] = {'d', 'u', 'm', 'm', 'y', 'b', 'o',
                                   'd', 'y', '4', '_', 'j', 'n', 't'};
  static const char_T bname[10] = {'d', 'u', 'm', 'm', 'y',
                                   'b', 'o', 'd', 'y', '4'};
  static const char_T b_cv1[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  static const int8_T b_iv[6] = {0, 0, 1, 0, 0, 0};
  static const int8_T iv1[6] = {0, 0, 0, 0, 0, 1};
  c_robotics_manip_internal_Rigid *b_obj;
  emlrtStack st;
  emxArray_char_T *switch_expression;
  real_T poslim_data[12];
  int32_T exitg1;
  int32_T i;
  int32_T loop_ub;
  char_T *switch_expression_data;
  int8_T msubspace_data[36];
  int8_T b_I[9];
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  b_obj = obj;
  i = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
  b_obj->NameInternal->size[0] = 1;
  b_obj->NameInternal->size[1] = 10;
  emxEnsureCapacity_char_T(sp, b_obj->NameInternal, i, &uc_emlrtRTEI);
  for (i = 0; i < 10; i++) {
    b_obj->NameInternal->data[i] = bname[i];
  }
  st.site = &o_emlrtRSI;
  iobj_1->InTree = false;
  for (i = 0; i < 16; i++) {
    iobj_1->JointToParentTransform[i] = iv[i];
  }
  for (i = 0; i < 16; i++) {
    iobj_1->ChildToJointTransform[i] = iv[i];
  }
  i = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  emxEnsureCapacity_char_T(&st, iobj_1->NameInternal, i, &jc_emlrtRTEI);
  for (i = 0; i < 14; i++) {
    iobj_1->NameInternal->data[i] = jname[i];
  }
  i = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  emxEnsureCapacity_char_T(&st, iobj_1->Type, i, &kc_emlrtRTEI);
  for (i = 0; i < 5; i++) {
    iobj_1->Type->data[i] = b_cv[i];
  }
  emxInit_char_T(&st, &switch_expression, &lc_emlrtRTEI);
  i = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  emxEnsureCapacity_char_T(&st, switch_expression, i, &lc_emlrtRTEI);
  switch_expression_data = switch_expression->data;
  loop_ub = iobj_1->Type->size[1];
  for (i = 0; i < loop_ub; i++) {
    switch_expression_data[i] = iobj_1->Type->data[i];
  }
  b_bool = false;
  if (switch_expression->size[1] == 8) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 8) {
        if (switch_expression_data[loop_ub] != b_cv1[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    loop_ub = 0;
  } else {
    b_bool = false;
    if (switch_expression->size[1] == 9) {
      loop_ub = 0;
      do {
        exitg1 = 0;
        if (loop_ub < 9) {
          if (switch_expression_data[loop_ub] != cv[loop_ub]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      loop_ub = 1;
    } else {
      loop_ub = -1;
    }
  }
  emxFree_char_T(&st, &switch_expression);
  switch (loop_ub) {
  case 0:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv[i];
    }
    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  case 1:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = iv1[i];
    }
    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  default:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0;
    }
    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }
  i = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->MotionSubspace, i, &mc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    iobj_1->MotionSubspace->data[i] = msubspace_data[i];
  }
  i = iobj_1->PositionLimitsInternal->size[0] *
      iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(&st, iobj_1->PositionLimitsInternal, i,
                           &nc_emlrtRTEI);
  for (i = 0; i < 2; i++) {
    iobj_1->PositionLimitsInternal->data[i] = poslim_data[i];
  }
  i = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->HomePositionInternal, i, &oc_emlrtRTEI);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  b_obj->JointInternal = iobj_1;
  b_obj->Index = -1.0;
  b_obj->ParentIndex = -1.0;
  b_obj->MassInternal = 1.0;
  b_obj->CenterOfMassInternal[0] = 0.0;
  b_obj->CenterOfMassInternal[1] = 0.0;
  b_obj->CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    b_I[i] = 0;
  }
  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (i = 0; i < 9; i++) {
    b_obj->InertiaInternal[i] = b_I[i];
  }
  for (i = 0; i < 36; i++) {
    msubspace_data[i] = 0;
  }
  for (loop_ub = 0; loop_ub < 6; loop_ub++) {
    msubspace_data[loop_ub + 6 * loop_ub] = 1;
  }
  for (i = 0; i < 36; i++) {
    b_obj->SpatialInertia[i] = msubspace_data[i];
  }
  st.site = &p_emlrtRSI;
  b_obj->CollisionsInternal = CollisionSet_CollisionSet(&st, iobj_0, 0.0);
  b_obj->matlabCodegenIsDeleted = false;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return b_obj;
}

c_robotics_manip_internal_Rigid *e_RigidBody_RigidBody(
    const emlrtStack *sp, c_robotics_manip_internal_Rigid *obj,
    d_robotics_manip_internal_Colli *iobj_0, rigidBodyJoint *iobj_1)
{
  static const char_T jname[14] = {'d', 'u', 'm', 'm', 'y', 'b', 'o',
                                   'd', 'y', '5', '_', 'j', 'n', 't'};
  static const char_T bname[10] = {'d', 'u', 'm', 'm', 'y',
                                   'b', 'o', 'd', 'y', '5'};
  static const char_T b_cv1[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  static const int8_T b_iv[6] = {0, 0, 1, 0, 0, 0};
  static const int8_T iv1[6] = {0, 0, 0, 0, 0, 1};
  c_robotics_manip_internal_Rigid *b_obj;
  emlrtStack st;
  emxArray_char_T *switch_expression;
  real_T poslim_data[12];
  int32_T exitg1;
  int32_T i;
  int32_T loop_ub;
  char_T *switch_expression_data;
  int8_T msubspace_data[36];
  int8_T b_I[9];
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  b_obj = obj;
  i = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
  b_obj->NameInternal->size[0] = 1;
  b_obj->NameInternal->size[1] = 10;
  emxEnsureCapacity_char_T(sp, b_obj->NameInternal, i, &uc_emlrtRTEI);
  for (i = 0; i < 10; i++) {
    b_obj->NameInternal->data[i] = bname[i];
  }
  st.site = &o_emlrtRSI;
  iobj_1->InTree = false;
  for (i = 0; i < 16; i++) {
    iobj_1->JointToParentTransform[i] = iv[i];
  }
  for (i = 0; i < 16; i++) {
    iobj_1->ChildToJointTransform[i] = iv[i];
  }
  i = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  emxEnsureCapacity_char_T(&st, iobj_1->NameInternal, i, &jc_emlrtRTEI);
  for (i = 0; i < 14; i++) {
    iobj_1->NameInternal->data[i] = jname[i];
  }
  i = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  emxEnsureCapacity_char_T(&st, iobj_1->Type, i, &kc_emlrtRTEI);
  for (i = 0; i < 5; i++) {
    iobj_1->Type->data[i] = b_cv[i];
  }
  emxInit_char_T(&st, &switch_expression, &lc_emlrtRTEI);
  i = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  emxEnsureCapacity_char_T(&st, switch_expression, i, &lc_emlrtRTEI);
  switch_expression_data = switch_expression->data;
  loop_ub = iobj_1->Type->size[1];
  for (i = 0; i < loop_ub; i++) {
    switch_expression_data[i] = iobj_1->Type->data[i];
  }
  b_bool = false;
  if (switch_expression->size[1] == 8) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 8) {
        if (switch_expression_data[loop_ub] != b_cv1[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    loop_ub = 0;
  } else {
    b_bool = false;
    if (switch_expression->size[1] == 9) {
      loop_ub = 0;
      do {
        exitg1 = 0;
        if (loop_ub < 9) {
          if (switch_expression_data[loop_ub] != cv[loop_ub]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      loop_ub = 1;
    } else {
      loop_ub = -1;
    }
  }
  emxFree_char_T(&st, &switch_expression);
  switch (loop_ub) {
  case 0:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv[i];
    }
    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  case 1:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = iv1[i];
    }
    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  default:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0;
    }
    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }
  i = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->MotionSubspace, i, &mc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    iobj_1->MotionSubspace->data[i] = msubspace_data[i];
  }
  i = iobj_1->PositionLimitsInternal->size[0] *
      iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(&st, iobj_1->PositionLimitsInternal, i,
                           &nc_emlrtRTEI);
  for (i = 0; i < 2; i++) {
    iobj_1->PositionLimitsInternal->data[i] = poslim_data[i];
  }
  i = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->HomePositionInternal, i, &oc_emlrtRTEI);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  b_obj->JointInternal = iobj_1;
  b_obj->Index = -1.0;
  b_obj->ParentIndex = -1.0;
  b_obj->MassInternal = 1.0;
  b_obj->CenterOfMassInternal[0] = 0.0;
  b_obj->CenterOfMassInternal[1] = 0.0;
  b_obj->CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    b_I[i] = 0;
  }
  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (i = 0; i < 9; i++) {
    b_obj->InertiaInternal[i] = b_I[i];
  }
  for (i = 0; i < 36; i++) {
    msubspace_data[i] = 0;
  }
  for (loop_ub = 0; loop_ub < 6; loop_ub++) {
    msubspace_data[loop_ub + 6 * loop_ub] = 1;
  }
  for (i = 0; i < 36; i++) {
    b_obj->SpatialInertia[i] = msubspace_data[i];
  }
  st.site = &p_emlrtRSI;
  b_obj->CollisionsInternal = CollisionSet_CollisionSet(&st, iobj_0, 0.0);
  b_obj->matlabCodegenIsDeleted = false;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return b_obj;
}

c_robotics_manip_internal_Rigid *f_RigidBody_RigidBody(
    const emlrtStack *sp, c_robotics_manip_internal_Rigid *obj,
    d_robotics_manip_internal_Colli *iobj_0, rigidBodyJoint *iobj_1)
{
  static const char_T jname[14] = {'d', 'u', 'm', 'm', 'y', 'b', 'o',
                                   'd', 'y', '6', '_', 'j', 'n', 't'};
  static const char_T bname[10] = {'d', 'u', 'm', 'm', 'y',
                                   'b', 'o', 'd', 'y', '6'};
  static const char_T b_cv1[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  static const int8_T b_iv[6] = {0, 0, 1, 0, 0, 0};
  static const int8_T iv1[6] = {0, 0, 0, 0, 0, 1};
  c_robotics_manip_internal_Rigid *b_obj;
  emlrtStack st;
  emxArray_char_T *switch_expression;
  real_T poslim_data[12];
  int32_T exitg1;
  int32_T i;
  int32_T loop_ub;
  char_T *switch_expression_data;
  int8_T msubspace_data[36];
  int8_T b_I[9];
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  b_obj = obj;
  i = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
  b_obj->NameInternal->size[0] = 1;
  b_obj->NameInternal->size[1] = 10;
  emxEnsureCapacity_char_T(sp, b_obj->NameInternal, i, &uc_emlrtRTEI);
  for (i = 0; i < 10; i++) {
    b_obj->NameInternal->data[i] = bname[i];
  }
  st.site = &o_emlrtRSI;
  iobj_1->InTree = false;
  for (i = 0; i < 16; i++) {
    iobj_1->JointToParentTransform[i] = iv[i];
  }
  for (i = 0; i < 16; i++) {
    iobj_1->ChildToJointTransform[i] = iv[i];
  }
  i = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  emxEnsureCapacity_char_T(&st, iobj_1->NameInternal, i, &jc_emlrtRTEI);
  for (i = 0; i < 14; i++) {
    iobj_1->NameInternal->data[i] = jname[i];
  }
  i = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  emxEnsureCapacity_char_T(&st, iobj_1->Type, i, &kc_emlrtRTEI);
  for (i = 0; i < 5; i++) {
    iobj_1->Type->data[i] = b_cv[i];
  }
  emxInit_char_T(&st, &switch_expression, &lc_emlrtRTEI);
  i = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  emxEnsureCapacity_char_T(&st, switch_expression, i, &lc_emlrtRTEI);
  switch_expression_data = switch_expression->data;
  loop_ub = iobj_1->Type->size[1];
  for (i = 0; i < loop_ub; i++) {
    switch_expression_data[i] = iobj_1->Type->data[i];
  }
  b_bool = false;
  if (switch_expression->size[1] == 8) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 8) {
        if (switch_expression_data[loop_ub] != b_cv1[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    loop_ub = 0;
  } else {
    b_bool = false;
    if (switch_expression->size[1] == 9) {
      loop_ub = 0;
      do {
        exitg1 = 0;
        if (loop_ub < 9) {
          if (switch_expression_data[loop_ub] != cv[loop_ub]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      loop_ub = 1;
    } else {
      loop_ub = -1;
    }
  }
  emxFree_char_T(&st, &switch_expression);
  switch (loop_ub) {
  case 0:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv[i];
    }
    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  case 1:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = iv1[i];
    }
    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  default:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0;
    }
    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }
  i = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->MotionSubspace, i, &mc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    iobj_1->MotionSubspace->data[i] = msubspace_data[i];
  }
  i = iobj_1->PositionLimitsInternal->size[0] *
      iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(&st, iobj_1->PositionLimitsInternal, i,
                           &nc_emlrtRTEI);
  for (i = 0; i < 2; i++) {
    iobj_1->PositionLimitsInternal->data[i] = poslim_data[i];
  }
  i = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->HomePositionInternal, i, &oc_emlrtRTEI);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  b_obj->JointInternal = iobj_1;
  b_obj->Index = -1.0;
  b_obj->ParentIndex = -1.0;
  b_obj->MassInternal = 1.0;
  b_obj->CenterOfMassInternal[0] = 0.0;
  b_obj->CenterOfMassInternal[1] = 0.0;
  b_obj->CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    b_I[i] = 0;
  }
  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (i = 0; i < 9; i++) {
    b_obj->InertiaInternal[i] = b_I[i];
  }
  for (i = 0; i < 36; i++) {
    msubspace_data[i] = 0;
  }
  for (loop_ub = 0; loop_ub < 6; loop_ub++) {
    msubspace_data[loop_ub + 6 * loop_ub] = 1;
  }
  for (i = 0; i < 36; i++) {
    b_obj->SpatialInertia[i] = msubspace_data[i];
  }
  st.site = &p_emlrtRSI;
  b_obj->CollisionsInternal = CollisionSet_CollisionSet(&st, iobj_0, 0.0);
  b_obj->matlabCodegenIsDeleted = false;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return b_obj;
}

c_robotics_manip_internal_Rigid *g_RigidBody_RigidBody(
    const emlrtStack *sp, c_robotics_manip_internal_Rigid *obj,
    d_robotics_manip_internal_Colli *iobj_0, rigidBodyJoint *iobj_1)
{
  static const real_T dv[16] = {
      4.8965888601467475E-12, 0.0, -1.0, 0.0, 0.0,  1.0, 0.0, 0.0, 1.0, 0.0,
      4.8965888601467475E-12, 0.0, 0.0,  0.0, 0.19, 1.0};
  static const char_T b[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T b_cv1[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T jname[6] = {'j', 'o', 'i', 'n', 't', '3'};
  static const char_T b_cv[5] = {'l', 'i', 'n', 'k', '3'};
  static const int8_T b_iv[6] = {0, 0, 1, 0, 0, 0};
  static const int8_T iv1[6] = {0, 0, 0, 0, 0, 1};
  static const int8_T iv2[6] = {0, 1, 0, 0, 0, 0};
  c_robotics_manip_internal_Rigid *b_obj;
  emlrtStack st;
  emxArray_char_T *switch_expression;
  real_T poslim_data[12];
  int32_T exitg1;
  int32_T i;
  int32_T loop_ub;
  char_T *switch_expression_data;
  int8_T msubspace_data[36];
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  b_obj = obj;
  i = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
  b_obj->NameInternal->size[0] = 1;
  b_obj->NameInternal->size[1] = 5;
  emxEnsureCapacity_char_T(sp, b_obj->NameInternal, i, &ic_emlrtRTEI);
  for (i = 0; i < 5; i++) {
    b_obj->NameInternal->data[i] = b_cv[i];
  }
  b_obj->ParentIndex = 2.0;
  b_obj->MassInternal = 0.0;
  b_obj->CenterOfMassInternal[0] = 0.0;
  b_obj->CenterOfMassInternal[1] = 0.0;
  b_obj->CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    b_obj->InertiaInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    b_obj->SpatialInertia[i] = 0.0;
  }
  st.site = &u_emlrtRSI;
  iobj_1->InTree = false;
  for (i = 0; i < 16; i++) {
    iobj_1->JointToParentTransform[i] = iv[i];
  }
  for (i = 0; i < 16; i++) {
    iobj_1->ChildToJointTransform[i] = iv[i];
  }
  i = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 6;
  emxEnsureCapacity_char_T(&st, iobj_1->NameInternal, i, &jc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    iobj_1->NameInternal->data[i] = jname[i];
  }
  i = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 8;
  emxEnsureCapacity_char_T(&st, iobj_1->Type, i, &kc_emlrtRTEI);
  for (i = 0; i < 8; i++) {
    iobj_1->Type->data[i] = b[i];
  }
  emxInit_char_T(&st, &switch_expression, &lc_emlrtRTEI);
  i = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  emxEnsureCapacity_char_T(&st, switch_expression, i, &lc_emlrtRTEI);
  switch_expression_data = switch_expression->data;
  loop_ub = iobj_1->Type->size[1];
  for (i = 0; i < loop_ub; i++) {
    switch_expression_data[i] = iobj_1->Type->data[i];
  }
  b_bool = false;
  if (switch_expression->size[1] == 8) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 8) {
        if (switch_expression_data[loop_ub] != b_cv1[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    loop_ub = 0;
  } else {
    b_bool = false;
    if (switch_expression->size[1] == 9) {
      loop_ub = 0;
      do {
        exitg1 = 0;
        if (loop_ub < 9) {
          if (switch_expression_data[loop_ub] != cv[loop_ub]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      loop_ub = 1;
    } else {
      loop_ub = -1;
    }
  }
  emxFree_char_T(&st, &switch_expression);
  switch (loop_ub) {
  case 0:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv[i];
    }
    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  case 1:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = iv1[i];
    }
    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  default:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0;
    }
    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }
  i = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->MotionSubspace, i, &mc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    iobj_1->MotionSubspace->data[i] = msubspace_data[i];
  }
  i = iobj_1->PositionLimitsInternal->size[0] *
      iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(&st, iobj_1->PositionLimitsInternal, i,
                           &nc_emlrtRTEI);
  for (i = 0; i < 2; i++) {
    iobj_1->PositionLimitsInternal->data[i] = poslim_data[i];
  }
  i = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->HomePositionInternal, i, &oc_emlrtRTEI);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  b_obj->JointInternal = iobj_1;
  for (i = 0; i < 16; i++) {
    b_obj->JointInternal->JointToParentTransform[i] = dv[i];
  }
  for (i = 0; i < 16; i++) {
    b_obj->JointInternal->ChildToJointTransform[i] = iv[i];
  }
  i = b_obj->JointInternal->MotionSubspace->size[0] *
      b_obj->JointInternal->MotionSubspace->size[1];
  b_obj->JointInternal->MotionSubspace->size[0] = 6;
  b_obj->JointInternal->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(sp, b_obj->JointInternal->MotionSubspace, i,
                           &pc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    b_obj->JointInternal->MotionSubspace->data[i] = iv2[i];
  }
  b_obj->JointInternal->InTree = true;
  i = b_obj->JointInternal->PositionLimitsInternal->size[0] *
      b_obj->JointInternal->PositionLimitsInternal->size[1];
  b_obj->JointInternal->PositionLimitsInternal->size[0] = 1;
  b_obj->JointInternal->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(sp, b_obj->JointInternal->PositionLimitsInternal, i,
                           &qc_emlrtRTEI);
  b_obj->JointInternal->PositionLimitsInternal->data[0] = -1.92;
  b_obj->JointInternal->PositionLimitsInternal->data[1] = 1.31;
  b_obj->JointInternal->JointAxisInternal[0] = 0.0;
  b_obj->JointInternal->JointAxisInternal[1] = 1.0;
  b_obj->JointInternal->JointAxisInternal[2] = 0.0;
  i = b_obj->JointInternal->HomePositionInternal->size[0];
  b_obj->JointInternal->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(sp, b_obj->JointInternal->HomePositionInternal, i,
                           &rc_emlrtRTEI);
  b_obj->JointInternal->HomePositionInternal->data[0] = 0.0;
  st.site = &v_emlrtRSI;
  b_obj->CollisionsInternal = CollisionSet_CollisionSet(&st, iobj_0, 0.0);
  b_obj->matlabCodegenIsDeleted = false;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return b_obj;
}

c_robotics_manip_internal_Rigid *h_RigidBody_RigidBody(
    const emlrtStack *sp, c_robotics_manip_internal_Rigid *obj,
    d_robotics_manip_internal_Colli *iobj_0, rigidBodyJoint *iobj_1)
{
  static const real_T dv[16] = {1.0, 0.0, -0.0, 0.0, 0.0,   1.0, 0.0, 0.0,
                                0.0, 0.0, 1.0,  0.0, -0.06, 0.0, 0.0, 1.0};
  static const char_T b[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T b_cv1[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T jname[6] = {'j', 'o', 'i', 'n', 't', '4'};
  static const char_T b_cv[5] = {'l', 'i', 'n', 'k', '4'};
  static const int8_T b_iv[6] = {0, 0, 1, 0, 0, 0};
  static const int8_T iv1[6] = {0, 0, 0, 0, 0, 1};
  c_robotics_manip_internal_Rigid *b_obj;
  emlrtStack st;
  emxArray_char_T *switch_expression;
  real_T poslim_data[12];
  int32_T exitg1;
  int32_T i;
  int32_T loop_ub;
  char_T *switch_expression_data;
  int8_T msubspace_data[36];
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  b_obj = obj;
  i = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
  b_obj->NameInternal->size[0] = 1;
  b_obj->NameInternal->size[1] = 5;
  emxEnsureCapacity_char_T(sp, b_obj->NameInternal, i, &ic_emlrtRTEI);
  for (i = 0; i < 5; i++) {
    b_obj->NameInternal->data[i] = b_cv[i];
  }
  b_obj->ParentIndex = 3.0;
  b_obj->MassInternal = 0.0;
  b_obj->CenterOfMassInternal[0] = 0.0;
  b_obj->CenterOfMassInternal[1] = 0.0;
  b_obj->CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    b_obj->InertiaInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    b_obj->SpatialInertia[i] = 0.0;
  }
  st.site = &u_emlrtRSI;
  iobj_1->InTree = false;
  for (i = 0; i < 16; i++) {
    iobj_1->JointToParentTransform[i] = iv[i];
  }
  for (i = 0; i < 16; i++) {
    iobj_1->ChildToJointTransform[i] = iv[i];
  }
  i = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 6;
  emxEnsureCapacity_char_T(&st, iobj_1->NameInternal, i, &jc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    iobj_1->NameInternal->data[i] = jname[i];
  }
  i = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 8;
  emxEnsureCapacity_char_T(&st, iobj_1->Type, i, &kc_emlrtRTEI);
  for (i = 0; i < 8; i++) {
    iobj_1->Type->data[i] = b[i];
  }
  emxInit_char_T(&st, &switch_expression, &lc_emlrtRTEI);
  i = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  emxEnsureCapacity_char_T(&st, switch_expression, i, &lc_emlrtRTEI);
  switch_expression_data = switch_expression->data;
  loop_ub = iobj_1->Type->size[1];
  for (i = 0; i < loop_ub; i++) {
    switch_expression_data[i] = iobj_1->Type->data[i];
  }
  b_bool = false;
  if (switch_expression->size[1] == 8) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 8) {
        if (switch_expression_data[loop_ub] != b_cv1[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    loop_ub = 0;
  } else {
    b_bool = false;
    if (switch_expression->size[1] == 9) {
      loop_ub = 0;
      do {
        exitg1 = 0;
        if (loop_ub < 9) {
          if (switch_expression_data[loop_ub] != cv[loop_ub]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      loop_ub = 1;
    } else {
      loop_ub = -1;
    }
  }
  emxFree_char_T(&st, &switch_expression);
  switch (loop_ub) {
  case 0:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv[i];
    }
    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  case 1:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = iv1[i];
    }
    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  default:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0;
    }
    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }
  i = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->MotionSubspace, i, &mc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    iobj_1->MotionSubspace->data[i] = msubspace_data[i];
  }
  i = iobj_1->PositionLimitsInternal->size[0] *
      iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(&st, iobj_1->PositionLimitsInternal, i,
                           &nc_emlrtRTEI);
  for (i = 0; i < 2; i++) {
    iobj_1->PositionLimitsInternal->data[i] = poslim_data[i];
  }
  i = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->HomePositionInternal, i, &oc_emlrtRTEI);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  b_obj->JointInternal = iobj_1;
  for (i = 0; i < 16; i++) {
    b_obj->JointInternal->JointToParentTransform[i] = dv[i];
  }
  for (i = 0; i < 16; i++) {
    b_obj->JointInternal->ChildToJointTransform[i] = iv[i];
  }
  i = b_obj->JointInternal->MotionSubspace->size[0] *
      b_obj->JointInternal->MotionSubspace->size[1];
  b_obj->JointInternal->MotionSubspace->size[0] = 6;
  b_obj->JointInternal->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(sp, b_obj->JointInternal->MotionSubspace, i,
                           &pc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    b_obj->JointInternal->MotionSubspace->data[i] = b_iv[i];
  }
  b_obj->JointInternal->InTree = true;
  i = b_obj->JointInternal->PositionLimitsInternal->size[0] *
      b_obj->JointInternal->PositionLimitsInternal->size[1];
  b_obj->JointInternal->PositionLimitsInternal->size[0] = 1;
  b_obj->JointInternal->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(sp, b_obj->JointInternal->PositionLimitsInternal, i,
                           &qc_emlrtRTEI);
  b_obj->JointInternal->PositionLimitsInternal->data[0] = -2.44;
  b_obj->JointInternal->PositionLimitsInternal->data[1] = 2.44;
  b_obj->JointInternal->JointAxisInternal[0] = 0.0;
  b_obj->JointInternal->JointAxisInternal[1] = 0.0;
  b_obj->JointInternal->JointAxisInternal[2] = 1.0;
  i = b_obj->JointInternal->HomePositionInternal->size[0];
  b_obj->JointInternal->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(sp, b_obj->JointInternal->HomePositionInternal, i,
                           &rc_emlrtRTEI);
  b_obj->JointInternal->HomePositionInternal->data[0] = 0.0;
  st.site = &v_emlrtRSI;
  b_obj->CollisionsInternal = CollisionSet_CollisionSet(&st, iobj_0, 0.0);
  b_obj->matlabCodegenIsDeleted = false;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return b_obj;
}

c_robotics_manip_internal_Rigid *i_RigidBody_RigidBody(
    const emlrtStack *sp, c_robotics_manip_internal_Rigid *obj,
    d_robotics_manip_internal_Colli *iobj_0, rigidBodyJoint *iobj_1)
{
  static const real_T dv[16] = {1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0,  0.0,
                                0.0, 0.0, 1.0,  0.0, 0.0, 0.0, 0.29, 1.0};
  static const char_T b[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T b_cv1[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T jname[6] = {'j', 'o', 'i', 'n', 't', '5'};
  static const char_T b_cv[5] = {'l', 'i', 'n', 'k', '5'};
  static const int8_T b_iv[6] = {0, 0, 1, 0, 0, 0};
  static const int8_T iv1[6] = {0, 0, 0, 0, 0, 1};
  static const int8_T iv2[6] = {0, 1, 0, 0, 0, 0};
  c_robotics_manip_internal_Rigid *b_obj;
  emlrtStack st;
  emxArray_char_T *switch_expression;
  real_T poslim_data[12];
  int32_T exitg1;
  int32_T i;
  int32_T loop_ub;
  char_T *switch_expression_data;
  int8_T msubspace_data[36];
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  b_obj = obj;
  i = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
  b_obj->NameInternal->size[0] = 1;
  b_obj->NameInternal->size[1] = 5;
  emxEnsureCapacity_char_T(sp, b_obj->NameInternal, i, &ic_emlrtRTEI);
  for (i = 0; i < 5; i++) {
    b_obj->NameInternal->data[i] = b_cv[i];
  }
  b_obj->ParentIndex = 4.0;
  b_obj->MassInternal = 0.0;
  b_obj->CenterOfMassInternal[0] = 0.0;
  b_obj->CenterOfMassInternal[1] = 0.0;
  b_obj->CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    b_obj->InertiaInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    b_obj->SpatialInertia[i] = 0.0;
  }
  st.site = &u_emlrtRSI;
  iobj_1->InTree = false;
  for (i = 0; i < 16; i++) {
    iobj_1->JointToParentTransform[i] = iv[i];
  }
  for (i = 0; i < 16; i++) {
    iobj_1->ChildToJointTransform[i] = iv[i];
  }
  i = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 6;
  emxEnsureCapacity_char_T(&st, iobj_1->NameInternal, i, &jc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    iobj_1->NameInternal->data[i] = jname[i];
  }
  i = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 8;
  emxEnsureCapacity_char_T(&st, iobj_1->Type, i, &kc_emlrtRTEI);
  for (i = 0; i < 8; i++) {
    iobj_1->Type->data[i] = b[i];
  }
  emxInit_char_T(&st, &switch_expression, &lc_emlrtRTEI);
  i = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  emxEnsureCapacity_char_T(&st, switch_expression, i, &lc_emlrtRTEI);
  switch_expression_data = switch_expression->data;
  loop_ub = iobj_1->Type->size[1];
  for (i = 0; i < loop_ub; i++) {
    switch_expression_data[i] = iobj_1->Type->data[i];
  }
  b_bool = false;
  if (switch_expression->size[1] == 8) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 8) {
        if (switch_expression_data[loop_ub] != b_cv1[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    loop_ub = 0;
  } else {
    b_bool = false;
    if (switch_expression->size[1] == 9) {
      loop_ub = 0;
      do {
        exitg1 = 0;
        if (loop_ub < 9) {
          if (switch_expression_data[loop_ub] != cv[loop_ub]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      loop_ub = 1;
    } else {
      loop_ub = -1;
    }
  }
  emxFree_char_T(&st, &switch_expression);
  switch (loop_ub) {
  case 0:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv[i];
    }
    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  case 1:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = iv1[i];
    }
    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  default:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0;
    }
    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }
  i = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->MotionSubspace, i, &mc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    iobj_1->MotionSubspace->data[i] = msubspace_data[i];
  }
  i = iobj_1->PositionLimitsInternal->size[0] *
      iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(&st, iobj_1->PositionLimitsInternal, i,
                           &nc_emlrtRTEI);
  for (i = 0; i < 2; i++) {
    iobj_1->PositionLimitsInternal->data[i] = poslim_data[i];
  }
  i = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->HomePositionInternal, i, &oc_emlrtRTEI);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  b_obj->JointInternal = iobj_1;
  for (i = 0; i < 16; i++) {
    b_obj->JointInternal->JointToParentTransform[i] = dv[i];
  }
  for (i = 0; i < 16; i++) {
    b_obj->JointInternal->ChildToJointTransform[i] = iv[i];
  }
  i = b_obj->JointInternal->MotionSubspace->size[0] *
      b_obj->JointInternal->MotionSubspace->size[1];
  b_obj->JointInternal->MotionSubspace->size[0] = 6;
  b_obj->JointInternal->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(sp, b_obj->JointInternal->MotionSubspace, i,
                           &pc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    b_obj->JointInternal->MotionSubspace->data[i] = iv2[i];
  }
  b_obj->JointInternal->InTree = true;
  i = b_obj->JointInternal->PositionLimitsInternal->size[0] *
      b_obj->JointInternal->PositionLimitsInternal->size[1];
  b_obj->JointInternal->PositionLimitsInternal->size[0] = 1;
  b_obj->JointInternal->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(sp, b_obj->JointInternal->PositionLimitsInternal, i,
                           &qc_emlrtRTEI);
  b_obj->JointInternal->PositionLimitsInternal->data[0] = -1.22;
  b_obj->JointInternal->PositionLimitsInternal->data[1] = 1.05;
  b_obj->JointInternal->JointAxisInternal[0] = 0.0;
  b_obj->JointInternal->JointAxisInternal[1] = 1.0;
  b_obj->JointInternal->JointAxisInternal[2] = 0.0;
  i = b_obj->JointInternal->HomePositionInternal->size[0];
  b_obj->JointInternal->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(sp, b_obj->JointInternal->HomePositionInternal, i,
                           &rc_emlrtRTEI);
  b_obj->JointInternal->HomePositionInternal->data[0] = 0.0;
  st.site = &v_emlrtRSI;
  b_obj->CollisionsInternal = CollisionSet_CollisionSet(&st, iobj_0, 0.0);
  b_obj->matlabCodegenIsDeleted = false;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return b_obj;
}

c_robotics_manip_internal_Rigid *j_RigidBody_RigidBody(
    const emlrtStack *sp, c_robotics_manip_internal_Rigid *obj,
    d_robotics_manip_internal_Colli *iobj_0, rigidBodyJoint *iobj_1)
{
  static const real_T dv[16] = {1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0,   0.0,
                                0.0, 0.0, 1.0,  0.0, 0.0, 0.0, 0.055, 1.0};
  static const char_T b[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T b_cv1[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T jname[6] = {'j', 'o', 'i', 'n', 't', '6'};
  static const char_T b_cv[5] = {'l', 'i', 'n', 'k', '6'};
  static const int8_T b_iv[6] = {0, 0, 1, 0, 0, 0};
  static const int8_T iv1[6] = {0, 0, 0, 0, 0, 1};
  c_robotics_manip_internal_Rigid *b_obj;
  emlrtStack st;
  emxArray_char_T *switch_expression;
  real_T poslim_data[12];
  int32_T exitg1;
  int32_T i;
  int32_T loop_ub;
  char_T *switch_expression_data;
  int8_T msubspace_data[36];
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  b_obj = obj;
  i = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
  b_obj->NameInternal->size[0] = 1;
  b_obj->NameInternal->size[1] = 5;
  emxEnsureCapacity_char_T(sp, b_obj->NameInternal, i, &ic_emlrtRTEI);
  for (i = 0; i < 5; i++) {
    b_obj->NameInternal->data[i] = b_cv[i];
  }
  b_obj->ParentIndex = 5.0;
  b_obj->MassInternal = 0.0;
  b_obj->CenterOfMassInternal[0] = 0.0;
  b_obj->CenterOfMassInternal[1] = 0.0;
  b_obj->CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    b_obj->InertiaInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    b_obj->SpatialInertia[i] = 0.0;
  }
  st.site = &u_emlrtRSI;
  iobj_1->InTree = false;
  for (i = 0; i < 16; i++) {
    iobj_1->JointToParentTransform[i] = iv[i];
  }
  for (i = 0; i < 16; i++) {
    iobj_1->ChildToJointTransform[i] = iv[i];
  }
  i = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 6;
  emxEnsureCapacity_char_T(&st, iobj_1->NameInternal, i, &jc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    iobj_1->NameInternal->data[i] = jname[i];
  }
  i = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 8;
  emxEnsureCapacity_char_T(&st, iobj_1->Type, i, &kc_emlrtRTEI);
  for (i = 0; i < 8; i++) {
    iobj_1->Type->data[i] = b[i];
  }
  emxInit_char_T(&st, &switch_expression, &lc_emlrtRTEI);
  i = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  emxEnsureCapacity_char_T(&st, switch_expression, i, &lc_emlrtRTEI);
  switch_expression_data = switch_expression->data;
  loop_ub = iobj_1->Type->size[1];
  for (i = 0; i < loop_ub; i++) {
    switch_expression_data[i] = iobj_1->Type->data[i];
  }
  b_bool = false;
  if (switch_expression->size[1] == 8) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 8) {
        if (switch_expression_data[loop_ub] != b_cv1[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    loop_ub = 0;
  } else {
    b_bool = false;
    if (switch_expression->size[1] == 9) {
      loop_ub = 0;
      do {
        exitg1 = 0;
        if (loop_ub < 9) {
          if (switch_expression_data[loop_ub] != cv[loop_ub]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      loop_ub = 1;
    } else {
      loop_ub = -1;
    }
  }
  emxFree_char_T(&st, &switch_expression);
  switch (loop_ub) {
  case 0:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv[i];
    }
    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  case 1:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = iv1[i];
    }
    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;
  default:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0;
    }
    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }
  i = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->MotionSubspace, i, &mc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    iobj_1->MotionSubspace->data[i] = msubspace_data[i];
  }
  i = iobj_1->PositionLimitsInternal->size[0] *
      iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(&st, iobj_1->PositionLimitsInternal, i,
                           &nc_emlrtRTEI);
  for (i = 0; i < 2; i++) {
    iobj_1->PositionLimitsInternal->data[i] = poslim_data[i];
  }
  i = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(&st, iobj_1->HomePositionInternal, i, &oc_emlrtRTEI);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  b_obj->JointInternal = iobj_1;
  for (i = 0; i < 16; i++) {
    b_obj->JointInternal->JointToParentTransform[i] = dv[i];
  }
  for (i = 0; i < 16; i++) {
    b_obj->JointInternal->ChildToJointTransform[i] = iv[i];
  }
  i = b_obj->JointInternal->MotionSubspace->size[0] *
      b_obj->JointInternal->MotionSubspace->size[1];
  b_obj->JointInternal->MotionSubspace->size[0] = 6;
  b_obj->JointInternal->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(sp, b_obj->JointInternal->MotionSubspace, i,
                           &pc_emlrtRTEI);
  for (i = 0; i < 6; i++) {
    b_obj->JointInternal->MotionSubspace->data[i] = b_iv[i];
  }
  b_obj->JointInternal->InTree = true;
  i = b_obj->JointInternal->PositionLimitsInternal->size[0] *
      b_obj->JointInternal->PositionLimitsInternal->size[1];
  b_obj->JointInternal->PositionLimitsInternal->size[0] = 1;
  b_obj->JointInternal->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(sp, b_obj->JointInternal->PositionLimitsInternal, i,
                           &qc_emlrtRTEI);
  b_obj->JointInternal->PositionLimitsInternal->data[0] = -2.09;
  b_obj->JointInternal->PositionLimitsInternal->data[1] = 2.09;
  b_obj->JointInternal->JointAxisInternal[0] = 0.0;
  b_obj->JointInternal->JointAxisInternal[1] = 0.0;
  b_obj->JointInternal->JointAxisInternal[2] = 1.0;
  i = b_obj->JointInternal->HomePositionInternal->size[0];
  b_obj->JointInternal->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(sp, b_obj->JointInternal->HomePositionInternal, i,
                           &rc_emlrtRTEI);
  b_obj->JointInternal->HomePositionInternal->data[0] = 0.0;
  st.site = &v_emlrtRSI;
  b_obj->CollisionsInternal = CollisionSet_CollisionSet(&st, iobj_0, 0.0);
  b_obj->matlabCodegenIsDeleted = false;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return b_obj;
}

/* End of code generation (RigidBody.c) */
