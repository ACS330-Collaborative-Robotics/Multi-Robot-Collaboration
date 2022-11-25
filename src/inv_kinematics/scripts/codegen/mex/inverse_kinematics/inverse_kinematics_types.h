/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * inverse_kinematics_types.h
 *
 * Code generation for function 'inverse_kinematics'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include <stddef.h>

/* Type Definitions */
#ifndef typedef_c_robotics_core_internal_System
#define typedef_c_robotics_core_internal_System
typedef struct {
  emlrtTimespec StartTime;
} c_robotics_core_internal_System;
#endif /* typedef_c_robotics_core_internal_System */

#ifndef struct_emxArray_char_T
#define struct_emxArray_char_T
struct emxArray_char_T {
  char_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_char_T */
#ifndef typedef_emxArray_char_T
#define typedef_emxArray_char_T
typedef struct emxArray_char_T emxArray_char_T;
#endif /* typedef_emxArray_char_T */

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T {
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_real_T */
#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T
typedef struct emxArray_real_T emxArray_real_T;
#endif /* typedef_emxArray_real_T */

#ifndef typedef_rigidBodyJoint
#define typedef_rigidBodyJoint
typedef struct {
  emxArray_char_T *Type;
  real_T VelocityNumber;
  real_T PositionNumber;
  emxArray_real_T *MotionSubspace;
  boolean_T InTree;
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
  emxArray_char_T *NameInternal;
  emxArray_real_T *PositionLimitsInternal;
  emxArray_real_T *HomePositionInternal;
  real_T JointAxisInternal[3];
} rigidBodyJoint;
#endif /* typedef_rigidBodyJoint */

#ifndef struct_emxArray_char_T_1x200
#define struct_emxArray_char_T_1x200
struct emxArray_char_T_1x200 {
  char_T data[200];
  int32_T size[2];
};
#endif /* struct_emxArray_char_T_1x200 */
#ifndef typedef_emxArray_char_T_1x200
#define typedef_emxArray_char_T_1x200
typedef struct emxArray_char_T_1x200 emxArray_char_T_1x200;
#endif /* typedef_emxArray_char_T_1x200 */

#ifndef struct_emxArray_real_T_1x1
#define struct_emxArray_real_T_1x1
struct emxArray_real_T_1x1 {
  real_T data[1];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_1x1 */
#ifndef typedef_emxArray_real_T_1x1
#define typedef_emxArray_real_T_1x1
typedef struct emxArray_real_T_1x1 emxArray_real_T_1x1;
#endif /* typedef_emxArray_real_T_1x1 */

#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  emxArray_char_T_1x200 JointName;
  emxArray_real_T_1x1 JointPosition;
} struct_T;
#endif /* typedef_struct_T */

#ifndef typedef_emxArray_struct_T
#define typedef_emxArray_struct_T
typedef struct {
  struct_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
} emxArray_struct_T;
#endif /* typedef_emxArray_struct_T */

#ifndef struct_emxArray_real_T_1x7
#define struct_emxArray_real_T_1x7
struct emxArray_real_T_1x7 {
  real_T data[7];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_1x7 */
#ifndef typedef_emxArray_real_T_1x7
#define typedef_emxArray_real_T_1x7
typedef struct emxArray_real_T_1x7 emxArray_real_T_1x7;
#endif /* typedef_emxArray_real_T_1x7 */

#ifndef typedef_b_struct_T
#define typedef_b_struct_T
typedef struct {
  emxArray_char_T_1x200 JointName;
  emxArray_real_T_1x7 JointPosition;
} b_struct_T;
#endif /* typedef_b_struct_T */

#ifndef typedef_b_emxArray_struct_T
#define typedef_b_emxArray_struct_T
typedef struct {
  b_struct_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
} b_emxArray_struct_T;
#endif /* typedef_b_emxArray_struct_T */

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T
struct emxArray_int32_T {
  int32_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_int32_T */
#ifndef typedef_emxArray_int32_T
#define typedef_emxArray_int32_T
typedef struct emxArray_int32_T emxArray_int32_T;
#endif /* typedef_emxArray_int32_T */

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T
struct emxArray_boolean_T {
  boolean_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_boolean_T */
#ifndef typedef_emxArray_boolean_T
#define typedef_emxArray_boolean_T
typedef struct emxArray_boolean_T emxArray_boolean_T;
#endif /* typedef_emxArray_boolean_T */

#ifndef struct_emxArray_ptrdiff_t
#define struct_emxArray_ptrdiff_t
struct emxArray_ptrdiff_t {
  ptrdiff_t *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_ptrdiff_t */
#ifndef typedef_emxArray_ptrdiff_t
#define typedef_emxArray_ptrdiff_t
typedef struct emxArray_ptrdiff_t emxArray_ptrdiff_t;
#endif /* typedef_emxArray_ptrdiff_t */

#ifndef typedef_c_robotics_manip_internal_Colli
#define typedef_c_robotics_manip_internal_Colli
typedef struct {
  void *CollisionPrimitive;
  real_T LocalPose[16];
  real_T WorldPose[16];
} c_robotics_manip_internal_Colli;
#endif /* typedef_c_robotics_manip_internal_Colli */

#ifndef typedef_c_emxArray_robotics_manip_inter
#define typedef_c_emxArray_robotics_manip_inter
typedef struct {
  c_robotics_manip_internal_Colli *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
} c_emxArray_robotics_manip_inter;
#endif /* typedef_c_emxArray_robotics_manip_inter */

#ifndef typedef_d_robotics_manip_internal_Colli
#define typedef_d_robotics_manip_internal_Colli
typedef struct {
  boolean_T matlabCodegenIsDeleted;
  c_emxArray_robotics_manip_inter *CollisionGeometries;
  real_T MaxElements;
  real_T Size;
} d_robotics_manip_internal_Colli;
#endif /* typedef_d_robotics_manip_internal_Colli */

#ifndef typedef_c_robotics_manip_internal_Rigid
#define typedef_c_robotics_manip_internal_Rigid
typedef struct {
  boolean_T matlabCodegenIsDeleted;
  real_T Index;
  emxArray_char_T *NameInternal;
  rigidBodyJoint *JointInternal;
  real_T ParentIndex;
  real_T MassInternal;
  real_T CenterOfMassInternal[3];
  real_T InertiaInternal[9];
  real_T SpatialInertia[36];
  d_robotics_manip_internal_Colli *CollisionsInternal;
} c_robotics_manip_internal_Rigid;
#endif /* typedef_c_robotics_manip_internal_Rigid */

#ifndef typedef_d_robotics_manip_internal_Rigid
#define typedef_d_robotics_manip_internal_Rigid
typedef struct {
  boolean_T matlabCodegenIsDeleted;
  real_T NumBodies;
  c_robotics_manip_internal_Rigid Base;
  real_T Gravity[3];
  c_robotics_manip_internal_Rigid *Bodies[6];
  real_T NumNonFixedBodies;
  d_robotics_manip_internal_Colli _pobj0[13];
  rigidBodyJoint _pobj1[13];
  c_robotics_manip_internal_Rigid _pobj2[12];
} d_robotics_manip_internal_Rigid;
#endif /* typedef_d_robotics_manip_internal_Rigid */

#ifndef typedef_e_robotics_manip_internal_Rigid
#define typedef_e_robotics_manip_internal_Rigid
typedef struct {
  boolean_T matlabCodegenIsDeleted;
  real_T NumBodies;
  c_robotics_manip_internal_Rigid Base;
  c_robotics_manip_internal_Rigid *Bodies[6];
  real_T NumNonFixedBodies;
  real_T PositionNumber;
  real_T VelocityNumber;
  real_T PositionDoFMap[12];
  real_T VelocityDoFMap[12];
  c_robotics_manip_internal_Rigid _pobj0[6];
  d_robotics_manip_internal_Colli _pobj1[7];
  rigidBodyJoint _pobj2[7];
} e_robotics_manip_internal_Rigid;
#endif /* typedef_e_robotics_manip_internal_Rigid */

#ifndef typedef_c_robotics_manip_internal_IKExt
#define typedef_c_robotics_manip_internal_IKExt
typedef struct {
  boolean_T matlabCodegenIsDeleted;
  e_robotics_manip_internal_Rigid *Robot;
  real_T WeightMatrix[36];
  emxArray_char_T *BodyName;
  real_T Tform[16];
  emxArray_real_T *ErrTemp;
  real_T CostTemp;
  emxArray_real_T *GradTemp;
} c_robotics_manip_internal_IKExt;
#endif /* typedef_c_robotics_manip_internal_IKExt */

#ifndef typedef_c_robotics_core_internal_Damped
#define typedef_c_robotics_core_internal_Damped
typedef struct {
  boolean_T matlabCodegenIsDeleted;
  emxArray_real_T *ConstraintMatrix;
  emxArray_real_T *ConstraintBound;
  boolean_T ConstraintsOn;
  real_T SolutionTolerance;
  boolean_T RandomRestart;
  c_robotics_manip_internal_IKExt *ExtraArgs;
  real_T MaxNumIteration;
  real_T MaxTime;
  emxArray_real_T *SeedInternal;
  real_T MaxTimeInternal;
  real_T MaxNumIterationInternal;
  real_T StepTolerance;
  c_robotics_core_internal_System TimeObj;
  real_T GradientTolerance;
  real_T ArmijoRuleBeta;
  real_T ArmijoRuleSigma;
  c_robotics_core_internal_System TimeObjInternal;
} c_robotics_core_internal_Damped;
#endif /* typedef_c_robotics_core_internal_Damped */

#ifndef typedef_rigidBodyTree
#define typedef_rigidBodyTree
typedef struct {
  boolean_T matlabCodegenIsDeleted;
  e_robotics_manip_internal_Rigid *TreeInternal;
  d_robotics_manip_internal_Colli _pobj0;
  e_robotics_manip_internal_Rigid _pobj1;
} rigidBodyTree;
#endif /* typedef_rigidBodyTree */

#ifndef typedef_inverseKinematics
#define typedef_inverseKinematics
typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  c_robotics_core_internal_Damped *Solver;
  emxArray_real_T *Limits;
  e_robotics_manip_internal_Rigid *RigidBodyTreeInternal;
  c_robotics_manip_internal_IKExt _pobj0;
  rigidBodyJoint _pobj1[12];
  c_robotics_manip_internal_Rigid _pobj2[6];
  d_robotics_manip_internal_Colli _pobj3[13];
  e_robotics_manip_internal_Rigid _pobj4;
  rigidBodyTree _pobj5;
  c_robotics_core_internal_Damped _pobj6;
} inverseKinematics;
#endif /* typedef_inverseKinematics */

#ifndef typedef_d_struct_T
#define typedef_d_struct_T
typedef struct {
  real_T JointPosition;
} d_struct_T;
#endif /* typedef_d_struct_T */

#ifndef typedef_c_emxArray_struct_T
#define typedef_c_emxArray_struct_T
typedef struct {
  d_struct_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
} c_emxArray_struct_T;
#endif /* typedef_c_emxArray_struct_T */

/* End of code generation (inverse_kinematics_types.h) */
