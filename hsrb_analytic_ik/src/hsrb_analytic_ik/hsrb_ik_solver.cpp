/*
Copyright (c) 2015 TOYOTA MOTOR CORPORATION
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include "hsrb_analytic_ik/hsrb_ik_solver.hpp"

#include <map>
#include <string>
#include <vector>

#include <boost/foreach.hpp>

#include "hsr_common_ik_solver.hpp"
#include "robot_optimizer.hpp"

using tmc_robot_kinematics_model::IKSolver;
using tmc_robot_kinematics_model::IKRequest;
using tmc_robot_kinematics_model::IKResult;
using tmc_manipulation_types::JointState;
using Eigen::Translation3d;
using Eigen::Affine3d;
using Eigen::AngleAxisd;
using Eigen::Vector3d;

namespace {

const double kBaselinkToArmLiftJointZ = 0.340;
const double kArmLiftJointToArmFlexJointX = 0.141;
const double kArmLiftJointToArmFlexJointY = 0.078;
const double kArmFlexJointToArmRollJointX = 0.005;
const double kArmFlexJointToArmRollJointZ = 0.345;
const double kWristRollJointToHandPalmJointX = 0.012;
const double kWristRollJointToHandPalmJointZ = 0.1405;
const double kArmLiftJointMin = 0.0;
const double kArmLiftJointMax = 0.69;
const double kArmFlexJointMin = -2.62;
const double kArmFlexJointMax = 0.0;
const double kArmRollJointMin = -1.92;
const double kArmRollJointMax = 3.67;
const double kWristFlexJointMin = -1.92;
const double kWristFlexJointMax = 1.22;
const double kWristRollJointMin = -1.92;
const double kWristRollJointMax = 3.67;

}  // namespace

namespace hsrb_analytic_ik {

/// IKを解く
/// @param [IN] request: IKの入力
/// @param [OUT] solution_angle_out: 解姿勢
/// @param [OUT] 解の姿勢
IKResult HsrbIKSolver::Solve(const IKRequest& request,
                             JointState& solution_angle_out,
                             Eigen::Affine3d& origin_to_end_out) {
  return Next_(request, solution_angle_out, origin_to_end_out);
}

/// IKを解く ただしbaseの移動も許可する
/// @param [IN] request IKの入力
/// @param [OUT] solution_angle_out アームの解
/// @param [OUT] origin_to_base_out 解のロボットの位置姿勢
/// @param [OUT] origin_to_end_out 解の姿勢
/// @retval kSuccess 成功
/// @retval kConverge 解でないところに収束
/// @retval kMaxItr 最大繰り返し回数に到達
/// @retval kFail 失敗．解なし．
IKResult HsrbIKSolver::Solve(const IKRequest& request,
                             JointState& solution_angle_out,
                             Eigen::Affine3d& origin_to_base_out,
                             Eigen::Affine3d& origin_to_end_out) {
  // 以下の条件でのみHybridIKを使う
  // 1. BaseMovementTypeがkPlanar
  // 2. use_jointsが arm_lift_joint, arm_flex_joint, arm_roll_joint, wrist_flex_joint, wrist_roll_joint(順不同)
  // 3. frame_nameがhand_palm_link or hand_palm_joint

  if (!SuitBaseMovement(request) ||
      !SuitUseJoint(request) ||
      !SuitFrame(request)) {
    return Next_(request, solution_angle_out,
                 origin_to_base_out, origin_to_end_out);
  }

  opt::RobotFunction2Parameter function_param;
  function_param.L3 = kBaselinkToArmLiftJointZ;
  function_param.L41 = kArmLiftJointToArmFlexJointX;
  function_param.L42 = kArmLiftJointToArmFlexJointY;
  function_param.L51 = kArmFlexJointToArmRollJointX;
  function_param.L52 = kArmFlexJointToArmRollJointZ;
  function_param.L81 = kWristRollJointToHandPalmJointX;
  function_param.L82 = kWristRollJointToHandPalmJointZ;
  function_param.t3_min = kArmLiftJointMin;
  function_param.t3_max = kArmLiftJointMax;
  function_param.t4_min = kArmFlexJointMin;
  function_param.t4_max = kArmFlexJointMax;
  function_param.t5_min = kArmRollJointMin;
  function_param.t5_max = kArmRollJointMax;
  function_param.t6_min = kWristFlexJointMin;
  function_param.t6_max = kWristFlexJointMax;
  function_param.t7_min = kWristRollJointMin;
  function_param.t7_max = kWristRollJointMax;

  return SolveIK(
      request,
      function_param,
      solution_angle_out,
      origin_to_base_out,
      origin_to_end_out);
}

}  // namespace hsrb_analytic_ik
