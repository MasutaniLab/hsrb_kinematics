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

#include "hsr_common_ik_solver.hpp"

#include <map>
#include <string>
#include <vector>

#include <boost/foreach.hpp>

#include "robot_optimizer.hpp"

using tmc_robot_kinematics_model::IKSolver;
using tmc_robot_kinematics_model::IKRequest;
using tmc_robot_kinematics_model::IKResult;
using tmc_manipulation_types::JointState;
using Eigen::Translation3d;
using Eigen::Affine3d;
using Eigen::AngleAxisd;
using Eigen::Vector3d;


namespace hsrb_analytic_ik {

/// BaseMoveMentがx,yの並進とzの回転であるか調べる
bool SuitBaseMovement(const tmc_robot_kinematics_model::IKRequest& request) {
  if (request.linear_base_movements.size() != 2) {
    return false;
  }
  if (request.rotational_base_movements.size() != 1) {
    return false;
  }
  if (request.linear_base_movements[0] != Eigen::Vector3d::UnitX()) {
    return false;
  }
  if (request.linear_base_movements[1] != Eigen::Vector3d::UnitY()) {
    return false;
  }
  if (request.rotational_base_movements[0] != Eigen::Vector3d::UnitZ()) {
    return false;
  }
  return true;
}

/// Jointが腕の軸全てを含むか調べる
bool SuitUseJoint(const tmc_robot_kinematics_model::IKRequest& request) {
  std::string arm_joints[] =
      {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};

  BOOST_FOREACH(std::string arm_joint, arm_joints) {
    if (std::find(request.use_joints.begin(),
                  request.use_joints.end(),
                  arm_joint) == request.use_joints.end()) {
      return false;
    }
  }
  return true;
}

/// frame_nameがhand_palm_linkか調べる
bool SuitFrame(const tmc_robot_kinematics_model::IKRequest& request) {
  return ((request.frame_name == "hand_palm_link") ||
          (request.frame_name == "hand_palm_joint"));
}

/// 関節名とIDのマップを作る
bool MapJointAndId(const std::vector<std::string>& use_joints,
                   std::map<std::string, uint32_t>& map_out) {
  std::string arm_joints[] =
      {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};
  map_out.clear();
  BOOST_FOREACH(std::string arm_joint, arm_joints) {
    std::vector<std::string>::const_iterator it =
        std::find(use_joints.begin(), use_joints.end(), arm_joint);
    if (it == use_joints.end()) {
      return false;
    } else {
      map_out[arm_joint] = std::distance(use_joints.begin(), it);
    }
  }
  return true;
}

/// joint_stateからjoint_namesの値を出力する．見つからない場合はdefault_posを返す．
double ExtractJointPosition(
    const tmc_manipulation_types::JointState& joint_state,
    const std::string& joint_name,
    double default_pos) {
  double pos = default_pos;
  if (joint_state.name.size() != joint_state.position.size()) {
    return pos;
  }
  for (uint32_t i = 0; i < joint_state.name.size(); ++i) {
    if (joint_state.name[i] == joint_name) {
      return joint_state.position(i);
    }
  }
  return default_pos;
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
IKResult SolveIK(const IKRequest& request,
                 const opt::RobotFunction2Parameter& function_param,
                 JointState& solution_angle_out,
                 Eigen::Affine3d& origin_to_base_out,
                 Eigen::Affine3d& origin_to_end_out
                 ) {
  std::map<std::string, uint32_t> joint_map;
  MapJointAndId(request.use_joints, joint_map);

  solution_angle_out.name = request.use_joints;
  solution_angle_out.position.resize(request.use_joints.size());
  // チェーン外の関節角度はinitialをコピー
  for (uint32_t i = 0; i < request.use_joints.size(); ++i) {
    solution_angle_out.position[i] =
        ExtractJointPosition(request.initial_angle,
                             request.use_joints[i], 0.0);
  }

  origin_to_base_out = Eigen::Affine3d::Identity();
  origin_to_end_out = Eigen::Affine3d::Identity();

  // 入力をコピーします。
  opt::RobotFunction2Request function_req;
  // 同時変換行列の参照値 (T_ref) を設定します。
  function_req.R11 = request.ref_origin_to_end(0, 0);
  function_req.R21 = request.ref_origin_to_end(1, 0);
  function_req.R31 = request.ref_origin_to_end(2, 0);

  function_req.R12 = request.ref_origin_to_end(0, 1);
  function_req.R22 = request.ref_origin_to_end(1, 1);
  function_req.R32 = request.ref_origin_to_end(2, 1);

  function_req.R13 = request.ref_origin_to_end(0, 2);
  function_req.R23 = request.ref_origin_to_end(1, 2);
  function_req.R33 = request.ref_origin_to_end(2, 2);

  function_req.px = request.ref_origin_to_end(0, 3);
  function_req.py = request.ref_origin_to_end(1, 3);
  function_req.pz = request.ref_origin_to_end(2, 3);

  // 重み行列 (対角行列) の対角成分 w_i (i=0..7) を設定します。
  function_req.w0 = request.weight(request.use_joints.size());
  function_req.w1 = request.weight(request.use_joints.size() + 1);
  function_req.w2 = request.weight(request.use_joints.size() + 2);
  function_req.w3 = request.weight(joint_map["arm_lift_joint"]);
  function_req.w4 = request.weight(joint_map["arm_flex_joint"]);
  function_req.w5 = request.weight(joint_map["arm_roll_joint"]);
  function_req.w6 = request.weight(joint_map["wrist_flex_joint"]);
  function_req.w7 = request.weight(joint_map["wrist_roll_joint"]);

  // パラメータの参照値 θ^ref_i (i=0..7) を設定します。
  function_req.r0 = request.origin_to_base(0, 3);
  function_req.r1 = request.origin_to_base(1, 3);
  function_req.r2 = std::atan2(request.origin_to_base(1, 0), request.origin_to_base(0, 0));
  function_req.r3 = ExtractJointPosition(request.initial_angle, "arm_lift_joint", 0.0);
  function_req.r4 = ExtractJointPosition(request.initial_angle, "arm_flex_joint", 0.0);
  function_req.r5 = ExtractJointPosition(request.initial_angle, "arm_roll_joint", 0.0);
  function_req.r6 = ExtractJointPosition(request.initial_angle, "wrist_flex_joint", 0.0);
  function_req.r7 = ExtractJointPosition(request.initial_angle, "wrist_roll_joint", 0.0);

  // 最適化の目的関数を作成します。
  opt::RobotFunction2 f(function_req, function_param);

  // 最適化を実行します。
  opt::OptResult result = opt::RobotOptimizer::Optimize(f);

  // 何らかの理由で最適化に失敗した場合は、解なしとします。
  // (使用する最適化法によっては、OptFail は返されないことがあります。)
  if (result == opt::OptFail) {
    return tmc_robot_kinematics_model::kFail;
  }

  // 最大反復数に至っても、最適化は成功したものとして扱います。
  if (result == opt::OptMaxItor) {
    // return kMaxItr;
  }

  // 最適解が実行可能領域外にある場合、その外れ具合がわずかであれば、
  // 強制的に実行可能領域内に引き戻して解とし、最適化が成功したものとして扱います。
  // 実行可能領域が一直線上にあるケースでは、計算誤差によって内点に収束することは望めないので、
  // この処理が必要になってきます。
  // テストドライバの LimitTest ではこのようなケースがあります。
  double outerGrade = f.GetOuterGrade();
  if (outerGrade > 0) {
    if (outerGrade <= 1e-6) {
      f.ForceFeasible();
    } else {
      return tmc_robot_kinematics_model::kFail;
    }
  }

  // 出力をコピーします。
  opt::RobotFunction2Response function_res(f.response());

  // solution_angle_out.position を作成します。
  solution_angle_out.position(joint_map["arm_lift_joint"]) = function_res.t3;
  solution_angle_out.position(joint_map["arm_flex_joint"]) = function_res.t4;
  solution_angle_out.position(joint_map["arm_roll_joint"]) = function_res.t5;
  solution_angle_out.position(joint_map["wrist_flex_joint"]) = function_res.t6;
  solution_angle_out.position(joint_map["wrist_roll_joint"]) = function_res.t7;

  // パラメータ θ_i に対応するリンク間の同時変換行列を作成します。
  Translation3d   T0 = Translation3d(function_res.t0, 0, 0);
  Translation3d   T1 = Translation3d(0, function_res.t1, 0);
  AngleAxisd      T2 = AngleAxisd(function_res.t2, Vector3d(0, 0, 1));
  Translation3d   T3 = Translation3d(0, 0, function_res.t3 + function_param.L3);
  Affine3d        T4 = Translation3d(function_param.L41, function_param.L42, 0)
                     * AngleAxisd(-function_res.t4, Vector3d(0, 1, 0));
  Affine3d        T5 = Translation3d(function_param.L51, 0, function_param.L52)
                     * AngleAxisd(function_res.t5, Vector3d(0, 0, 1));
  AngleAxisd      T6 = AngleAxisd(-function_res.t6, Vector3d(0, 1, 0));
  AngleAxisd      T7 = AngleAxisd(function_res.t7, Vector3d(0, 0, 1));
  Affine3d        T8 = Translation3d(function_param.L81, 0, function_param.L82)
                     * AngleAxisd(M_PI, Vector3d(0, 0, 1));

  // origin_to_base_out, origin_to_end_out を作成します。
  origin_to_base_out = T0 * T1 * T2;
  origin_to_end_out = origin_to_base_out * T3 * T4 * T5 * T6 * T7 * T8;

  return tmc_robot_kinematics_model::kSuccess;
}
}  // namespace hsrb_analytic_ik
