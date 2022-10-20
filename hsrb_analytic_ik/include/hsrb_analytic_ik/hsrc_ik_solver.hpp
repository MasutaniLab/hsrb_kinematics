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
#ifndef HSRB_ANALYTIC_IK_HSRC_IK_SOLVER_HPP_
#define HSRB_ANALYTIC_IK_HSRC_IK_SOLVER_HPP_

#include <string>

#include <tmc_robot_kinematics_model/ik_solver.hpp>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>

namespace hsrb_analytic_ik {

class HsrcIKSolver : public tmc_robot_kinematics_model::IKSolver {
 public:
  HsrcIKSolver() {}
  /// @param [IN] successor Nextで次に渡したい場合のIK
  /// @param [IN] 初期化に必要なものは追加してください．
  explicit HsrcIKSolver(tmc_robot_kinematics_model::IKSolver::Ptr successor) :
      IKSolver(successor) {}
  virtual ~HsrcIKSolver() {}

  /// IKを解く
  /// @param [IN] request: IKの入力
  /// @param [OUT] solution_angle_out: 解姿勢
  /// @param [OUT] 解の姿勢
  virtual tmc_robot_kinematics_model::IKResult Solve(
      const tmc_robot_kinematics_model::IKRequest& request,
      tmc_manipulation_types::JointState& solution_angle_out,
      Eigen::Affine3d& origin_to_end_out);

  /// IKを解く ただしbaseの移動も許可する
  /// @param [IN] request 目標手先位置や初期値など
  /// @param [OUT] solution_angle_out アームの関節角度の解
  /// @param [OUt] origin_to_base_out 台車部分の解
  /// @param [OUT] origin_to_end_out  解の手先位置．目標手先位置にほぼ一致するはず
  /// @retval kSuccess 成功
  /// @retval kConverge 解でないところに収束
  /// @retval kMaxItr 最大繰り返し回数に到達
  /// @retval kFail 失敗．解なし．
  virtual tmc_robot_kinematics_model::IKResult Solve(
      const tmc_robot_kinematics_model::IKRequest& request,
      tmc_manipulation_types::JointState& solution_angle_out,
      Eigen::Affine3d& origin_to_base_out,
      Eigen::Affine3d& origin_to_end_out);
};

}  // namespace hsrb_analytic_ik
#endif  // HSRB_ANALYTIC_IK_HSRC_IK_SOLVER_HPP_
