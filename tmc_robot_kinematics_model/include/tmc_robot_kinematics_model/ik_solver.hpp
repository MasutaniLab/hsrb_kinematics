/*
Copyright (c) 2012 TOYOTA MOTOR CORPORATION
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
#ifndef ROBOT_KINEMATICS_MODEL_IK_SOLVER_HPP__
#define ROBOT_KINEMATICS_MODEL_IK_SOLVER_HPP__

#include <string>
#include <vector>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>

namespace tmc_robot_kinematics_model {
enum IKResult {
  kSuccess,
  kConverge,
  kMaxItr,
  kFail
};

/// IKのリクエスト
struct IKRequest {
  IKRequest() {}
  /// Baseの動作がある時はBaseMovementTypeを指定
  explicit IKRequest(tmc_manipulation_types::BaseMovementType base_type) {
    switch (base_type) {
      case tmc_manipulation_types::kFloat:
        linear_base_movements.push_back(Eigen::Vector3d::UnitX());
        linear_base_movements.push_back(Eigen::Vector3d::UnitY());
        linear_base_movements.push_back(Eigen::Vector3d::UnitZ());
        rotational_base_movements.push_back(Eigen::Vector3d::UnitX());
        rotational_base_movements.push_back(Eigen::Vector3d::UnitY());
        rotational_base_movements.push_back(Eigen::Vector3d::UnitZ());
        break;
      case tmc_manipulation_types::kPlanar:
        linear_base_movements.push_back(Eigen::Vector3d::UnitX());
        linear_base_movements.push_back(Eigen::Vector3d::UnitY());
        rotational_base_movements.push_back(Eigen::Vector3d::UnitZ());
        break;
      case tmc_manipulation_types::kRailX:
        linear_base_movements.push_back(Eigen::Vector3d::UnitX());
        break;
      case tmc_manipulation_types::kRailY:
        linear_base_movements.push_back(Eigen::Vector3d::UnitY());
        break;
      case tmc_manipulation_types::kRailZ:
        linear_base_movements.push_back(Eigen::Vector3d::UnitZ());
        break;
      case tmc_manipulation_types::kRotationX:
        rotational_base_movements.push_back(Eigen::Vector3d::UnitX());
        break;
      case tmc_manipulation_types::kRotationY:
        rotational_base_movements.push_back(Eigen::Vector3d::UnitY());
        break;
      case tmc_manipulation_types::kRotationZ:
        rotational_base_movements.push_back(Eigen::Vector3d::UnitZ());
        break;
      case tmc_manipulation_types::kNone:
        break;
      default:
        break;
    }
  }

  /// 対象とするフレーム名
  std::string frame_name;
  /// 対象フレームからのオフセット
  Eigen::Affine3d frame_to_end;
  /// 目標位置初期姿勢
  Eigen::Affine3d ref_origin_to_end;
  /// ロボット位置姿勢
  Eigen::Affine3d origin_to_base;
  /// 初期姿勢 全関節を入れると良い.
  /// そうでない場合すでに設定されている関節角が使われる．
  tmc_manipulation_types::JointState initial_angle;
  /// 対象とする関節名
  std::vector<std::string> use_joints;
  /// 関節毎の重み use_joints+base_dofと同じ長さにしない場合は無視される
  Eigen::VectorXd weight;
  /// 並進方向のbase移動
  std::vector<Eigen::Vector3d> linear_base_movements;
  /// 回転方向のbase移動
  std::vector<Eigen::Vector3d> rotational_base_movements;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/// IKのインターフェイス
/// chain of responsibilityでIKの方法をカスタマイズ可能
/// 例: 腕6軸を使った解析解があるのでそのようなリクエストの際は解析解を使用
class IKSolver {
 public:
  typedef boost::shared_ptr<IKSolver> Ptr;
  IKSolver() {}
  /// @param [IN] successor Nextで次に渡したい場合のIK
  explicit IKSolver(IKSolver::Ptr successor) : successor_(successor) {}
  virtual ~IKSolver() {}
  void set_successor(IKSolver::Ptr successor) { successor_ = successor;}
  /// IKを解く
  /// @param [IN] request IKのリクエスト
  /// @param [OUT] solution_angle_out 解の関節角度
  /// @param [OUT] origin_to_end_out 最終的な目標位置姿勢
  /// @retval kSuccess 成功
  /// @retval kConverge 終端の誤差は大きいが収束した
  /// @retval kMaxItr 最大繰り返し回数到達
  /// @retval kFail 失敗　解なし
  virtual IKResult Solve(const IKRequest& request,
                         tmc_manipulation_types::JointState& solution_angle_out,
                         Eigen::Affine3d& origin_to_end_out)  {
    return Next_(request, solution_angle_out, origin_to_end_out);
  }

  /// IKを解く
  /// @param [IN] request IKのリクエスト
  /// @param [OUT] solution_angle_out 解の関節角度
  /// @param [OUT] origin_to_base_out 最終的な基準からみた目標位置
  /// @param [OUT] origin_to_end_out 最終的な基準からみた目標位置姿勢
  /// @retval kSuccess 成功
  /// @retval kConverge 終端の誤差は大きいが収束した
  /// @retval kMaxItr 最大繰り返し回数到達
  /// @retval kFail 失敗　解なし
  virtual IKResult Solve(const IKRequest& request,
                         tmc_manipulation_types::JointState& solution_angle_out,
                         Eigen::Affine3d& origin_to_base_out,
                         Eigen::Affine3d& origin_to_end_out) {
    return Next_(request, solution_angle_out,
                 origin_to_end_out, origin_to_end_out);
  }

 protected:
  /// 後任(succesor)に委譲する
  /// @param [IN] request IKのリクエスト
  /// @param [OUT] solution_angle_out 解の関節角度
  /// @param [OUT] origin_to_end_out 最終的な基準から見た目標の位置姿勢
  /// @retval kSuccess 成功
  /// @retval kConverge 終端の誤差は大きいが収束した
  /// @retval kMaxItr 最大繰り返し回数到達
  /// @retval kFail 失敗　解なし
  IKResult Next_(const IKRequest& request,
                 tmc_manipulation_types::JointState& solution_angle_out,
                 Eigen::Affine3d& origin_to_end_out) {
    if (successor_) {
      return successor_->Solve(request, solution_angle_out, origin_to_end_out);
    } else {
      return kFail;
    }
  }

  /// 後任(succesor)に委譲する
  /// @param [IN] request IKのリクエスト
  /// @param [OUT] solution_angle_out 解の関節角度
  /// @param [OUT] origin_to_base_out 最終的な基準から見たロボット位置
  /// @param [OUT] origin_to_end_out 最終的な基準から見た目標の位置姿勢
  /// @retval kSuccess 成功
  /// @retval kConverge 終端の誤差は大きいが収束した
  /// @retval kMaxItr 最大繰り返し回数到達
  /// @retval kFail 失敗　解なし
  IKResult Next_(const IKRequest& request,
                 tmc_manipulation_types::JointState& solution_angle_out,
                 Eigen::Affine3d& origin_to_base_out,
                 Eigen::Affine3d& origin_to_end_out) {
    if (successor_) {
      return successor_->Solve(request, solution_angle_out,
                               origin_to_base_out, origin_to_end_out);
    } else {
      return kFail;
    }
  }

 private:
  Ptr successor_;
};
}  // namespace tmc_robot_kinematics_model
#endif
