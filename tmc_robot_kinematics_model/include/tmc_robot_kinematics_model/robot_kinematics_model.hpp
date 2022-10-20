/*
Copyright (c) 2016 TOYOTA MOTOR CORPORATION
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
#ifndef TMC_ROBOT_KINEMATICS_MODEL_ROBOT_KINEMATICS_MODEL_HPP_
#define TMC_ROBOT_KINEMATICS_MODEL_ROBOT_KINEMATICS_MODEL_HPP_
#include <stdint.h>

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <tmc_manipulation_types/manipulation_types.hpp>

namespace tmc_robot_kinematics_model {

/// JointStateが不正な例外
class JointStateError : public std::domain_error {
 public:
  explicit JointStateError(const std::string &error) :
    std::domain_error("error: " + error + " failed") {}
};

/// ロボットの運動学モデル
/// とりあえず干渉チェックまわりで必要な機能だけ
class IRobotKinematicsModel {
 public:
  typedef boost::shared_ptr<IRobotKinematicsModel> Ptr;
  virtual ~IRobotKinematicsModel() {}

  /// @brief  ロボットの位置姿勢を入力
  /// @param  [in] transform ロボットの姿勢
  virtual void SetRobotTransform(const Eigen::Affine3d& transform) = 0;

  /// @brief  ロボットの位置姿勢を取得
  /// @return Eigen::Affine3d ロボットの姿勢
  virtual Eigen::Affine3d GetRobotTransform(void) const = 0;

  /// @brief  ロボットの関節名を指定し，その関節角を入力
  /// @param  [in] angle ロボット関節情報
  /// @exception domain_error 存在しない関節角名を入力した場合は例外を投げる
  /// @exception domain_error 関節角度と関節名のサイズが違っても例外を投げる
  virtual void SetNamedAngle(
      const tmc_manipulation_types::JointState& angle) = 0;

  /// @brief  ロボットの関節情報を取得
  /// @return JointState ロボットの関節情報
  virtual tmc_manipulation_types::JointState GetNamedAngle(void) const = 0;

  /// @brief  関節名を指定してロボットの関節情報を取得
  /// @param  [in] joint_names 関節名のvector
  /// @return JointState ロボットの関節情報
  /// @exception domain_error 存在しない関節角名を入力した場合は例外を投げる
  virtual tmc_manipulation_types::JointState GetNamedAngle(
      const tmc_manipulation_types::NameSeq& joint_names) const = 0;

  /// @brief  オブジェクトの位置姿勢を取得
  /// @param  [in] name 取得するオブジェクト名
  /// @return Eigen::Affine3d オブジェクトの位置姿勢
  /// @exception domain_error 存在しないオブジェクト名を入力した場合は例外を投げる．
  virtual Eigen::Affine3d GetObjectTransform(
      const std::string& name) const = 0;
  /// オブジェクトの相対位置姿勢を取得
  virtual Eigen::Affine3d GetObjectRelativeTransform(
      const std::string& base_name, const std::string& name) const = 0;

  /// @brief  動的にフレームを追加する
  /// @param  [in] parent_frame_name 親フレームの名前
  /// @param  [in] transform 親フレームを基準とする追加するフレームの位置姿勢
  /// @param  [in] new_frame_name 追加するフレーム名取得するオブジェクト名
  /// @exception domain_error 存在しないオブジェクト名を親フレームとした場合，例外を投げる．
  virtual void CreateFrame(
      const std::string& parent_frame_name, const Eigen::Affine3d& transform,
      const std::string& new_frame_name) = 0;

  /// @brief  動的にフレームを削除する
  /// @param  [in] frame_name 削除するフレーム名
  /// @exception domain_error Createしていないフレーム名を入力した場合，例外を投げる．
  virtual void DestroyFrame(const std::string& frame_name) = 0;

  /// @brief  Jacobianを取得する
  /// @param  [in] frame_name 対象フレーム
  /// @param  [in] frame_to_end 対象フレームからのオフセット
  /// @param  [in] use_joints 対象関節リスト
  /// @return Eigen::MatrixXd Jacobian
  /// @exception domain_error 存在していないフレーム名を入力した場合，例外を投げる．
  /// @exception domain_error 存在していない関節名を入力した場合，例外を投げる．
  virtual Eigen::MatrixXd GetJacobian(const std::string& frame_name,
                                      const Eigen::Affine3d& frame_to_end,
                                      const tmc_manipulation_types::NameSeq& use_joints) = 0;

  /// @brief  関節のMinとMaxを取得する
  /// @param  [in] use_joints 取得する関節名
  /// @param  [out] min 関節角の下限列
  /// @param  [out] max 関節角の上限列
  /// @exception domain_error 存在していない関節名を入力した場合，例外を投げる．
  virtual void GetMinMax(const tmc_manipulation_types::NameSeq& use_joints,
                         Eigen::VectorXd& min, Eigen::VectorXd& max) const = 0;
};
}  // namespace tmc_robot_kinematics_model
#endif /* ROBOT_KINEMATICS_MODEL_HPP_ */
