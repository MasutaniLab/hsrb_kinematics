/*
Copyright (c) 2014 TOYOTA MOTOR CORPORATION
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
#ifndef TARP3_WRAPPER_HPP_
#define TARP3_WRAPPER_HPP_

#include <exception>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/scoped_ptr.hpp>
#include <tmc_manipulation_types/manipulation_types.hpp>
#include "tmc_robot_kinematics_model/robot_kinematics_model.hpp"

namespace tmc_robot_kinematics_model {
/// tarp3の関数が失敗した
class Tarp3Error : public std::domain_error {
 public:
  explicit Tarp3Error(const std::string &error) :
    std::domain_error("error: " + error + " failed") {}
};

/// tarp3のラッパー
class Tarp3Wrapper : public IRobotKinematicsModel {
 public:
  explicit Tarp3Wrapper(const std::string& robot_model_config);
  virtual ~Tarp3Wrapper();

  /// ロボットの位置姿勢を入力
  virtual void SetRobotTransform(const Eigen::Affine3d& transform);
  /// ロボットの位置姿勢を取得b
  virtual Eigen::Affine3d GetRobotTransform(void) const;

  /// ロボットの関節名を指定し，その関節角を入力
  virtual void SetNamedAngle(const tmc_manipulation_types::JointState& angle);
  /// ロボットの関節名とその角度を取得
  virtual tmc_manipulation_types::JointState GetNamedAngle(void) const;
  /// ロボットの関節名とその角度を取得
  virtual tmc_manipulation_types::JointState GetNamedAngle(
      const tmc_manipulation_types::NameSeq& joint_names) const;

  /// オブジェクトの位置姿勢を取得
  virtual Eigen::Affine3d GetObjectTransform(const std::string& name) const;
  /// オブジェクトの相対位置姿勢を取得
  virtual Eigen::Affine3d GetObjectRelativeTransform(
      const std::string& base_name, const std::string& name) const;

  /// 動的にフレームを追加する
  virtual void CreateFrame(const std::string& parent_frame_name,
                           const Eigen::Affine3d& transform,
                           const std::string& new_frame_name);
  /// 動的にフレームを削除する
  virtual void DestroyFrame(const std::string& frame_name);
  /// Jacobianを取得する
  virtual Eigen::MatrixXd GetJacobian(
      const std::string& frame_name, const Eigen::Affine3d& frame_to_end,
      const std::vector<std::string>& use_joints);
  /// 関節のMinとMaxを取得する
  virtual void GetMinMax(const tmc_manipulation_types::NameSeq& use_joints,
                         Eigen::VectorXd& min,
                         Eigen::VectorXd& max) const;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 private:
  class Impl;
  boost::scoped_ptr<Impl> pimpl_;
};
}  // namespace tmc_robot_kinematics_model

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus
  // wrapper function for ctypes
  void* load_xml_create_robot(char*);
#ifdef __cplusplus
}
#endif  // __cplusplus

#endif /* TARP3_WRAPPER_HPP_ */
