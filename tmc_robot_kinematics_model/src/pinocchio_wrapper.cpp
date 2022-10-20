/*
Copyright (c) 2022 TOYOTA MOTOR CORPORATION
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
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>


namespace {

void Convert(const pinocchio::Model& model, const std::map<std::string, double>& joint_angles,
             Eigen::VectorXd& angle_out) {
  angle_out = Eigen::VectorXd::Zero(model.nq);
  for (int i = 0; i < model.njoints; ++i) {
    if (joint_angles.find(model.names[i]) != joint_angles.end()) {
      angle_out[model.idx_qs[i]] = joint_angles.at(model.names[i]);
    }
  }
}

}  // namespace

namespace tmc_robot_kinematics_model {

PinocchioWrapper::PinocchioWrapper(const std::string& robot_model_xml) {
  try {
    pinocchio::urdf::buildModelFromXML(robot_model_xml, model_);
  } catch(std::invalid_argument& e) {
    // わざわざキャッチして投げ直すのは無駄な気もするけれど，既存のテストとの整合性のため，仕方ない
    throw PinocchioError(e.what());
  }

  jacobian_index_ = model_.addBodyFrame("jacobian_frame", 0);

  for (int i = 0; i < model_.njoints; ++i) {
    // qの数が1の関節が操作対象の関節
    if (model_.nqs[i] == 1) {
      joint_angles_.insert(std::make_pair(model_.names[i], 0.0));
    }
  }
}

// ロボットの位置姿勢を入力
void PinocchioWrapper::SetRobotTransform(const Eigen::Affine3d& transform) {
  // TODO(Takeshita) 必要なときだけデータを更新する
  origin_to_robot_ = transform;
}

// ロボットの位置姿勢を取得
Eigen::Affine3d PinocchioWrapper::GetRobotTransform() const {
  return origin_to_robot_;
}

// ロボットの関節名を指定し，その関節角を入力
void PinocchioWrapper::SetNamedAngle(const tmc_manipulation_types::JointState& angle) {
    if (angle.name.size() != angle.position.size()) {
      throw PinocchioError("Joint name size and position size are mismatch.");
    }
  for (int i = 0; i < angle.name.size(); ++i) {
    if (joint_angles_.find(angle.name[i]) == joint_angles_.end()) {
      throw PinocchioError("Joint "+ angle.name[i] + " is not found.");
    }
    joint_angles_[angle.name[i]] = angle.position[i];
  }
}

// ロボットの関節名とその角度を取得
tmc_manipulation_types::JointState PinocchioWrapper::GetNamedAngle() const {
  // TODO(Takeshita) たかだが10, 20だからmapでいいと思うけれど，別のコンテナも検討する
  tmc_manipulation_types::JointState joint_state;
  joint_state.position.resize(joint_angles_.size());
  for (std::map<std::string, double>::const_iterator it = joint_angles_.cbegin(); it != joint_angles_.cend(); ++it) {
    const auto index = std::distance(joint_angles_.cbegin(), it);
    joint_state.name.push_back(it->first);
    joint_state.position[index] = it->second;
  }
  return joint_state;
}

// ロボットの関節名とその角度を取得
tmc_manipulation_types::JointState PinocchioWrapper::GetNamedAngle(
    const tmc_manipulation_types::NameSeq& joint_names) const {
  tmc_manipulation_types::JointState joint_state;
  joint_state.name = joint_names;
  joint_state.position.resize(joint_names.size());
  for (int i = 0; i < joint_names.size(); ++i) {
    if (joint_angles_.find(joint_names[i]) == joint_angles_.end()) {
      throw PinocchioError("Joint "+ joint_names[i] + " is not found.");
    }
    joint_state.position[i] = joint_angles_.at(joint_names[i]);
  }
  return joint_state;
}

// オブジェクトの位置姿勢を取得
Eigen::Affine3d PinocchioWrapper::GetObjectTransform(const std::string& name) const {
  const auto index = model_.getFrameId(name);
  if (index == model_.nframes) {
    throw PinocchioError("object "+ name + " is not found.");
  }

  pinocchio::Data data(model_);
  ExecuteFK(data);
  auto pose = pinocchio::updateFramePlacement(model_, data, index);
  return origin_to_robot_ * (Eigen::Translation3d(pose.translation()) * Eigen::Quaterniond(pose.rotation()));
}

// オブジェクトの相対位置姿勢を取得
Eigen::Affine3d PinocchioWrapper::GetObjectRelativeTransform(
    const std::string& base_name, const std::string& name) const {
  const auto base_index = model_.getFrameId(base_name);
  if (base_index == model_.nframes) {
    throw PinocchioError("object "+ base_name + " is not found.");
  }

  const auto index = model_.getFrameId(name);
  if (index == model_.nframes) {
    throw PinocchioError("object "+ name + " is not found.");
  }

  pinocchio::Data data(model_);
  ExecuteFK(data);
  auto root_to_base = pinocchio::updateFramePlacement(model_, data, base_index);
  auto root_to_target = pinocchio::updateFramePlacement(model_, data, index);
  auto base_to_target = root_to_base.inverse() * root_to_target;
  return Eigen::Translation3d(base_to_target.translation()) * Eigen::Quaterniond(base_to_target.rotation());
}

// 動的にフレームを追加する
void PinocchioWrapper::CreateFrame(
    const std::string& parent_frame_name,
    const Eigen::Affine3d& transform,
    const std::string& new_frame_name) {
  const auto new_frame_index = model_.addBodyFrame("jacobian_frame", 0);
  const auto parent_frame_index = model_.getFrameId(parent_frame_name);
  if (parent_frame_index == model_.nframes) {
    throw PinocchioError("object "+ parent_frame_name + " is not found.");
  }
  model_.addBodyFrame(new_frame_name, model_.frames[parent_frame_index].parent,
                      pinocchio::SE3(transform.linear(), transform.translation()), parent_frame_index);
}

// 動的にフレームを削除する
void PinocchioWrapper::DestroyFrame(const std::string& frame_name) {
  // addFrameの実装的には,これで削除できるはず
  // ただし,inertiaを扱い出すと,そちらの削除も必要になる
  const auto it = std::find_if(model_.frames.begin(), model_.frames.end(),
                               [&frame_name](decltype(model_.frames.front()) x) { return x.name == frame_name; });
  if (it != model_.frames.end()) {
    model_.frames.erase(it);
    --model_.nframes;
  } else {
    throw PinocchioError("frame "+ frame_name + " is not found.");
  }
}

// Jacobianを取得する
Eigen::MatrixXd PinocchioWrapper::GetJacobian(
    const std::string& frame_name,
    const Eigen::Affine3d& frame_to_end,
    const std::vector<std::string>& use_joints) {
  const auto index = model_.getFrameId(frame_name);
  if (index == model_.nframes) {
    throw PinocchioError("object "+ frame_name + " is not found.");
  }
  model_.frames[jacobian_index_].parent = model_.frames[index].parent;
  model_.frames[jacobian_index_].previousFrame = model_.frames[index].previousFrame;
  model_.frames[jacobian_index_].placement =
      model_.frames[index].placement * pinocchio::SE3(frame_to_end.linear(), frame_to_end.translation());

  Eigen::VectorXd joint_angles;
  Convert(model_, joint_angles_, joint_angles);
  pinocchio::Data data(model_);

  pinocchio::Data::Matrix6x jacobian(6, model_.nv);
  jacobian.setZero();
  pinocchio::computeFrameJacobian(
      model_, data, joint_angles, jacobian_index_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);

  // use_jointsだけ取り出すが，謎の0番目の関節universeの影響で1個ズレる
  Eigen::MatrixXd result(6, use_joints.size());
  for (int joint_name_index = 0; joint_name_index < use_joints.size(); ++joint_name_index) {
    const auto joint_id = model_.getJointId(use_joints[joint_name_index]);
    if (joint_id == model_.njoints) {
      throw PinocchioError("Joint "+ use_joints[joint_name_index] + " is not found.");
    }
    result.col(joint_name_index) = jacobian.col(joint_id - 1);
  }

  Eigen::MatrixXd origin_to_robot = Eigen::MatrixXd::Zero(6, 6);
  origin_to_robot.block<3, 3>(0, 0) = origin_to_robot_.linear();
  origin_to_robot.block<3, 3>(3, 3) = origin_to_robot_.linear();
  return origin_to_robot * result;
}

// 関節のMinとMaxを取得する
void PinocchioWrapper::GetMinMax(
    const tmc_manipulation_types::NameSeq& use_joints,
    Eigen::VectorXd& min,
    Eigen::VectorXd& max) const {
  min.resize(use_joints.size());
  max.resize(use_joints.size());
  // ここも0番目の関節universeの影響で1個ズレる
  for (int i = 0; i < use_joints.size(); ++i) {
    const auto joint_id = model_.getJointId(use_joints[i]);
    if (joint_id == model_.njoints) {
      throw PinocchioError("Joint "+ use_joints[i] + " is not found.");
    }
    min[i] = model_.lowerPositionLimit[joint_id - 1];
    max[i] = model_.upperPositionLimit[joint_id - 1];
  }
}

void PinocchioWrapper::ExecuteFK(pinocchio::Data& data) const {
  // TODO(Takeshita) この計算を毎回やるのがもったいないので，必要なときだけやるようにしたい
  Eigen::VectorXd joint_angles;
  Convert(model_, joint_angles_, joint_angles);
  pinocchio::forwardKinematics(model_, data, joint_angles);
}

}  // namespace tmc_robot_kinematics_model

