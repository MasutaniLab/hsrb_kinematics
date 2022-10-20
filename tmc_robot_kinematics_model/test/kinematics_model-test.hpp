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
#include <algorithm>
#include <fstream>
#include <string>

#include <boost/foreach.hpp>
#include <gtest/gtest.h>

namespace tmc_robot_kinematics_model {

using tmc_manipulation_types::JointState;
using tmc_manipulation_types::NameSeq;

const uint32_t kJointNameNum = 6;
const uint32_t kTotalJointNum = 6;
const char* const kNonExistFile = "hoge.urdf";
// ロボットモデルのパス
const char* const kRobotModel = "stanford.urdf";
const char* const kJointName[kJointNameNum] = {
  "joint1", "joint2", "joint3",
  "joint4", "joint5", "joint6"};
const double kDoubleEps = 1e-10;

/// ２つの姿勢が近いことを計算する
/// @param [in] expect 期待される姿勢
/// @param [in] value  比較する値
/// @param [in] angular_eps 角度ずれ許容値[rad]
/// @param [in] translation_eps 位置ずれ許容値[m]
void ExpectTransformsNear(Eigen::Affine3d expect,
                          Eigen::Affine3d value,
                          double angular_eps = 1.0e-6,
                          double translation_eps = 1.0e-6) {
  Eigen::Affine3d expect_to_value = expect.inverse() * value;
  double diff_angle = Eigen::AngleAxisd(expect_to_value.linear()).angle();
  EXPECT_NEAR(0.0, diff_angle, angular_eps);
  double diff_trans = expect_to_value.translation().norm();
  EXPECT_NEAR(0.0, diff_trans, translation_eps);
}

template<typename T>
class RobotKinematicsModelTest : public ::testing::Test {
 public:
  virtual ~RobotKinematicsModelTest() = default;
  // ~RobotKinematicsModelTest() override = default;

  void SetUp() override;

 protected:
  IRobotKinematicsModel::Ptr robot_model_;
  NameSeq joint_names_;
};

template<typename T>
void RobotKinematicsModelTest<T>::SetUp() {
  // urdf読み込み
  std::fstream xml_file(kRobotModel, std::fstream::in);
  std::string xml_string;
  while (xml_file.good()) {
    std::string line;
    std::getline(xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();

  robot_model_ = IRobotKinematicsModel::Ptr(new T(xml_string));
  for (uint32_t i = 0; i < kJointNameNum; i++) {
    joint_names_.push_back(kJointName[i]);
  }
  JointState angle;
  robot_model_->SetNamedAngle(angle);
}

TYPED_TEST_CASE_P(RobotKinematicsModelTest);

// ファイルが開けない
TYPED_TEST_P(RobotKinematicsModelTest, InvalidFile) {
  EXPECT_THROW(
      IRobotKinematicsModel::Ptr robot_model_(new TypeParam(kNonExistFile)),
      std::domain_error);
}

// ロボット座標系のセットが出来ることをチェック
TYPED_TEST_P(RobotKinematicsModelTest, SetAndGetRobotTransform) {
  Eigen::Affine3d transform(Eigen::Affine3d::Identity());
  transform.translation().setRandom();
  Eigen::Affine3d get_transform;
  this->robot_model_->SetRobotTransform(transform);
  get_transform = this->robot_model_->GetRobotTransform();
  ExpectTransformsNear(transform, get_transform);
}

// ロボットから全関節角度が取得出来ることをチェック
TYPED_TEST_P(RobotKinematicsModelTest, GetAllJointAngles) {
  JointState angles;
  angles = this->robot_model_->GetNamedAngle();
  EXPECT_EQ(kTotalJointNum, angles.name.size());
  EXPECT_EQ(kTotalJointNum, angles.position.size());
}

// ロボットから一部の関節角度が取得出来ることをチェック
TYPED_TEST_P(RobotKinematicsModelTest, GetPartialJointAngles) {
  JointState angles;
  angles = this->robot_model_->GetNamedAngle(this->joint_names_);
  EXPECT_EQ(angles.name.size(), this->joint_names_.size());
  EXPECT_EQ(angles.position.size(), this->joint_names_.size());
}

// ロボットに適当な名前の関節でGetnNamedAngleを読んだ場合例外
TYPED_TEST_P(RobotKinematicsModelTest, ErrorInvalidPartialJointAngles) {
  JointState angles;
  NameSeq invalid_name_seq;
  invalid_name_seq.push_back("hoge_piyo");
  EXPECT_THROW({
      angles = this->robot_model_->GetNamedAngle(invalid_name_seq);
    }, std::domain_error);
}

// ロボットに関節角度を設定できることを確認
TYPED_TEST_P(RobotKinematicsModelTest, SetPartialJointAngles) {
  JointState target_angles;
  JointState result_angles;
  target_angles.name = this->joint_names_;
  target_angles.position.resize(kJointNameNum);
  target_angles.position << 0.0, 0.1, 0.2, 0.3, 0.4, 0.5;
  this->robot_model_->SetNamedAngle(target_angles);
  result_angles = this->robot_model_->GetNamedAngle(this->joint_names_);
  ASSERT_EQ(result_angles.name.size(), target_angles.name.size());
  ASSERT_EQ(result_angles.position.size(), target_angles.position.size());

  for (uint32_t i = 0; i < this->joint_names_.size(); ++i) {
    EXPECT_EQ(result_angles.name[i], target_angles.name[i]);
    EXPECT_FLOAT_EQ(result_angles.position(i), target_angles.position(i));
  }
}

// 不正な名前のセットの際に例外が投げられることを確認
TYPED_TEST_P(RobotKinematicsModelTest, InvalidNameSetPartialJointAngles) {
  JointState target_angles;
  target_angles.name = this->joint_names_;
  target_angles.position.resize(this->joint_names_.size());
  target_angles.name[0] = "hoge_piyo";
  EXPECT_THROW({
      this->robot_model_->SetNamedAngle(target_angles);
    }, std::domain_error);
}

// 名前と角度の配列の長さが違うときに例外が投げられることを確認
TYPED_TEST_P(RobotKinematicsModelTest, UnevenJointstateSetPartialJointAngles) {
  JointState target_angles;
  target_angles.name = this->joint_names_;
  target_angles.position.resize(this->joint_names_.size() + 1);
  EXPECT_THROW({
      this->robot_model_->SetNamedAngle(target_angles);
    }, std::domain_error);
}

// オブジェクトの位置姿勢が取得できるか確認
TYPED_TEST_P(RobotKinematicsModelTest, GetObjectTransform) {
  std::string name = "link2";
  Eigen::Affine3d origin_to_object;
  Eigen::Affine3d origin_to_robot(Eigen::Affine3d::Identity());
  Eigen::Affine3d get_transform;
  this->robot_model_->SetRobotTransform(origin_to_robot);
  origin_to_object = this->robot_model_->GetObjectTransform(name);
  ExpectTransformsNear(Eigen::Affine3d::Identity(), origin_to_object);
}

// URDFのルートリンクの位置姿勢を取得できるか確認
TYPED_TEST_P(RobotKinematicsModelTest, GetObjectTransformRoot) {
  std::string name = "link1";
  Eigen::Affine3d origin_to_object;
  Eigen::Affine3d origin_to_robot(Eigen::Affine3d::Identity());
  Eigen::Affine3d get_transform;
  this->robot_model_->SetRobotTransform(origin_to_robot);
  origin_to_object = this->robot_model_->GetObjectTransform(name);
  ExpectTransformsNear(origin_to_robot, origin_to_object);
}

// fixed_jointを介したオブジェクトの位置姿勢が取得できるか確認
TYPED_TEST_P(RobotKinematicsModelTest, GetObjectTransformFixed) {
  std::string name = "frame1";
  Eigen::Affine3d origin_to_object;
  Eigen::Affine3d origin_to_robot(Eigen::Affine3d::Identity());
  Eigen::Affine3d get_transform;
  this->robot_model_->SetRobotTransform(origin_to_robot);
  origin_to_object = this->robot_model_->GetObjectTransform(name);
  Eigen::Affine3d expected_origin_to_object =
      Eigen::Translation3d(1.0, 2.0, 3.0)
      * Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ());
  ExpectTransformsNear(expected_origin_to_object, origin_to_object);
}

// オブジェクト不正な名前を与えた場合にdomain_errorがでる
TYPED_TEST_P(RobotKinematicsModelTest, InvalidObjectTransform) {
  std::string name = "hoge_piyo";
  Eigen::Affine3d robot_to_object;
  Eigen::Affine3d origin_to_robot(Eigen::Affine3d::Identity());
  this->robot_model_->SetRobotTransform(origin_to_robot);
  EXPECT_THROW({
      robot_to_object = this->robot_model_->GetObjectTransform(name);
    }, std::domain_error);
}

// オブジェクトの相対位置姿勢がとれるか確認
TYPED_TEST_P(RobotKinematicsModelTest, GetObjectRelativeTransform) {
  std::string parent_name = "joint3";
  std::string child_name = "joint4";
  Eigen::Affine3d parent_to_child(Eigen::Affine3d::Identity());
  parent_to_child = this->robot_model_->GetObjectRelativeTransform(parent_name, child_name);
  Eigen::Affine3d expect_parent_to_child(Eigen::Affine3d::Identity());
  expect_parent_to_child.translation().x() = 0.0;
  expect_parent_to_child.translation().y() = 0.0;
  expect_parent_to_child.translation().z() = 0.25;
  ExpectTransformsNear(expect_parent_to_child, parent_to_child);
}

// URDFのルートリンクとオブジェクトとの相対位置姿勢がとれるか確認
TYPED_TEST_P(RobotKinematicsModelTest, GetObjectRelativeTransformRoot) {
  std::string root_name = "link1";
  std::string target_name = "link2";
  Eigen::Affine3d root_to_target(Eigen::Affine3d::Identity());
  Eigen::Affine3d origin_to_robot(Eigen::Affine3d::Identity());
  origin_to_robot.translation().x() = 1.0;
  this->robot_model_->SetRobotTransform(origin_to_robot);
  root_to_target = this->robot_model_->GetObjectRelativeTransform(root_name, target_name);
  ExpectTransformsNear(Eigen::Affine3d::Identity(), root_to_target);
}

// オブジェクト不正な名前を与えた場合にdomain_errorがでる
TYPED_TEST_P(RobotKinematicsModelTest, InvalidObjectRelativeTransform) {
  std::string parent_name = "joint3";
  std::string child_name = "joint4";
  std::string invalid_name = "hoge_piyo";
  EXPECT_THROW({
      this->robot_model_->GetObjectRelativeTransform(parent_name, invalid_name);
    }, std::domain_error);
  EXPECT_THROW({
      this->robot_model_->GetObjectRelativeTransform(invalid_name, child_name);
    }, std::domain_error);
  EXPECT_THROW({
      this->robot_model_->GetObjectRelativeTransform(invalid_name, invalid_name);
    }, std::domain_error);
}

// 動的フレーム追加のテスト
TYPED_TEST_P(RobotKinematicsModelTest, CreateNewFrame) {
  const std::string parent_name = "joint1";
  Eigen::Affine3d parent_to_frame(Eigen::Affine3d::Identity());
  parent_to_frame.translation().setRandom();
  const std::string frame_name = "new_frame";

  this->robot_model_->CreateFrame(parent_name, parent_to_frame, frame_name);

  Eigen::Affine3d parent_to_frame_got =
      this->robot_model_->GetObjectRelativeTransform(parent_name, frame_name);
  ExpectTransformsNear(parent_to_frame, parent_to_frame_got);
}

// 動的フレームの追加テスト失敗
TYPED_TEST_P(RobotKinematicsModelTest, CreateToInvalidFrame) {
  const std::string parent_name = "hoge_piyo";
  Eigen::Affine3d parent_to_frame(Eigen::Affine3d::Identity());
  parent_to_frame.translation().setRandom();
  const std::string frame_name = "new_frame";
  EXPECT_THROW({
      this->robot_model_->CreateFrame(parent_name, parent_to_frame, frame_name);
    }, std::domain_error);
}

// 動的フレーム削除
TYPED_TEST_P(RobotKinematicsModelTest, DeleteDynamicFrame) {
  const std::string parent_name = "joint1";
  Eigen::Affine3d parent_to_frame(Eigen::Affine3d::Identity());
  parent_to_frame.translation().setRandom();
  const std::string frame_name = "new_frame";

  this->robot_model_->CreateFrame(parent_name, parent_to_frame, frame_name);
  this->robot_model_->DestroyFrame(frame_name);

  EXPECT_THROW({
      Eigen::Affine3d parent_to_frame_got =
          this->robot_model_->GetObjectRelativeTransform(parent_name, frame_name);
    }, std::domain_error);
}

// 動的フレーム削除失敗
TYPED_TEST_P(RobotKinematicsModelTest, DeleteInvalidFrame) {
  const std::string parent_name = "joint1";
  Eigen::Affine3d parent_to_frame(Eigen::Affine3d::Identity());
  parent_to_frame.translation().setRandom();
  const std::string frame_name = "new_frame";

  this->robot_model_->CreateFrame(parent_name, parent_to_frame, frame_name);
  EXPECT_THROW({
      this->robot_model_->DestroyFrame("hoge_piyo");
    }, std::domain_error);
}

// Jacobianの取得
TYPED_TEST_P(RobotKinematicsModelTest, GetJacobianNormal) {
  std::string frame_name("link7");
  Eigen::Affine3d frame_to_end(Eigen::Affine3d::Identity());
  tmc_manipulation_types::NameSeq use_joints;
  use_joints.push_back("joint3");
  Eigen::MatrixXd jacobian;
  jacobian = this->robot_model_->GetJacobian(frame_name, frame_to_end, use_joints);
  ASSERT_EQ(6, jacobian.rows());
  ASSERT_EQ(use_joints.size(), jacobian.cols());
  // zだけ1.0になるはず
  EXPECT_NEAR(0.0, jacobian(0, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(1, 0), kDoubleEps);
  EXPECT_NEAR(1.0, jacobian(2, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(3, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(4, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(5, 0), kDoubleEps);
}

// Jacobianの取得台座を動かしてJacobianが変化
TYPED_TEST_P(RobotKinematicsModelTest, GetJacobianRotateLinear) {
  std::string frame_name("link7");
  Eigen::Affine3d frame_to_end(Eigen::Affine3d::Identity());
  tmc_manipulation_types::NameSeq use_joints;
  use_joints.push_back("joint3");
  Eigen::MatrixXd jacobian;
  Eigen::Affine3d origin_to_robot = Eigen::Affine3d::Identity() *
      Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX());
  this->robot_model_->SetRobotTransform(origin_to_robot);
  jacobian = this->robot_model_->GetJacobian(frame_name, frame_to_end, use_joints);
  ASSERT_EQ(6, jacobian.rows());
  ASSERT_EQ(use_joints.size(), jacobian.cols());
  // yだけ-1.0になるはず
  EXPECT_NEAR(0.0, jacobian(0, 0), kDoubleEps);
  EXPECT_NEAR(-1.0, jacobian(1, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(2, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(3, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(4, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(5, 0), kDoubleEps);
}

// Jacobianの取得台座を動かしてJacobianが変化
TYPED_TEST_P(RobotKinematicsModelTest, GetJacobianRotateRotate) {
  std::string frame_name("link7");
  Eigen::Affine3d frame_to_end(Eigen::Affine3d::Identity());
  tmc_manipulation_types::NameSeq use_joints;
  use_joints.push_back("joint1");
  Eigen::MatrixXd jacobian;
  Eigen::Affine3d origin_to_robot = Eigen::Affine3d::Identity() *
      Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX());
  this->robot_model_->SetRobotTransform(origin_to_robot);
  jacobian = this->robot_model_->GetJacobian(frame_name, frame_to_end, use_joints);
  ASSERT_EQ(6, jacobian.rows());
  ASSERT_EQ(use_joints.size(), jacobian.cols());
  // -rotate_yは-1.0になるはず
  EXPECT_NEAR(0.0, jacobian(3, 0), kDoubleEps);
  EXPECT_NEAR(-1.0, jacobian(4, 0), kDoubleEps);
  EXPECT_NEAR(0.0, jacobian(5, 0), kDoubleEps);
}

// jacobianの取得で存在しないフレームが取得された
TYPED_TEST_P(RobotKinematicsModelTest, GetJacobianInvalidFrame) {
  std::string frame_name("hoge_piyo");
  Eigen::Affine3d frame_to_end(Eigen::Affine3d::Identity());
  tmc_manipulation_types::NameSeq use_joints;
  use_joints.push_back("joint3");
  Eigen::MatrixXd jacobian;

  EXPECT_THROW({
      jacobian = this->robot_model_->GetJacobian(frame_name, frame_to_end, use_joints);
    }, std::domain_error);
}

// jacobianの取得で存在しない関節が取得された
TYPED_TEST_P(RobotKinematicsModelTest, GetJacobianInvalidJoint) {
  std::string frame_name("link6");
  Eigen::Affine3d frame_to_end(Eigen::Affine3d::Identity());
  tmc_manipulation_types::NameSeq use_joints;
  use_joints.push_back("hoge_piyo");
  Eigen::MatrixXd jacobian;

  EXPECT_THROW({
      jacobian = this->robot_model_->GetJacobian(frame_name, frame_to_end, use_joints);
    }, std::domain_error);
}

// 関節の上下限取得テスト
TYPED_TEST_P(RobotKinematicsModelTest, GetJointMinMax) {
  tmc_manipulation_types::NameSeq use_joints;
  use_joints.push_back("joint1");
  use_joints.push_back("joint3");
  Eigen::VectorXd min;
  Eigen::VectorXd max;
  this->robot_model_->GetMinMax(use_joints, min, max);
  ASSERT_EQ(use_joints.size(), min.size());
  ASSERT_EQ(use_joints.size(), max.size());
  EXPECT_DOUBLE_EQ(-3.14, min(0));
  EXPECT_DOUBLE_EQ(3.14, max(0));
  EXPECT_DOUBLE_EQ(0.0, min(1));
  EXPECT_DOUBLE_EQ(0.5, max(1));
}

// 関節の上下限取得で存在しない名前を指定
TYPED_TEST_P(RobotKinematicsModelTest, GetInvalidJointMinMax) {
  tmc_manipulation_types::NameSeq use_joints;
  use_joints.push_back("hoge_piyo");
  Eigen::VectorXd min;
  Eigen::VectorXd max;
  EXPECT_THROW({
      this->robot_model_->GetMinMax(use_joints, min, max);
    }, std::domain_error);
}

REGISTER_TYPED_TEST_CASE_P(RobotKinematicsModelTest,
                           InvalidFile,
                           SetAndGetRobotTransform,
                           GetAllJointAngles,
                           GetPartialJointAngles,
                           ErrorInvalidPartialJointAngles,
                           SetPartialJointAngles,
                           InvalidNameSetPartialJointAngles,
                           UnevenJointstateSetPartialJointAngles,
                           GetObjectTransform,
                           GetObjectTransformRoot,
                           GetObjectTransformFixed,
                           InvalidObjectTransform,
                           GetObjectRelativeTransform,
                           GetObjectRelativeTransformRoot,
                           InvalidObjectRelativeTransform,
                           CreateNewFrame,
                           CreateToInvalidFrame,
                           DeleteDynamicFrame,
                           DeleteInvalidFrame,
                           GetJacobianNormal,
                           GetJacobianRotateLinear,
                           GetJacobianRotateRotate,
                           GetJacobianInvalidFrame,
                           GetJacobianInvalidJoint,
                           GetJointMinMax,
                           GetInvalidJointMinMax);

}  // namespace tmc_robot_kinematics_model
