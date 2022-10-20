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
#include <fstream>
#include <iostream>
#include <random>
#include <string>

#include <gtest/gtest.h>
#include <sys/time.h>

// pinocchioのヘッダをboost系のあとにincludeするとビルドが通らないので先にincludeする
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>

#include <tmc_robot_kinematics_model/numeric_ik_solver.hpp>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>

using tmc_robot_kinematics_model::IRobotKinematicsModel;
using tmc_robot_kinematics_model::PinocchioWrapper;
using tmc_robot_kinematics_model::IKSolver;
using tmc_robot_kinematics_model::IKRequest;
using tmc_robot_kinematics_model::IKResult;
using tmc_robot_kinematics_model::kSuccess;
using tmc_robot_kinematics_model::kFail;
using tmc_robot_kinematics_model::kMaxItr;
using tmc_robot_kinematics_model::kConverge;
using tmc_robot_kinematics_model::NumericIKSolver;
using tmc_manipulation_types::JointState;
using tmc_manipulation_types::NameSeq;

namespace {
const int32_t kRandomItr = 1000;
// リミットにたいするマージン
const double kDeltaJoint = 0.1;
// ハンドに与える微小摂動
const double kHandDelta = 0.005;
// ロボットモデルのパス
const char* const kRobotModel = "stanford.urdf";

double GetPosition(JointState joint_state, std::string name) {
  if (joint_state.name.size() !=
      static_cast<uint32_t>(joint_state.position.size())) {
    throw std::invalid_argument("name size and position size must be even");
  } else {
    for (uint32_t i = 0; i < joint_state.name.size(); ++i) {
      if (joint_state.name[i] == name) {
        return joint_state.position(i);
      }
    }
  }
  throw std::invalid_argument("cannot found joint");
}

/// 2つの姿勢の変位を計算
double CalcPoseDiff(const Eigen::Affine3d pose_src,
                    const Eigen::Affine3d pose_dst) {
  Eigen::Matrix<double, 6, 1> diff;
  Eigen::Vector3d diff_pos = pose_src.translation() - pose_dst.translation();
  Eigen::Quaterniond rotation_diff(
      pose_dst.linear().transpose() * pose_src.linear());
  rotation_diff = rotation_diff.normalized();
  Eigen::AngleAxisd diff_angle = Eigen::AngleAxisd(rotation_diff);
  diff << diff_pos, diff_angle.angle() * diff_angle.axis();
  return diff.norm();
}
}  // anonymous namespace

/// Testフィスクチャ
class NumericIKTest : public ::testing::Test {
 protected:
  NumericIKTest() {
    Eigen::Affine3d unit(Eigen::Affine3d::Identity());

    // urdf読み込み
    std::fstream xml_file(kRobotModel, std::fstream::in);
    std::string xml_string;
    while (xml_file.good()) {
      std::string line;
      std::getline(xml_file, line);
      xml_string += (line + "\n");
    }
    xml_file.close();

    robot_.reset(new PinocchioWrapper(xml_string));
    robot_->SetRobotTransform(unit);
    JointState angle;
    JointState numeric_angle;

    angle.position.resize(6);
    angle.position(0) = 0.1;
    angle.position(1) = 0.2;
    angle.position(2) = 0.3;
    angle.position(3) = 0.4;
    angle.position(4) = 0.5;
    angle.position(5) = 0.6;
    NameSeq use_name(6);
    use_name[0] = ("joint1");
    use_name[1] = ("joint2");
    use_name[2] = ("joint3");
    use_name[3] = ("joint4");
    use_name[4] = ("joint5");
    use_name[5] = ("joint6");

    angle.name = use_name;
    robot_->SetNamedAngle(angle);
    initial_angle_ = robot_->GetNamedAngle();
    use_name_ = use_name;
  }
  IRobotKinematicsModel::Ptr robot_;
  JointState initial_angle_;
  NameSeq use_name_;
};

// 簡単なケース
TEST_F(NumericIKTest, SimpleCase) {
  double epsilon = 1.0e-3;
  double converge_threshold = 1.0e-10;
  int32_t max_itr = 1000;

  // 数値解のみ
  IKSolver::Ptr solver(
      new NumericIKSolver(IKSolver::Ptr(),
                          robot_, max_itr, epsilon, converge_threshold));

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  Eigen::Affine3d hand = Eigen::Translation3d(-0.2, -0.2, 1.0)
      * Eigen::Quaterniond(0.7214, -0.69634, -0.0874548, 0.0176877);

  IKRequest req;
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d hand_result;
  ASSERT_EQ(kSuccess, solver->Solve(req, solution, hand_result));
  EXPECT_LT(CalcPoseDiff(hand_result, hand), epsilon);
}

// 存在しない関節の指定で例外がでること
TEST_F(NumericIKTest, NoExistJoint) {
  double epsilon = 1.0e-3;
  double converge_threshold = 1.0e-10;
  int32_t max_itr = 1000;
  // 数値解のみ
  IKSolver::Ptr solver(new NumericIKSolver(
      IKSolver::Ptr(), robot_, max_itr, epsilon, converge_threshold));
  Eigen::Affine3d unit(Eigen::Affine3d::Identity());
  Eigen::Affine3d hand = Eigen::Translation3d(0.3, 0.0, 0.05) *
      Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
  IKRequest req;
  NameSeq use_name(6);
  use_name[0] = ("no_exist_joint");
  use_name[1] = ("joint2");
  use_name[2] = ("joint3");
  use_name[3] = ("joint4");
  use_name[4] = ("joint5");
  use_name[5] = ("joint6");

  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name;

  JointState solution;
  Eigen::Affine3d hand_result;
  EXPECT_ANY_THROW(solver->Solve(req, solution, hand_result););
}

// 存在しないフレームの指定で例外がでること
TEST_F(NumericIKTest, NoExistFrame) {
  double epsilon = 1.0e-3;
  double converge_threshold = 1.0e-10;
  int32_t max_itr = 1000;
  // 数値解のみ
  IKSolver::Ptr solver(new NumericIKSolver(
      IKSolver::Ptr(), robot_, max_itr, epsilon, converge_threshold));
  Eigen::Affine3d unit(Eigen::Affine3d::Identity());
  Eigen::Affine3d hand = Eigen::Translation3d(0.3, 0.0, 0.05) *
      Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
  IKRequest req;
  NameSeq use_name(6);
  use_name[0] = ("joint1");
  use_name[1] = ("joint2");
  use_name[2] = ("joint3");
  use_name[3] = ("joint4");
  use_name[4] = ("joint5");
  use_name[5] = ("joint6");

  req.frame_name = "no_exist_frame";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name;

  JointState solution;
  Eigen::Affine3d hand_result;
  EXPECT_ANY_THROW(solver->Solve(req, solution, hand_result););
}

// 6自由度以下の設定で例外がでること
TEST_F(NumericIKTest, UnderSixAxis) {
  double epsilon = 1.0e-3;
  double converge_threshold = 1.0e-10;
  int32_t max_itr = 1000;
  // 数値解のみ
  IKSolver::Ptr solver(new NumericIKSolver(
      IKSolver::Ptr(), robot_, max_itr, epsilon, converge_threshold));
  Eigen::Affine3d unit(Eigen::Affine3d::Identity());
  Eigen::Affine3d hand = Eigen::Translation3d(0.3, 0.0, 0.05) *
      Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
  IKRequest req;
  NameSeq use_name(5);
  use_name[0] = ("joint1");
  use_name[1] = ("joint2");
  use_name[2] = ("joint3");
  use_name[3] = ("joint4");
  use_name[4] = ("joint5");

  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name;

  JointState solution;
  Eigen::Affine3d hand_result;
  EXPECT_ANY_THROW(solver->Solve(req, solution, hand_result););
}

// リーチ外 Converge or MaxItrで終わる
TEST_F(NumericIKTest, OutOfRange) {
  double epsilon = 1.0e-3;
  double converge_threshold = 1.0e-10;
  int32_t max_itr = 1000;
  // 数値解のみ
  IKSolver::Ptr solver(new NumericIKSolver(
      IKSolver::Ptr(), robot_, max_itr, epsilon, converge_threshold));
  Eigen::Affine3d unit(Eigen::Affine3d::Identity());
  Eigen::Affine3d hand = Eigen::Translation3d(2.0, 0.0, 0.05) *
      Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());

  IKRequest req;
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;
  JointState solution;
  Eigen::Affine3d hand_result;
  IKResult result = solver->Solve(req, solution, hand_result);
  EXPECT_TRUE((result == kMaxItr) || (result == kConverge));
}

// 平面移動を使った解が出ること
TEST_F(NumericIKTest, PlanarBaseCase) {
  double epsilon = 1.0e-3;
  double converge_threshold = 1.0e-10;
  int32_t max_itr = 1000;

  // 数値解のみ
  IKSolver::Ptr solver(
      new NumericIKSolver(IKSolver::Ptr(),
                          robot_, max_itr, epsilon, converge_threshold));

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  Eigen::Affine3d origin_to_hand = Eigen::Translation3d(1.2, -0.2, 1.0)
      * Eigen::Quaterniond(0.7214, -0.69634, -0.0874548, 0.0176877);

  IKRequest req(tmc_manipulation_types::kPlanar);
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = origin_to_hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base;
  ASSERT_EQ(kSuccess, solver->Solve(
      req,
      solution,
      origin_to_base,
      origin_to_hand_result));
  // 手先位置が正しい
  EXPECT_LT(CalcPoseDiff(origin_to_hand, origin_to_hand_result), epsilon);
  // 拘束が正しい
  Eigen::Translation3d trans(origin_to_base.translation());
  Eigen::AngleAxisd rotation(origin_to_base.rotation());
  EXPECT_NEAR(0.0, trans.z(), epsilon);
  EXPECT_NEAR(0.0, rotation.axis().x(), epsilon);
  EXPECT_NEAR(0.0, rotation.axis().y(), epsilon);
}

// 浮遊移動を使った解が出ること
TEST_F(NumericIKTest, FloatBaseCase) {
  double epsilon = 1.0e-3;
  double converge_threshold = 1.0e-10;
  int32_t max_itr = 1000;

  // 数値解のみ
  IKSolver::Ptr solver(
      new NumericIKSolver(
          IKSolver::Ptr(),
          robot_, max_itr, epsilon, converge_threshold));

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  Eigen::Affine3d origin_to_hand = Eigen::Translation3d(1.2, -0.2, 1.0)
      * Eigen::Quaterniond(0.7214, -0.69634, -0.0874548, 0.0176877);

  IKRequest req(tmc_manipulation_types::kFloat);
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = origin_to_hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base;
  ASSERT_EQ(kSuccess, solver->Solve(
      req,
      solution,
      origin_to_base,
      origin_to_hand_result));
  // 手先位置が正しい
  EXPECT_LT(CalcPoseDiff(origin_to_hand, origin_to_hand_result), epsilon);
}

// 水平Xを使った解が出ること
TEST_F(NumericIKTest, RailXBaseCase) {
  double epsilon = 1.0e-3;
  double converge_threshold = 1.0e-10;
  int32_t max_itr = 1000;

  // 数値解のみ
  IKSolver::Ptr solver(
      new NumericIKSolver(IKSolver::Ptr(),
                          robot_, max_itr, epsilon, converge_threshold));

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  Eigen::Affine3d origin_to_hand = Eigen::Translation3d(0.3, -0.2, 1.0)
      * Eigen::Quaterniond(0.7214, -0.69634, -0.0874548, 0.0176877);

  IKRequest req(tmc_manipulation_types::kRailX);
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = origin_to_hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base;
  ASSERT_EQ(kSuccess, solver->Solve(
      req,
      solution,
      origin_to_base,
      origin_to_hand_result));
  // 手先位置が正しい
  EXPECT_LT(CalcPoseDiff(origin_to_hand, origin_to_hand_result), epsilon);
  // 拘束が正しい
  Eigen::Translation3d trans(origin_to_base.translation());
  Eigen::AngleAxisd rotation(origin_to_base.rotation());
  EXPECT_NEAR(0.0, trans.y(), epsilon);
  EXPECT_NEAR(0.0, trans.z(), epsilon);
  EXPECT_NEAR(0.0, rotation.angle(), epsilon);
}

// 水平Yを使った解が出ること
TEST_F(NumericIKTest, RailYBaseCase) {
  double epsilon = 1.0e-3;
  double converge_threshold = 1.0e-10;
  int32_t max_itr = 1000;

  // 数値解のみ
  IKSolver::Ptr solver(
      new NumericIKSolver(IKSolver::Ptr(),
                          robot_, max_itr, epsilon, converge_threshold));

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  Eigen::Affine3d origin_to_hand = Eigen::Translation3d(0.3, -1.2, 1.0)
      * Eigen::Quaterniond(0.7214, -0.69634, -0.0874548, 0.0176877);

  IKRequest req(tmc_manipulation_types::kRailY);
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = origin_to_hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base;
  ASSERT_EQ(kSuccess, solver->Solve(
      req,
      solution,
      origin_to_base,
      origin_to_hand_result));
  // 手先位置が正しい
  EXPECT_LT(CalcPoseDiff(origin_to_hand, origin_to_hand_result), epsilon);
  // 拘束が正しい
  Eigen::Translation3d trans(origin_to_base.translation());
  Eigen::AngleAxisd rotation(origin_to_base.rotation());
  EXPECT_NEAR(0.0, trans.x(), epsilon);
  EXPECT_NEAR(0.0, trans.z(), epsilon);
  EXPECT_NEAR(0.0, rotation.angle(), epsilon);
}

// 水平Zを使った解が出ること
TEST_F(NumericIKTest, RailZBaseCase) {
  double epsilon = 1.0e-3;
  double converge_threshold = 1.0e-10;
  int32_t max_itr = 1000;

  // 数値解のみ
  IKSolver::Ptr solver(
      new NumericIKSolver(IKSolver::Ptr(),
                          robot_, max_itr, epsilon, converge_threshold));

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  Eigen::Affine3d origin_to_hand = Eigen::Translation3d(0.3, -0.2, 4.0)
      * Eigen::Quaterniond(0.7214, -0.69634, -0.0874548, 0.0176877);

  IKRequest req(tmc_manipulation_types::kRailZ);
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = origin_to_hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base;
  ASSERT_EQ(kSuccess, solver->Solve(
      req,
      solution,
      origin_to_base,
      origin_to_hand_result));
  // 手先位置が正しい
  EXPECT_LT(CalcPoseDiff(origin_to_hand, origin_to_hand_result), epsilon);
  // 拘束が正しい
  Eigen::Translation3d trans(origin_to_base.translation());
  Eigen::AngleAxisd rotation(origin_to_base.rotation());
  EXPECT_NEAR(0.0, trans.x(), epsilon);
  EXPECT_NEAR(0.0, trans.y(), epsilon);
  EXPECT_NEAR(0.0, rotation.angle(), epsilon);
}

// 回転Xを使った解が出ること
TEST_F(NumericIKTest, RotationXBaseCase) {
  double epsilon = 1.0e-3;
  double converge_threshold = 1.0e-10;
  int32_t max_itr = 1000;

  // 数値解のみ
  IKSolver::Ptr solver(
      new NumericIKSolver(IKSolver::Ptr(),
                          robot_, max_itr, epsilon, converge_threshold));

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  Eigen::Affine3d origin_to_hand = Eigen::Translation3d(0.3, 0.2, 1.0)
      * Eigen::Quaterniond(0.7214, -0.69634, -0.0874548, 0.0176877);

  IKRequest req(tmc_manipulation_types::kRotationX);
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = origin_to_hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base;
  ASSERT_EQ(kSuccess, solver->Solve(
      req,
      solution,
      origin_to_base,
      origin_to_hand_result));
  // 手先位置が正しい
  EXPECT_LT(CalcPoseDiff(origin_to_hand, origin_to_hand_result), epsilon);
  // 拘束が正しい
  Eigen::Translation3d trans(origin_to_base.translation());
  Eigen::AngleAxisd rotation(origin_to_base.rotation());
  EXPECT_NEAR(0.0, trans.x(), epsilon);
  EXPECT_NEAR(0.0, trans.y(), epsilon);
  EXPECT_NEAR(0.0, trans.z(), epsilon);
  EXPECT_NEAR(0.0, rotation.axis().y(), epsilon);
  EXPECT_NEAR(0.0, rotation.axis().z(), epsilon);
}

// 回転Yを使った解が出ること
TEST_F(NumericIKTest, RotationYBaseCase) {
  double epsilon = 1.0e-3;
  double converge_threshold = 1.0e-10;
  int32_t max_itr = 1000;

  // 数値解のみ
  IKSolver::Ptr solver(
      new NumericIKSolver(IKSolver::Ptr(),
                          robot_, max_itr, epsilon, converge_threshold));

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  Eigen::Affine3d origin_to_hand = Eigen::Translation3d(0.3, 0.0, 1.0)
      * Eigen::Quaterniond(0.7214, -0.69634, -0.0874548, 0.0176877);

  IKRequest req(tmc_manipulation_types::kRotationY);
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = origin_to_hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base;
  ASSERT_EQ(kSuccess, solver->Solve(
      req,
      solution,
      origin_to_base,
      origin_to_hand_result));
  // 手先位置が正しい
  EXPECT_LT(CalcPoseDiff(origin_to_hand, origin_to_hand_result), epsilon);
  // 拘束が正しい
  Eigen::Translation3d trans(origin_to_base.translation());
  Eigen::AngleAxisd rotation(origin_to_base.rotation());
  EXPECT_NEAR(0.0, trans.x(), epsilon);
  EXPECT_NEAR(0.0, trans.y(), epsilon);
  EXPECT_NEAR(0.0, trans.z(), epsilon);
  EXPECT_NEAR(0.0, rotation.axis().x(), epsilon);
  EXPECT_NEAR(0.0, rotation.axis().z(), epsilon);
}

// 回転Zを使った解が出ること
TEST_F(NumericIKTest, RotationZBaseCase) {
  double epsilon = 1.0e-3;
  double converge_threshold = 1.0e-10;
  int32_t max_itr = 1000;

  // 数値解のみ
  IKSolver::Ptr solver(
      new NumericIKSolver(IKSolver::Ptr(),
                          robot_, max_itr, epsilon, converge_threshold));

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  Eigen::Affine3d origin_to_hand = Eigen::Translation3d(0.3, -0.2, 1.0)
      * Eigen::Quaterniond(0.7214, -0.69634, -0.0874548, 0.0176877);

  IKRequest req(tmc_manipulation_types::kRotationZ);
  req.frame_name = "link7";
  req.frame_to_end = unit;
  req.origin_to_base = unit;
  req.ref_origin_to_end = origin_to_hand;
  req.initial_angle = initial_angle_;
  req.use_joints = use_name_;

  JointState solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base;
  ASSERT_EQ(kSuccess, solver->Solve(
      req,
      solution,
      origin_to_base,
      origin_to_hand_result));
  // 手先位置が正しい
  EXPECT_LT(CalcPoseDiff(origin_to_hand, origin_to_hand_result), epsilon);
  // 拘束が正しい
  Eigen::Translation3d trans(origin_to_base.translation());
  Eigen::AngleAxisd rotation(origin_to_base.rotation());
  EXPECT_NEAR(0.0, trans.x(), epsilon);
  EXPECT_NEAR(0.0, trans.y(), epsilon);
  EXPECT_NEAR(0.0, trans.z(), epsilon);
  EXPECT_NEAR(0.0, rotation.axis().y(), epsilon);
  EXPECT_NEAR(0.0, rotation.axis().x(), epsilon);
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
