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
#ifndef HSRB_ANALYTIC_IK_HSRB_IK_TEST_DRIVER_HPP__
#define HSRB_ANALYTIC_IK_HSRB_IK_TEST_DRIVER_HPP__

#include <string>
#include <vector>

#include <gtest/gtest.h>

// pinocchioのヘッダをboost系のあとにincludeするとビルドが通らないので先にincludeする
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>

#include <boost/foreach.hpp>  // NOLINT
#include <boost/lexical_cast.hpp>  // NOLINT
#include <Eigen/Geometry>  // NOLINT

#include <hsrb_analytic_ik/hsrb_ik_solver.hpp>
#include <tmc_robot_kinematics_model/numeric_ik_solver.hpp>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>

namespace tmc_robot_kinematics_model {

// ファイルから1x8のコンフィギュレーションをあるだけ読み出す．
void LoadDataFile(const std::string& file_name,
                  std::vector<Eigen::VectorXd>& data_out);

// テストドライバ
class HsrbIKTestDrvier : public ::testing::TestWithParam<std::string> {
 protected:
  HsrbIKTestDrvier();
  virtual ~HsrbIKTestDrvier() {}

  IRobotKinematicsModel::Ptr robot_;
  IKSolver::Ptr numeric_solver_;
  IKSolver::Ptr analytic_solver_;

  tmc_manipulation_types::NameSeq use_name_;
  Eigen::VectorXd weight_vector_;
  Eigen::VectorXd joint_min_;
  Eigen::VectorXd joint_max_;

  // 数値解を算出
  bool SolveByNumericIK(const Eigen::Affine3d& ref_origin_to_end,
                        const Eigen::VectorXd& initial_config,
                        Eigen::VectorXd& solution_config_out);
  // 解析解を算出
  bool SolveByAnalyticIK(const Eigen::Affine3d& ref_origin_to_end,
                         const Eigen::VectorXd& initial_config,
                         Eigen::VectorXd& solution_config_out);

  // FKをしてconfigを手先に変換
  void FK(const Eigen::VectorXd& config,
          Eigen::Affine3d& origin_to_end_out);

  // データに対してテストを走らせる
  void RunTest(const std::string ref_config_file,
               const std::string init_config_file);

  // 重み付きNormを計算
  double CalcWeightedNorm(const Eigen::VectorXd& config1,
                          const Eigen::VectorXd& config2) const;

  // 最大値，最小値チェック
  void CheckMinMax(const Eigen::VectorXd& joint) const;
};
}  // namespace tmc_robot_kinematics_model
#endif  // HSRB_ANALYTIC_IK_HSRB_IK_TEST_DRIVER_HPP__
