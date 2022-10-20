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
#include "hsrc_ik_test_driver.hpp"

#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <sys/time.h>

#include <gtest/gtest.h>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

using tmc_robot_kinematics_model::IRobotKinematicsModel;
using tmc_robot_kinematics_model::PinocchioWrapper;
using hsrb_analytic_ik::HsrcIKSolver;
using tmc_manipulation_types::JointState;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Affine3d;
using Eigen::AngleAxisd;
using Eigen::Translation3d;
using std::vector;
using std::string;

namespace {
const char* const kHandName = "hand_palm_link";
const uint32_t kMaxItr = 1000;
const double kEpsilon = 0.001;
const double kConvergeThreshold = 1e-6;
// 手先の許容誤差並進[m]
const double kTransThreshold = 1e-3;
// 手先の許容誤差角度[rad]
const double kRotThreshold = 1e-3;
// 数値計算の場合と比べた最適性の許容限界 10%
const double kOptimizeThreshold = 1.1;
// 数値解の場合と比べた高速性
const double kAnalyticExpectTimes = 2.0;

// 時間計測 gettimeofdayはあまり適切でないので出来れば修正願います．
double get_dtime(void) {
  /*
   struct timeval tv;
   gettimeofday(&tv, NULL);
   return ((double)(tv.tv_sec) + (double)(tv.tv_usec) * 0.001 * 0.001);
   */

  //
  // スレッド固有の CPU 時間を計測します。
  // これによって実行環境の負荷状況に依存しにくい性能テストが行えます。
  // (実行環境に全く影響を受けないわけではありません)
  //
  // 【注意】Ubuntu 12 上で clock_gettime 関数を使うためには、<sys/time.h> をインクルードし、
  // librt.so ライブラリをリンクする必要があります。
  // src/hsrb_analytic_ik/CMakeLists.txt を修正し、
  // CMake の target_link_libraries コマンドによって、リンクするライブラリを追加しています。
  //
  struct timespec t;
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t);
  double seconds = static_cast<double>(t.tv_sec) + static_cast<double>(t.tv_nsec) * 1e-9;
  return seconds;
}

// ロボットの自由度
const uint32_t kDOF = 8;
const uint32_t kArmDOF = 5;
// configurationをangleとbaseの姿勢に変換
void ConfigurationToAngleAndBase(const VectorXd& config, VectorXd& angle_out,
                                 Affine3d& origin_to_base_out) {
  origin_to_base_out = Translation3d(config(0), config(1), 0.0);
  origin_to_base_out.linear() =
  AngleAxisd(config(2), Vector3d::UnitZ()).matrix();
  angle_out = config.tail(kArmDOF);
}

// angleとbaseをconfigurationに変換
void AngleAndBaseToConfiguration(const VectorXd& angle,
                                 const Affine3d& origin_to_base,
                                 VectorXd& config_out) {
  config_out.resize(kDOF);
  config_out(0) = origin_to_base.translation().x();
  config_out(1) = origin_to_base.translation().y();
  config_out(2) = AngleAxisd(origin_to_base.rotation()).axis().z()
  * AngleAxisd(origin_to_base.rotation()).angle();
  config_out.tail(kArmDOF) = angle;
}

// ２つの姿勢が十分近いか判定
// 並進がkTransThreshold[m] 回転がkRotThreshold[rad]以内で近いと判定
bool IsNearAffine(const Affine3d& ref, const Affine3d& cur) {
  double trans_diff = (ref.translation() - cur.translation()).norm();
  double rot_diff =
  fabs(AngleAxisd(ref.linear() * cur.linear().transpose()).angle());
  return ((trans_diff < kTransThreshold) && (rot_diff < kRotThreshold));
}

/**
 * 統計情報を集計するためのクラスです。
 */
class Stats {
 public:
  Stats() {
    Reset();
  }

  void Reset() {
    count_ = 0;
    min_ = +std::numeric_limits<double>::infinity();
    max_ = -std::numeric_limits<double>::infinity();
    sum1_ = 0;
    sum2_ = 0;
  }
  void Add(double value) {
    count_++;
    if (value < min_)
      min_ = value;
    if (max_ < value)
      max_ = value;
    sum1_ += value;
    sum2_ += value * value;
  }
  double Min() const {
    return min_;
  }
  double Max() const {
    return max_;
  }
  double Mean() const {
    return sum1_ / count_;
  }
  double Var() const {
    double m = Mean();
    return (sum2_ - count_ * m * m) / count_;
  }
  double Sd() const {
    return std::sqrt(Var());
  }

 private:
  double count_;
  double min_;
  double max_;
  double sum1_;
  double sum2_;
};

}  // namespace

namespace tmc_robot_kinematics_model {

/// ファイルから1x8のコンフィギュレーションをあるだけ読み出す．
void LoadDataFile(const string& file_name, vector<VectorXd>& data_out) {
  data_out.clear();
  string buf;
  std::ifstream ifs(file_name.c_str());
  while (ifs && getline(ifs, buf)) {
    VectorXd config(kDOF);
    std::stringstream input_line(buf);
    for (uint32_t i = 0; i < kDOF; ++i) {
      input_line >> config(i);
    }
    data_out.push_back(config);
  }
}

HsrcIKTestDrvier::HsrcIKTestDrvier() {
  // ロボットモデル読み込み
  // urdf読み込み
  std::string xml_string;
  std::fstream xml_file("hsrc1s.urdf", std::fstream::in);
  while (xml_file.good()) {
    std::string line;
    std::getline(xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();
  robot_.reset(new PinocchioWrapper(xml_string));

  // 利用関節名
  use_name_.push_back("arm_lift_joint");
  use_name_.push_back("arm_flex_joint");
  use_name_.push_back("arm_roll_joint");
  use_name_.push_back("wrist_flex_joint");
  use_name_.push_back("wrist_roll_joint");
  weight_vector_.resize(8);
  weight_vector_ << 10.0, 10.0, 1.0, 10.0, 1.0, 1.0, 1.0, 1.0;
  numeric_solver_.reset(
                        new NumericIKSolver(
                                            IKSolver::Ptr(),
                                            robot_,
                                            ::kMaxItr,
                                            ::kEpsilon,
                                            ::kConvergeThreshold));
  analytic_solver_.reset(new HsrcIKSolver(IKSolver::Ptr()));

  // 関節の上下限を取得
  robot_->GetMinMax(use_name_, joint_min_, joint_max_);
}

void HsrcIKTestDrvier::FK(const VectorXd& config, Affine3d& origin_to_end_out) {
  Affine3d origin_to_base;
  JointState joint_state;
  joint_state.name = use_name_;
  ConfigurationToAngleAndBase(config, joint_state.position, origin_to_base);
  robot_->SetRobotTransform(origin_to_base);
  robot_->SetNamedAngle(joint_state);
  origin_to_end_out = robot_->GetObjectTransform(kHandName);
}

/// 数値ソルバを使ってIKを解く
/// @param[in] ref_origin_to_end 目標手先位置
/// @param[in] initial_config 初期の台車位置と関節角度
/// @param[out] solution_config_out 解の台車位置と関節角度
/// @return 成功か失敗か
bool HsrcIKTestDrvier::SolveByNumericIK(const Affine3d& ref_origin_to_end,
                                        const VectorXd& initial_config,
                                        VectorXd& solution_config_out) {
  IKRequest req(tmc_manipulation_types::kPlanar);
  req.frame_name = "hand_palm_link";
  req.frame_to_end = Affine3d::Identity();
  req.initial_angle.name = use_name_;
  req.use_joints = use_name_;
  req.weight = weight_vector_;
  req.ref_origin_to_end = ref_origin_to_end;
  ConfigurationToAngleAndBase(
                              initial_config,
                              req.initial_angle.position,
                              req.origin_to_base);

  JointState solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base_solution;
  tmc_robot_kinematics_model::IKResult result;

  result = numeric_solver_->Solve(
                                  req,
                                  solution,
                                  origin_to_base_solution,
                                  origin_to_hand_result);
  if (result == kSuccess) {
    AngleAndBaseToConfiguration(
                                solution.position,
                                origin_to_base_solution,
                                solution_config_out);
    return true;
  } else {
    return false;
  }
}

/// 解析ソルバを使ってIKを解く
/// @param[in] ref_origin_to_end 目標手先位置
/// @param[in] initial_config 初期の台車位置と関節角度
/// @param[out] solution_config_out 解の台車位置と関節角度
/// @return 成功か失敗か
bool HsrcIKTestDrvier::SolveByAnalyticIK(const Affine3d& ref_origin_to_end,
                                         const VectorXd& initial_config,
                                         VectorXd& solution_config_out) {
  IKRequest req(tmc_manipulation_types::kPlanar);
  req.frame_name = "hand_palm_link";
  req.frame_to_end = Affine3d::Identity();
  req.initial_angle.name = use_name_;
  req.use_joints = use_name_;
  req.weight = weight_vector_;
  req.ref_origin_to_end = ref_origin_to_end;
  ConfigurationToAngleAndBase(
                              initial_config,
                              req.initial_angle.position,
                              req.origin_to_base);

  JointState solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base_solution;
  tmc_robot_kinematics_model::IKResult result;

  result = analytic_solver_->Solve(
                                   req,
                                   solution,
                                   origin_to_base_solution,
                                   origin_to_hand_result);
  if (result == kSuccess) {
    AngleAndBaseToConfiguration(
                                solution.position,
                                origin_to_base_solution,
                                solution_config_out);
    return true;
  } else {
    return false;
  }
}

double HsrcIKTestDrvier::CalcWeightedNorm(const VectorXd& config1,
                                          const VectorXd& config2) const {
  VectorXd diff = config1 - config2;
  // diff(2)は無限回転なので正規化
  if (fabs(diff(2)) > M_PI)
    diff(2) = M_PI - fabs(diff(2));
  return (weight_vector_.cwiseProduct(diff)).norm();
}

// 関節部分の最大最小をチェック
void HsrcIKTestDrvier::CheckMinMax(const VectorXd& joint) const {
  for (uint32_t i = 0; i < kArmDOF; ++i) {
    EXPECT_GE(joint(i), joint_min_(i)) << "[" << use_name_[i]
    << "] : lower limit";
    EXPECT_LE(joint(i), joint_max_(i)) << "[" << use_name_[i]
    << "] : higher limit";
  }
}

// データに対してテストを走らせる
void HsrcIKTestDrvier::RunTest(const std::string ref_config_file,
                               const std::string init_config_file) {
  // ランダムな可動域内の関節角．
  std::vector<VectorXd> ref_configs;
  // 初期値の集合．最適化はこれに対して行う．
  std::vector<VectorXd> init_configs;
  LoadDataFile(ref_config_file, ref_configs);
  LoadDataFile(init_config_file, init_configs);

  // 最適性と計算時間を統計的に集計するためのオブジェクトを作成します。
  Stats norm_stats;
  Stats time_stats;

  int32_t numeric_ik_solve = 0;

  // 解のパターンのループ
  BOOST_FOREACH(VectorXd ref_config, ref_configs) {
  // 目標値をFKで作成
  Eigen::Affine3d ref_origin_to_hand;
  FK(ref_config, ref_origin_to_hand);

  // 初期値のパターンのループ
  BOOST_FOREACH(VectorXd init_config, init_configs) {
    // 解析IK+最適化で解を生成 まずは解けることを確認
    VectorXd analytic_solution(8);
    VectorXd numeric_solution(8);
    bool solved;

    double start = get_dtime();
    EXPECT_TRUE(solved = SolveByAnalyticIK(ref_origin_to_hand,
    init_config,
    analytic_solution))
    << "[test case ref " << ref_config.transpose()
    << " ] : IK cannot solve.";

    double end = get_dtime();
    double analytic_elapsed = end - start;

    // 解けないケースは以降のチェックは無意味
    if (!solved) {
      continue;
    }

    // きちんと解になっていることを確認
    Eigen::Affine3d ik_origin_to_hand;
    FK(analytic_solution, ik_origin_to_hand);

    // 目標手先位置と一致するかチェック
    EXPECT_TRUE(IsNearAffine(ref_origin_to_hand, ik_origin_to_hand))
    << "[test case ref " << ref_config.transpose()
    << " ] : IK solution is too far"
    << "\nref_origin_to_hand=\n" << ref_origin_to_hand.matrix()
    << "\nik_origin_to_hand=\n" << ik_origin_to_hand.matrix();  // CJS added.

    // 関節角度内に入るかチェック
    CheckMinMax(analytic_solution.tail(kArmDOF));

    double start_n = get_dtime();
    solved = SolveByNumericIK(ref_origin_to_hand,
    init_config,
    numeric_solution);
    double end_n = get_dtime();
    double numeric_elapsed = end_n - start_n;
    if (solved) {
      ++numeric_ik_solve;

      double analytic_weighted_norm = CalcWeightedNorm(init_config, analytic_solution);
      double numeric_weighted_norm = CalcWeightedNorm(init_config, numeric_solution);

      /*
       // 数値解と最適性を比較して(kOptimizeThreshold)倍以内に収まることを確認
       EXPECT_LE(analytic_weighted_norm,
       numeric_weighted_norm * kOptimizeThreshold);
       // 数値解と計算時間を比較して(kAnalyticExpectTimes)倍早いことを確認
       // これが困難の場合は，init_configsでの平均で比較してください
       EXPECT_LE(analytic_elapsed * kAnalyticExpectTimes,
       numeric_elapsed);
       */

      // テストケース個別の判定から、統計的判定に変更します。
      // (統計データが無限大になるようなケースは除外します)
      if (numeric_weighted_norm > 0 && analytic_elapsed > 0) {
        norm_stats.Add(analytic_weighted_norm / numeric_weighted_norm);
        time_stats.Add(numeric_elapsed / analytic_elapsed);
      }

    } else {
      continue;
    }
  }
}

  std::cout << "numeric_ik_solves : " << numeric_ik_solve << std::endl;

  // 最適性と計算時間の統計的値によりテストを実施します。

  // 数値解と最適性を比較して(kOptimizeThreshold)倍以内に収まることを確認
  EXPECT_LE(norm_stats.Mean(), kOptimizeThreshold);

  // 数値解と計算時間を比較して(kAnalyticExpectTimes)倍早いことを確認
  EXPECT_LE(kAnalyticExpectTimes, time_stats.Mean());

  // 参考までに、統計的値がどれぐらいかを出力します。
  std::cout << "Norm Stats: " << "   mean=" << norm_stats.Mean() << " , sd="
  << norm_stats.Sd() << " , min=" << norm_stats.Min() << " , max="
  << norm_stats.Max() << std::endl;
  std::cout << "Time Stats: " << "   mean=" << time_stats.Mean() << " , sd="
  << time_stats.Sd() << " , min=" << time_stats.Min() << " , max="
  << time_stats.Max() << std::endl;
}

// ランダム目標値でテスト ただし全て必ず解はある
TEST_P(HsrcIKTestDrvier, RandomTest) {
  RunTest(GetParam(), "joint_configs/init_config.dat");
}

// 関節角リミット目標値でテスト ただし全て必ず解はある
TEST_F(HsrcIKTestDrvier, LimitTest) {
  RunTest(
          "joint_configs/limit_config_501.dat",
          "joint_configs/small_init_config.dat");
}

// 特異目標値でテスト ただし全て必ず解はある
TEST_F(HsrcIKTestDrvier, SingularTest) {
  RunTest(
          "joint_configs/singular_config.dat",
          "joint_configs/small_init_config.dat");
}

// 解けない目標値が失敗するテスト
TEST_F(HsrcIKTestDrvier, UnSolveTest) {
  std::vector<VectorXd> init_configs;
  LoadDataFile("joint_configs/small_init_config.dat", init_configs);

  // 初期値のパターンのループ
  BOOST_FOREACH(VectorXd init_config, init_configs) {
  // 解を生成 解が出ないことを確認
  VectorXd analytic_solution(8);
  bool solved;
  Affine3d ref_origin_to_hand = Affine3d::Identity();
  // z = 2mは手が届かない
  ref_origin_to_hand.translation().z() += 2.0;
  EXPECT_FALSE(SolveByAnalyticIK(ref_origin_to_hand,
  init_config,
  analytic_solution));
}
}

INSTANTIATE_TEST_CASE_P(
                        TestDataFiles,
                        HsrcIKTestDrvier,
                        ::testing::Values(
                                          "joint_configs/random_config1.dat",
                                          "joint_configs/random_config2.dat",
                                          "joint_configs/random_config3.dat",
                                          "joint_configs/random_config4.dat",
                                          "joint_configs/random_config5.dat",
                                          "joint_configs/random_config6.dat",
                                          "joint_configs/random_config7.dat",
                                          "joint_configs/random_config8.dat",
                                          "joint_configs/random_config9.dat",
                                          "joint_configs/random_config10.dat"));

}  // namespace tmc_robot_kinematics_model

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  // ::testing::GTEST_FLAG(filter) = "*RandomTest*";
  // ::testing::GTEST_FLAG(filter) = "*UnSolveTest*";
  // ::testing::GTEST_FLAG(filter) = "*SingularTest*";
  // ::testing::GTEST_FLAG(filter) = "*LimitTest*";

  return RUN_ALL_TESTS();
}
