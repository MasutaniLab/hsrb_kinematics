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
#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include <tmc_robot_kinematics_model/numeric_ik_solver.hpp>

using tmc_manipulation_types::JointState;

namespace {
const uint32_t kSE3Dim = 6;
// 杉原法に使う微小バイアス sugihara 2009 より
const double kWn = 1e-3;

/// 関節角度をリミット内に収める
/// @param [IN] min 関節角度の最小値
/// @param [IN] min 関節角度の最大値
/// @param [IN,OUT] angle 関節角度
void SaturateAngle(const Eigen::VectorXd& min, const Eigen::VectorXd& max,
                   Eigen::VectorXd& angle) {
  for (int32_t i = 0; i < angle.size(); ++i) {
    angle(i) = std::max(min(i), angle(i));
    angle(i) = std::min(max(i), angle(i));
  }
}
}  // anonymous namespace

namespace tmc_robot_kinematics_model {

/// 数値的にIKを解く use_jointsが６以下だと失敗
/// @param [IN] request: IKの入力
/// @param [OUT] solution_angle_out: 解姿勢
/// @param [OUT] 解の姿勢
IKResult NumericIKSolver::Solve(const IKRequest& request,
                                JointState& solution_angle_out,
                                Eigen::Affine3d& origin_to_end_out) {
  Eigen::Affine3d origin_to_base_dummy;
  return Solve(request,
               solution_angle_out,
               origin_to_base_dummy,
               origin_to_end_out);
}

/// 数値的にIKを解く ただしbaseの移動も許可する
/// @param [IN] request IKの入力
/// @param [OUT] solution_angle_out 解姿勢
/// @param [OUT] origin_to_base_out 解のロボットの位置姿勢
/// @param [OUT] origin_to_end_out 解の姿勢
IKResult NumericIKSolver::Solve(const IKRequest& request,
                                JointState& solution_angle_out,
                                Eigen::Affine3d& origin_to_base_out,
                                Eigen::Affine3d& origin_to_end_out) {
  double delta = 0.0;
  double delta_old = 0.0;
  uint32_t dof = request.use_joints.size();
  robot_model_->SetRobotTransform(request.origin_to_base);
  robot_model_->SetNamedAngle(request.initial_angle);

  Eigen::Affine3d origin_to_current(Eigen::Affine3d::Identity());
  Eigen::Affine3d origin_to_base(request.origin_to_base);
  Eigen::Affine3d ref_origin_to_frame(Eigen::Affine3d::Identity());
  Eigen::Affine3d ref_to_current(Eigen::Affine3d::Identity());
  Eigen::MatrixXd jacobian(kSE3Dim, dof);
  Eigen::VectorXd angle_diff(dof);
  Eigen::Matrix<double, kSE3Dim, 1> diff;
  Eigen::AngleAxisd diff_rot;
  Eigen::Vector3d diff_pos;
  JointState current_joint;
  Eigen::VectorXd angle_max(dof);
  Eigen::VectorXd angle_min(dof);

  std::vector<Eigen::Vector3d> linear_base_movements =
      request.linear_base_movements;
  std::vector<Eigen::Vector3d> rotational_base_movements =
      request.rotational_base_movements;

  uint32_t total_dof = dof +
      linear_base_movements.size() + rotational_base_movements.size();
  Eigen::MatrixXd linear_base_jacobian(
      kSE3Dim, linear_base_movements.size());
  Eigen::MatrixXd rotational_base_jacobian(
      kSE3Dim, rotational_base_movements.size());

  // 位置のjacobianを作成
  for (uint32_t i = 0; i < linear_base_movements.size(); ++i) {
    linear_base_jacobian.col(i) <<
        linear_base_movements[i], Eigen::Vector3d::Zero();
  }


  Eigen::MatrixXd jacobian_with_base(kSE3Dim, total_dof);

  // 関節数+base自由度が6以下なら例外送出
  if (total_dof < kSE3Dim) {
    throw std::invalid_argument(
        "use_joints's + base dof has to be at least 6.");
  }

  Eigen::MatrixXd wn = Eigen::MatrixXd::Identity(total_dof, total_dof);
  // weightのサイズがDOFと同じならwnに重みをつける
  if (request.weight.size() == static_cast<int32_t>(total_dof)) {
    for (uint32_t i = 0; i < dof; ++i) {
      // 重みが負なら例外送出
      if (request.weight(i) < 0.0) {
        throw std::invalid_argument(
            "ik joint weights have to be positive double.");
      } else {
        wn(i, i) = request.weight(i);
      }
    }
    for (uint32_t i = dof; i < dof + linear_base_movements.size(); ++i) {
      wn(i, i) = request.weight(i);
    }
    for (uint32_t i = dof + linear_base_movements.size();
         i < total_dof; ++i) {
      wn(i, i) = request.weight(i);
    }
  }

  ref_origin_to_frame = request.ref_origin_to_end
      * (request.frame_to_end).inverse();

  robot_model_->GetMinMax(request.use_joints, angle_min, angle_max);
  for (uint32_t i = 0; i < max_itr_; ++i) {
    origin_to_current = robot_model_->GetObjectTransform(request.frame_name);
    current_joint = robot_model_->GetNamedAngle(request.use_joints);
    Eigen::Quaterniond diff_quat(origin_to_current.linear().transpose()
                                 * ref_origin_to_frame.linear());
    diff_rot = Eigen::AngleAxisd(diff_quat.normalized());
    diff_pos = ref_origin_to_frame.translation()
        - origin_to_current.translation();
    diff << diff_pos,
        origin_to_current.linear() * diff_rot.angle() * diff_rot.axis();

    delta_old = delta;
    delta = diff.norm();
    if (delta < epsilon_) {
      solution_angle_out = current_joint;
      origin_to_end_out = origin_to_current * request.frame_to_end;
      origin_to_base_out = origin_to_base;
      return kSuccess;
    } else if (fabs(delta - delta_old) < converge_threshold_) {
      solution_angle_out = current_joint;
      origin_to_end_out = origin_to_current * request.frame_to_end;
      origin_to_base_out = origin_to_base;
      return kConverge;
    }
    if (!request.use_joints.empty()) {
      jacobian = robot_model_->GetJacobian(request.frame_name,
                                           request.frame_to_end,
                                           request.use_joints);
    }

    // 回転のjacobianを作成
    for (uint32_t i = 0; i < rotational_base_movements.size(); ++i) {
      rotational_base_jacobian.col(i) <<
          rotational_base_movements[i].cross(origin_to_current.translation() -
                                             origin_to_base.translation()),
          rotational_base_movements[i];
    }

    // ベース移動のヤコビアンを追加
    if (request.use_joints.empty()) {
      if (linear_base_movements.empty()) {
        if (rotational_base_movements.empty()) {
        } else {
          jacobian_with_base <<
              rotational_base_jacobian;
        }
      } else {
        if (rotational_base_movements.empty()) {
          jacobian_with_base <<
              linear_base_jacobian;
        } else {
          jacobian_with_base <<
              linear_base_jacobian, rotational_base_jacobian;
        }
      }
    } else {
      if (linear_base_movements.empty()) {
        if (rotational_base_movements.empty()) {
          jacobian_with_base = jacobian;
        } else {
          jacobian_with_base <<
              jacobian, rotational_base_jacobian;
        }
      } else {
        if (rotational_base_movements.empty()) {
          jacobian_with_base <<
              jacobian, linear_base_jacobian;
        } else {
          jacobian_with_base <<
              jacobian, linear_base_jacobian, rotational_base_jacobian;
        }
      }
    }
    // Newton-Rapson
    // angle_diff = jacobian_with_base.transpose() * (jacobian_with_base *
    //                                      jacobian_with_base.transpose()).inverse() * diff;
    // LM法[Sugihara 2009]
    angle_diff = (jacobian_with_base.transpose() * jacobian_with_base
                  + (diff.transpose() * diff)(0, 0)
                  * wn
                  + kWn * wn).inverse()
        * jacobian_with_base.transpose() * diff;
    current_joint.position += angle_diff.head(dof);
    // 関節角度限界を守る用に修正
    SaturateAngle(angle_min, angle_max, current_joint.position);
    // ベースの位置の修正分を計算
    Eigen::Vector3d mod_base_pos = Eigen::Vector3d::Zero();
    for (uint32_t i = 0; i < linear_base_movements.size(); ++i) {
      mod_base_pos += angle_diff(dof + i) * linear_base_movements[i];
    }
    // ベースの回転の修正分を計算
    for (uint32_t i = 0; i < rotational_base_movements.size(); ++i) {
      double angle = angle_diff(dof + linear_base_movements.size() + i);
      origin_to_base = origin_to_base *
          Eigen::AngleAxisd(
              angle,
              origin_to_base.rotation() * rotational_base_movements[i]);
    }
    origin_to_base = Eigen::Translation3d(mod_base_pos) * origin_to_base;

    robot_model_->SetRobotTransform(origin_to_base);
    robot_model_->SetNamedAngle(current_joint);
  }
  solution_angle_out = current_joint;
  origin_to_base_out = origin_to_base;
  origin_to_end_out = origin_to_current * request.frame_to_end;
  return kMaxItr;
}
}  // namespace tmc_robot_kinematics_model

/// @brief IKRequestを生成するctypes向けのwrapper関数
/// @param [in] movement ベースのタイプ
/// @return req_p IKRequestのポインタ
void* create_request(tmc_manipulation_types::BaseMovementType movement) {
  tmc_robot_kinematics_model::IKRequest* req =
    new tmc_robot_kinematics_model::IKRequest(movement);
  void* req_p = reinterpret_cast<void*>(req);
  return req_p;
}

/// @brief IKを解くフレームの名前をsetするctypes向けのwrapper関数
/// @param [in,out] req_p IKRequestのポインタ
/// @param [in] frame_name フレームの名前
void set_req_frame_name(void* req_p, char* frame_name) {
  struct tmc_robot_kinematics_model::IKRequest* req =
    (struct tmc_robot_kinematics_model::IKRequest*)req_p;
  req->frame_name = frame_name;
}

/// @brief IKを解くフレームからend coordinatesへの変換をsetするctypes向けのwrapper関数
/// @param [in,out] req_p IKRequestのポインタ
/// @param [in] mat フレームからend coordinatesへの変換。4x4行列
void set_req_frame_to_end(void* req_p, double* mat) {
  Eigen::Matrix4d  matrix;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      matrix(i, j) = mat[i * 4 + j];
    }
  }
  struct tmc_robot_kinematics_model::IKRequest* req =
    (struct tmc_robot_kinematics_model::IKRequest*)req_p;
  req->frame_to_end.matrix() = matrix;
}

/// @brief IKRequestに原点からベースへの変換をsetするctypes向けのwrapper関数
/// @param [in,out] req_p IKRequestのポインタ
/// @param [in] mat 原点からベースへの変換。4x4行列
void set_req_origin_to_base(void* req_p, double* mat) {
  Eigen::Matrix4d  matrix;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      matrix(i, j) = mat[i * 4 + j];
    }
  }
  struct tmc_robot_kinematics_model::IKRequest* req =
    (struct tmc_robot_kinematics_model::IKRequest*)req_p;
  req->origin_to_base.matrix() = matrix;
}

/// @brief IKRequestにIK開始時点の関節の名前setするctypes向けのwrapper関数
/// @param [in,out] req_p IKRequestのポインタ
/// @param [in] angle_names 関節の名前
/// @param [in] num_elements 設定する関節数
void set_req_initial_angle_name(void* req_p, char* angle_names[], int num_elements) {
  tmc_manipulation_types::NameSeq use_name;
  struct tmc_robot_kinematics_model::IKRequest* req =
    reinterpret_cast<struct tmc_robot_kinematics_model::IKRequest*>(req_p);
  req->initial_angle.name.resize(num_elements);
  for (int index = 0; index < num_elements; ++index) {
    char* angle_name = angle_names[index];
    use_name.push_back(angle_name);
  }
  req->initial_angle.name = use_name;
  req->use_joints = use_name;
}

/// @brief IKRequestにIK開始時点の関節角度列をsetするctypes向けのwrapper関数
/// @param [in,out] req_p IKRequestのポインタ
/// @param [in] pos 関節角度列
/// @param [in] len_pos 設定する関節数
void set_req_initial_angle_position(void* req_p, float pos[], int len_pos) {
  struct tmc_robot_kinematics_model::IKRequest* req =
    reinterpret_cast<struct tmc_robot_kinematics_model::IKRequest*>(req_p);
  req->initial_angle.position.resize(len_pos);

  for (int i = 0; i < len_pos; i++) {
    req->initial_angle.position[i] = pos[i];
  }
}

/// @brief IKRequestに重みをsetするctypes向けのwrapper関数
/// @param [in,out] req_p IKRequestのポインタ
/// @param [in] weight 重み配列
/// @param [in] len_weight 重み配列の長さ
void set_req_weight(void* req_p, float weight[], int len_weight) {
  struct tmc_robot_kinematics_model::IKRequest* req =
    reinterpret_cast<struct tmc_robot_kinematics_model::IKRequest*>(req_p);
  req->weight.resize(len_weight);
  for (int i = 0; i < len_weight; i++) {
    req->weight[i] = weight[i];
  }
}

/// @brief IKRequestに原点からend coordinatesの変換をsetするctypes向けのwrapper関数
/// @param [in,out] req_p IKRequestのポインタ
/// @param [in] mat 原点からend coordinatesの変換。4x4行列
void set_req_ref_origin_to_end(void* req_p, double* mat) {
  struct tmc_robot_kinematics_model::IKRequest* req =
    reinterpret_cast<struct tmc_robot_kinematics_model::IKRequest*>(req_p);
  Eigen::Matrix4d  matrix;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      matrix(i, j) = mat[i * 4 + j];
    }
  }
  req->ref_origin_to_end.matrix() = matrix;
}

/// @brief JointStateオブジェクトのポインタを返すctypes向けのwrapper関数
/// @return res JointStateのオブジェクトのポインタ
void* jointstate() {
  JointState* js = new JointState;

  void* res = reinterpret_cast<void*>(js);
  return res;
}

/// @brief Affine3dのオブジェクトのポインタを返すctypes向けのwrapper関数
/// @return Affine3dのオブジェクトのポインタ
void* affine3d() {
  Eigen::Affine3d* affine3d = new Eigen::Affine3d;
  return reinterpret_cast<void*>(affine3d);
}

/// @brief IK solverオブジェクトを作るctypes向けのwrapper関数
/// @param [in] robot_p IK用ロボットモデル(tarp3_wrapper)のポインタ
/// @param [in] max_itr 繰り返し計算の最大回数
/// @param [in] epsilon 許容誤差
/// @param [in] converge_threshold 一回の繰り返しでこれより変化が小さければ収束とみなす
/// @return IK solverのオブジェクトのポインタ
void* create_solver(void* robot_p, int max_itr,
                    float epsilon, float converge_threshold) {
  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr* robot =
    reinterpret_cast<
    tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr*>(robot_p);
  tmc_robot_kinematics_model::IKSolver::Ptr* numeric_solver
    = new tmc_robot_kinematics_model::IKSolver::Ptr;
  numeric_solver->reset(
    new tmc_robot_kinematics_model::NumericIKSolver(
        tmc_robot_kinematics_model::IKSolver::Ptr(),
        *robot,
        max_itr,
        epsilon,
        converge_threshold));
  return reinterpret_cast<void*>(numeric_solver);
}

/// @brief IKを解くctypes向けのwrapper関数
/// @param [in] solver_p solverのポインタ
/// @param [out] solution_p IKを解いた結果の関節角度列のポインタ
/// @param [out] origin_to_base_solution_p IKを解いた結果の、原点からベースへの変換のポインタ
/// @param [out] origin_to_hand_result_p IKを解いた結果の、原点から手先への変換のポインタ
/// @param [out] req_p リクエストのポインタ
void solve(void* solver_p, void* solution_p,
           void* origin_to_base_solution_p,
           void* origin_to_hand_result_p,
           void* req_p) {
  tmc_robot_kinematics_model::IKSolver::Ptr* numeric_solver =
    reinterpret_cast<tmc_robot_kinematics_model::IKSolver::Ptr*>(solver_p);
  JointState* solution = reinterpret_cast<JointState*>(solution_p);
  Eigen::Affine3d* origin_to_hand_result =
    reinterpret_cast<Eigen::Affine3d*>(origin_to_hand_result_p);
  Eigen::Affine3d* origin_to_base_solution =
    reinterpret_cast<Eigen::Affine3d*>(origin_to_base_solution_p);
  struct tmc_robot_kinematics_model::IKRequest* req =
    reinterpret_cast<struct tmc_robot_kinematics_model::IKRequest*>(req_p);
  tmc_robot_kinematics_model::IKResult result;
  // Solve.
  result = (*numeric_solver)->Solve(*req,
                                    *solution,
                                    *origin_to_base_solution,
                                    *origin_to_hand_result);
  // Print result.
  if (result == tmc_robot_kinematics_model::kSuccess) {
  } else {
    std::cout << "ik cannot be solved" << std::endl;
  }
}

/// @brief IKを解いた結果の、関節角度列を返すctypes向けのwrapper関数
/// @param [in] sol_p 関節角度列(JointState)へのポインタ
/// @param [out] result 関節角度列
/// @param [in] len 利用する関節数
void get_joint_angle(void* sol_p,
                     float* result,
                     int len) {
  JointState* solution = reinterpret_cast<JointState*>(sol_p);
  for (int i=0; i < len; i++) {
    result[i] = solution->position[i];
  }
}

/// @brief IKを解いた結果の、原点からベースへの変換を返すctypes向けのwrapper関数
/// @param [in] origin_to_base 原点からベースへの変換(Affine3d)へのポインタ
/// @param [out] result 原点からベースへの変換(4x4行列(serialized))
void get_origin_to_base(void* origin_to_base,
                        double* result) {
  Eigen::Affine3d* otb = reinterpret_cast<Eigen::Affine3d*>(origin_to_base);
  Eigen::MatrixXd  mat = otb->matrix();
  for (int i=0; i < 4; i++) {
    for (int j=0; j < 4; j++) {
      result[4*i + j] = mat(i, j);
    }
  }
}
