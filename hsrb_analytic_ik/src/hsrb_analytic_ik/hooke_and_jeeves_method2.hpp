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
#ifndef HSRB_ANALYTIC_IK_HOOKE_AND_JEEVES_METHOD_2_HPP_
#define HSRB_ANALYTIC_IK_HOOKE_AND_JEEVES_METHOD_2_HPP_

#include "common.hpp"
#include "function_adapters.hpp"

namespace opt {

/**
 * Hooke-and-Jeeves 法による最適化を行うクラスです。
 * 勾配を使わない最適化法です。
 */
class HookeAndJeevesMethod2 {
 public:
  /**
   * コンストラクタ
   */
  explicit HookeAndJeevesMethod2(int max_iteration = 100,
                                 double epsilon = 1e-4)
      : max_iteration_(max_iteration), epsilon_(epsilon) {
  }

  /**
   * 探索を行います。
   *
   * @param	func	目的関数
   * @param	x0		初期値
   * @param	step	ステップ幅 (直線探索に使用します)
   * @param	maxItor	最大反復
   * @param	epsilon	収束条件 (反復によって移動する距離がこの距離以下になると収束したとみなします)
   */
  template<class Function2, class BiLineSearch>
  OptResult Search(Function2& func, BiLineSearch& lineSearch, const Vector2& x0,
                   double step) {
    // 探索結果を初期化します。
    result_ = OptFail;
    iteration_ = 0;
    solution_.Zero();

    // 点列を初期化します。
    Vector2 x = x0;

    // 前回の位置 z_prev を初期化します。
    Vector2 z_prev = x0;

    // 反復 k を繰り返します。
    for (int k = 1; k <= max_iteration_; k++) {
      // 点 x から (1,0) 方向に直線探索を行なって、点 y を決定します。
      Vector2 y;
      {
        DirectionAdapterFunction2<Function2> func_x1(func, x, Vector2(1, 0));
        OptResult result_x1 = lineSearch.Search(func_x1, step);
        if (result_x1 == OptFail) {
          result_ = OptFail;
          return result_;
        }
        double solution_x1 = lineSearch.solution();
        y.v1 = x.v1 + solution_x1;
        y.v2 = x.v2;
      }

      // 点 y から (0,1) 方向に直線探索を行って、点 z を決定します。
      Vector2 z;
      {
        DirectionAdapterFunction2<Function2> func_x2(func, y, Vector2(0, 1));
        OptResult result_x2 = lineSearch.Search(func_x2, step);
        if (result_x2 == OptFail) {
          result_ = OptFail;
          return result_;
        }
        double solution_x2 = lineSearch.solution();
        z.v1 = y.v1;
        z.v2 = y.v2 + solution_x2;
      }

      // Hooke-and-Jeeves 法により、パターン探索を行います。
      // x1, x2 方向探索前の x に基いた方向 d=z-x への探索ではなく、
      // 前回の位置 z_prev に基いた方向 d=z-z_prev への探索を行います。
      Vector2 d = z - z_prev;

      // 点 z と点 z_prev が十分近ければ、反復を終了します。
      if (d.Norm() <= epsilon_) {
        result_ = OptSuccess;
        iteration_ = k;
        solution_ = (func.Value(z) <= func.Value(z_prev)) ? z : z_prev;
        return result_;
      }

      // 点 z から方向 d に直線探索を行って、点 x_next を決定します。
      Vector2 x_next;
      {
        d.Normalize();
        DirectionAdapterFunction2<Function2> func_d(func, z, d);
        OptResult result_d = lineSearch.Search(func_d, step);
        if (result_d == OptFail) {
          result_ = OptFail;
          return result_;
        }
        double solution_d = lineSearch.solution();
        x_next = z + solution_d * d;
      }

      // 点 x と点 x_next の距離を計算します。
      double dx = Vector2::Norm(x, x_next);

      // 点 x と点 x_next が十分近ければ、反復を終了します。
      if (dx <= epsilon_) {
        result_ = OptSuccess;
        iteration_ = k;
        solution_ = (func.Value(x_next) <= func.Value(x)) ? x_next : x;
        return result_;
      }

      // 前回の点 z を記録します。
      z_prev = z;

      // 注目点 x を更新します。
      x = x_next;
    }

    // 最大反復回数に達したので、OptMaxItor を返します。
    result_ = OptMaxItor;
    iteration_ = max_iteration_;
    solution_ = x;
    return result_;
  }

  /**
   * 最大反復数を設定します。
   */
  void set_max_iteration(int max_iteration) {
    max_iteration_ = max_iteration;
  }

  /**
   * 収束条件を設定します。
   */
  void set_epsilon(double epsilon) {
    epsilon_ = epsilon;
  }

  /**
   * 探索結果を取得します。
   */
  OptResult result() const {
    return result_;
  }

  /**
   * 探索にかかった反復数を取得します。
   */
  int iteration() const {
    return iteration_;
  }

  /**
   * 探索後の解を取得します。
   */
  Vector2 solution() const {
    return solution_;
  }

 private:
  OPT_CLASS_UNCOPYABLE(HookeAndJeevesMethod2)

  /**
   * 最大反復数
   */
  int max_iteration_;

  /**
   * 収束条件
   * (反復によって移動する距離がこの距離以下になると収束したとみなします)
   */
  double epsilon_;

  /**
   * 探索結果を保持します。
   */
  OptResult result_;

  /**
   * 探索にかかった反復数を保持します。
   */
  int iteration_;

  /**
   * 探索後の解を保持します。
   */
  Vector2 solution_;
};

}  // namespace opt
#endif  // HSRB_ANALYTIC_IK_HOOKE_AND_JEEVES_METHOD_2_HPP_
