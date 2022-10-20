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
#ifndef HSRB_ANALYTIC_IK_GOLDEN_SECTION_LINE_SEARCH_HPP_
#define HSRB_ANALYTIC_IK_GOLDEN_SECTION_LINE_SEARCH_HPP_

#include "common.hpp"

namespace opt {

/**
 * 黄金分割直線探索法を行うクラスです。
 */
class GoldenSectionLineSearch {
 public:
  /**
   * コンストラクタ
   */
  explicit GoldenSectionLineSearch(int max_iteration = 100, double epsilon = 1e-4)
      : max_iteration_(max_iteration),
        epsilon_(epsilon),
        result_(OptFail),
        iteration_(0),
        solution_(0),
        value_(0) {}

  /**
   * 探索を行います。
   *
   * 一変数関数 f が探索区間 [a, b] において狭義凸であれば、
   * 探索により極小値に収束します。
   *
   * 探索中に狭義凸でないことが検出された場合は、探索失敗となり、OptFail を返します。
   * ただし、必ずしも検出されるわけでなく、検出されない場合は局所解を求めます。
   *
   * @param	f		一変数関数
   * @param	a		探索区間 [a, b] の下限値
   * @param	b		探索区間 [a, b] の上限値
   * @param	maxItor	最大反復回数
   * @param	epsilon	収束条件です。不確定区間の幅がこの値以下になれば、収束したものとします。
   * @return
   */
  template<class Function1>
  OptResult Search(Function1& f, double a, double b) {
    // 探索結果を初期化します。
    result_ = OptFail;
    iteration_ = 0;
    solution_ = 0;
    value_ = 0;

    // 不確定区間を初期化します。
    double a_k = a;
    double b_k = b;

    // 不確定区間の端点 a_k, b_k における関数値 f_a_k, f_b_k を計算します。
    double f_a_k = f.Value(a_k);
    double f_b_k = f.Value(b_k);

    // 黄金比を設定します。
    const double _alpha = 0.61803398874989484820458683436563811772;

    // 不確定区間 [a_k, b_k] 内の評価点 s_k, t_k を選びます。
    double s_k = a_k + (1 - _alpha) * (b_k - a_k);
    double t_k = a_k + _alpha * (b_k - a_k);

    // 評価点 s_k, t_k における関数値 f_s_k, f_t_k を計算します。
    double f_s_k = f.Value(s_k);
    double f_t_k = f.Value(t_k);

    // 反復 k を繰り返します。
    for (int k = 1; k <= max_iteration_; k++) {
      // 次の不確定区間として [a_k, t_k] を選択するときは true、
      // [s_k, b_k] を選択するときは false のフラグです。
      bool left = (f_a_k <= f_s_k && f_a_k <= f_t_k && f_a_k <= f_b_k)
               || (f_s_k <= f_a_k && f_s_k <= f_t_k && f_s_k <= f_b_k);

      // 次の不確定区間を決定します。
      if (left) {
        // a_k はそのまま
        b_k = t_k;
        f_b_k = f_t_k;
        t_k = s_k;
        f_t_k = f_s_k;
        s_k = a_k + (1 - _alpha) * (b_k - a_k);
        f_s_k = f.Value(s_k);
      } else {
        // b_k はそのまま
        a_k = s_k;
        f_a_k = f_s_k;
        s_k = t_k;
        f_s_k = f_t_k;
        t_k = a_k + _alpha * (b_k - a_k);
        f_t_k = f.Value(t_k);
      }

      // 収束条件を判定します。
      if (b_k - a_k <= epsilon_) {
        result_ = OptSuccess;
        iteration_ = k;
        if (f_a_k <= f_b_k) {
          solution_ = a_k;
          value_ = f_a_k;
        } else {
          solution_ = b_k;
          value_ = f_b_k;
        }
        return result_;
      }
    }

    // 最大反復回数に達したので、OptMaxItor を返します。
    result_ = OptMaxItor;
    iteration_ = max_iteration_;
    if (f_a_k <= f_b_k) {
      solution_ = a_k;
      value_ = f_a_k;
    } else {
      solution_ = b_k;
      value_ = f_b_k;
    }
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
  double solution() const {
    return solution_;
  }

  /**
   * 探索後の目的関数値を取得します。
   */
  double value() const {
    return value_;
  }

 private:
  OPT_CLASS_UNCOPYABLE(GoldenSectionLineSearch)

  /**
   * 最大反復数
   */
  int max_iteration_;

  /**
   * 収束条件
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
  double solution_;

  /**
   * 探索後の目的関数値を保持します。
   */
  double value_;
};

}  // namespace opt
#endif  // HSRB_ANALYTIC_IK_GOLDEN_SECTION_LINE_SEARCH_HPP_
