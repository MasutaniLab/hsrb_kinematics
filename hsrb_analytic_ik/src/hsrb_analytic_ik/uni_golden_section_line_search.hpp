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
#ifndef HSRB_ANALYTIC_IK_UNI_GOLDEN_SECTION_LINE_SEARCH_HPP_
#define HSRB_ANALYTIC_IK_UNI_GOLDEN_SECTION_LINE_SEARCH_HPP_

#include "common.hpp"
#include "golden_section_line_search.hpp"

namespace opt {

/**
 * 単方向の黄金分割直線探索法を行うクラスです。
 */
class UniGoldenSectionLineSearch {
 public:
  /**
   * コンストラクタ
   */
  explicit UniGoldenSectionLineSearch(int max_iteration = 100, double epsilon = 1e-4)
      : max_iteration_(max_iteration),
        epsilon_(epsilon),
        result_(OptFail),
        iteration_(0),
        solution_(0),
        value_(0) {}

  /**
   * 区間 (0,+∞) の範囲で探索を行います。
   *
   * @param	f		一変数関数
   * @param	step	探索区間 [0, step] の初期値
   * @return
   */
  template<class Function1>
  OptResult Search(Function1& f, double step) {
    // 探索結果を初期化します。
    result_ = OptFail;
    iteration_ = 0;
    solution_ = 0;
    value_ = 0;

    // 黄金分割直線探索を作成します。
    GoldenSectionLineSearch search(max_iteration_, epsilon_);

    // 探索区間を初期化します。
    const double a = 0;
    double b = step;

    // 最初の１回めの探索を行います。
    OptResult result = search.Search(f, a, b);

    // 探索成功なら、その解を記録しておきます。
    iteration_ = search.iteration();
    solution_ = search.solution();
    value_ = search.value();
    if (result == OptSuccess) {
      result_ = OptSuccess;
      // 引き続き、区間を拡大して、解を探索します。
    } else if (result == OptMaxItor) {
      result_ = OptMaxItor;
      return result_;
    }

    // もし、最初の黄金分割探索で、非準凸が検出された場合は、
    // ステップ幅を縮小して、区間内で準凸となるようにします。
    if (result == OptFail) {
      // 反復 k を繰り返します。
      for (int k = 2; k <= max_iteration_; k++) {
        // 区間を縮小します。
        b *= 0.5;

        // 探索を行います。
        OptResult result = search.Search(f, a, b);

        if (result == OptSuccess) {
          result_ = OptSuccess;
          iteration_ = search.iteration();
          solution_ = search.solution();
          value_ = search.value();
          return result_;
        }
      }

      result_ = OptMaxItor;
      return result_;
    } else {
      // 反復 k を繰り返します。
      for (int k = 2; k <= max_iteration_; k++) {
        // 区間を拡大します。
        b *= 2;

        // 探索を行います。
        OptResult result = search.Search(f, a, b);

        if (result == OptSuccess) {
          double comparativeSolution = search.solution();
          double comparativeValue = search.value();

          // 前回の解と今回の解が十分近ければ、その両者のうち、
          // 目的関数値の小さいほうを解とします。
          // これを行わずに、目的関数値の大小のみで判断すると、
          // 微小な計算誤差のために無駄な反復を行うことになります。
          if (std::abs(solution_ - comparativeSolution) <= epsilon_) {
            if (comparativeValue < value_) {
              iteration_ = search.iteration();
              solution_ = comparativeSolution;
              value_ = comparativeValue;
            }
            return result_;
          } else if (comparativeValue < value_) {
            // 今回の解と前回の解が、離れているときは、
            // 目的関数値が小さくなっていれば、さらに探索を続けます。
            iteration_ = search.iteration();
            solution_ = comparativeSolution;
            value_ = comparativeValue;
            continue;
          } else {
            // 前回成功した解を返します。
            return result_;
          }
        } else {
          // OptMaxItor または OptFail の場合
          // 前回成功した解を返します。
          return result_;
        }
      }

      // 最低 1 回は探索に成功しているので、それを解とします。
      return result_;
    }
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
  OPT_CLASS_UNCOPYABLE(UniGoldenSectionLineSearch)

  /**
   * 最大反復数
   */
  int max_iteration_;

  /**
   * 収束条件
   * (不確定区間の幅がこの値以下になれば、収束したものとします)
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
   * 解の目的関数値を保持します。
   */
  double value_;
};

}  // namespace opt
#endif  // HSRB_ANALYTIC_IK_UNI_GOLDEN_SECTION_LINE_SEARCH_HPP_
