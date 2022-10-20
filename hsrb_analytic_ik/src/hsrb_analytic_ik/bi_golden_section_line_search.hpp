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
#ifndef HSRB_ANALYTIC_IK_BI_GOLDEN_SECTION_LINE_SEARCH_HPP_
#define HSRB_ANALYTIC_IK_BI_GOLDEN_SECTION_LINE_SEARCH_HPP_

#include "common.hpp"
#include "function_adapters.hpp"
#include "uni_golden_section_line_search.hpp"

namespace opt {

/**
 * 双方向の黄金分割直線探索法を行うクラスです。
 */
class BiGoldenSectionLineSearch {
 public:
  /**
   * コンストラクタ
   */
  explicit BiGoldenSectionLineSearch(int max_iteration = 100, double epsilon = 1e-4) {
    max_iteration_ = max_iteration;
    epsilon_ = epsilon;
    result_ = OptFail;
    iteration_ = 0;
    solution_ = 0;
    value_ = 0;
  }

  /**
   * 区間 (-∞,+∞) の範囲で探索を行います。
   *
   * @param	f		一変数関数
   * @param	step	探索区間 [-step, step] の初期値
   * @return
   */
  template<class Function1>
  OptResult Search(Function1& f, double step) {
    // 正方向に単方向探索を行います。
    {
      UniGoldenSectionLineSearch search(max_iteration_, epsilon_);
      OptResult result = search.Search(f, step);

      // 探索に失敗した場合
      if (result == OptFail) {
        result_ = OptFail;
        // 負方向への探索を継続します。
      } else {
        // 探索に成功、または、最大反復回数に達した場合
        // 解を記録して、負方向の探索結果と比較します。
        result_ = result;
        iteration_ = search.iteration();
        solution_ = search.solution();
        value_ = search.value();
      }
    }

    // 負方向に単方向探索を行います。
    {
      // 反転関数を作成します。
      ReverseAdapterFunction1<Function1> revserseFunc(f);

      UniGoldenSectionLineSearch search(max_iteration_, epsilon_);
      OptResult result = search.Search(revserseFunc, step);

      // 探索に失敗した場合は、正方向の結果を返します。
      if (result == OptFail) {
        return result_;
      } else {
        // 探索に成功、または、最大反復回数に達した場合
        // 正方向で失敗している、または、正方向で成功しているが、
        // 負方向のほうが良い結果となっている場合、それを最終解とします。
        if ((result_ == OptFail) || (search.value() < value_)) {
          result_ = result;
          iteration_ = search.iteration();
          solution_ = -search.solution();  // マイナスをかけるので注意
          value_ = search.value();
        }
        return result_;
      }
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
  OPT_CLASS_UNCOPYABLE(BiGoldenSectionLineSearch)

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
#endif  // HSRB_ANALYTIC_IK_BI_GOLDEN_SECTION_LINE_SEARCH_HPP_
