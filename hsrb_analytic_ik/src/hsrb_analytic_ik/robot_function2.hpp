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
#ifndef HSRB_ANALYTIC_IK_ROBOT_FUNCTION_2_HPP_
#define HSRB_ANALYTIC_IK_ROBOT_FUNCTION_2_HPP_

#include <algorithm>
#include <utility>

#include "common.hpp"
#include "vector2.hpp"

namespace opt {

/**
 * IK最適化のための目的関数の入力
 * 現在の関節角度等
 */
struct RobotFunction2Request {
  /**
   * 同時変換行列の参照値 (T_ref)
   */
  double R11, R12, R13, px;
  double R21, R22, R23, py;
  double R31, R32, R33, pz;

  /**
   * 重み行列 (対角行列) の対角成分 w_i (i=0..7)
   */
  double w0, w1, w2, w3, w4, w5, w6, w7;

  /**
   * パラメータの参照値 θ^ref_i (i=0..7)
   */
  double r0, r1, r2, r3, r4, r5, r6, r7;
};

/**
 * IK最適化のための目的関数の出力
 * IKの結果等
 */
struct RobotFunction2Response {
  /**
   * パラメータ θ_i (i=0..7)
   */
  double t0, t1, t2, t3, t4, t5, t6, t7;
};

/**
 * IK最適化のための目的関数のパラメータ
 * 関節角度の上下限等
 */
struct RobotFunction2Parameter {
  /**
   * 機構の設定値
   */
  double L3;
  double L41;
  double L42;
  double L51;
  double L52;
  double L81;
  double L82;

  /**
   * パラメータの可動範囲
   */
  double t3_min, t3_max;
  double t4_min, t4_max;
  double t5_min, t5_max;
  double t6_min, t6_max;
  double t7_min, t7_max;

  RobotFunction2Parameter()
      : L3(0.340), L41(0.141), L42(0.078),
        L51(0.005), L52(0.345), L81(0.012), L82(0.1405),
        t3_min(0.00), t3_max(0.69), t4_min(-2.62), t4_max(0.00),
        t5_min(-1.92), t5_max(3.67), t6_min(-1.92), t6_max(1.22),
        t7_min(-1.92), t7_max(3.67) {}
};


/**
 * IK最適化のための目的関数です。
 *
 * x = (θ_2, θ_4) とするとき、
 * f(x) = || W * ( θ^ref(x) - θ(x) ) ||^2
 * です。
 */
class RobotFunction2 {
 public:
  /**
   * 付加するペナルティのタイプです。
   */
  enum PenaltyType {
    /**
     * ペナルティなしです。
     */
    PenaltyNone = 0,

    /**
     * 実現可能領域外に対して、巨大な値になりますが、
     * 領域からはみ出るほど大きな値になるようにします。
     */
    PenaltyBigDiscontinuous = 1,

    /**
     * 実現可能領域外に対して、
     * 領域からはみ出るほど大きな値になりますが、
     * 連続性は保持されます。
     */
    PenaltyBigProportional = 2,
  };

  /**
   * コンストラクタ
   */
  RobotFunction2(const RobotFunction2Request& request,
                 const RobotFunction2Parameter& parameter)
      : request_(request),
        parameter_(parameter),
        penalty_type_(PenaltyBigProportional), penalty_coeff_(1000) {}

  RobotFunction2Response response() const {
    return response_;
  }

  void set_penalty_coeff(double penalty_coeff) {
    penalty_coeff_ = penalty_coeff;
  }

  void set_penalty_type(PenaltyType penalty_type) {
    penalty_type_ = penalty_type;
  }

  bool t6_use_plus() const {
    return t6_use_plus_;
  }

  /**
   * 実現可能点であるか判定します。
   */
  bool IsFeasible(const Vector2& x) {
    CalculateTheta_(x);
    return IsFeasibleFromMembers();
  }

  /**
   * 現在のメンバ変数に基づいて、実現可能点であるか判定します。
   */
  bool IsFeasibleFromMembers() {
    return ((parameter_.t3_min <= response_.t3 && response_.t3 <= parameter_.t3_max) &&
            (parameter_.t4_min <= response_.t4 && response_.t4 <= parameter_.t4_max) &&
            (parameter_.t5_min <= response_.t5 && response_.t5 <= parameter_.t5_max) &&
            (parameter_.t6_min <= response_.t6 && response_.t6 <= parameter_.t6_max) &&
            (parameter_.t7_min <= response_.t7 && response_.t7 <= parameter_.t7_max));
  }

  /**
   * 実現可能領域内に強制的に引き戻します。
   */
  void ForceFeasible() {
    if (response_.t4 < parameter_.t4_min) {
      CalculateTheta_(Vector2(response_.t2, parameter_.t4_min));
    } else if (response_.t4 > parameter_.t4_max) {
      CalculateTheta_(Vector2(response_.t2, parameter_.t4_max));
    }

    if (response_.t3 < parameter_.t3_min)
      response_.t3 = parameter_.t3_min;
    else if (response_.t3 > parameter_.t3_max)
      response_.t3 = parameter_.t3_max;

    if (response_.t5 < parameter_.t5_min)
      response_.t5 = parameter_.t5_min;
    else if (response_.t5 > parameter_.t5_max)
      response_.t5 = parameter_.t5_max;

    if (response_.t6 < parameter_.t6_min)
      response_.t6 = parameter_.t6_min;
    else if (response_.t6 > parameter_.t6_max)
      response_.t6 = parameter_.t6_max;

    if (response_.t7 < parameter_.t7_min)
      response_.t7 = parameter_.t7_min;
    else if (response_.t7 > parameter_.t7_max)
      response_.t7 = parameter_.t7_max;
  }

  /**
   * 関数値を取得します。
   */
  double Value(const Vector2& x) {
    // θ_2, θ_4 から他の θ_i を計算します。
    CalculateTheta_(x);

    return ValueFromMembers();
  }

  /**
   * 現在設定されている t_i (i=0,..,7) から関数値を取得します。
   */
  double ValueFromMembers() {
    if (penalty_type_ == PenaltyBigDiscontinuous) {
      // 実現可能領域内にある場合
      if (IsFeasibleFromMembers()) {
        // 関数値を計算します。
        return ValueWithoutPenaltyFromMembers();
      } else {
        // 実現可能領域外の場合は、はみ出し具合に応じて、不連続なバリアを張ります。
        // ペナルティの大きさは、大きくとりすぎると、double の精度レンジに収まらなくなり、
        // 最適化が適切に作用しなくなるので、注意してください。
        return penalty_coeff_ + GetPenaltyGrade();
      }
    } else if (penalty_type_ == PenaltyBigProportional) {
      // 実現可能領域外の場合は、はみ出し具合に応じて、連続なバリアを張ります。
      return ValueWithoutPenaltyFromMembers() + penalty_coeff_ * GetPenaltyGrade();
    } else {
      // 関数値を計算します。
      return ValueWithoutPenaltyFromMembers();
    }
  }

  /**
   * 実行可能領域からの外れ具合を、重み付きで計算します。
   * この値はペナルティの計算に使用します。
   */
  double GetPenaltyGrade() {
    return request_.w3 * Plus_(parameter_.t3_min - response_.t3) +
           request_.w3 * Plus_(response_.t3 - parameter_.t3_max) +
           request_.w4 * Plus_(parameter_.t4_min - response_.t4) +
           request_.w4 * Plus_(response_.t4 - parameter_.t4_max) +
           request_.w5 * Plus_(parameter_.t5_min - response_.t5) +
           request_.w5 * Plus_(response_.t5 - parameter_.t5_max) +
           request_.w6 * Plus_(parameter_.t6_min - response_.t6) +
           request_.w6 * Plus_(response_.t6 - parameter_.t6_max) +
           request_.w7 * Plus_(parameter_.t7_min - response_.t7) +
           request_.w7 * Plus_(response_.t7 - parameter_.t7_max);
  }

  /**
   * 実行可能領域からの外れ具合を、重みなしで計算します。
   * この値は、実行可能領域が一直線になり、
   * 計算誤差から外点に収束してしまう場合に、
   * 内点に強制引き戻しを行ってよいかどうかの判定に使用できます。
   */
  double GetOuterGrade() {
    return Plus_(parameter_.t3_min - response_.t3) +
           Plus_(response_.t3 - parameter_.t3_max) +
           Plus_(parameter_.t4_min - response_.t4) +
           Plus_(response_.t4 - parameter_.t4_max) +
           Plus_(parameter_.t5_min - response_.t5) +
           Plus_(response_.t5 - parameter_.t5_max) +
           Plus_(parameter_.t6_min - response_.t6) +
           Plus_(response_.t6 - parameter_.t6_max) +
           Plus_(parameter_.t7_min - response_.t7) +
           Plus_(response_.t7 - parameter_.t7_max);
  }

  /**
   * 現在のメンバ変数に基づいて、ペナルティなしの目的関数値を計算します。
   */
  double ValueWithoutPenaltyFromMembers() {
    double V = 0;
    double a;
    a = request_.w0 * (request_.r0 - response_.t0);
    V += a * a;
    a = request_.w1 * (request_.r1 - response_.t1);
    V += a * a;
    a = request_.w2 * (request_.r2 - response_.t2);
    V += a * a;
    a = request_.w3 * (request_.r3 - response_.t3);
    V += a * a;
    a = request_.w4 * (request_.r4 - response_.t4);
    V += a * a;
    a = request_.w5 * (request_.r5 - response_.t5);
    V += a * a;
    a = request_.w6 * (request_.r6 - response_.t6);
    V += a * a;
    a = request_.w7 * (request_.r7 - response_.t7);
    V += a * a;
    return V;
  }

  /**
   * 勾配値を取得します。
   */
  Vector2 Gradient(const Vector2& x) {
    response_.t2 = x.v1;  // θ_2 を受け取ります。
    response_.t4 = x.v2;  // θ_4 を受け取ります。

    // θ_2, θ_4 から他の θ_i を計算します。
    CalculateTheta_(x);

    const double S2 = sin(response_.t2);
    const double C2 = cos(response_.t2);
    const double S4 = sin(response_.t4);
    const double C4 = cos(response_.t4);

    // t0 の偏微分値を計算します。
    double d0_2 = - parameter_.L52 * S2 * S4 + parameter_.L51 * S2 * C4
                  + parameter_.L42 * C2 + parameter_.L41 * S2;
    double d0_4 = parameter_.L52 * C2 * C4 + parameter_.L51 * C2 * S4;

    // t1 の偏微分値を計算します。
    double d1_2 = parameter_.L52 * C2 * S4 - parameter_.L51 * C2 * C4
                + parameter_.L42 * S2 - parameter_.L41 * C2;
    double d1_4 = parameter_.L52 * S2 * C4 + parameter_.L51 * S2 * S4;

    // t2 の偏微分値を計算します。
    double d2_2 = 1;
    double d2_4 = 0;

    // t3 の偏微分値を計算します。
    double d3_2 = 0;
    double d3_4 = parameter_.L52 * S4 - parameter_.L51 * C4;

    // t4 の偏微分値を計算します。
    double d4_2 = 0;
    double d4_4 = 1;

    // t6 の偏微分値を計算します。
    double d6_2;
    double d6_4;
    {
      double a = request_.R23 * S2 * S4 + request_.R13 * C2 * S4 - request_.R33 * C4;
      a *= a;
      double b = request_.R13 * S2 * S4 - request_.R23 * C2 * S4;
      double c = -(request_.R33 * S4 + request_.R23 * S2 * C4 + request_.R13 * C2 * C4);

      double d = 1 / std::sqrt(1 - a);
      d6_2 = b * d;
      d6_4 = c * d;

      // t6 を計算したときの±を反転させて符号を選びます。
      if (t6_use_plus_) {
        d6_2 = -d6_2;
        d6_4 = -d6_4;
      }
    }

    // t5 の偏微分値を計算します。
    double d5_2;
    double d5_4;
    {
      double a1 = (request_.R23 * S2 + request_.R13 * C2);
      double P1 = a1 * request_.R33 * S4
                + (request_.R23 * request_.R23 + request_.R13 * request_.R13) * C4;
      double Q1 = 2 * request_.R13 * request_.R23 * C2 * S2
                + (request_.R13 * request_.R13 - request_.R23 * request_.R23) * C2 * C2
                - request_.R33 * request_.R33 + request_.R23 * request_.R23;
      double R1 = -2 * a1 * request_.R33 * C4 * S4
                - (request_.R23 * request_.R23 + request_.R13 * request_.R13);
      d5_2 = P1 / (Q1 * S4 * S4 + R1);

      double b1 = request_.R13 * S2 - request_.R23 * C2;
      double P2 = a1 * b1 * S4
                + (request_.R23 * request_.R33 * C2 - request_.R13 * request_.R33 * S2) * C4;
      double Q2 = request_.R33 * request_.R33 * S4 * S4
                + 2 * a1 * request_.R33 * C4 * S4;
      double R2 = a1 * a1 * C4 * C4 + b1 * b1;
      d5_4 = -P2 / (Q2 + R2);
    }

    // t7 の偏微分値を計算します。
    double d7_2 = 0;
    double d7_4 = 0;
    {
      double a2 = request_.R12 * request_.R31 - request_.R11 * request_.R32;
      double b2 = request_.R21 * request_.R32 - request_.R22 * request_.R31;
      double c2 = 2 * (request_.R12 * request_.R22 + request_.R11 * request_.R21) * C2 * S2;

      double Z = -2 * ((request_.R22 * request_.R32 + request_.R21 * request_.R31) * S2
               + (request_.R12 * request_.R32 + request_.R11 * request_.R31) * C2) * C4 * S4;

      double P3 = (request_.R11 * request_.R22 - request_.R12 * request_.R21) * S4 * S4
                + (a2 * S2 + b2 * C2) * C4 * S4;
      double Q3 = (c2 + (- (request_.R22 * request_.R22 + request_.R21 * request_.R21)
                         + (request_.R12 * request_.R12 + request_.R11 * request_.R11)) * C2 * C2
                   + request_.R22 * request_.R22 + request_.R21 * request_.R21) * S4 * S4;
      double U3 = Z + (request_.R32 * request_.R32 + request_.R31 * request_.R31) * C4 * C4;

      d7_2 = -P3 / (Q3 + U3);

      double P4 = b2 * S2 - a2 * C2;
      double Q4 = ((request_.R22 * request_.R22 + request_.R21 * request_.R21) * S2 * S2 + c2
                   + (request_.R12 * request_.R12 + request_.R11 * request_.R11) * C2 * C2
                   - (request_.R32 * request_.R32 + request_.R31 * request_.R31)) * S4 * S4;
      double U4 = Z + request_.R32 * request_.R32 + request_.R31 * request_.R31;

      d7_4 = -P4 / (Q4 + U4);
    }

    // 勾配値を計算します。
    double a0 = request_.w0 * request_.w0 * (request_.r0 - response_.t0);
    double a1 = request_.w1 * request_.w1 * (request_.r1 - response_.t1);
    double a2 = request_.w2 * request_.w2 * (request_.r2 - response_.t2);
    double a3 = request_.w3 * request_.w3 * (request_.r3 - response_.t3);
    double a4 = request_.w4 * request_.w4 * (request_.r4 - response_.t4);
    double a5 = request_.w5 * request_.w5 * (request_.r5 - response_.t5);
    double a6 = request_.w6 * request_.w6 * (request_.r6 - response_.t6);
    double a7 = request_.w7 * request_.w7 * (request_.r7 - response_.t7);

    Vector2 g;
    g.v1 = d0_2 * a0 + d1_2 * a1 + d2_2 * a2 + d3_2 * a3 + d4_2 * a4 + d5_2 * a5
    + d6_2 * a6 + d7_2 * a7;
    g.v2 = d0_4 * a0 + d1_4 * a1 + d2_4 * a2 + d3_4 * a3 + d4_4 * a4 + d5_4 * a5
    + d6_4 * a6 + d7_4 * a7;

    g = -2.0 * g;

    // ペナルティなしの場合は、計算した勾配を返します。
    if (penalty_type_ == PenaltyNone) {
      return g;
    }

    // 実行可能領域外にはみ出た程度に応じて、勾配を計算します。
    double h2 = 0;
    double h4 = 0;

    if (response_.t3 > parameter_.t3_max) {
      h2 += request_.w3 * d3_2;
      h4 += request_.w3 * d3_4;
    } else if (response_.t3 < parameter_.t3_min) {
      h2 -= request_.w3 * d3_2;
      h4 -= request_.w3 * d3_4;
    }

    if (response_.t4 > parameter_.t4_max) {
      h2 += request_.w4 * d4_2;
      h4 += request_.w4 * d4_4;
    } else if (response_.t4 < parameter_.t4_min) {
      h2 -= request_.w4 * d4_2;
      h4 -= request_.w4 * d4_4;
    }

    if (response_.t5 > parameter_.t5_max) {
      h2 += request_.w5 * d5_2;
      h4 += request_.w5 * d5_4;
    } else if (response_.t5 < parameter_.t5_min) {
      h2 -= request_.w5 * d5_2;
      h4 -= request_.w5 * d5_4;
    }

    if (response_.t6 > parameter_.t6_max) {
      h2 += request_.w6 * d6_2;
      h4 += request_.w6 * d6_4;
    } else if (response_.t6 < parameter_.t6_min) {
      h2 -= request_.w6 * d6_2;
      h4 -= request_.w6 * d6_4;
    }

    if (response_.t7 > parameter_.t7_max) {
      h2 += request_.w7 * d7_2;
      h4 += request_.w7 * d7_4;
    } else if (response_.t7 < parameter_.t7_min) {
      h2 -= request_.w7 * d7_2;
      h4 -= request_.w7 * d7_4;
    }

    Vector2 h(h2, h4);

    // 不連続ペナルティの場合
    if (penalty_type_ == PenaltyBigDiscontinuous) {
      return h;
    } else if (penalty_type_ == PenaltyBigProportional) {
    // 連続ペナルティの場合
      return g + penalty_coeff_ * h;
    } else {
      return g;
    }
  }

  /**
   * θ3 の制限範囲からθ4 のとり得る範囲を絞り込みます。
   *
   * @param lower θ4 の下限値を返します。t4_min 以上の値になります。
   * @param upper θ4 の上限値を返します。t4_max 以下の値になります。
   * @return      θ4 に可能な範囲がない場合は false を返します。
   *          この場合、lower, upper は不定になります。
   */
  bool GetTheta4Boundary(double& lower, double& upper) {
    const double A = parameter_.L52;
    const double B = parameter_.L51;
    const double C0 = -request_.R33 * parameter_.L82 + request_.R31 * parameter_.L81 - parameter_.L3 + request_.pz;
    const double C1 = C0 - parameter_.t3_max;
    const double C2 = C0 - parameter_.t3_min;

    const double AA_BB = A * A + B * B;
    const double SQRT_AA_BB = std::sqrt(AA_BB);

    const double Cmax = +SQRT_AA_BB;
    const double Cmin = -SQRT_AA_BB;

    const double X_Cmax = +A / SQRT_AA_BB;
    const double X_Cmin = -A / SQRT_AA_BB;

    double Xmin;
    double Xmax;

    // Cmin, A, Cmax と C1, C2 の位置関係によって、Xmin, Xmax を求めます。
    if (C1 < Cmin) {
      if (C2 < Cmin) {
        return false;
      } else if (C2 <= A) {
        Xmin = X_Cmin;
        Xmax = X_C_(C2);
      } else {
        Xmin = X_Cmin;
        Xmax = 1.0;
      }
    } else if (C1 <= A) {
      if (C2 <= A) {
        Xmin = X_C_(C1);
        Xmax = X_C_(C2);
      } else if (C2 <= Cmax) {
        Xmin = std::min(X_C_(C1), X_C_(C2));
        Xmax = 1.0;
      } else {
        Xmin = std::min(X_C_(C1), X_Cmax);
        Xmax = 1.0;
      }
    } else if (C1 <= Cmax) {
      if (C2 <= Cmax) {
        Xmin = X_C_(C2);
        Xmax = X_C_(C1);
      } else {
        Xmin = X_Cmax;
        Xmax = X_C_(C1);
      }
    } else {
      return false;
    }

    // 計算誤差によって Xmin > Xmax となる場合は、入れ替えます。
    if (Xmin > Xmax) {
      std::swap(Xmin, Xmax);
    }

    // Xmin, Xmax が計算誤差によって [-1,+1] の範囲を超え、
    // 後続の acos で NaN が生成されないようにします。
    Xmin = std::max(-1.0, Xmin);
    Xmax = std::min(+1.0, Xmax);

    // Xmin, Xmax から θ4 の下限と上限を求めます。
    lower = std::max(-acos(Xmin), parameter_.t4_min);
    upper = std::min(-acos(Xmax), parameter_.t4_max);

    if (lower > upper) {
      std::swap(lower, upper);
    }

    return true;
  }


 private:
  OPT_CLASS_UNCOPYABLE(RobotFunction2)

  RobotFunction2Request request_;
  RobotFunction2Parameter parameter_;
  RobotFunction2Response response_;

  /**
   * t2, t4 から t6 を計算したときに、符号の＋を採用したときは true、
   * 符号の−を採用したときは false を記録します。
   * このメンバ変数は CalculateTheta 関数で更新され、
   * Gradient メソッドで使用されます。
   */
  bool t6_use_plus_;

  /**
   * ペナルティ タイプです。
   * デフォルトは PenaltyBigProportional です。
   */
  PenaltyType penalty_type_;

  /**
   * ペナルティ係数です。
   * 実行可能領域外のはみ出し具合に、この係数を乗算した値だけ、
   * ペナルティがかけられます。
   * デフォルトは 1000 です。
   */
  double penalty_coeff_;

  /**
   * 引数に渡された (θ_2, θ_4) の値から、t_i (i=0,...,7) を計算します。
   */
  void CalculateTheta_(const Vector2& x) {
    response_.t2 = x.v1;  // θ_2 を受け取ります。
    response_.t4 = x.v2;  // θ_4 を受け取ります。

    const double S2 = sin(response_.t2);
    const double C2 = cos(response_.t2);
    const double S4 = sin(response_.t4);
    const double C4 = cos(response_.t4);

    response_.t0 = - request_.R13 * parameter_.L82
                   + request_.R11 * parameter_.L81
                   + parameter_.L52 * C2 * S4
                   - parameter_.L51 * C2 * C4
                   + parameter_.L42 * S2
                   - parameter_.L41 * C2
                   + request_.px;

    response_.t1 = - request_.R23 * parameter_.L82
                   + request_.R21 * parameter_.L81
                   + parameter_.L52 * S2 * S4
                   - parameter_.L51 * S2 * C4
                   - parameter_.L42 * C2
                   - parameter_.L41 * S2
                   + request_.py;

    response_.t3 = - request_.R33 * parameter_.L82
                   + request_.R31 * parameter_.L81
                   - parameter_.L52 * C4
                   - parameter_.L51 * S4
                   - parameter_.L3
                   + request_.pz;

    // t6 を計算します。
    {
      double b = -request_.R23 * S2 * S4 - request_.R13 * C2 * S4 + request_.R33 * C4;
      double a = std::sqrt(1 - b * b);
      response_.t6 = std::atan2(a, b);

      // ここで t6 には ± の不定性がありますが、
      // 目的関数値が小さくなるように選択します。
      // 目的関数値は、t6 に依存している t5, t7 によっても変わるため、
      // t5, t7 計算後に、符号を選択します。
    }

    const double S6 = sin(response_.t6);

    if (S6 != 0) {
      // 除算は高価なため、逆数を乗算します。
      // const double S6_inv = 1.0 / S6;

      // sin(t6) による除算は、数値計算が不安定になると考えられるため、
      // atan2 の引数に符号のみを乗算するようにしています。
      const double S6_inv = (S6 > 0) ? 1.0 : -1.0;

      // t5 を計算します。
      double t5_plus, t5_minus;
      {
        double a = (request_.R23 * C2 - request_.R13 * S2) * S6_inv;
        double b = -(request_.R33 * S4 + request_.R23 * S2 * C4 + request_.R13 * C2 * C4) * S6_inv;
        t5_plus = -std::atan2(a, b);
        t5_minus = -std::atan2(-a, -b);

        // t5 が [t5_min,t5_max] の範囲内になるように正規化します。
        // t5 は atan2 の結果として与えられるため、[-PI,+PI] の範囲内であり、
        // したがって、最大でも１度 2*PI を加算するだけで十分です。
        if (t5_plus < parameter_.t5_min) {
          t5_plus += 2 * M_PI;
        }
        if (t5_minus < parameter_.t5_min) {
          t5_minus += 2 * M_PI;
        }
      }

      // t7 を計算します。
      double t7_plus, t7_minus;
      {
        double a = (-request_.R22 * S2 * S4 - request_.R12 * C2 * S4 + request_.R32 * C4) * S6_inv;
        double b = -(-request_.R21 * S2 * S4 - request_.R11 * C2 * S4 + request_.R31 * C4) * S6_inv;
        t7_plus = std::atan2(a, b);
        t7_minus = std::atan2(-a, -b);

        // t7 が [t7_min,t7_max] の範囲内になるように正規化します。
        // t5 の場合と同様です。
        if (t7_plus < parameter_.t7_min) {
          t7_plus += 2 * M_PI;
        }
        if (t7_minus < parameter_.t7_min) {
          t7_minus += 2 * M_PI;
        }
      }

      // t6 の正符号と負符号の場合の目的関数値を計算し、
      // 符号を選択します。
      {
        double t6_plus = response_.t6;
        double t6_minus = -response_.t6;

        response_.t5 = t5_plus;
        response_.t6 = t6_plus;
        response_.t7 = t7_plus;
        double value_plus = ValueFromMembers();

        response_.t5 = t5_minus;
        response_.t6 = t6_minus;
        response_.t7 = t7_minus;
        double value_minus = ValueFromMembers();

        if (value_plus < value_minus) {
          t6_use_plus_ = true;
          response_.t5 = t5_plus;
          response_.t6 = t6_plus;
          response_.t7 = t7_plus;
        } else {
          t6_use_plus_ = false;
        }
      }
    } else {
      // t6=0,PI のいずれかの場合。
      // t6=PI は t6 の可動範囲外であるため、ここでは考慮しません。
      double a = request_.R11 * S2 - request_.R21 * C2;
      double b = request_.R12 * S2 - request_.R22 * C2;
      double A = std::atan2(a, b);

      // t5 を計算します。
      response_.t5 = (request_.w7 * request_.w7 * (A - request_.r7)
                      + request_.r5 * request_.w5 * request_.w5) /
                     (request_.w7 * request_.w7 + request_.w5 * request_.w5);

      if (parameter_.t5_max < response_.t5) {
        response_.t5 = std::min(parameter_.t5_max, A - parameter_.t7_min);
      } else if (response_.t5 < parameter_.t5_min) {
        response_.t5 = std::max(parameter_.t5_min, A - parameter_.t7_max);
      }

      // t7 を計算します。
      response_.t7 = -response_.t5 + A;

      // t5, t7 の正規化は？
    }
  }

  /**
   * プラス関数です。
   */
  inline static double Plus_(double x) {
    return (x > 0) ? x : 0;
  }

  inline double X_C_(double C) {
    const double A = parameter_.L52;
    const double B = parameter_.L51;
    const double AA_BB = A * A + B * B;

    double AA_BB_CC = AA_BB - C * C;

    // 計算誤差によって A*A+B*B-C*C が負数になることがあるので、
    // 適切にクリップします。
    if (AA_BB_CC < 0)
      AA_BB_CC = 0;

    return (A * C + B * std::sqrt(AA_BB_CC)) / AA_BB;
  }
};

}  // namespace opt
#endif  // HSRB_ANALYTIC_IK_ROBOT_FUNCTION_2_HPP_
