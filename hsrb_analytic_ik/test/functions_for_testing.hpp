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
#ifndef HSRB_ANALYTIC_IK_FUNCTION_FOR_TESTING_HPP_
#define HSRB_ANALYTIC_IK_FUNCTION_FOR_TESTING_HPP_

#include <limits>

#include "vector2.hpp"

namespace opt {

/**
 * テスト用の一次関数 A です。
 * f(x) = 2*x + 1
 * 最小値は持ちません。
 */
class LinearFunction1A {
 public:
  double Value(double x) {
    return (2.0 * x + 1.0);
  }
  double Gradient(double x) {
    return 2.0;
  }
  double Hessian(double x) {
    return 0;
  }
};

/**
 * テスト用の二次関数 A です。
 * f(x) = (x-1)^2 + 2
 * x=1 において最小値 f(x)=2 を取ります。
 */
class QuadraticFunction1A {
 public:
  double Value(double x) {
    return (x * x - 2.0 * x + 3.0);
  }
  double Gradient(double x) {
    return 2.0 * (x - 1.0);
  }
  double Hessian(double x) {
    return 2.0;
  }
};

/**
 * テスト用の二次関数 B です。
 * f(x) = 1/2*(x+1)^2 - 1/2
 * x=-1 において最小値 f(x)=1/2 を取ります。
 */
class QuadraticFunction1B {
 public:
  double Value(double x) {
    return (0.5 * x * (x + 2.0));
  }
  double Gradient(double x) {
    return (x + 1.0);
  }
  double Hessian(double x) {
    return 1.0;
  }
};

/**
 * テスト用の微分不可能関数です。
 *
 * f(x) = -x/2        (if x <= -1)
 * f(x) = 2*x + 5/2   (if x > -1)
 *
 * x=-1 で最小値 f(x)=0.5 を持ちます。
 */
class NonDiffenrentialFunction1A {
 public:
  double Value(double x) {
    if (x <= -1)
      return -0.5 * x;
    else
      return 2.0 * x + 2.5;
  }
  double Gradient(double x) {
    if (x <= -1)
      return -0.5;
    else
      return 2.0;
  }
  double Hessian(double x) {
    return 0;
  }
};

/**
 * テスト用の逆台形関数です。
 *
 * f(x) = -x + 2 (if x < -1)
 * f(x) = 3      (if  -1 <= x <= 2);
 * f(x) = x+1    (if 2 < x)
 *
 * x=-1 で最小値 f(x)=0.5 を持ちます。
 */
class InvertedTrapeziumFunction1A {
 public:
  double Value(double x) {
    if (x < -1.0)
      return 2.0 - x;
    if (x < 2.0)
      return 3.0;
    return x + 1.0;
  }
  double Gradient(double x) {
    if (x < -1.0)
      return -1.0;
    if (x < 2.0)
      return 0.0;
    return 1.0;
  }
  double Hessian(double x) {
    return 0;
  }
};


/**
 * テスト用の不連続関数 A です。
 *
 * f(x) = -2*x + 1 (if x < 1)
 * f(x) = x-1      (if 1 <= x < 2);
 * f(x) = x        (if 2 <= x)
 *
 * x=-1 で最小値 f(x)=0.5 を持ちます。
 */
class DiscontinuousFunction1A {
 public:
  double Value(double x) {
    if (x < 1.0)
      return -2.0 * x + 1.0;
    if (x < 2.0)
      return x - 1;
    return x;
  }
  double Gradient(double x) {
    if (x < 1.0)
      return -2.0;
    if (x < 2.0)
      return 1.0;
    return 1.0;
  }
  double Hessian(double x) {
    return 0;
  }
};

/**
 * テスト用の不連続関数 B です。
 * RobotoFunction2 のペナルティ関数に類似のペナルティを課します。
 *
 * f(x) = x^2        (if -1 <= x <= 1)
 * f(x) = +x + BIG   (if 1 < x)
 * f(x) = -x + BIG   (if x < -1)
 *
 * x=-1 で最小値 f(x)=0.5 を持ちます。
 */
class DiscontinuousFnction1B {
 public:
  explicit DiscontinuousFnction1B(double penalty = std::numeric_limits<double>::max())
      : penalty_(penalty) {}

  double Value(double x) {
    if (x < -1) {
      return -x + penalty_;
    } else if (x < 1) {
      return x * x;
    } else {
      return +x + penalty_;
    }
  }
  double Gradient(double x) {
    if (x < -1) {
      return -1;
    } else if (x < 1) {
      return 2 * x;
    } else {
      return +1;
    }
  }
  double Hessian(double x) {
    if (x < -1) {
      return 0;
    } else if (x < 1) {
      return 2;
    } else {
      return 0;
    }
  }

 private:
  double penalty_;
};

/**
 * テスト用の非凸関数です。
 * sin 関数を点 x=2n で連結し、振幅を abs(n) としたものです。
 * x=2n で微分不可能ですが、区間内での最小点がわかりやすいです。
 *
 * f(x) = sin (PI * x) * abs(n)   (if x∈[2n-2,2n]);
 *
 * 区間 [0, 1] では x=0,1 で最小値をとります。
 * 区間 [0, 2] では x=3/2 で最小値をとります。
 * 区間 [0, 4] では x=7/2 で最小値をとります。
 *
 */
class NonConvexFunction1A {
 public:
  double Value(double x) {
    int n = 1 + std::abs(static_cast<int>(std::floor(x / 2)));
    return sin(M_PI * x) * n;
  }
  double Gradient(double x) {
    int n = 1 + std::abs(static_cast<int>(std::floor(x / 2)));
    return M_PI * cos(M_PI * x) * n;
  }
  double Hessian(double x) {
    int n = 1 + std::abs(static_cast<int>(std::floor(x / 2)));
    return -M_PI * M_PI * sin(M_PI * x) * n;
  }
};

/**
 * テスト用の四次関数 A です。
 *
 * f(x) = (x1-2)^4 + (x1-2*x2)^2
 *
 * 点 (2,1) で最小値 0 をとります。
 *
 */
class QuarticFunction2A {
 public:
  bool IsFeasible(Vector2 x) {
    return true;
  }
  double Value(Vector2 x) {
    double a = x.v1 - 2;
    double b = x.v1 - 2 * x.v2;
    return (a*a*a*a + b*b);
  }
  Vector2 Gradient(Vector2 x) {
    double a = x.v1 - 2;
    double b = x.v1 - 2 * x.v2;
    return Vector2(4*a*a*a + 2*b, -4*b);
  }
};

}  // namespace opt
#endif  // HSRB_ANALYTIC_IK_FUNCTION_FOR_TESTING_HPP_
