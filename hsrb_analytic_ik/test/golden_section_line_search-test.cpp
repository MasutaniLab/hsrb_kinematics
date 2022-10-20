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
#include <gtest/gtest.h>

#include "functions_for_testing.hpp"
#include "golden_section_line_search.hpp"

namespace opt {
////////////////////////////////////////////////////////////////////////////////
//// 一次関数 A によるテスト

TEST(GoldenSectionLineSearch_Test, LinearFunction1A_Test) {
  int maxItor = 100;
  double epsilon = 1e-8;

  // 区間 [0,2] で探索します。x=0 が解になります。
  {
    LinearFunction1A func;
    GoldenSectionLineSearch search(maxItor, epsilon);
    double a = 0.0;
    double b = 2.0;
    double expected = 0.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(0 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }
}

////////////////////////////////////////////////////////////////////////////////
//// 二次関数 A によるテスト

TEST(GoldenSectionLineSearch_Test, QuadraticFunction1A_Test) {
  QuadraticFunction1A func;

  int maxItor = 100;
  double epsilon = 1e-8;
  GoldenSectionLineSearch search(maxItor, epsilon);

  // 区間 [0,2] で探索します。x=1 が解になります。
  {
    double a = 0.0;
    double b = 2.0;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    // 【メモ】関数値の計算誤差によって、真値との誤差が
    //  不確定区間の誤差以下になるとは限らないようなので、
    //  ここでは誤差を 10 倍して検証しています。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // 区間 [0,1] で探索します。x=1 が解になります。
  {
    double a = 0.0;
    double b = 1.0;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // 区間 [1,2] で探索します。x=1 が解になります。
  {
    double a = 1.0;
    double b = 2.0;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // 区間 [-4,-1] で探索します。x=-1 が解になります。
  {
    double a = -4.0;
    double b = -1.0;
    double expected = -1.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // 区間 [2, 8] で探索します。x=2 が解になります。
  {
    double a = 2.0;
    double b = 8.0;
    double expected = 2.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // 区間 [0,2] で探索します。x=1 が解になります。
  // 反復回数を限定して OptMaxItor が返されることを検証します。
  {
    double a = 0.0;
    double b = 2.0;
    double expected = 1.0;

    // 探索を行って、反復回数を取得します。
    OptResult result = search.Search(func, a, b);
    int iteration = search.iteration();

    // 反復回数を 1 つだけ少なくして探索を行います。
    search.set_max_iteration(iteration - 1);
    result = search.Search(func, a, b);
    EXPECT_EQ(OptMaxItor, result);
    EXPECT_EQ(OptMaxItor, search.result());

    // 反復回数が最大になっていることを確認します。
    EXPECT_EQ(iteration - 1, search.iteration());
  }
}

////////////////////////////////////////////////////////////////////////////////
//// 微分不可能関数によるテスト

TEST(GoldenSectionLineSearch_Test, NonDiffenrentialFunction1A_Test) {
  NonDiffenrentialFunction1A func;

  int maxItor = 100;
  double epsilon = 1e-8;
  GoldenSectionLineSearch search(maxItor, epsilon);

  // 区間 [-2, 2] で探索します。x=-1 が解になります。
  {
    double a = -2.0;
    double b = 2.0;
    double expected = -1.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }

  // 区間 [-2, -1] で探索します。x=-1 が解になります。
  {
    double a = -2.0;
    double b = -1.0;
    double expected = -1.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }

  // 区間 [-1, 2] で探索します。x=-1 が解になります。
  {
    double a = -1.0;
    double b = 2.0;
    double expected = -1.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }

  // 区間 [-4, -3] で探索します。x=-3 が解になります。
  {
    double a = -4.0;
    double b = -3.0;
    double expected = -3.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }

  // 区間 [3, 4] で探索します。x=3 が解になります。
  {
    double a = 3.0;
    double b = 4.0;
    double expected = 3.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }
}

////////////////////////////////////////////////////////////////////////////////
//// 逆台形関数によるテスト

TEST(GoldenSectionLineSearch_Test, InvertedTrapeziumFunction1A_Test) {
  InvertedTrapeziumFunction1A func;

  int maxItor = 100;
  double epsilon = 1e-8;
  GoldenSectionLineSearch search(maxItor, epsilon);

  // 区間 [-5, 5] で探索します。x∈[-1,2] が解になります。
  {
    double a = -5.0;
    double b = 5.0;
    double expected1 = -1.0;
    double expected2 = 2.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_TRUE(
                expected1 - epsilon <= solution
                && solution <= expected2 + epsilon);
  }

  // 区間 [-1/2, 1/2] で探索します。x∈[-1/2,1/2] が解になります。
  {
    double a = -0.5;
    double b = 0.5;
    double expected1 = -0.5;
    double expected2 = 0.5;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_TRUE(
                expected1 - epsilon <= solution
                && solution <= expected2 + epsilon);
  }

  // 区間 [-5, 0] で探索します。x∈[-1,0] が解になります。
  {
    double a = -5.0;
    double b = 0.0;
    double expected1 = -1.0;
    double expected2 = 0.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_TRUE(
                expected1 - epsilon <= solution
                && solution <= expected2 + epsilon);
  }

  // 区間 [1, 3] で探索します。x∈[1,2] が解になります。
  {
    double a = 1.0;
    double b = 3.0;
    double expected1 = 1.0;
    double expected2 = 2.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_TRUE(
                expected1 - epsilon <= solution
                && solution <= expected2 + epsilon);
  }
}

////////////////////////////////////////////////////////////////////////////////
//// 不連続関数 A によるテスト

TEST(GoldenSectionLineSearch_Test, DiscontinuousFunction1A_Test) {
  DiscontinuousFunction1A func;

  int maxItor = 100;
  double epsilon = 1e-8;
  GoldenSectionLineSearch search(maxItor, epsilon);

  // 区間 [-5, 5] で探索します。x=1 が解になります。
  {
    double a = -5.0;
    double b = 5.0;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }

  // 区間 [0, 3] で探索します。x=1 が解になります。
  {
    double a = 0.0;
    double b = 3.0;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }

  // 区間 [0, 1] で探索します。x=1 が解になります。
  {
    double a = 0.0;
    double b = 1.0;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }

  // 区間 [0, 2] で探索します。x=1 が解になります。
  {
    double a = 0.0;
    double b = 1.0;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }

  // 区間 [1, 3] で探索します。x=1 が解になります。
  {
    double a = 1.0;
    double b = 3.0;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }

  // 区間 [1.5, 3] で探索します。x=1.5 が解になります。
  {
    double a = 1.5;
    double b = 3.0;
    double expected = 1.5;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon);
  }
}

////////////////////////////////////////////////////////////////////////////////
//// 不連続関数 B によるテスト

TEST(GoldenSectionLineSearch_Test, DiscontinuousFunction1B_Test) {
  double penalty = 1e8;
  DiscontinuousFnction1B func(penalty);
  int maxItor = 100;
  double epsilon = 1e-8;
  GoldenSectionLineSearch search(maxItor, epsilon);

  // 区間 [-100, 100] で探索します。x=0 が解になります。
  {
    double a = -20;
    double b = 20;
    double expected = 0.0;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }
}

////////////////////////////////////////////////////////////////////////////////
//// 非凸関数による異常系のテスト

TEST(GoldenSectionLineSearch_Test, NonConvexFunction1A_Test) {
  NonConvexFunction1A func;

  int maxItor = 100;
  double epsilon = 1e-8;
  GoldenSectionLineSearch search(maxItor, epsilon);

  // 区間 [0, 4] で探索します。
  // 探索結果は 1.5 になるはずです。
  {
    double a = 0;
    double b = 4;
    double expected = 1.5;

    // 探索を行います。
    OptResult result = search.Search(func, a, b);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }
}

}  // namespace opt

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
