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

#include "bi_golden_section_line_search.hpp"
#include "functions_for_testing.hpp"

namespace opt {
////////////////////////////////////////////////////////////////////////////////
//// 二次関数 A によるテスト

/**
 * 二次関数 QuadraticFunction1A  に対して単方向と同じ結果が出るかテストします。
 */
TEST(BiGoldenSectionLineSearch_Test, QuadraticFunction1A_Test) {
  int maxItor = 100;
  double epsilon = 1e-8;

  // 初期ステップ長 0.1 で探索します。x=1 が解になります。
  {
    QuadraticFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.1;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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

  // 初期ステップ長 0.5 で探索します。x=1 が解になります。
  {
    QuadraticFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.5;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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

  // 初期ステップ長 1.0 で探索します。x=1 が解になります。
  {
    QuadraticFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 1.0;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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

  // 初期ステップ長 2.0 で探索します。x=1 が解になります。
  {
    QuadraticFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 2.0;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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

  // 初期ステップ長 100.0 で探索します。x=1 が解になります。
  {
    QuadraticFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 100.0;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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

  // 初期ステップ長 0.01 で探索します。x=1 が解になります。
  {
    QuadraticFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.01;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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

  // x=-1 で最小値をとる二次関数でテストします。
  // 初期ステップ長 0.1 で探索します。
  {
    ShiftAdapterFunction1<QuadraticFunction1A> func(QuadraticFunction1A(), -2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.1;
    double expected = -1;

    // 探索を行います。
    OptResult result = search.Search(func, step);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(0 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }
}

////////////////////////////////////////////////////////////////////////////////
//// 二次関数 B によるテスト

/**
 * 二次関数 QuadraticFunction1B  に対して単方向と同じ結果が出るかテストします。
 */
TEST(BiGoldenSectionLineSearch_Test, QuadraticFunction1B_Test) {
  int maxItor = 100;
  double epsilon = 1e-8;

  // 初期ステップ長 0.1 で探索します。x=-1 が解になります。
  {
    QuadraticFunction1B func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.1;
    double expected = -1.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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

  // 初期ステップ長 0.5 で探索します。x=-1 が解になります。
  {
    QuadraticFunction1B func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.5;
    double expected = -1.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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

  // 初期ステップ長 1.0 で探索します。x=-1 が解になります。
  {
    QuadraticFunction1B func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 1.0;
    double expected = -1.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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

  // 初期ステップ長 2.0 で探索します。x=-1 が解になります。
  {
    QuadraticFunction1B func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 2.0;
    double expected = -1.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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

  // 初期ステップ長 100.0 で探索します。x=-1 が解になります。
  {
    QuadraticFunction1B func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 100.0;
    double expected = -1.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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

  // 初期ステップ長 0.01 で探索します。x=-1 が解になります。
  {
    QuadraticFunction1B func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.01;
    double expected = -1.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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

  // x=2 で最小値をとる二次関数でテストします。
  // 初期ステップ長 0.1 で探索します。
  {
    ShiftAdapterFunction1<QuadraticFunction1B> func(QuadraticFunction1B(), 3.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.1;
    double expected = 2.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(0 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }
}

////////////////////////////////////////////////////////////////////////////////
//// 微分不可能関数によるテスト

TEST(BiGoldenSectionLineSearch_Test, NonDiffenrentialFunction1A_Test) {
  int maxItor = 100;
  double epsilon = 1e-8;

  // 初期ステップ長 0.1 で探索します。x=1 が解になります。
  {
    // NonDiffenrentialFunction1A を +2 だけシフトした関数を使用します。
    // x=1 で最小値を持ちます。
    ShiftAdapterFunction1<NonDiffenrentialFunction1A> func(NonDiffenrentialFunction1A(), +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.1;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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

  // 初期ステップ長 0.5 で探索します。x=1 が解になります。
  {
    // NonDiffenrentialFunction1A を +2 だけシフトした関数を使用します。
    // x=1 で最小値を持ちます。
    ShiftAdapterFunction1<NonDiffenrentialFunction1A> func(
                                                           NonDiffenrentialFunction1A(),
                                                           +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.5;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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

  // 初期ステップ長 1.0 で探索します。x=1 が解になります。
  {
    // NonDiffenrentialFunction1A を +2 だけシフトした関数を使用します。
    // x=1 で最小値を持ちます。
    ShiftAdapterFunction1<NonDiffenrentialFunction1A> func(
                                                           NonDiffenrentialFunction1A(),
                                                           +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 1.0;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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

  // 初期ステップ長 2.0 で探索します。x=1 が解になります。
  {
    // NonDiffenrentialFunction1A を +2 だけシフトした関数を使用します。
    // x=1 で最小値を持ちます。
    ShiftAdapterFunction1<NonDiffenrentialFunction1A> func(
                                                           NonDiffenrentialFunction1A(),
                                                           +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 2.0;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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

  // 初期ステップ長 100.0 で探索します。x=1 が解になります。
  {
    // NonDiffenrentialFunction1A を +2 だけシフトした関数を使用します。
    // x=1 で最小値を持ちます。
    ShiftAdapterFunction1<NonDiffenrentialFunction1A> func(
                                                           NonDiffenrentialFunction1A(),
                                                           +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 100.0;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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

  // 初期ステップ長 0.01 で探索します。x=1 が解になります。
  {
    // NonDiffenrentialFunction1A を +2 だけシフトした関数を使用します。
    // x=1 で最小値を持ちます。
    ShiftAdapterFunction1<NonDiffenrentialFunction1A> func(
                                                           NonDiffenrentialFunction1A(),
                                                           +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.01;
    double expected = 1.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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
//// 逆台形関数によるテスト

TEST(BiGoldenSectionLineSearch_Test, InvertedTrapeziumFunction1A_Test) {
  int maxItor = 100;
  double epsilon = 1e-8;

  // 初期ステップ長 0.1 で探索します。x∈[-1,2] が解になります。
  {
    InvertedTrapeziumFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);

    double step = 0.1;
    double expected1 = -1.0;
    double expected2 = 2.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

    // 探索結果を検証します。
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // 反復回数を検証します。
    int iteration = search.iteration();
    EXPECT_TRUE(0 <= iteration && iteration <= maxItor);

    // 解を検証します。
    double solution = search.solution();
    EXPECT_TRUE(
                expected1 - epsilon <= solution
                && solution <= expected2 + epsilon);
  }

  // 初期ステップ長 0.1 で探索します。x∈[1,4] が解になります。
  {
    // 逆台形関数を +2 だけシフトします。
    // x∈[1,4] で最小値をとります。
    ShiftAdapterFunction1<InvertedTrapeziumFunction1A> func(InvertedTrapeziumFunction1A(), +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);

    double step = 0.1;
    double expected1 = 1.0;
    double expected2 = 4.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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

  // 初期ステップ長 100 で探索します。x∈[1,4] が解になります。
  {
    // 逆台形関数を +2 だけシフトします。
    // x∈[1,4] で最小値をとります。
    ShiftAdapterFunction1<InvertedTrapeziumFunction1A> func(
                                                            InvertedTrapeziumFunction1A(),
                                                            +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);

    double step = 100.0;
    double expected1 = 1.0;
    double expected2 = 4.0;

    // 探索を行います。
    OptResult result = search.Search(func, step);

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

}  // namespace opt

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
