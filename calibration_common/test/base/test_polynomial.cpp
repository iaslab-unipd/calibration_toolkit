/*
 *  Copyright (c) 2015-, Filippo Basso <bassofil@gmail.com>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder(s) nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <calibration_common/base/polynomial.h>

using namespace unipd::calib;

TEST(Polynomial_, Polynomial_)
{
  Polynomial_<double, 3, 0> polynomial1;
  Polynomial_<double, 3, 2> polynomial2;
  Polynomial_<double, 1, 1> polynomial3;
  SUCCEED();
}

TEST(PolynomialX_, Polynomial_1)
{
  PolynomialX_<double> polynomial(4, 2);
  EXPECT_EQ(polynomial.maxDegree(), 4);
  EXPECT_EQ(polynomial.minDegree(), 2);
  EXPECT_EQ(polynomial.size(), 3);
}

TEST(PolynomialX_, PolynomialX_2)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  PolynomialX_<double> polynomial;
  EXPECT_DEATH(polynomial.maxDegree(), "initialized_");
  EXPECT_DEATH(polynomial.minDegree(), "initialized_");
  EXPECT_DEATH(polynomial.size(), "initialized_");
}

TEST(Polynomial_, identityCoefficients)
{
  Polynomial_<double, 3, 0> polynomial;
  auto coeffs = polynomial.identityCoefficients();
  for (int i = 0; i < Polynomial_<double, 3, 0>::Size; ++i)
    EXPECT_EQ(coeffs[i], i == 1 ? 1.0 : 0.0);
}

TEST(PolynomialX_, identityCoefficients)
{
  PolynomialX_<double> polynomial(3, 0);
  auto coeffs = polynomial.identityCoefficients();
  for (int i = 0; i < polynomial.size(); ++i)
    EXPECT_EQ(coeffs[i], i == 1 ? 1.0 : 0.0);
}

TEST(PolynomialX_, identityCoefficients_death)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  PolynomialX_<double> polynomial(3, 2);
  EXPECT_DEATH(polynomial.identityCoefficients(), "min_degree_ <= 1");
}

TEST(Polynomial_, createCoefficients)
{
  Polynomial_<double, 5, 3> polynomial;
  auto coeffs = polynomial.createCoefficients();
  EXPECT_EQ(coeffs.SizeAtCompileTime, 3);
}

TEST(PolynomialX_, createCoefficients)
{
  PolynomialX_<double> polynomial(4, 1);
  auto coeffs = polynomial.createCoefficients();
  EXPECT_EQ(coeffs.size(), 4);
}

TEST(PolynomialX_, createCoefficients_death)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  PolynomialX_<double> polynomial;
  EXPECT_DEATH(polynomial.createCoefficients(), "initialized_");
}

TEST(Polynomial_, evaluate)
{
  Polynomial_<double, 3, 0> polynomial;
  auto coeffs = Polynomial_<double, 3, 0>::Coefficients();
  coeffs[0] = 4;
  coeffs[1] = 3;
  coeffs[2] = 2;
  coeffs[3] = 1;
  auto x = 1.5;
  auto y = coeffs[0] + coeffs[1] * x + coeffs[2] * x * x + coeffs[3] * x * x * x;
  EXPECT_NEAR(polynomial.evaluate(coeffs, x), y, 1e-5);
}

TEST(PolynomialX_, evaluate)
{
  PolynomialX_<double> polynomial(3, 1);
  auto coeffs = polynomial.createCoefficients();
  coeffs[0] = 4;
  coeffs[1] = 3;
  coeffs[2] = 2;
  auto x = 2.5;
  auto y = coeffs[0] * x + coeffs[1] * x * x + coeffs[2] * x * x * x;
  EXPECT_NEAR(polynomial.evaluate(coeffs, x), y, 1e-5);
}
