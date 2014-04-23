/*
 *  Copyright (c) 2013-2014, Filippo Basso <bassofil@dei.unipd.it>
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
#include <calibration_common/base/matrix.h>

using calibration::Matrix;

TEST(Matrix, Matrix_fixed)
{
  Matrix<int, 2, 5> matrix;
  EXPECT_EQ(matrix.xSize(), 2);
  EXPECT_EQ(matrix.ySize(), 5);
}

TEST(Matrix, Matrix_dynamic)
{
  Matrix<int> matrix(2, 5);
  EXPECT_EQ(matrix.xSize(), 2);
  EXPECT_EQ(matrix.ySize(), 5);
}

TEST(Matrix, Matrix_dynamic_2)
{
  Matrix<int, 2, Eigen::Dynamic> matrix(2, 5);
  EXPECT_EQ(matrix.xSize(), 2);
  EXPECT_EQ(matrix.ySize(), 5);
}

TEST(Matrix, Matrix_dynamic_3)
{
  Matrix<int, Eigen::Dynamic, 5> matrix(2, 5);
  EXPECT_EQ(matrix.xSize(), 2);
  EXPECT_EQ(matrix.ySize(), 5);
}

TEST(Matrix, Matrix_fixed_value)
{
  std::vector<int> conf(10, 7);
  Matrix<int, 2, 5> matrix(7);
  EXPECT_EQ(matrix.container(), conf);
}

TEST(Matrix, Matrix_dynamic_value)
{
  std::vector<int> conf(10, 7);
  Matrix<int> matrix(2, 5, 7);
  EXPECT_EQ(matrix.container(), conf);
}

TEST(Matrix, Matrix_dynamic_value_2)
{
  std::vector<int> conf(10, 7);
  Matrix<int, 2> matrix(2, 5, 7);
  EXPECT_EQ(matrix.container(), conf);
}

TEST(Matrix, Matrix_dynamic_value_3)
{
  std::vector<int> conf(10, 7);
  Matrix<int, Eigen::Dynamic, 5> matrix(2, 5, 7);
  EXPECT_EQ(matrix.container(), conf);
}

TEST(Matrix, Matrix_data)
{
  std::vector<int> data(10, 7);
  Matrix<int, 2, 5> matrix(data);
  EXPECT_EQ(matrix.container(), data);
}

TEST(Matrix, reshape)
{
  std::vector<int> conf(10, 7);
  Matrix<int> matrix(2, 5, 7);
  matrix.reshape(5, 2);
  EXPECT_EQ(matrix.container(), conf);
}
