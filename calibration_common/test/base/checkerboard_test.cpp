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
#include <calibration_common/base/point_matrix.h>
#include <calibration_common/objects/checkerboard.h>

using calibration::Checkerboard;
using namespace calibration;

#define ERROR 1e-6

TEST(Checkerboard, Checkerboard)
{
  Checkerboard cb(3, 2, 1.0, 2.0);

  EXPECT_EQ(cb.rows(), 2);
  EXPECT_EQ(cb.cols(), 3);
  EXPECT_NEAR(cb.cellWidth(), 1.0, ERROR);
  EXPECT_NEAR(cb.cellHeight(), 2.0, ERROR);

  EXPECT_NEAR(cb.height(), 6.0, ERROR);
  EXPECT_NEAR(cb.width(), 4.0, ERROR);
  EXPECT_NEAR(cb.area(), 24.0, ERROR);

  Point3 center(1.0, 1.0, 0.0);
  EXPECT_NEAR((cb.center() - center).norm(), 0.0, ERROR);
}

TEST(Checkerboard, corners)
{
  Checkerboard cb(3, 2, 1.0, 2.0);

  Point3 p00(0.0, 0.0, 0.0);
  Point3 p10(1.0, 0.0, 0.0);
  Point3 p20(2.0, 0.0, 0.0);
  Point3 p01(0.0, 2.0, 0.0);
  Point3 p11(1.0, 2.0, 0.0);
  Point3 p21(2.0, 2.0, 0.0);

  EXPECT_NEAR((cb(0, 0) - p00).norm(), 0.0, ERROR);
  EXPECT_NEAR((cb(1, 0) - p10).norm(), 0.0, ERROR);
  EXPECT_NEAR((cb(2, 0) - p20).norm(), 0.0, ERROR);
  EXPECT_NEAR((cb(0, 1) - p01).norm(), 0.0, ERROR);
  EXPECT_NEAR((cb(1, 1) - p11).norm(), 0.0, ERROR);
  EXPECT_NEAR((cb(2, 1) - p21).norm(), 0.0, ERROR);
}

TEST(Checkerboard, transform)
{
  Checkerboard cb(3, 2, 1.0, 2.0);

  Eigen::Translation3d translation(1.0, 2.0, 3.0);
  Eigen::AngleAxisd rotation(M_PI_2, Eigen::Vector3d::UnitZ());
  Eigen::Affine3d transform(translation * rotation);

  Point3 p00(0.0, 0.0, 0.0);
  Point3 p10(1.0, 0.0, 0.0);
  Point3 p20(2.0, 0.0, 0.0);
  Point3 p01(0.0, 2.0, 0.0);
  Point3 p11(1.0, 2.0, 0.0);
  Point3 p21(2.0, 2.0, 0.0);

  p00 = transform * p00;
  p10 = transform * p10;
  p20 = transform * p20;
  p01 = transform * p01;
  p11 = transform * p11;
  p21 = transform * p21;

  cb.transform(transform);

  EXPECT_NEAR((cb(0, 0) - p00).norm(), 0.0, ERROR);
  EXPECT_NEAR((cb(1, 0) - p10).norm(), 0.0, ERROR);
  EXPECT_NEAR((cb(2, 0) - p20).norm(), 0.0, ERROR);
  EXPECT_NEAR((cb(0, 1) - p01).norm(), 0.0, ERROR);
  EXPECT_NEAR((cb(1, 1) - p11).norm(), 0.0, ERROR);
  EXPECT_NEAR((cb(2, 1) - p21).norm(), 0.0, ERROR);
}
