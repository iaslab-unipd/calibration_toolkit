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
#include <calibration_common/base/opencv_eigen_conversions.h>

TEST(opencv_eigen, eigen2opencv)
{
  Eigen::Matrix<double, 3, 2> in;
  in.col(0) = Eigen::Vector3d(1, 2, 3);
  in.col(1) = Eigen::Vector3d(4, 5, 6);

  cv::Mat_<cv::Vec3d> out = unipd::calib::eigen2opencv(in);

  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 2; ++j)
      EXPECT_EQ(in(i, j), out.at<cv::Vec3d>(j)[i]);

}

TEST(opencv_eigen, opencv2eigen)
{
  std::vector<cv::Point2d> in(3); // (rows, cols)
  in[0] = cv::Point2d(1, 2);
  in[1] = cv::Point2d(3, 4);
  in[2] = cv::Point2d(5, 6);

  auto out = unipd::calib::opencv2eigen<Eigen::Matrix<double, 2, Eigen::Dynamic>>(in);

  for (int i = 0; i < 3; ++i)
  {
    EXPECT_EQ(out(0, i), in[i].x);
    EXPECT_EQ(out(1, i), in[i].y);
  }

}
