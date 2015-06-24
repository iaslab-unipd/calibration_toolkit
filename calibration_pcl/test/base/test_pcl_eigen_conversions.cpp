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
#include <calibration_pcl/base/pcl_eigen_conversions.h>

TEST(pcl_eigen, pcl2eigen_1)
{
  pcl::PointCloud<pcl::PointXYZ> in(3, 2); // (width, height)
  in.at(0, 0) = pcl::PointXYZ{1, 2, 3};
  in.at(0, 1) = pcl::PointXYZ{4, 5, 6};
  in.at(1, 0) = pcl::PointXYZ{7, 8, 9};
  in.at(1, 1) = pcl::PointXYZ{9, 8, 7};
  in.at(2, 0) = pcl::PointXYZ{6, 0, 4};
  in.at(2, 1) = pcl::PointXYZ{3, 2, 1};

  auto out = unipd::calib::pcl2eigen<Eigen::Matrix<double, 3, 6>>(in);

  for (int i = 0; i < 6; ++i)
  {
    EXPECT_EQ(in[i].x, out.col(i).x());
    EXPECT_EQ(in[i].y, out.col(i).y());
    EXPECT_EQ(in[i].z, out.col(i).z());
  }
}
TEST(pcl_eigen, pcl2eigen_2)
{
  pcl::PointCloud<pcl::PointXYZ> in(3, 2); // (width, height)
  in.at(0, 0) = pcl::PointXYZ{1, 2, 3};
  in.at(0, 1) = pcl::PointXYZ{4, 5, 6};
  in.at(1, 0) = pcl::PointXYZ{7, 8, 9};
  in.at(1, 1) = pcl::PointXYZ{9, 8, 7};
  in.at(2, 0) = pcl::PointXYZ{6, 0, 4};
  in.at(2, 1) = pcl::PointXYZ{3, 2, 1};

  std::vector<int> indices{1, 2, 4};

  auto out = unipd::calib::pcl2eigen<Eigen::Matrix<double, 3, Eigen::Dynamic>>(in, indices);

  for (int i = 0; i < 3; ++i)
  {
    EXPECT_EQ(in[indices[i]].x, out.col(i).x());
    EXPECT_EQ(in[indices[i]].y, out.col(i).y());
    EXPECT_EQ(in[indices[i]].z, out.col(i).z());
  }
}
