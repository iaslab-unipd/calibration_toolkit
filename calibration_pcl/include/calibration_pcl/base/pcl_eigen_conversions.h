/*
 *  Copyright (c) 2013-, Filippo Basso <bassofil@gmail.com>
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

#ifndef UNIPD_CALIBRATION_CALIBRATION_PCL_BASE_PCL_EIGEN_CONVERSIONS_H_
#define UNIPD_CALIBRATION_CALIBRATION_PCL_BASE_PCL_EIGEN_CONVERSIONS_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>

namespace unipd
{
namespace calib
{

template <typename EigenT_>
  EigenT_
  pcl2eigen (const pcl::PointCloud<pcl::PointXYZ> & in)
  {
    auto out = EigenT_(3, in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
      const auto & p = in[i];
      out.col(i) << p.x, p.y, p.z;
    }
    return out;
  }

template <typename EigenT_>
  EigenT_
  pcl2eigen (const pcl::PointCloud<pcl::PointXYZ> & in,
             const std::vector<int> & indices)
  {
    auto out = EigenT_(3, indices.size());
    for (size_t i = 0; i < indices.size(); ++i)
    {
      const auto & p = in[indices[i]];
      out.col(i) << p.x, p.y, p.z;
    }
    return out;
  }

} // namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_CALIBRATION_PCL_BASE_PCL_EIGEN_CONVERSIONS_H_
