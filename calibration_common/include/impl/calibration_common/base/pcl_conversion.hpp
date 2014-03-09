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

#ifndef CALIBRATION_COMMON_IMPL_BASE_PCL_CONVERSION_HPP_
#define CALIBRATION_COMMON_IMPL_BASE_PCL_CONVERSION_HPP_

#include <calibration_common/base/pcl_conversion.h>

namespace calibration
{

template <typename ScalarT_>
  template <typename PCLPointT_>
    void PCLConversion<ScalarT_>::toPointMatrix(const pcl::PointCloud<PCLPointT_> & point_cloud,
                                                PointMatrix<ScalarT_, 2> & point_matrix)
    {
      for (size_t i = 0; i < point_cloud.size(); ++i)
      {
        const PCLPointT_ & p = point_cloud.points[i];
        point_matrix[i] << p.x, p.y;
      }
    }

template <typename ScalarT_>
  template <typename PCLPointT_>
    void PCLConversion<ScalarT_>::toPointMatrix(const pcl::PointCloud<PCLPointT_> & point_cloud,
                                                PointMatrix<ScalarT_, 3> & point_matrix)
    {
      for (size_t i = 0; i < point_cloud.size(); ++i)
      {
        const PCLPointT_ & p = point_cloud.points[i];
        point_matrix[i] << p.x, p.y, p.z;
      }
    }

template <typename ScalarT_>
  template <typename PCLPointT_>
    PointMatrix<ScalarT_, 3> PCLConversion<ScalarT_>::toPointMatrix(const pcl::PointCloud<PCLPointT_> & point_cloud)
    {
      PointMatrix<ScalarT_, 3> point_matrix(point_cloud.width, point_cloud.height);
      toPointMatrix(point_cloud, point_matrix);
      return point_matrix;
    }

template <typename ScalarT_>
  template <typename PCLPointT_>
    void PCLConversion<ScalarT_>::toPointMatrix(const pcl::PointCloud<PCLPointT_> & point_cloud,
                                                const std::vector<int> & indices,
                                                PointMatrix<ScalarT_, 3> & point_matrix)
    {
      for (size_t i = 0; i < indices.size(); ++i)
      {
        const PCLPointT_ & p = point_cloud.points[indices[i]];
        point_matrix[i] << p.x, p.y, p.z;
      }
    }

template <typename ScalarT_>
  template <typename PCLPointT_>
    PointMatrix<ScalarT_, 3> PCLConversion<ScalarT_>::toPointMatrix(const pcl::PointCloud<PCLPointT_> & point_cloud,
                                                                    const std::vector<int> & indices)
    {
      PointMatrix<ScalarT_, 3> point_matrix(indices.size(), 1);
      toPointMatrix(point_cloud, indices, point_matrix);
      return point_matrix;
    }

template <typename ScalarT_>
  template <typename PCLPointT_>
    void PCLConversion<ScalarT_>::toPCL(const Eigen::Matrix<ScalarT_, 3, 1> & point_in,
                                        PCLPointT_ & point_out)
    {
      point_out.x = float(point_in[0]);
      point_out.y = float(point_in[1]);
      point_out.z = float(point_in[2]);
    }

template <typename ScalarT_>
  template <typename PCLPointT_>
    PCLPointT_ PCLConversion<ScalarT_>::toPCL(const Eigen::Matrix<ScalarT_, 3, 1> & point_in)
    {
      PCLPointT_ point_out;
      toPCL(point_in, point_out);
      return point_out;
    }

template <typename ScalarT_>
  template <typename PCLPointT_>
    void PCLConversion<ScalarT_>::toEigen(const PCLPointT_ & point_in,
                                          Eigen::Matrix<ScalarT_, 3, 1> & point_out)
    {
      point_out[0] = ScalarT_(point_in.x);
      point_out[1] = ScalarT_(point_in.y);
      point_out[2] = ScalarT_(point_in.z);
    }

template <typename ScalarT_>
  template <typename PCLPointT_>
    Eigen::Matrix<ScalarT_, 3, 1> PCLConversion<ScalarT_>::toEigen(const PCLPointT_ & point_in)
    {
      Eigen::Matrix<ScalarT_, 3, 1> point_out;
      toPCL(point_in, point_out);
      return point_out;
    }

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_IMPL_BASE_PCL_CONVERSION_HPP_ */
