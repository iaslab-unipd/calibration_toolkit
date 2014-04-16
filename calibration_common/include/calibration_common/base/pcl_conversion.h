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

#ifndef CALIBRATION_COMMON_BASE_PCL_CONVERSION_H_
#define CALIBRATION_COMMON_BASE_PCL_CONVERSION_H_

#include <pcl/point_cloud.h>
#include <calibration_common/base/point_matrix.h>

namespace calibration
{

template <typename ScalarT>
  struct PCLConversion
  {

    template <typename PCLPointT_>
      static void toPointMatrix(const pcl::PointCloud<PCLPointT_> & point_cloud,
                                PointMatrix<ScalarT, 2> & point_matrix);

    template <typename PCLPointT_>
      static void toPointMatrix(const pcl::PointCloud<PCLPointT_> & point_cloud,
                                PointMatrix<ScalarT, 3> & point_matrix);

    template <typename PCLPointT_>
      static PointMatrix<ScalarT, 3> toPointMatrix(const pcl::PointCloud<PCLPointT_> & point_cloud);

    template <typename PCLPointT_>
      static void toPointMatrix(const pcl::PointCloud<PCLPointT_> & point_cloud,
                                const std::vector<int> & indices,
                                PointMatrix<ScalarT, 3> & point_matrix);

    template <typename PCLPointT_>
      static PointMatrix<ScalarT, 3> toPointMatrix(const pcl::PointCloud<PCLPointT_> & point_cloud,
                                                   const std::vector<int> & indices);

    template <typename PCLPointT_>
      static void toPCL(const Eigen::Matrix<ScalarT, 3, 1> & point_in,
                        PCLPointT_ & point_out);

    template <typename PCLPointT_>
      static PCLPointT_ toPCL(const Eigen::Matrix<ScalarT, 3, 1> & point_in);

    template <typename PCLPointT_>
      static void toEigen(const PCLPointT_ & point_in,
                          Eigen::Matrix<ScalarT, 3, 1> & point_out);

    template <typename PCLPointT_>
      static Eigen::Matrix<ScalarT, 3, 1> toEigen(const PCLPointT_ & point_in);

  };

} /* namespace calibration */

#include <impl/calibration_common/base/pcl_conversion.hpp>

#endif /* CALIBRATION_COMMON_BASE_PCL_CONVERSION_H_ */
