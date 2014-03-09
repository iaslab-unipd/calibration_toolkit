/*
 *  Copyright (C) 2013 - Filippo Basso <bassofil@dei.unipd.it>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
