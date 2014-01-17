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

    template <typename PointT>
      static void toPointMatrix(const pcl::PointCloud<PointT> & point_cloud,
                                PointMatrix<ScalarT, 2> & point_matrix);

    template <typename PointT>
      static void toPointMatrix(const pcl::PointCloud<PointT> & point_cloud,
                                PointMatrix<ScalarT, 3> & point_matrix);

    template <typename PointT>
      static PointMatrix<ScalarT, 3> toPointMatrix(const pcl::PointCloud<PointT> & point_cloud);

    template <typename PointT>
      static void toPointMatrix(const pcl::PointCloud<PointT> & point_cloud,
                                const std::vector<int> & indices,
                                PointMatrix<ScalarT, 3> & point_matrix);

    template <typename PointT>
      static PointMatrix<ScalarT, 3> toPointMatrix(const pcl::PointCloud<PointT> & point_cloud,
                                                   const std::vector<int> & indices);

    template <typename PointT>
      static void toPCL(const Eigen::Matrix<ScalarT, 3, 1> & point_in,
                        PointT & point_out);

    template <typename PointT>
      static PointT toPCL(const Eigen::Matrix<ScalarT, 3, 1> & point_in);

    template <typename PointT>
      static void toEigen(const PointT & point_in,
                          Eigen::Matrix<ScalarT, 3, 1> & point_out);

    template <typename PointT>
      static Eigen::Matrix<ScalarT, 3, 1> toEigen(const PointT & point_in);

  };

} /* namespace calibration */

#include <impl/calibration_common/base/pcl_conversion.hpp>

#endif /* CALIBRATION_COMMON_BASE_PCL_CONVERSION_H_ */
