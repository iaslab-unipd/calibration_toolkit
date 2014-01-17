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

#ifndef CALIBRATION_COMMON_BASE_OPENCV_CONVERSION_H_
#define CALIBRATION_COMMON_BASE_OPENCV_CONVERSION_H_

#include <opencv2/core/core.hpp>
#include "point_matrix.h"

namespace calibration
{

template <typename ScalarT>
  class OpenCVConversion
  {
  public:

    template <typename OtherScalarT>
      static void toPointMatrix(const std::vector<cv::Point_<OtherScalarT> > & cv_points,
                                PointMatrix<ScalarT, 2> & point_matrix);

    template <typename OtherScalarT>
      static PointMatrix<ScalarT, 2> toPointMatrix(const std::vector<cv::Point_<OtherScalarT> > & cv_points);

    template <typename OtherScalarT>
      static void toPointMatrix(const std::vector<cv::Point3_<OtherScalarT> > & cv_points,
                                PointMatrix<ScalarT, 3> & point_matrix);

    template <typename OtherScalarT>
      static PointMatrix<ScalarT, 3> toPointMatrix(const std::vector<cv::Point3_<OtherScalarT> > & cv_points);

    template <int Dimension, int Elements>
      static void toOpenCV(const Eigen::Matrix<ScalarT, Dimension, Elements> & in,
                           cv::Mat_<cv::Vec<ScalarT, Dimension> > & out);

  };

} /* namespace calibration */

#include <impl/calibration_common/base/opencv_conversion.hpp>

#endif /* CALIBRATION_COMMON_BASE_OPENCV_CONVERSION_H_ */
