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

#include <calibration_common/base/point_matrix.h>

#include <opencv2/core/core.hpp>

namespace calibration
{

template <typename ScalarT_>
  class OpenCVConversion
  {
  public:

    template <typename OtherScalarT_>
      static void toPointMatrix(const std::vector<cv::Point_<OtherScalarT_> > & cv_points,
                                PointMatrix<ScalarT_, 2> & point_matrix);

    template <typename OtherScalarT_>
      static PointMatrix<ScalarT_, 2> toPointMatrix(const std::vector<cv::Point_<OtherScalarT_> > & cv_points);

    template <typename OtherScalarT_>
      static void toPointMatrix(const std::vector<cv::Point3_<OtherScalarT_> > & cv_points,
                                PointMatrix<ScalarT_, 3> & point_matrix);

    template <typename OtherScalarT_>
      static PointMatrix<ScalarT_, 3> toPointMatrix(const std::vector<cv::Point3_<OtherScalarT_> > & cv_points);

    template <int Dimension_, int Elements_>
      static void toOpenCV(const Eigen::Matrix<ScalarT_, Dimension_, Elements_> & in,
                           cv::Mat_<cv::Vec<ScalarT_, Dimension_> > & out);

  };

} /* namespace calibration */

#include <impl/calibration_common/base/opencv_conversion.hpp>

#endif /* CALIBRATION_COMMON_BASE_OPENCV_CONVERSION_H_ */
