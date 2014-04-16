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
