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

#ifndef CALIBRATION_COMMON_IMPL_BASE_OPENCV_CONVERSION_HPP_
#define CALIBRATION_COMMON_IMPL_BASE_OPENCV_CONVERSION_HPP_

#include <calibration_common/base/opencv_conversion.h>

namespace calibration
{

template <typename ScalarT>
  template <typename OtherScalarT>
    void OpenCVConversion<ScalarT>::toPointMatrix(const std::vector<cv::Point_<OtherScalarT> > & cv_points,
                                                  PointMatrix<ScalarT, 2> & point_matrix)
    {
      for (size_t i = 0; i < cv_points.size(); ++i)
      {
        const cv::Point_<OtherScalarT> & p = cv_points[i];
        point_matrix[i] << ScalarT(p.x), ScalarT(p.y);
      }
    }

template <typename ScalarT>
  template <typename OtherScalarT>
    PointMatrix<ScalarT, 2> OpenCVConversion<ScalarT>::toPointMatrix(const std::vector<cv::Point_<OtherScalarT> > & cv_points)
    {
      PointMatrix<ScalarT, 2> point_matrix(cv_points.size(), 1);
      toPointMatrix(cv_points, point_matrix);
      return point_matrix;
    }

template <typename ScalarT>
  template <typename OtherScalarT>
    void OpenCVConversion<ScalarT>::toPointMatrix(const std::vector<cv::Point3_<OtherScalarT> > & cv_points,
                                                  PointMatrix<ScalarT, 3> & point_matrix)
    {
      for (size_t i = 0; i < cv_points.size(); ++i)
      {
        const cv::Point3_<OtherScalarT> & p = cv_points[i];
        point_matrix[i] << ScalarT(p.x), ScalarT(p.y), ScalarT(p.z);
      }
    }

template <typename ScalarT>
  template <typename OtherScalarT>
    PointMatrix<ScalarT, 3> OpenCVConversion<ScalarT>::toPointMatrix(const std::vector<cv::Point3_<OtherScalarT> > & cv_points)
    {
      PointMatrix<ScalarT, 3> point_matrix(cv_points.size(), 1);
      toPointMatrix(cv_points, point_matrix);
      return point_matrix;
    }

template <typename ScalarT>
  template <int Dimension, int Elements>
    void OpenCVConversion<ScalarT>::toOpenCV(const Eigen::Matrix<ScalarT, Dimension, Elements> & in,
                                             cv::Mat_<cv::Vec<ScalarT, Dimension> > & out)
    {
      out = cv::Mat_<cv::Vec<ScalarT, Dimension> >(in.cols(), 1);
      for (int i = 0; i < in.cols(); ++i)
        out(i) = cv::Vec<ScalarT, Dimension>(in.col(i).data());
    }

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_IMPL_BASE_OPENCV_CONVERSION_HPP_ */
