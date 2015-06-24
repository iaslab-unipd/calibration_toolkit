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

#ifndef UNIPD_CALIBRATION_CALIBRATION_COMMON_BASE_OPENCV_EIGEN_CONVERSIONS_H_
#define UNIPD_CALIBRATION_CALIBRATION_COMMON_BASE_OPENCV_EIGEN_CONVERSIONS_H_

#include <opencv2/core/core.hpp>
#include <Eigen/Dense>

namespace unipd
{
namespace calib
{

template <typename ScalarT_, int Dimension_, int Elements_>
  cv::Mat_<cv::Vec<ScalarT_, Dimension_>>
  eigen2opencv (const Eigen::Matrix<ScalarT_, Dimension_, Elements_> & in)
  {
    auto out = cv::Mat_<cv::Vec<ScalarT_, Dimension_>>(in.cols(), 1);
    for (int i = 0; i < in.cols(); ++i)
      out(i) = cv::Vec<ScalarT_, Dimension_>(in.col(i).data());
    return out;
  }

template <typename EigenT_, typename ScalarT_>
  EigenT_
  opencv2eigen (const std::vector<cv::Point_<ScalarT_>> & in)
  {
    auto out = EigenT_(2, in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
      const auto & p = in[i];
      out.col(i) << p.x, p.y;
    }
    return out;
  }

} // namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_CALIBRATION_COMMON_BASE_OPENCV_EIGEN_CONVERSIONS_H_
