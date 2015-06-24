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

#include <tuple>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <calibration_common/objects/checkerboard.h>
#include <calibration_common/base/opencv_eigen_conversions.h>
#include <calibration_algorithms/checkerboard_corners_extraction.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace unipd
{
namespace calib
{

void
CheckerboardCornersExtraction::setImage (const cv::Mat & image)
{
  image_ = image;
  if (image_.channels() > 1)
    cv::cvtColor(image_, gray_, CV_BGR2GRAY);
  else if (image_.depth() == CV_16U)
  {
    cv::Mat tmp;
    double min, max;
    cv::minMaxLoc(image_, &min, &max);
    image_.convertTo(tmp, CV_8U, 255.0 / max);
    cv::equalizeHist(tmp, gray_);
  }
  else
    gray_ = image_;
}

auto
CheckerboardCornersExtraction::perform (const Checkerboard & checkerboard) const -> ExtractedCorners
{
  std::vector<cv::Point2f> cv_corners;
  cv::Size pattern_size(checkerboard.cols(), checkerboard.rows());

  if (cv::findChessboardCorners(gray_, pattern_size, cv_corners, cv::CALIB_CB_FAST_CHECK))
  {
    cv::cornerSubPix(gray_, cv_corners, cv::Size(2, 2), cv::Size(-1, -1),
                     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.01));
    return ExtractedCorners{true, Cloud2(opencv2eigen<Cloud2::Container>(cv_corners), checkerboard.corners().size())};
  }
  else
  {
    return ExtractedCorners{false, Cloud2(Size2{0, 0})};
  }

}

} // namespace calib
} // namespace unipd
