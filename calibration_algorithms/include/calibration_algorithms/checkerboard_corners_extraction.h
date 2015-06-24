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

#ifndef UNIPD_CALIBRATION_CALIBRATION_ALGORITHMS_CHECKERBOARD_CORNERS_EXTRACTION_H_
#define UNIPD_CALIBRATION_CALIBRATION_ALGORITHMS_CHECKERBOARD_CORNERS_EXTRACTION_H_

#include <opencv2/core/core.hpp>
#include <calibration_common/base/eigen_cloud.h>

namespace unipd
{
namespace calib
{

class Checkerboard;

/**
 * @brief Automatically extract checkerboard corners from an image.
 */
class CheckerboardCornersExtraction
{
public:

  struct ExtractedCorners
  {
    bool pattern_found;
    Cloud2 corners;
  };

  void
  setImage(const cv::Mat & image);

//  bool
//  perform(const Checkerboard & checkerboard,
//          std::vector<cv::Point2f> & corners) const;

  ExtractedCorners
  perform(const Checkerboard & checkerboard) const;

protected:

  cv::Mat image_; ///< The image.
  cv::Mat gray_;  ///< The grayscale image.

};


} // namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_CALIBRATION_ALGORITHMS_CHECKERBOARD_CORNERS_EXTRACTION_H_
