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

#ifndef CALIBRATION_COMMON_ALGORITHMS_INTERACTIVE_CHECKERBOARD_FINDER_H_
#define CALIBRATION_COMMON_ALGORITHMS_INTERACTIVE_CHECKERBOARD_FINDER_H_

#include <calibration_common/objects/checkerboard.h>
#include <opencv2/core/core.hpp>
#include <vector>

namespace calibration
{

/**
 * @brief Interactively extract checkerboard corners from an image.\n
 * Firstly the user should draw a rectangle around the checkerboard and press any key to search for the checkerboard inside.
 * If the corners are not found, the user is asked to click manually over them in row-major.
 * @todo Make the class derive from CheckerboardFinder.
 */
class InteractiveCheckerboardFinder
{
public:

  /**
   * @brief Set the image where to look for the checkerboard.
   * @param image The image where to look for the checkerboard.
   */
  inline void setImage(const cv::Mat & image)
  {
    image_ = image;
  }

  /**
   * @brief Extract the corners of the given checkerboard.
   * @param [in] checkerboard The checkerboard to look for in the image.
   * @param [out] corners The extracted corners.
   * @param [in] try_automatically Set whether the function should try to extract the corners automatically.
   * @see AutomaticCheckerboardFinder
   * @return @c true if the checkerboard is found in the image, @c false otherwise.
   */
  bool find(const Checkerboard & checkerboard,
            std::vector<cv::Point2d> & corners,
            bool try_automatically = true);

  /**
   * @brief Extract the corners of the given checkerboard.
   * @param [in] checkerboard The checkerboard to look for in the image.
   * @param [out] corners The extracted corners.
   * @param [in] try_automatically Set whether the function should try to extract the corners automatically.
   * @see AutomaticCheckerboardFinder
   * @return @c true if the checkerboard is found in the image, @c false otherwise.
   */
  bool find(const Checkerboard & checkerboard,
            Cloud2 & corners,
            bool try_automatically = true);

  static void selectCornersCallback(int event,
                                    int x,
                                    int y,
                                    int flags,
                                    void * param);

  static void selectSubImageCallback(int event,
                                     int x,
                                     int y,
                                     int flags,
                                     void * param);

protected:

  cv::Mat image_;                          ///< The image.
  cv::Rect sub_image_rect_;                ///< The selected ROI.
  std::vector<cv::Point2f> corners_float_; ///< The selected corners.

};

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_ALGORITHMS_INTERACTIVE_CHECKERBOARD_FINDER_H_ */
