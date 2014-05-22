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

#ifndef CALIBRATION_COMMON_ALGORITHMS_AUTOMATIC_CHECKERBOARD_FINDER_H_
#define CALIBRATION_COMMON_ALGORITHMS_AUTOMATIC_CHECKERBOARD_FINDER_H_

#include <calibration_common/algorithms/checkerboard_finder.h>


/**
 * @namespace calibration
 */
namespace calibration
{

/**
 * @addtogroup Algorithms
 * @{
 */

/**
 * @brief Automatically extract checkerboard corners from an image.
 */
class AutomaticCheckerboardFinder : public CheckerboardFinder
{
public:

  typedef boost::shared_ptr<AutomaticCheckerboardFinder> Ptr;
  typedef boost::shared_ptr<const AutomaticCheckerboardFinder> ConstPtr;

  virtual ~AutomaticCheckerboardFinder()
  {
    // Do nothing
  }

  using CheckerboardFinder::find;

  /**
   * @brief Extract the corners of the given checkerboard.
   * @param [in] checkerboard The checkerboard to look for in the image.
   * @param [out] corners The extracted corners.
   * @return @c true if the checkerboard is found in the image, @c false otherwise.
   */
  virtual bool find(const Checkerboard & checkerboard,
                    std::vector<cv::Point2f> & corners) const;

};

/**
 * @}
 */

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_ALGORITHMS_AUTOMATIC_CHECKERBOARD_FINDER_H_ */
