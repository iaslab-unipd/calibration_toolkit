/*
 *  Copyright (c) 2015-, Filippo Basso <bassofil@gmail.com>
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

#ifndef UNIPD_CALIBRATION_CALIBRATION_ALGORITHMS_CHECKERBOARD_EXTRACTION_H_
#define UNIPD_CALIBRATION_CALIBRATION_ALGORITHMS_CHECKERBOARD_EXTRACTION_H_

#include <opencv2/core/core.hpp>
#include <calibration_common/base/geometry.h>
#include <calibration_pcl/base/definitions.h>

namespace unipd
{
namespace calib
{
class PlanarObject;
class Checkerboard;
class CheckerboardCorners;
class PinholeSensor;
class PinholeDepthSensor;

template <typename ObjectT_>
  class PinholePointsView;

template <typename ObjectT_>
  class DepthViewPCL;

class CheckerboardExtraction
{
public:

  struct ImageResult
  {
    bool extracted;
    std::shared_ptr<PinholePointsView<Checkerboard>> view;
  };

  struct DepthResult
  {
    bool extracted;
    std::shared_ptr<DepthViewPCL<PlanarObject>> view;
  };

  CheckerboardExtraction ()
    : color_to_depth_transform_(Transform3::Identity())
  {
    // Do nothing
  }

  void
  setExtractionNumber (int number);

  void
  setImage (const cv::Mat & image);

  void
  setCloud (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud);

  void
  setColorSensor (const std::shared_ptr<const PinholeSensor> & sensor);

  void
  setDepthSensor (const std::shared_ptr<const PinholeDepthSensor> & sensor);

  void
  setCheckerboard (const std::shared_ptr<const Checkerboard> & checkerboard);

  void
  setColorToDepthTransform (const Transform3 & transform);

  ImageResult
  performImage () const;

  DepthResult
  performDepth (const ImageResult & image_result) const;

private:

  int number_ = 0;

  cv::Mat image_;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_;

  std::shared_ptr<const PinholeSensor> color_sensor_;
  std::shared_ptr<const PinholeDepthSensor> depth_sensor_;
  Transform3 color_to_depth_transform_;

  std::shared_ptr<const Checkerboard> checkerboard_;

};

} // namespace calib
} // namespace unipd

#endif // UNIPD_CALIBRATION_CALIBRATION_ALGORITHMS_CHECKERBOARD_EXTRACTION_H_
