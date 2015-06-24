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

#include <calibration_common/objects/checkerboard.h>
#include <calibration_common/pinhole/pinhole.h>
#include <calibration_common/depth/depth.h>

//#include <calibration_pcl/utilities/point_plane_extraction.h>
#include <calibration_pcl/depth/view.h>

#include <calibration_algorithms/checkerboard_corners_extraction.h>
#include <calibration_algorithms/checkerboard_extraction.h>

namespace unipd
{
namespace calib
{

void
CheckerboardExtraction::setExtractionNumber (int number)
{
  assert(number >= 0);
  number_ = number;
}

void
CheckerboardExtraction::setImage (const cv::Mat & image)
{
  image_ = image;
}

void
CheckerboardExtraction::setCloud (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud)
{
  cloud_ = cloud;
}

void
CheckerboardExtraction::setColorSensor (const std::shared_ptr<const PinholeSensor> & sensor)
{
  color_sensor_= sensor;
}

void
CheckerboardExtraction::setDepthSensor (const std::shared_ptr<const PinholeDepthSensor> & sensor)
{
  depth_sensor_= sensor;
}

void
CheckerboardExtraction::setCheckerboard (const std::shared_ptr<const Checkerboard> & checkerboard)
{
  checkerboard_ = checkerboard;
}

void
CheckerboardExtraction::setColorToDepthTransform (const Transform3 & transform)
{
  color_to_depth_transform_ = transform;
}

auto
CheckerboardExtraction::performImage () const -> ImageResult
{
  assert(checkerboard_ and color_sensor_ and not image_.empty());

  CheckerboardCornersExtraction cb_extraction;
  cb_extraction.setImage(image_);
  auto result = cb_extraction.perform(*checkerboard_);
  if (not result.pattern_found)
    return ImageResult{false};

  std::stringstream ss;
  ss << color_sensor_->frameId() << "/view/" << number_;

  auto color_view = std::make_shared<PinholePointsView<Checkerboard>>();
  color_view->setId(ss.str());
  color_view->setObject(checkerboard_);
  color_view->setSensor(color_sensor_);
  color_view->setPoints(std::move(result.corners));

  return ImageResult{true, color_view};
}

//auto
//CheckerboardExtraction::performDepth (const ImageResult & image_result) const -> DepthResult
//{
//  if (not image_result.extracted)
//    return DepthResult{false};

//  assert(depth_sensor_ and cloud_);

//  PointPlaneExtraction<pcl::PointXYZ> plane_extractor;
//  plane_extractor.setInputCloud(cloud_);
//  plane_extractor.setRadius(std::min(checkerboard_->width(), checkerboard_->height()) / 1.5);

//  Point3_<float> centroid = (color_to_depth_transform_ * Checkerboard(*image_result.view).center()).cast<float>();
//  plane_extractor.setPoint(pcl::PointXYZ{centroid[0], centroid[1], centroid[2]});

//  PlaneInfo plane_info;

//  if (not plane_extractor.extract(plane_info))
//    return DepthResult{false};

//  std::stringstream ss;
//  ss << depth_sensor_->frameId() << "/view/" << number_;

//  auto depth_view = std::make_shared<DepthViewPCL<PlanarObject>>();
//  depth_view->setId(ss.str());
//  depth_view->setObject(checkerboard_);
//  depth_view->setSensor(depth_sensor_);
//  depth_view->setData(cloud_);
//  depth_view->setIndices(boost2std(plane_info.indices));

//  return DepthResult{true, depth_view};
//}

auto
CheckerboardExtraction::performDepth (const ImageResult & image_result) const -> DepthResult
{
  if (not image_result.extracted)
    return DepthResult{false};

  assert(depth_sensor_ and cloud_);

  Size2 corner_index[] = {Size2{0, 0}, Size2{0, 1}, Size2{1, 1}, Size2{1, 0}};

  Cloud3 e_corners = Checkerboard(*image_result.view).externalCorners();
  e_corners.transform(color_to_depth_transform_);
  Cloud2_<float> e_corners_pixel = depth_sensor_->cameraModel().project3dToPixel(e_corners).cast<float>();

  float min_x = e_corners_pixel(corner_index[0]).x(), max_x = min_x;
  float min_y = e_corners_pixel(corner_index[0]).y(), max_y = min_y;
  for (int i = 1; i < 4; ++i)
  {
    min_x = std::min(min_x, e_corners_pixel(corner_index[i]).x());
    max_x = std::max(max_x, e_corners_pixel(corner_index[i]).x());
    min_y = std::min(min_y, e_corners_pixel(corner_index[i]).y());
    max_y = std::max(max_y, e_corners_pixel(corner_index[i]).y());
  }

  min_x = std::max(min_x, 0.0f);
  max_x = std::min(max_x, static_cast<float>(cloud_->width));
  min_y = std::max(min_y, 0.0f);
  max_y = std::min(max_y, static_cast<float>(cloud_->height));

  Line2_<float> lines[4];
  Eigen::Array4f multipliers;
  for (int i = 0; i < 4; ++i)
  {
    lines[i] = Line2_<float>::Through(e_corners_pixel(corner_index[(i + 1) % 4]), e_corners_pixel(corner_index[i]));
    multipliers[i] = (lines[i].signedDistance(e_corners_pixel(corner_index[(i + 2) % 4])) > 0) ? 1.0f : -1.0f;
  }

  std::shared_ptr<Indices1> indices = std::make_shared<Indices1>();
  indices->reserve(static_cast<size_t>((max_x - min_x) * (max_y - min_y)));

  for (float y = std::ceil(min_y); y < max_y; ++y)
  {
    for (float x = std::ceil(min_x); x < max_x; ++x)
    {
      bool ok = true;
      for (int i = 0; i < 4; ++i)
      {
        if (lines[i].signedDistance(Eigen::Vector2f(x, y)) * multipliers[i] < 0)
        {
          ok = false;
          break;
        }
      }
      if (ok)
        indices->push_back(static_cast<Index1>(y * cloud_->width + x));
    }
  }

  std::stringstream ss;
  ss << depth_sensor_->frameId() << "/view/" << number_;

  auto depth_view = std::make_shared<DepthViewPCL<PlanarObject>>();
  depth_view->setId(ss.str());
  depth_view->setObject(checkerboard_);
  depth_view->setSensor(depth_sensor_);
  depth_view->setData(cloud_);
  depth_view->setIndices(indices);

  return DepthResult{true, depth_view};

}

} // namespace calib
} // namespace unipd
