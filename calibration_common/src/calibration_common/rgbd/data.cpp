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

#include <calibration_common/rgbd/data.h>
#include <calibration_common/base/pcl_conversion.h>
#include <omp.h>

namespace calibration
{

void RGBDData::setDepthData(const PCLCloud3 & depth_data)
{
  assert(depth_sensor_);
  depth_data_ = boost::make_shared<PCLCloud3>(depth_data);
  depth_data_->header.frame_id = depth_sensor_->frameId();

  fused_data_ = boost::make_shared<PCLCloudRGB>();
  fused_data_->width = depth_data_->width;
  fused_data_->height = depth_data_->height;
  fused_data_->is_dense = depth_data_->is_dense;
  fused_data_->header.frame_id = depth_data_->header.frame_id;

  for (size_t p_index = 0; p_index < depth_data_->points.size(); ++p_index)
  {
    const PCLPoint3 & point = depth_data_->points[p_index];

    PCLPointRGB point_rgb;
    point_rgb.x = point.x;
    point_rgb.y = point.y;
    point_rgb.z = point.z;
    point_rgb.b = 100;
    point_rgb.g = 100;
    point_rgb.r = 100;

    fused_data_->points.push_back(point_rgb);
  }

}

PCLCloudRGB::Ptr RGBDData::fusedData() const
{
  return fused_data_;
}

void RGBDData::fuseData() const
{
  assert(depth_sensor_ and color_sensor_);
  fused_data_->header.stamp = depth_data_->header.stamp;

  Transform t = color_sensor_->pose().inverse();

//  pcl::PointCloud<pcl::PointXYZ> depth_data_tmp(*depth_data_);
//  depth_sensor_->undistortion()->undistort(depth_data_tmp);

#pragma omp parallel for
  for (size_t i = 0; i < depth_data_->size(); ++i)
  {
    PCLPointRGB & point_rgb = fused_data_->points[i];
    Point3 point_eigen((*depth_data_)[i].x, (*depth_data_)[i].y, (*depth_data_)[i].z);

    point_rgb.x = point_eigen.x();
    point_rgb.y = point_eigen.y();
    point_rgb.z = point_eigen.z();

    point_eigen = t * point_eigen;
    Point2 point_image = color_sensor_->cameraModel()->project3dToPixel(point_eigen);

    int x = static_cast<int>(point_image[0]);
    int y = static_cast<int>(point_image[1]);

    if (x >= 0 and x < color_data_.cols and y >= 0 and y < color_data_.rows)
    {
      cv::Point p(x, y);
      const cv::Vec3b & image_data = color_data_.at<cv::Vec3b>(p);
      point_rgb.b = image_data[0];
      point_rgb.g = image_data[1];
      point_rgb.r = image_data[2];
    }
    else
    {
      point_rgb.b = 100;
      point_rgb.g = 100;
      point_rgb.r = 100;
    }
  }

}

} /* namespace calibration */
