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

#ifndef IMPL_CALIBRATION_COMMON_PCL_UTILS_HPP_
#define IMPL_CALIBRATION_COMMON_PCL_UTILS_HPP_

#include <omp.h>
#include <calibration_common/pcl/utils.h>
#include <image_geometry/pinhole_camera_model.h>

namespace calibration
{

template <typename Scalar_, typename PCLPoint_>
  void convertToPointCloud(const sensor_msgs::Image & depth_msg,
                           const sensor_msgs::CameraInfo & info_msg,
                           typename pcl::PointCloud<PCLPoint_>::Ptr & cloud)
  {
    image_geometry::PinholeCameraModel camera_model;
    camera_model.fromCameraInfo(info_msg);

    if (not cloud)
      cloud = boost::make_shared<pcl::PointCloud<PCLPoint_> >();

    cloud->header = pcl_conversions::toPCL(depth_msg.header);
    cloud->height = depth_msg.height;
    cloud->width = depth_msg.width;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = SensorDepthTraits<Scalar_>::toMeters(Scalar_(1));
    float constant_x = unit_scaling / camera_model.fx();
    float constant_y = unit_scaling / camera_model.fy();
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    typename pcl::PointCloud<PCLPoint_>::iterator pt_iter = cloud->begin();
    const Scalar_ * depth_row = reinterpret_cast<const Scalar_ *>(&depth_msg.data[0]);
    int row_step = depth_msg.step / sizeof(Scalar_);

#pragma omp parallel for
    for (int v = 0; v < (int)cloud->height; ++v, depth_row += row_step)
    {
      for (int u = 0; u < (int)cloud->width; ++u)
      {
        PCLPoint_ & pt = cloud->points[u + v * cloud->width];
        Scalar_ depth = depth_row[u];

        // Missing points denoted by NaNs
        if (not SensorDepthTraits<Scalar_>::valid(depth))
        {
          pt.x = pt.y = pt.z = bad_point;
          continue;
        }

        // Fill in XYZ
        pt.x = (u - camera_model.cx()) * depth * constant_x;
        pt.y = (v - camera_model.cy()) * depth * constant_y;
        pt.z = SensorDepthTraits<Scalar_>::toMeters(depth);
      }
    }
  }

} /* namespace calibration */

#endif /* IMPL_CALIBRATION_COMMON_PCL_UTILS_HPP_ */
