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

#ifndef CALIBRATION_COMMON_RGBD_DATA_H_
#define CALIBRATION_COMMON_RGBD_DATA_H_

#include <opencv2/opencv.hpp>
#include <calibration_common/pinhole/sensor.h>
#include <calibration_common/depth/sensor.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace calibration
{

typedef pcl::PointXYZRGB PCLPointRGB;
typedef pcl::PointCloud<PCLPointRGB> PCLCloudRGB;

class RGBDData
{
public:

  typedef boost::shared_ptr<RGBDData> Ptr;
  typedef boost::shared_ptr<const RGBDData> ConstPtr;

  RGBDData(int id)
    : id_(id)
  {
    // Do nothing
  }

  RGBDData(const RGBDData & other)
    : id_(other.id_),
      color_data_(other.color_data_.clone()),
      depth_data_(boost::make_shared<PCLCloud3>(*other.depth_data_)),
      fused_data_(boost::make_shared<PCLCloudRGB>(*other.fused_data_)),
      color_sensor_(other.color_sensor_),
      depth_sensor_(other.depth_sensor_)
  {
    // Do nothing
  }

  int id() const
  {
    return id_;
  }

  void setId(int id)
  {
    id_ = id;
  }

  const cv::Mat & colorData() const
  {
    return color_data_;
  }

  void setColorData(const cv::Mat & color_data)
  {
    color_data_ = color_data.clone();
  }

  const PCLCloud3::Ptr & depthData() const
  {
    return depth_data_;
  }

  void setDepthData(const PCLCloud3 & depth_data);

  void fuseData() const;

  PCLCloudRGB::Ptr fusedData() const;

  const PinholeSensor::ConstPtr & colorSensor() const
  {
    return color_sensor_;
  }

  void setColorSensor(const PinholeSensor::ConstPtr & color_sensor)
  {
    color_sensor_ = color_sensor;
  }

  const DepthSensor::ConstPtr & depthSensor() const
  {
    return depth_sensor_;
  }

  void setDepthSensor(const DepthSensor::ConstPtr & depth_sensor)
  {
    depth_sensor_ = depth_sensor;
  }

private:

  int id_;

  PinholeSensor::ConstPtr color_sensor_;
  cv::Mat color_data_;

  DepthSensor::ConstPtr depth_sensor_;
  PCLCloud3::Ptr depth_data_;

  PCLCloudRGB::Ptr fused_data_;

};

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_RGBD_DATA_H_ */
