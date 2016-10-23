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

/**
 * @brief The RGBDData class
 */
class RGBDData
{
public:

  typedef boost::shared_ptr<RGBDData> Ptr;
  typedef boost::shared_ptr<const RGBDData> ConstPtr;

  /**
   * @brief RGBDData
   * @param id
   */
  RGBDData(int id)
    : id_(id)
  {
    // Do nothing
  }

  /**
   * @brief RGBDData
   * @param other
   */
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

  /**
   * @brief id
   * @return
   */
  inline int id() const
  {
    return id_;
  }

  /**
   * @brief setId
   * @param id
   */
  inline void setId(int id)
  {
    id_ = id;
  }

  /**
   * @brief colorData
   * @return
   */
  inline const cv::Mat & colorData() const
  {
    return color_data_;
  }

  /**
   * @brief setColorData
   * @param color_data
   */
  inline void setColorData(const cv::Mat & color_data)
  {
    color_data_ = color_data.clone();
  }

  /**
   * @brief depthData
   * @return
   */
  inline const PCLCloud3::Ptr & depthData() const
  {
    return depth_data_;
  }

  /**
   * @brief setDepthData
   * @param depth_data
   */
  void setDepthData(const PCLCloud3 & depth_data);

  /**
   * @brief fuseData
   */
  void fuseData() const;

  /**
   * @brief fusedData
   * @return
   */
  PCLCloudRGB::Ptr fusedData() const;

  inline const PCLCloud3::Ptr & registeredDepthData () const
  {
    return registered_depth_data_;
  }

  /**
   * @brief colorSensor
   * @return
   */
  inline const PinholeSensor::ConstPtr & colorSensor() const
  {
    return color_sensor_;
  }

  /**
   * @brief setColorSensor
   * @param color_sensor
   */
  inline void setColorSensor(const PinholeSensor::ConstPtr & color_sensor)
  {
    color_sensor_ = color_sensor;
  }

  /**
   * @brief depthSensor
   * @return
   */
  inline const DepthSensor::ConstPtr & depthSensor() const
  {
    return depth_sensor_;
  }

  /**
   * @brief setDepthSensor
   * @param depth_sensor
   */
  inline void setDepthSensor(const DepthSensor::ConstPtr & depth_sensor)
  {
    depth_sensor_ = depth_sensor;
  }

private:

  int id_;

  PinholeSensor::ConstPtr color_sensor_;
  cv::Mat color_data_;

  DepthSensor::ConstPtr depth_sensor_;
  PCLCloud3::Ptr depth_data_;
  PCLCloud3::Ptr registered_depth_data_;

  PCLCloudRGB::Ptr fused_data_;

};

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_RGBD_DATA_H_ */
