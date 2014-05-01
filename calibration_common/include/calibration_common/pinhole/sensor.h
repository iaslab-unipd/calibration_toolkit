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

#ifndef CALIBRATION_COMMON_PINHOLE_SENSOR_H_
#define CALIBRATION_COMMON_PINHOLE_SENSOR_H_

#include <calibration_common/color/sensor.h>
#include <calibration_common/pinhole/camera_model.h>

namespace calibration
{

/**
 * @brief The PinholeSensor class
 */
class PinholeSensor : public ColorSensor
{
public:

  typedef boost::shared_ptr<PinholeSensor> Ptr;
  typedef boost::shared_ptr<const PinholeSensor> ConstPtr;

  /**
   * @brief PinholeSensor
   */
  PinholeSensor()
    : ColorSensor()
  {
    // Do nothing
  }

  /**
   * @brief cameraModel
   * @return
   */
  inline const PinholeCameraModel::ConstPtr & cameraModel() const
  {
    return camera_model_;
  }

  /**
   * @brief setCameraModel
   * @param camera_model
   */
  inline void setCameraModel(const PinholeCameraModel::ConstPtr & camera_model)
  {
    camera_model_ = camera_model;
  }

  /**
   * @brief estimatePose
   * @param points_image
   * @param points_object
   * @return
   */
  inline Pose estimatePose(const Cloud2 & points_image,
                           const Cloud3 & points_object) const
  {
    return camera_model_->estimatePose(points_image, points_object);
  }

  /**
   * @brief estimatePose
   * @param points_image
   * @param points_object
   * @return
   */
  template <typename ScalarT_>
    inline typename Types<ScalarT_>::Pose estimatePose(const typename Types<ScalarT_>::Cloud2 & points_image,
                                                const typename Types<ScalarT_>::Cloud3 & points_object) const
    {
      return camera_model_->estimatePose(points_image, points_object);
    }

private:

  typename PinholeCameraModel::ConstPtr camera_model_;

};

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_PINHOLE_SENSOR_H_ */
