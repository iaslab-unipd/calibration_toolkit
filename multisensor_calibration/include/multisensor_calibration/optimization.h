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

#ifndef UNIPD_CALIBRATION_MULTISENSOR_CALIBRATION_OPTIMIZATION_H_
#define UNIPD_CALIBRATION_MULTISENSOR_CALIBRATION_OPTIMIZATION_H_

#include <ceres/ceres.h>

#include <ros/console.h>

#include <calibration_common/objects/checkerboard.h>
#include <calibration_common/depth/sensor.h>

#include <multisensor_calibration/common.h>
#include <multisensor_calibration/corner_based/plane_errors.h>

namespace unipd
{
namespace calib
{

template <int R_, int t_>
  struct RawTransform_
  {
    RawTransform_ (const Pose3 & pose)
    {
      Pose3Parser<R_, t_> parser;
      parser.fromPose3(pose, data);
    }

    Scalar data[R_ + t_];
  };

class Optimization
{
public:

  using RawTransform = RawTransform_<4, 3>;
  using IntensityError = IntensityError_<4, 3>;
  using DepthError = DepthError_<4, 3>;
  using OnPlaneIntensityError = OnPlaneIntensityError_<4, 3>;
  using OnPlaneDepthError = OnPlaneDepthError_<4, 3>;
  using PlaneParameterization = PlaneParameterization_<4, 3>;
  using Pose2DParameterization = Pose2DParameterization_<4, 3>;
  using Pose3DParameterization = Pose3DParameterization_<4, 3>;

  void
  addObject (const std::shared_ptr<BaseObject> & object);

  void
  setVariable (const std::shared_ptr<BaseObject> & object);

  void
  setFixed (const std::shared_ptr<BaseObject> & object);

  void
  set6DOFTransform (const std::shared_ptr<BaseObject> & object);

  void
  setPlaneTransform (const std::shared_ptr<BaseObject> & object);

  void
  set3DOFTransform (const std::shared_ptr<BaseObject> & object);

  void
  addConstraint (const std::shared_ptr<Checkerboard> & checkerboard,
                 const std::shared_ptr<PinholeSensor> & sensor,
                 const std::shared_ptr<const IntensityData> & data);

  void
  addConstraint (const std::shared_ptr<Checkerboard> & checkerboard,
                 const std::shared_ptr<DepthSensor> & sensor,
                 const std::shared_ptr<const DepthData> & data);

  void
  addConstraint (const std::shared_ptr<Checkerboard> & checkerboard,
                 const std::shared_ptr<PlanarObject> & plane,
                 const std::shared_ptr<PinholeSensor> & sensor,
                 const std::shared_ptr<const IntensityData> & data);

  void
  addConstraint (const std::shared_ptr<Checkerboard> & checkerboard,
                 const std::shared_ptr<PlanarObject> & plane,
                 const std::shared_ptr<DepthSensor> & sensor,
                 const std::shared_ptr<const DepthData> & data);

  void
  perform ();

  Transform3
  estimatedTransform (const std::shared_ptr<const BaseObject> & object) const;

  void
  reset ();

private:

  std::string log_ = "[/optimization] ";

  std::map<const std::shared_ptr<const BaseObject>, std::shared_ptr<RawTransform>> transform_map_;

  ceres::Problem problem_;
  Pose3Parser<4, 3> parser_;

};

} // namespace calib
} // namespace unipd

#endif // UNIPD_CALIBRATION_MULTISENSOR_CALIBRATION_OPTIMIZATION_H_
