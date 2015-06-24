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

#ifndef UNIPD_CALIBRATION_CALIBRATION_COMMON_DEPTH_SENSOR_H_
#define UNIPD_CALIBRATION_CALIBRATION_COMMON_DEPTH_SENSOR_H_

#include <calibration_common/objects/sensor.h>
#include <calibration_common/base/polynomial.h>
#include <calibration_common/pinhole/camera_model.h>

namespace unipd
{
namespace calib
{

class DepthSensor;
class PinholeDepthSensor;

template <>
struct SensorTraits<DepthSensor>
{
  using Polynomial = PolynomialX_<Scalar>;
};

template <>
struct SensorTraits<PinholeDepthSensor> : public SensorTraits<DepthSensor>
{
  using CameraModel = PinholeCameraModel;
  using Polynomial = SensorTraits<DepthSensor>::Polynomial;
};

class DepthSensor : public Sensor
{
public:

  DepthSensor () = default;

  DepthSensor (const DepthSensor & other) = default;

  DepthSensor (DepthSensor && other) = default;

  DepthSensor & operator = (const DepthSensor & other) = default;

  DepthSensor & operator = (DepthSensor && other) = default;

  using Sensor::Sensor;

  explicit
  DepthSensor (const PolynomialX_<Scalar> & error_polynomial)
    : error_polynomial_(error_polynomial)
  {
    // Do nothing
  }

  const PolynomialX_<Scalar> &
  error () const
  {
    return error_polynomial_;
  }

  void
  setError (const PolynomialX_<Scalar> & error_polynomial)
  {
    error_polynomial_ = error_polynomial;
  }

  void
  setError (const PolynomialX_<Scalar> & error_polynomial,
            const std::vector<Scalar> & coefficients)
  {
    assert(error_polynomial.size() == coefficients.size());
    error_polynomial_ = error_polynomial;
    error_polynomial_.setCoefficients(PolynomialX_<Scalar>::Coefficients::Map(coefficients.data(), coefficients.size()));
  }

  Scalar
  estimateError (const Scalar & x)
  {
    return error_polynomial_.evaluate(x);
  }

private:

  PolynomialX_<Scalar> error_polynomial_;

};

class PinholeDepthSensor : public DepthSensor
{
public:

  PinholeDepthSensor () = default;

  PinholeDepthSensor (const PinholeDepthSensor & other) = default;

  PinholeDepthSensor (PinholeDepthSensor && other) = default;

  PinholeDepthSensor & operator = (const PinholeDepthSensor & other) = default;

  PinholeDepthSensor & operator = (PinholeDepthSensor && other) = default;

  using DepthSensor::DepthSensor;

  const PinholeCameraModel &
  cameraModel () const
  {
    return camera_model_;
  }

  void
  setCameraModel (const PinholeCameraModel & camera_model)
  {
    camera_model_ = camera_model;
  }

  void
  setCameraModel (PinholeCameraModel && camera_model)
  {
    camera_model_ = camera_model;
  }

private:

  PinholeCameraModel camera_model_;

};

} // namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_CALIBRATION_COMMON_DEPTH_SENSOR_H_
