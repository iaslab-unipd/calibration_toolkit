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

#ifndef UNIPD_CALIBRATION_CALIBRATION_MSGS_CALIBRATION_COMMON_CONVERSIONS_H_
#define UNIPD_CALIBRATION_CALIBRATION_MSGS_CALIBRATION_COMMON_CONVERSIONS_H_

#include <calibration_common/objects/objects.h>
#include <calibration_common/base/polynomial.h>
#include <calibration_common/pinhole/view.h>

#include <calibration_msgs/Checkerboard.h>
#include <calibration_msgs/Point2DArray.h>
#include <calibration_msgs/PointArray.h>
#include <calibration_msgs/Polynomial.h>

namespace unipd
{
namespace calib
{

Checkerboard
fromMessage (const calibration_msgs::Checkerboard & msg);

Checkerboard
fromMessage (const calibration_msgs::Checkerboard & msg,
             const std::shared_ptr<BaseObject> & parent);

calibration_msgs::Checkerboard
toMessage (const Checkerboard & checkerboard);

Cloud2
fromMessage (const calibration_msgs::Point2DArray & msg);

Cloud3
fromMessage (const calibration_msgs::PointArray & msg);

const calibration_msgs::Point2DArray
toMessage (const Cloud2 & points);

const calibration_msgs::PointArray
toMessage (const Cloud3 & points);

template <typename PinholeSensorT_>
  std::shared_ptr<PinholeSensorT_>
  fromMessage (const sensor_msgs::CameraInfo & msg)
  {
    auto sensor = std::make_shared<PinholeSensorT_>(msg.header.frame_id);
    sensor->setCameraModel(typename SensorTraits<PinholeSensorT_>::CameraModel(msg));
    return sensor;
  }

template <typename PolynomialT_, typename CoefficientsT_>
  calibration_msgs::Polynomial
  toMessage (const PolynomialT_ & polynomial,
             const CoefficientsT_ & coefficients)
  {
    assert(coefficients.size() == polynomial.size());
    calibration_msgs::Polynomial msg;
    msg.max_degree = polynomial.maxDegree();
    msg.min_degree = polynomial.minDegree();
    msg.coefficients.resize(coefficients.size());
    for (decltype(coefficients.size()) i = 0; i < coefficients.size(); ++i)
      msg.coefficients[i] = coefficients[i];
    return msg;
  }

template <typename ScalarT_>
  PolynomialX_<ScalarT_>
  fromMessage (const calibration_msgs::Polynomial & msg)
  {
    assert(msg.coefficients.size() == computePolynomialSize(msg.max_degree, msg.min_degree));
    PolynomialX_<ScalarT_> poly = PolynomialX_<ScalarT_>(msg.max_degree, msg.min_degree);
    poly.setCoefficients(PolynomialX_<double>::Coefficients::Map(msg.coefficients.data(), msg.coefficients.size()).template cast<ScalarT_>());
    return poly;
  }


} // namespace calib
} // namespace unipd

#endif // UNIPD_CALIBRATION_CALIBRATION_MSGS_CALIBRATION_COMMON_CONVERSIONS_H_
