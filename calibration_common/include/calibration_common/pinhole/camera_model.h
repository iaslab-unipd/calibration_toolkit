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

#ifndef UNIPD_CALIBRATION_CALIBRATION_COMMON_PINHOLE_CAMERA_MODEL_H_
#define UNIPD_CALIBRATION_CALIBRATION_COMMON_PINHOLE_CAMERA_MODEL_H_

#include <calibration_common/base/eigen_cloud.h>
#include <image_geometry/pinhole_camera_model.h>

namespace unipd
{
namespace calib
{

class PinholeCameraModel : public image_geometry::PinholeCameraModel
{
public:

  typedef image_geometry::PinholeCameraModel Base;

  PinholeCameraModel () = default;

  PinholeCameraModel (const PinholeCameraModel & other) = default;

  PinholeCameraModel (PinholeCameraModel && other) = default;

  PinholeCameraModel & operator = (const PinholeCameraModel & other) = default;

  PinholeCameraModel & operator = (PinholeCameraModel && other) = default;

  explicit
  PinholeCameraModel (const sensor_msgs::CameraInfo & msg)
    : PinholeCameraModel()
  {
    Base::fromCameraInfo(msg);

    f_ << fx(), fy();
    f_inv_ << 1.0 / fx(), 1.0 / fy();
    c_ << cx(), cy();
    T_ << Tx(), Ty();
    p_ << D_(0, 2), D_(0, 3);
    k_ << D_(0, 0), D_(0, 1), D_(0, 4), D_(0, 5), D_(0, 6), D_(0, 7);
    R_ = Eigen::Map<Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor>>(&Base::R_(0, 0));
  }

  template <typename ScalarT_>
    Point3_<ScalarT_>
    projectPixelTo3dRay (const Point2_<ScalarT_> & pixel_point) const;

  template <typename ScalarT_>
    Point2_<ScalarT_>
    project3dToPixel (const Point3_<ScalarT_> & world_point) const;

  template <typename ScalarT_>
    void
    projectPixelTo3dRay (const Cloud2_<ScalarT_> & pixel_points,
                         Cloud3_<ScalarT_> & world_points) const;

  template <typename ScalarT_>
    Cloud3_<ScalarT_>
    projectPixelTo3dRay (const Cloud2_<ScalarT_> & pixel_points) const;

  template <typename ScalarT_>
    void
    project3dToPixel (const Cloud3_<ScalarT_> & world_points,
                      Cloud2_<ScalarT_> & pixel_points) const;

  template <typename ScalarT_>
    Cloud2_<ScalarT_>
    project3dToPixel (const Cloud3_<ScalarT_> & world_points) const;

  template <typename ScalarT_>
    Point2_<ScalarT_>
    distortPoint (const Point2_<ScalarT_> & pixel_point) const;

  template <typename ScalarT_>
    Cloud2_<ScalarT_>
    distortPoints (const Cloud2_<ScalarT_> & pixel_points) const;

  template <typename ScalarT_>
    Pose3_<ScalarT_>
    estimatePose (const Cloud2_<ScalarT_> & points_image,
                  const Cloud3_<ScalarT_> & points_object) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:

    Eigen::Matrix<double, 2, 1> f_;
    Eigen::Matrix<double, 2, 1> f_inv_;
    Eigen::Matrix<double, 2, 1> c_;
    Eigen::Matrix<double, 2, 1> T_;
    Eigen::Matrix<double, 2, 1> p_;
    Eigen::Matrix<double, 1, 6> k_;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_;

};

} // namespace calib
} // namespace unipd

#include <impl/calibration_common/pinhole/camera_model.hpp>

#endif // UNIPD_CALIBRATION_CALIBRATION_COMMON_PINHOLE_CAMERA_MODEL_H_
