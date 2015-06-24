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

#ifndef UNIPD_CALIBRATION_MULTISENSOR_CALIBRATION_CORNER_BASED_PLANE_ERRORS_H_
#define UNIPD_CALIBRATION_MULTISENSOR_CALIBRATION_CORNER_BASED_PLANE_ERRORS_H_

#include <ceres/local_parameterization.h>

#include <multisensor_calibration/corner_based/errors.h>

namespace unipd
{
namespace calib
{

template <int R_, int t_>
  class PlaneParameterization_ : public ceres::LocalParameterization
  {
  public:

    virtual
    ~PlaneParameterization_ ();

    virtual bool
    Plus (const double * x,
          const double * delta,
          double * x_plus_delta) const override;

    virtual bool
    ComputeJacobian (const double * x,
                     double * jacobian) const override;

    virtual int
    GlobalSize() const override;

    virtual int
    LocalSize() const override;

  };

template <>
  class PlaneParameterization_<4, 3> : public ceres::LocalParameterization
  {
  public:

    virtual
    ~PlaneParameterization_ ()
    {
    }

    virtual bool
    Plus (const double * x,
          const double * delta, // delta = [alpha, angle, delta_z], axis = [cos(alpha), sin(alpha), 0]
          double * x_plus_delta) const override
    {
      Eigen::Quaterniond q_delta(std::cos(0.5 * delta[1]),                      // w
                                 std::cos(delta[0]) * std::sin(0.5 * delta[1]), // x
                                 std::sin(delta[0]) * std::sin(0.5 * delta[1]), // y
                                 0);                                            // z
      Eigen::Vector3d t_delta(0, 0, delta[2]);

      Eigen::Map<const Eigen::Quaterniond> q_x(&x[0]);
      Eigen::Map<const Eigen::Vector3d> t_x(&x[4]);

      Eigen::Map<Eigen::Quaterniond> q_x_plus_delta(&x_plus_delta[0]);
      Eigen::Map<Eigen::Vector3d> t_x_plus_delta(&x_plus_delta[4]);

      q_x_plus_delta = q_delta * q_x;
      t_x_plus_delta = q_delta * t_x + t_delta;

      return true;
    }

    virtual bool
    ComputeJacobian (const double * x,
                     double * jacobian) const override
    {
      jacobian[0]  = 0; jacobian[1]  =  0.5 * x[3]; jacobian[2]  = 0;  // NOLINT
      jacobian[3]  = 0; jacobian[4]  = -0.5 * x[2]; jacobian[5]  = 0;  // NOLINT
      jacobian[6]  = 0; jacobian[7]  =  0.5 * x[1]; jacobian[8]  = 0;  // NOLINT
      jacobian[9]  = 0; jacobian[10] = -0.5 * x[0]; jacobian[11] = 0;  // NOLINT
      jacobian[12] = 0; jacobian[13] =  0         ; jacobian[14] = 0;  // NOLINT
      jacobian[15] = 0; jacobian[16] =       -x[6]; jacobian[17] = 0;  // NOLINT
      jacobian[18] = 0; jacobian[19] =        x[5]; jacobian[20] = 1;  // NOLINT
      return true;
    }

    virtual int
    GlobalSize() const override
    {
      return 7;
    }

    virtual int
    LocalSize() const override
    {
      return 3;
    }

  };

template <int R_, int t_>
  class Pose2DParameterization_ : public ceres::LocalParameterization
  {
  public:

    virtual
    ~Pose2DParameterization_ ();

    virtual bool
    Plus (const double * x,
          const double * delta,
          double * x_plus_delta) const override;

    virtual bool
    ComputeJacobian (const double * x,
                     double * jacobian) const override;

    virtual int
    GlobalSize() const override;

    virtual int
    LocalSize() const override;

  };

template <>
  class Pose2DParameterization_<4, 3> : public ceres::LocalParameterization
  {
  public:

    virtual
    ~Pose2DParameterization_ ()
    {
    }

    virtual bool
    Plus (const double * x,
          const double * delta,
          double * x_plus_delta) const override
    {
      Eigen::Quaterniond q_delta(std::cos(0.5 * delta[2]), 0, 0, std::sin(0.5 * delta[2])); // [w, x, y, z]
      Eigen::Vector3d t_delta(delta[0], delta[1], 0);

      Eigen::Map<const Eigen::Quaterniond> q_x(&x[0]);
      Eigen::Map<const Eigen::Vector3d> t_x(&x[4]);

      Eigen::Map<Eigen::Quaterniond> q_x_plus_delta(&x_plus_delta[0]);
      Eigen::Map<Eigen::Vector3d> t_x_plus_delta(&x_plus_delta[4]);

      q_x_plus_delta = q_delta * q_x;
      t_x_plus_delta = q_delta * t_x + t_delta;

      return true;
    }

    virtual bool
    ComputeJacobian (const double * x,
                     double * jacobian) const override
    {
      jacobian[0]  = 0; jacobian[1]  = 0; jacobian[2]  = -0.5 * x[1];  // NOLINT
      jacobian[3]  = 0; jacobian[4]  = 0; jacobian[5]  =  0.5 * x[0];  // NOLINT
      jacobian[6]  = 0; jacobian[7]  = 0; jacobian[8]  =  0.5 * x[3];  // NOLINT
      jacobian[9]  = 0; jacobian[10] = 0; jacobian[11] = -0.5 * x[2];  // NOLINT
      jacobian[12] = 1; jacobian[13] = 0; jacobian[14] =       -x[5];  // NOLINT
      jacobian[15] = 0; jacobian[16] = 1; jacobian[17] =        x[4];  // NOLINT
      jacobian[18] = 0; jacobian[19] = 0; jacobian[20] =  0         ;  // NOLINT
      return true;
    }

    virtual int
    GlobalSize() const override
    {
      return 7;
    }

    virtual int
    LocalSize() const override
    {
      return 3;
    }

  };

template <int R_, int t_>
  class OnPlaneIntensityError_ : public IntensityError_<R_, t_>
  {
  public:

    OnPlaneIntensityError_ (const std::shared_ptr<const Checkerboard> & checkerboard,
                            const std::shared_ptr<const PlanarObject> & plane,
                            const std::shared_ptr<const PinholeSensor> & sensor,
                            const std::shared_ptr<const IntensityData> & data)
      : IntensityError_<R_, t_>(checkerboard, sensor, data),
        plane_(plane)
    {
    }

    virtual
    ~OnPlaneIntensityError_ () {}

    template <typename T>
      bool
      operator () (const T * const sensor_transform_ptr,
                   const T * const checkerboard_transform_ptr,
                   const T * const plane_transform_ptr,
                   T * residuals) const
      {
        Pose3_<T> sensor_pose = parser_.toPose3(sensor_transform_ptr) * sensor_->pose().template cast<T>();
        Pose3_<T> checkerboard_transform = parser_.toPose3(checkerboard_transform_ptr);
        Pose3_<T> plane_pose = parser_.toPose3(plane_transform_ptr) * plane_->pose().template cast<T>();
        Cloud3_<T> corners = checkerboard_->corners().template cast<T>();
        corners.transform(sensor_pose.inverse() * plane_pose * checkerboard_transform);
        Cloud2_<T> reprojected_corners = sensor_->cameraModel().template project3dToPixel<T>(corners);
        reprojected_corners = sensor_->cameraModel().template distortPoints<T>(reprojected_corners);

        Eigen::Map<typename Cloud2_<T>::Container> residuals_map(residuals, 2, data_->corners.elements());
        residuals_map = (reprojected_corners.container() - data_->corners.container().template cast<T>()) / T(0.5);

        return true;
      }

  protected:

    using IntensityError_<R_, t_>::checkerboard_;
    using IntensityError_<R_, t_>::parser_;
    using IntensityError_<R_, t_>::sensor_;
    using IntensityError_<R_, t_>::data_;
    const std::shared_ptr<const PlanarObject> plane_;

  };

template <int R_, int t_>
  class OnPlaneDepthError_ : public DepthError_<R_, t_>
  {
  public:

    OnPlaneDepthError_ (const std::shared_ptr<const Checkerboard> & checkerboard,
                        const std::shared_ptr<const PlanarObject> & plane,
                        const std::shared_ptr<const PinholeSensor> & sensor,
                        const std::shared_ptr<const IntensityData> & data)
      : DepthError_<R_, t_>(checkerboard, sensor, data),
        plane_(plane)
    {
    }

    virtual
    ~OnPlaneDepthError_ () {}

    template <typename T>
      bool
      operator () (const T * const sensor_transform_ptr,
                   const T * const checkerboard_transform_ptr,
                   const T * const plane_transform_ptr,
                   T * residuals) const
      {
        Pose3_<T> sensor_pose = parser_.toPose3(sensor_transform_ptr) * sensor_->pose().template cast<T>();
        Pose3_<T> checkerboard_transform = parser_.toPose3(checkerboard_transform_ptr);
        Pose3_<T> plane_pose = parser_.toPose3(plane_transform_ptr) * plane_->pose().template cast<T>();
        Cloud3_<T> corners = checkerboard_->corners().template cast<T>();
        corners.transform(sensor_pose.inverse() * plane_pose * checkerboard_transform);

        Plane3_<T> depth_plane = data_->plane.cast<T>();
        PolynomialX_<T> error = sensor_->error().cast<T>();

        Eigen::Map<typename Cloud3_<T>::Container> residuals_map(residuals, 3, corners.elements());
        for (Size1 i = 0; i < corners.elements(); ++i)
        {
          Line3_<T> line = Line3_<T>(Point3_<T>::Zero(), corners[i]);
          Point3_<T> diff = (line.intersectionPoint(depth_plane) - corners[i]);
          residuals_map.col(i) = diff / error.evaluate(corners[i].z());
        }

        return true;
      }

  protected:

    using DepthError_<R_, t_>::checkerboard_;
    using DepthError_<R_, t_>::parser_;
    using DepthError_<R_, t_>::sensor_;
    using DepthError_<R_, t_>::data_;
    const std::shared_ptr<const PlanarObject> plane_;

  };

} // namespace calib
} // namespace unipd

#endif // UNIPD_CALIBRATION_MULTISENSOR_CALIBRATION_CORNER_BASED_PLANE_ERRORS_H_
