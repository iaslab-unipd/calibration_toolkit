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

#ifndef UNIPD_CALIBRATION_MULTISENSOR_CALIBRATION_CORNER_BASED_ERRORS_H_
#define UNIPD_CALIBRATION_MULTISENSOR_CALIBRATION_CORNER_BASED_ERRORS_H_

#include <ceres/local_parameterization.h>

#include <calibration_common/objects/checkerboard.h>
#include <calibration_common/depth/sensor.h>

#include <multisensor_calibration/common.h>

namespace unipd
{
namespace calib
{

template <int RotationSize_, int TranslationSize_>
  struct Pose3Parser
  {

    template <typename ScalarT_>
      Pose3_<ScalarT_>
      toPose3 (const ScalarT_ * const data) const;

    template <typename ScalarT_>
      void
      fromPose3 (const Pose3_<ScalarT_> & pose,
                 ScalarT_ * data) const;

  };

template <>
  struct Pose3Parser<3, 3>
  {

    template <typename ScalarT_>
      Pose3_<ScalarT_>
      toPose3 (const ScalarT_ * const data) const
      {
        Eigen::Map<Vector3_<const ScalarT_>> rot_vec = Eigen::Map<Vector3_<const ScalarT_>>(data);

        if (rot_vec.isApprox(Vector3_<ScalarT_>::Zero(), ScalarT_(1e-5)))
        {
          Pose3_<ScalarT_> pose = Pose3_<ScalarT_>::Identity();
          pose.translation() = Eigen::Map<const Vector3_<ScalarT_>>(&data[3]);
          return pose;
        }
        else
        {
          Pose3_<ScalarT_> pose;
          pose.linear() = AngleAxis_<ScalarT_>(rot_vec.norm(), rot_vec.normalized()).matrix();
          pose.translation() = Eigen::Map<const Vector3_<ScalarT_>>(&data[3]);
          return pose;
        }
      }

    template <typename ScalarT_>
      void
      fromPose3 (const Pose3_<ScalarT_> & pose,
                 ScalarT_ * data) const
      {
        Eigen::Map<Eigen::Matrix<ScalarT_, 6, 1>> data_map(data);
        AngleAxis_<ScalarT_> rotation = AngleAxis_<ScalarT_>(pose.linear());
        data_map.template head<3>() = rotation.angle() * rotation.axis();
        data_map.template tail<3>() = pose.translation();
      }

  };

template <>
  struct Pose3Parser<4, 3>
  {

    template <typename ScalarT_>
      Pose3_<ScalarT_>
      toPose3 (const ScalarT_ * const data) const
      {
        Pose3_<ScalarT_> pose;
        pose.linear() = Eigen::Map<const Quaternion_<ScalarT_>>(&data[0]).matrix();
        pose.translation() = Eigen::Map<const Vector3_<ScalarT_>>(&data[4]);
        return pose;
      }

    template <typename ScalarT_>
      void
      fromPose3 (const Pose3_<ScalarT_> & pose,
                 ScalarT_ * data) const
      {
        Eigen::Map<Eigen::Matrix<ScalarT_, 7, 1>> data_map(data);
        data_map.template head<4>() = Quaternion_<ScalarT_>(pose.linear()).coeffs();
        data_map.template tail<3>() = pose.translation();
      }

  };

template <int R_, int t_>
  class Pose3DParameterization_ : public ceres::LocalParameterization
  {
  public:

    virtual
    ~Pose3DParameterization_ ();

    virtual bool
    Plus (const double * x,
          const double * delta,
          double * x_plus_delta) const;

    virtual bool
    ComputeJacobian (const double * x,
                     double * jacobian) const;

    virtual int
    GlobalSize() const;

    virtual int
    LocalSize() const;

  };

template <>
  class Pose3DParameterization_<4, 3> : public ceres::LocalParameterization
  {
  public:

    virtual
    ~Pose3DParameterization_ ()
    {
    }

    virtual bool
    Plus (const double * x,
          const double * delta,
          double * x_plus_delta) const
    {
      Eigen::Map<const Eigen::Vector3d> r_axis_delta(&delta[0]);
      double r_angle_delta = r_axis_delta.norm();

      Eigen::Quaterniond q_delta;
      q_delta.vec() = r_axis_delta / r_angle_delta * std::sin(0.5 * r_angle_delta);
      q_delta.w() = std::cos(0.5 * r_angle_delta);
      Eigen::Map<const Eigen::Vector3d> t_delta(&delta[3]);

      Eigen::Map<const Eigen::Quaterniond> q_x(&x[0]);
      Eigen::Map<const Eigen::Vector3d> t_x(&x[4]);

      Eigen::Map<Eigen::Quaterniond> q_x_plus_delta(&x_plus_delta[0]);
      Eigen::Map<Eigen::Vector3d> t_x_plus_delta(&x_plus_delta[4]);

      q_x_plus_delta = q_delta * q_x;
      t_x_plus_delta = q_delta * t_x + t_delta;

      return true;
    }

    /*
     * x = [q0x, q0y, q0z, q0w, t0x, t0y, t0z]'
     * delta = [r0x, r0y, r0z, t0x, t0y, t0z]'
     * Jacobian =
     * [ q0w/2,  q0z/2, -q0y/2, 0, 0, 0]
     * [-q0z/2,  q0w/2,  q0x/2, 0, 0, 0]
     * [ q0y/2, -q0x/2,  q0w/2, 0, 0, 0]
     * [-q0x/2, -q0y/2, -q0z/2, 0, 0, 0]
     * [     0,    t0z,   -t0y, 1, 0, 0]
     * [  -t0z,      0,    t0x, 0, 1, 0]
     * [   t0y,   -t0x,      0, 0, 0, 1]
     */
    virtual bool
    ComputeJacobian (const double * x,
                     double * jacobian) const
    {
      for (int i = 0; i < 42; ++i)
        jacobian[i] = 0;

      jacobian[0]  =  0.5 * x[3]; jacobian[1]  =  0.5 * x[2]; jacobian[2]  = -0.5 * x[1];  // NOLINT
      jacobian[6]  = -0.5 * x[2]; jacobian[7]  =  0.5 * x[3]; jacobian[8]  =  0.5 * x[0];  // NOLINT
      jacobian[12] =  0.5 * x[1]; jacobian[13] = -0.5 * x[0]; jacobian[14] =  0.5 * x[3];  // NOLINT
      jacobian[18] = -0.5 * x[0]; jacobian[10] = -0.5 * x[1]; jacobian[11] = -0.5 * x[2];  // NOLINT

      jacobian[25] =        x[6]; jacobian[26] =       -x[5]; jacobian[27] =  1.0       ;  // NOLINT
      jacobian[30] =       -x[6]; jacobian[32] =        x[4]; jacobian[34] =  1.0       ;  // NOLINT
      jacobian[36] =        x[5]; jacobian[37] =       -x[4]; jacobian[41] =  1.0       ;  // NOLINT

      return true;
    }

    virtual int
    GlobalSize() const
    {
      return 7;
    }

    virtual int
    LocalSize() const
    {
      return 6;
    }

  };

template <int R_, int t_>
  class BaseError_
  {
  public:

    BaseError_ (const std::shared_ptr<const Checkerboard> & checkerboard)
      : checkerboard_(checkerboard),
        parser_()
    {
    }

    virtual
    ~BaseError_ () {}

    virtual int
    residualSize () const = 0;

  protected:

    const std::shared_ptr<const Checkerboard> checkerboard_;
    const Pose3Parser<R_, t_> parser_;

  };

template <int R_, int t_>
  class IntensityError_ : public BaseError_<R_, t_>
  {
  public:

    IntensityError_ (const std::shared_ptr<const Checkerboard> & checkerboard,
                     const std::shared_ptr<const PinholeSensor> & sensor,
                     const std::shared_ptr<const IntensityData> & data)
      : BaseError_<R_, t_>(checkerboard),
        sensor_(sensor),
        data_(data)
    {
    }

    virtual
    ~IntensityError_ () {}

    template <typename T>
      bool operator () (const T * const sensor_transform_ptr,
                        const T * const checkerboard_transform_ptr,
                        T * residuals) const
      {
        Pose3_<T> sensor_pose = parser_.toPose3(sensor_transform_ptr) * sensor_->pose().cast<T>();
        Pose3_<T> checkerboard_transform = parser_.toPose3(checkerboard_transform_ptr);
        Cloud3_<T> corners = checkerboard_->corners().template cast<T>();
        corners.transform(sensor_pose.inverse() * checkerboard_transform);
        Cloud2_<T> reprojected_corners = sensor_->cameraModel().project3dToPixel<T>(corners);
        reprojected_corners = sensor_->cameraModel().distortPoints<T>(reprojected_corners);

        Eigen::Map<typename Cloud2_<T>::Container> residuals_map(residuals, 2, data_->corners.elements());
        residuals_map = (reprojected_corners.container() - data_->corners.container().cast<T>()) / T(0.2);
        residuals_map.cwiseAbs();

        return true;
      }

    virtual int
    residualSize () const override
    {
      return 2 * checkerboard_->corners().elements();
    }

  protected:

    using BaseError_<R_, t_>::checkerboard_;
    using BaseError_<R_, t_>::parser_;
    const std::shared_ptr<const PinholeSensor> sensor_;
    const std::shared_ptr<const IntensityData> data_;

  };

template <int R_, int t_>
  class DepthError_ : public BaseError_<R_, t_>
  {
  public:

    DepthError_ (const std::shared_ptr<const Checkerboard> & checkerboard,
                 const std::shared_ptr<const DepthSensor> & sensor,
                 const std::shared_ptr<const DepthData> & data)
      : BaseError_<R_, t_>(checkerboard),
        sensor_(sensor),
        data_(data)
    {
    }

    virtual
    ~DepthError_ () {}

    template <typename T>
      bool operator () (const T * const sensor_transform_ptr,
                        const T * const checkerboard_transform_ptr,
                        T * residuals) const
      {
        Pose3_<T> sensor_pose = parser_.toPose3(sensor_transform_ptr) * sensor_->pose().cast<T>();
        Pose3_<T> checkerboard_transform = parser_.toPose3(checkerboard_transform_ptr);
        Cloud3_<T> corners = checkerboard_->corners().template cast<T>();

        corners.transform(sensor_pose.inverse() * checkerboard_transform);

        Plane3_<T> depth_plane = data_->plane.cast<T>();
        PolynomialX_<T> error = sensor_->error().cast<T>();

        Eigen::Map<typename Cloud3_<T>::Container> residuals_map(residuals, 3, corners.elements());
        for (Size1 i = 0; i < corners.elements(); ++i)
        {
          Line3_<T> line = Line3_<T>(Point3_<T>::Zero(), corners[i]);
          Point3_<T> diff = (line.intersectionPoint(depth_plane) - corners[i]);
//          diff /= error.evaluate(corners[i].z());
//          residuals[3 * i] = diff.x();
//          residuals[3 * i + 1] = diff.y();
//          residuals[3 * i + 2] = diff.z();
          residuals_map.col(i) = diff / error.evaluate(corners[i].z());
        }

        return true;
      }

    virtual int
    residualSize () const override
    {
      return 3 * checkerboard_->corners().elements();
    }

  protected:

    using BaseError_<R_, t_>::checkerboard_;
    using BaseError_<R_, t_>::parser_;
    const std::shared_ptr<const DepthSensor> sensor_;
    const std::shared_ptr<const DepthData> data_;

  };

} // namespace calib
} // namespace unipd

#endif // UNIPD_CALIBRATION_MULTISENSOR_CALIBRATION_CORNER_BASED_ERRORS_H_
