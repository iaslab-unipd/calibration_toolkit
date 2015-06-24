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

#ifndef IMPL_UNIPD_CALIBRATION_CALIBRATON_COMMON_PINHOLE_CAMERA_MODEL_HPP_
#define IMPL_UNIPD_CALIBRATION_CALIBRATON_COMMON_PINHOLE_CAMERA_MODEL_HPP_

#include <calibration_common/pinhole/camera_model.h>
#include <calibration_common/base/opencv_eigen_conversions.h>
#include <calibration_common/base/polynomial.h>

namespace unipd
{
namespace calib
{

template <typename ScalarT_>
  Point3_<ScalarT_>
  PinholeCameraModel::projectPixelTo3dRay (const Point2_<ScalarT_> & pixel_point) const
  {
    assert(initialized());

    Point3_<ScalarT_> ray;
    ray.template head<2>() = f_inv_.template cast<ScalarT_>().asDiagonal() * (pixel_point - (c_ + T_).template cast<ScalarT_>());
    ray[2] = ScalarT_(1.0);

    return ray.normalized();
  }

template <typename ScalarT_>
  Point2_<ScalarT_>
  PinholeCameraModel::project3dToPixel (const Point3_<ScalarT_> & world_point) const
  {
    assert(initialized());
    assert(P_(2, 3) == 0.0);

    Point2_<ScalarT_> pixel_point = f_.template cast<ScalarT_>().asDiagonal() * world_point.template head<2>() + T_.template cast<ScalarT_>();
    return pixel_point / world_point[2] + c_.template cast<ScalarT_>();
  }

template <typename ScalarT_>
  void
  PinholeCameraModel::projectPixelTo3dRay (const Cloud2_<ScalarT_> & pixel_points,
                                           Cloud3_<ScalarT_> & world_points) const
  {
    assert(initialized());
    assert(pixel_points.size() == world_points.size());

    world_points.container().setOnes();
    world_points.container().template topRows<2>() = f_inv_.template cast<ScalarT_>().asDiagonal() * (pixel_points.container().colwise() - (c_ + T_).template cast<ScalarT_>());
    world_points.container().colwise().normalize();
  }

template <typename ScalarT_>
  inline Cloud3_<ScalarT_>
  PinholeCameraModel::projectPixelTo3dRay (const Cloud2_<ScalarT_> & pixel_points) const
  {
    Cloud3_<ScalarT_> world_points(pixel_points.size());
    projectPixelTo3dRay<ScalarT_>(pixel_points, world_points);
    return world_points;
  }

template <typename ScalarT_>
  void
  PinholeCameraModel::project3dToPixel (const Cloud3_<ScalarT_> & world_points,
                                        Cloud2_<ScalarT_> & pixel_points) const
  {
    assert(initialized());
    assert(P_(2, 3) == 0.0);
    assert(pixel_points.size() == world_points.size());

    Eigen::Matrix<ScalarT_, 2, Eigen::Dynamic> tmp = world_points.container().template topRows<2>();

    pixel_points.container() = f_.template cast<ScalarT_>().asDiagonal() * tmp;
    pixel_points.container().colwise() += T_.template cast<ScalarT_>();
    pixel_points.container().array().rowwise() /= world_points.container().array().template bottomRows<1>();
    pixel_points.container().colwise() += c_.template cast<ScalarT_>();
  }

template <typename ScalarT_>
  inline Cloud2_<ScalarT_>
  PinholeCameraModel::project3dToPixel (const Cloud3_<ScalarT_> & world_points) const
  {
    Cloud2_<ScalarT_> pixel_points(world_points.size());
    project3dToPixel<ScalarT_>(world_points, pixel_points);
    return pixel_points;
  }

template <typename ScalarT_>
  Point2_<ScalarT_>
  PinholeCameraModel::distortPoint (const Point2_<ScalarT_> & pixel_point) const
  {
    Point2_<ScalarT_> xy = f_inv_.template cast<ScalarT_>().asDiagonal() * (pixel_point - (c_ + T_).template cast<ScalarT_>());

    Point3_<ScalarT_> XYZ = R_.transpose().template cast<ScalarT_>() * xy.homogeneous();
    Point2_<ScalarT_> xyp = XYZ.hnormalized().template head<2>();

    Point2_<ScalarT_> xyp2 = xyp.cwiseAbs2();
    Point3_<ScalarT_> r;
    r[0] = xyp2.sum();
    r[1] = r[0] * r[0];
    r[2] = r[1] * r[0];

    ScalarT_ barrel_correction = ScalarT_(1.0) + k_.template head<3>() * r;
    if (Base::D_.cols == 8)
      barrel_correction /= ScalarT_(1.0) + k_.template tail<3>() * r;

    Eigen::Matrix<ScalarT_, 2, 2> a;
    a(0, 0) = a(1, 1) = 2 * xyp.prod();
    a(0, 1) = r[0] + 2 * xyp2[0];
    a(1, 0) = r[0] + 2 * xyp2[1];

    Point2_<ScalarT_> xypp = xyp * barrel_correction + a * p_.template cast<ScalarT_>();
    return Point2_<ScalarT_>(f_.template cast<ScalarT_>().asDiagonal() * xypp + c_.template cast<ScalarT_>());
  }

template <typename ScalarT_>
  Cloud2_<ScalarT_>
  PinholeCameraModel::distortPoints (const Cloud2_<ScalarT_> & pixel_points) const
  {
    using Matrix1X = Eigen::Matrix<ScalarT_, 1, Eigen::Dynamic>;
    using Matrix2X = Eigen::Matrix<ScalarT_, 2, Eigen::Dynamic>;
    using Matrix3X = Eigen::Matrix<ScalarT_, 3, Eigen::Dynamic>;

    Matrix2X xy = f_inv_.template cast<ScalarT_>().asDiagonal()
                  * (pixel_points.container().colwise() - (c_ + T_).template cast<ScalarT_>());

    Matrix3X XYZ = R_.transpose().template cast<ScalarT_>() * xy.colwise().homogeneous();
    Matrix2X xyp = (XYZ.array().rowwise() / XYZ.array().row(2)).template topRows<2>();

    Matrix2X xyp2 = xyp.cwiseAbs2();
    Matrix3X r(3, xyp.cols());
    r.row(0) = xyp2.colwise().sum();
    r.row(1) = r.row(0).cwiseProduct(r.row(0));
    r.row(2) = r.row(1).cwiseProduct(r.row(0));

    Matrix1X barrel_correction = (k_.template head<3>().template cast<ScalarT_>() * r).array() + ScalarT_(1.0);
    if (Base::D_.cols == 8)
      barrel_correction.array() /= (k_.template tail<3>().template cast<ScalarT_>() * r).array() + ScalarT_(1.0);

    Matrix2X a1(2, xyp.cols());
    a1.row(0) = ScalarT_(2) * xyp.colwise().prod();
    a1.row(1) = r.row(0) + ScalarT_(2) * xyp2.row(1);

    Matrix2X a2(2, xyp.cols());
    a2.row(0) = r.row(0) + ScalarT_(2) * xyp2.row(0);
    a2.row(1) = a1.row(0);

    Matrix2X xypp = (xyp.array().rowwise() * barrel_correction.array()).matrix()
                    + a1 * static_cast<ScalarT_>(p_.x()) + a2 * static_cast<ScalarT_>(p_.y());

    Cloud2_<ScalarT_> dist_points(pixel_points.size());
    dist_points.container() = (f_.template cast<ScalarT_>().asDiagonal() * xypp).colwise() + c_.template cast<ScalarT_>();

    return dist_points;
  }


template <typename ScalarT_>
  Pose3_<ScalarT_>
  PinholeCameraModel::estimatePose (const Cloud2_<ScalarT_> & points_image,
                                    const Cloud3_<ScalarT_> & points_object) const
  {
    assert(points_image.size() == points_object.size());

    cv::Mat_<cv::Vec<ScalarT_, 2> > cv_points_image = eigen2opencv(points_image.container());
    cv::Mat_<cv::Vec<ScalarT_, 3> > cv_points_object = eigen2opencv(points_object.container());

    cv::Vec<ScalarT_, 3> cv_r, cv_t;
    cv::solvePnP(cv_points_object, cv_points_image, intrinsicMatrix(), distortionCoeffs(), cv_r, cv_t);

    Vector3_<ScalarT_> r(cv_r[0], cv_r[1], cv_r[2]);
    Vector3_<ScalarT_> t(cv_t[0], cv_t[1], cv_t[2]);

    Vector3_<ScalarT_> r_normalized = r.normalized();
    AngleAxis_<ScalarT_> rotation(r.norm(), r_normalized);
    Translation3_<ScalarT_> translation(t);

    if (r_normalized.allFinite())
      return translation * rotation;
    else
      return Pose3_<ScalarT_>(translation);
  }

} // namespace calib
} // namespace unipd
#endif // IMPL_UNIPD_CALIBRATION_CALIBRATON_COMMON_PINHOLE_CAMERA_MODEL_HPP_
