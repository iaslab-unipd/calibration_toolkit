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

#ifndef IMPL_CALIBRATION_COMMON_PINHOLE_CAMERA_MODEL_HPP_
#define IMPL_CALIBRATION_COMMON_PINHOLE_CAMERA_MODEL_HPP_

#include <calibration_common/pinhole/camera_model.h>
#include <calibration_common/base/opencv_conversion.h>

namespace calibration
{

/*template <typename Scalar>
  typename Types<Scalar>::Point3 PinholeCameraModel::projectPixelTo3dRay(const typename Types<Scalar>::Point2 & pixel_point) const
  {
    assert(initialized());

    typename Types<Scalar>::Point3 ray;
    ray[0] = Scalar((pixel_point[0] - cx() - Tx()) / fx());
    ray[1] = Scalar((pixel_point[1] - cy() - Ty()) / fy());
    ray[2] = Scalar(1.0);
    ray.normalize();

    return ray;
  }

template <typename Scalar>
  typename Types<Scalar>::Point2 PinholeCameraModel::project3dToPixel(const typename Types<Scalar>::Point3 & world_point) const
  {
    assert(initialized());
    assert(P_(2, 3) == 0.0);

    typename Types<Scalar>::Point2 uv_rect;
    uv_rect[0] = Scalar((fx() * world_point[0] + Tx()) / world_point[2] + cx());
    uv_rect[1] = Scalar((fy() * world_point[1] + Ty()) / world_point[2] + cy());

    return uv_rect;
  }

template <typename Scalar>
  void PinholeCameraModel::projectPixelTo3dRay(const typename Types<Scalar>::Cloud2 & pixel_points,
                                               typename Types<Scalar>::Cloud3 & world_points) const
  {
    assert(initialized());
    assert(((pixel_points.size() == world_points.size()).all()));

    Eigen::Array<Scalar, 2, 1> sub(Scalar(-cx() - Tx()), Scalar(-cy() - Ty()));
    Eigen::Array<Scalar, 2, 1> div(Scalar(fx()), Scalar(fy()));

    world_points.container().template topRows<2>() = (pixel_points.container().array().colwise() + sub).colwise() / div;
    world_points.container().template bottomRows<1>().setOnes();
    world_points.container().array().rowwise() /= world_points.container().colwise().norm().array();
  }

template <typename Scalar>
  inline typename Types<Scalar>::Cloud3 PinholeCameraModel::projectPixelTo3dRay(const typename Types<Scalar>::Cloud2 & pixel_points) const
  {
    typename Types<Scalar>::Cloud3 world_points(pixel_points.xSize(), pixel_points.ySize());
    projectPixelTo3dRay<Scalar>(pixel_points, world_points);
    return world_points;
  }

template <typename Scalar>
  void PinholeCameraModel::project3dToPixel(const typename Types<Scalar>::Cloud3 & world_points,
                                            typename Types<Scalar>::Cloud2 & pixel_points) const
  {
    assert(initialized());
    assert(P_(2, 3) == 0.0);
    assert((pixel_points.size() == world_points.size()).all());

    Eigen::Array<double, 2, 1> prod(fx(), fy());
    Types<double>::Point2 sum(Tx(), Ty());
    Types<double>::Point2 sum_final(cx(), cy());

    pixel_points.container() = world_points.container().template topRows<2>();
    pixel_points.container().array().colwise() *= prod.template cast<Scalar>();
    pixel_points.container().colwise() += sum.template cast<Scalar>();
    pixel_points.container().array().rowwise() /= world_points.container().array().template bottomRows<1>();
    pixel_points.container().colwise() += sum_final.template cast<Scalar>();
  }

template <typename Scalar>
  inline typename Types<Scalar>::Cloud2 PinholeCameraModel::project3dToPixel(const typename Types<Scalar>::Cloud3 & world_points) const
  {
    typename Types<Scalar>::Cloud2 pixel_points(world_points.size());
    project3dToPixel<Scalar>(world_points, pixel_points);
    return pixel_points;
  }*/

template <typename Scalar>
  inline typename Types<Scalar>::Cloud2 PinholeCameraModel::project3dToPixel2(const typename Types<Scalar>::Cloud3 & world_points) const
  {
    typename Types<Scalar>::Cloud2 pixel_points(world_points.size());
    project3dToPixel2<Scalar>(world_points, pixel_points);
    return pixel_points;
  }

template <typename Scalar>
  typename Types<Scalar>::Point2 PinholeCameraModel::project3dToPixel2 (const typename Types<Scalar>::Point3 & point) const
  {
    typename Types<Scalar>::Point2 K_f_;
    typename Types<Scalar>::Point2 K_c_;

    K_f_ << Scalar(K_(0, 0)), Scalar(K_(1, 1));
    K_c_ << Scalar(K_(0, 2)), Scalar(K_(1, 2));

    typename Types<Scalar>::Point2 p = point.template head<2>() / point[2];
    p = K_f_.asDiagonal() * distort2d_<Scalar>(p) + K_c_;
    return p;
  }

template <typename Scalar>
  void PinholeCameraModel::project3dToPixel2(const typename Types<Scalar>::Cloud3 & world_points,
                                             typename Types<Scalar>::Cloud2 & pixel_points) const
  {
    pixel_points = typename Types<Scalar>::Cloud2(world_points.size());
    for (Size1 i = 0; i < world_points.elements(); ++i)
      pixel_points[i] = project3dToPixel2<Scalar>(world_points[i]);
  }

template <typename Scalar>
  typename Types<Scalar>::Pose PinholeCameraModel::estimatePose(const typename Types<Scalar>::Cloud2 & points_image,
                                                                const typename Types<Scalar>::Cloud3 & points_object) const
  {
    assert((points_image.size() == points_object.size()).all());

    cv::Mat_<cv::Vec<Scalar, 2> > cv_points_image;
    cv::Mat_<cv::Vec<Scalar, 3> > cv_points_object;
    OpenCVConversion<Scalar>::toOpenCV(points_image.container(), cv_points_image);
    OpenCVConversion<Scalar>::toOpenCV(points_object.container(), cv_points_object);

    cv::Vec<Scalar, 3> cv_r, cv_t;
    cv::solvePnP(cv_points_object, cv_points_image, intrinsicMatrix(), distortionCoeffs(), cv_r, cv_t);

    Vector3 r;
    r << cv_r[0], cv_r[1], cv_r[2];
    Vector3 t;
    t << cv_t[0], cv_t[1], cv_t[2];

    AngleAxis rotation(r.norm(), r.normalized());
    Translation3 translation(t);

    return translation * rotation;
  }

template <typename Scalar>
typename Types<Scalar>::Point2
PinholeCameraModel::distort2d_ (const typename Types<Scalar>::Point2 & normalized_point_rect_2d) const
{
  typename Types<Scalar>::Vector2 p_ = Types<Scalar>::Vector2::Zero();
  if (D_.rows >= 4)
  {
    p_[0] = Scalar(D_.at<double>(2, 0));
    p_[1] = Scalar(D_.at<double>(3, 0));
  }
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> k_;

  if (D_.rows <= 5)
    k_ = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(3);
  else
    k_ = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(6);

  if (D_.rows >= 2)
  {
    k_.template head<2>() << Scalar(D_.at<double>(0, 0)), Scalar(D_.at<double>(1, 0));
  }
  if (D_.rows >= 5)
  {
    k_[2] = Scalar(D_.at<double>(4, 0));
  }
  if (D_.rows == 8)
  {
    k_.template tail<3>() << Scalar(D_.at<double>(5, 0)), Scalar(D_.at<double>(6, 0)), Scalar(D_.at<double>(7, 0));
  }

 const typename Types<Scalar>::Point2 & xyp = normalized_point_rect_2d;
 typename Types<Scalar>::Point2 xyp2 = normalized_point_rect_2d.cwiseAbs2();
 typename Types<Scalar>::Point3 r;
 r[0] = xyp2.sum();
 r[1] = r[0] * r[0];
 r[2] = r[1] * r[0];

 Scalar barrel_correction = Scalar(1.0) + k_.template head<3>().dot(r);
 if (k_.rows() == 6)
   barrel_correction /= Scalar(1.0) + k_.template tail<3>().dot(r);

 Eigen::Matrix<Scalar, 2, 2> a;
 a(0, 0) = a(1, 1) = Scalar(2.0) * xyp.prod();
 a(0, 1) = r[0] + Scalar(2.0) * xyp2[0];
 a(1, 0) = r[0] + Scalar(2.0) * xyp2[1];

 return typename Types<Scalar>::Point2(xyp * barrel_correction + a * p_);
}

template <typename Scalar>
typename Types<Scalar>::Point2
PinholeCameraModel::undistort2d_ (const typename Types<Scalar>::Point2 & normalized_point_2d) const
{
  typename Types<Scalar>::Vector2 p_ = Types<Scalar>::Vector2::Zero();
  if (D_.rows >= 4)
  {
    p_[0] = Scalar(D_.at<double>(2, 0));
    p_[1] = Scalar(D_.at<double>(3, 0));
  }
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> k_;

  if (D_.rows <= 5)
    k_ = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(3);
  else
    k_ = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(6);

  if (D_.rows >= 2)
  {
    k_.template head<2>() << Scalar(D_.at<double>(0, 0)), Scalar(D_.at<double>(1, 0));
  }
  if (D_.rows >= 5)
  {
    k_[2] = Scalar(D_.at<double>(4, 0));
  }
  if (D_.rows == 8)
  {
    k_.template tail<3>() << Scalar(D_.at<double>(5, 0)), Scalar(D_.at<double>(6, 0)), Scalar(D_.at<double>(7, 0));
  }

  typename Types<Scalar>::Point2 p = normalized_point_2d;

 // Compensate distortion iteratively
  for (int i = 0; i < 10; ++i)
  {
    typename Types<Scalar>::Vector3 r;
    r[0] = p.cwiseAbs2().sum();
    r[1] = r[0] * r[0];
    r[2] = r[1] * r[0];

    Scalar ic_dist = static_cast<Scalar>(1.0) / (static_cast<Scalar>(1.0) + k_.template head<3>().dot(r));
    if (k_.rows() == 6)
     ic_dist *= static_cast<Scalar>(1.0) + k_.template tail<3>().dot(r);

    Eigen::Matrix<Scalar, 2, 2> a;
    a(0, 0) = a(1, 1) = 2 * p.prod();
    a(0, 1) = r[0] + 2 * p[0] * p[0];
    a(1, 0) = r[0] + 2 * p[1] * p[1];

    p = (normalized_point_2d - a * p_) * ic_dist;
  }

 return p;
}

} /* namespace calibration */

#endif /* IMPL_CALIBRATION_COMMON_PINHOLE_CAMERA_MODEL_HPP_ */
