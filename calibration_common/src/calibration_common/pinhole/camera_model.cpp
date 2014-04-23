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

#include <calibration_common/pinhole/camera_model.h>

namespace calibration
{

Point3 PinholeCameraModel::projectPixelTo3dRay(const Point2 & pixel_point) const
{
  //cv::Point3d cv_point = Base::projectPixelTo3dRay(cv::Point2d(pixel_point[0], pixel_point[1]));
  //return Eigen::Vector3d(cv_point.x, cv_point.y, cv_point.z).normalized();

  assert(initialized());

  Point3 ray;
  ray[0] = (pixel_point[0] - cx() - Tx()) / fx();
  ray[1] = (pixel_point[1] - cy() - Ty()) / fy();
  ray[2] = 1.0;
  ray.normalize();

  return ray;
}

Point2 PinholeCameraModel::project3dToPixel(const Point3 & world_point) const
{
  //cv::Point2d cv_point = Base::project3dToPixel(cv::Point3d(world_point[0], world_point[1], world_point[2]));
  //return Point2(cv_point.x, cv_point.y);

  assert(initialized());
  assert(P_(2, 3) == 0.0);

  Point2 uv_rect;
  uv_rect[0] = (fx() * world_point[0] + Tx()) / world_point[2] + cx();
  uv_rect[1] = (fy() * world_point[1] + Ty()) / world_point[2] + cy();

  return uv_rect;
}

void PinholeCameraModel::projectPixelTo3dRay(const Cloud2 & pixel_points,
                                             Cloud3 & world_points) const
{
  assert(initialized());
  assert(pixel_points.size() == world_points.size());

  Eigen::Array<Scalar, 2, 1> sub(-cx() - Tx(), -cy() - Ty());
  Eigen::Array<Scalar, 2, 1> div(fx(), fy());

  world_points.container().topRows<2>() = (pixel_points.container().array().colwise() + sub).colwise() / div;
  world_points.container().bottomRows<1>().setOnes();
  world_points.container().array().rowwise() /= world_points.container().colwise().norm().array();

}

Cloud3 PinholeCameraModel::projectPixelTo3dRay(const Cloud2 & pixel_points) const
{
  Cloud3 world_points(pixel_points.xSize(), pixel_points.ySize());
  projectPixelTo3dRay(pixel_points, world_points);
  return world_points;
}

void PinholeCameraModel::project3dToPixel(const Cloud3 & world_points,
                                          Cloud2 & pixel_points) const
{
  assert(initialized());
  assert(P_(2, 3) == 0.0);
  assert(pixel_points.size() == world_points.size());

  Eigen::Array<Scalar, 2, 1> prod(fx(), fy());
  Point2 sum(Tx(), Ty());
  Point2 sum_final(cx(), cy());

  pixel_points.container() = world_points.container().topRows<2>();
  pixel_points.container().array().colwise() *= prod;
  pixel_points.container().colwise() += sum;
  pixel_points.container().array().rowwise() /= world_points.container().array().bottomRows<1>();
  pixel_points.container().colwise() += sum_final;

}

Cloud2 PinholeCameraModel::project3dToPixel(const Cloud3 & world_points) const
{
  Cloud2 pixel_points(world_points.xSize(), world_points.ySize());
  project3dToPixel(world_points, pixel_points);
  return pixel_points;
}

Pose PinholeCameraModel::estimatePose(const Cloud2 & points_image,
                                      const Cloud3 & points_object) const
{
  assert(points_image.size() == points_object.size());

  cv::Mat_<cv::Vec<Scalar, 2> > cv_points_image;
  cv::Mat_<cv::Vec<Scalar, 3> > cv_points_object;
  OpenCVConversion<Scalar>::toOpenCV(points_image.container(), cv_points_image);
  OpenCVConversion<Scalar>::toOpenCV(points_object.container(), cv_points_object);

  cv::Vec<Scalar, 3> cv_r;
  cv::Vec<Scalar, 3> cv_t;
  cv::solvePnP(cv_points_object, cv_points_image, intrinsicMatrix(), distortionCoeffs(), cv_r, cv_t);

  Vector3 r;
  r << cv_r[0], cv_r[1], cv_r[2];
  Vector3 t;
  t << cv_t[0], cv_t[1], cv_t[2];

  AngleAxis rotation(r.norm(), r.normalized());
  Translation3 translation(t);

  return translation * rotation;
}

} /* namespace calibration */
