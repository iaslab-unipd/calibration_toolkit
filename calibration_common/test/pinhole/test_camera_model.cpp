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

#include <gtest/gtest.h>
#include <sensor_msgs/CameraInfo.h>
#include <calibration_common/pinhole/camera_model.h>

using namespace unipd::calib;
using sensor_msgs::CameraInfo;

CameraInfo
initCameraInfo ()
{
  CameraInfo camera_info = CameraInfo();
  camera_info.width = 640;
  camera_info.height = 480;
  camera_info.distortion_model = "plumb_bob";
  camera_info.K = CameraInfo::_K_type{500.0, 0.0, 320.0, 0.0, 450.0, 240.0, 0.0, 0.0, 1.0};
  camera_info.R = CameraInfo::_R_type{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camera_info.P = CameraInfo::_P_type{500.0, 0.0, 320.0, 0.0, 0.0, 450.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  camera_info.D = CameraInfo::_D_type{0.1723, -0.3400, 0.0021, 0.0032, 0.0};
  return camera_info;
}

TEST(PinholeCameraModel, PinholeCameraModel_CameraInfo)
{
  PinholeCameraModel(initCameraInfo());
  SUCCEED();
}

TEST(PinholeCameraModel, projectPixelTo3dRay_Point2)
{
  PinholeCameraModel model = PinholeCameraModel(initCameraInfo());
  double u = 200.0;
  double v = 300.0;
  Point2_<double> pixel = Point2_<double>(u, v);
  Point3_<double> point = model.projectPixelTo3dRay(pixel);

  double x = (u - model.cx() - model.Tx()) / model.fx();
  double y = (v - model.cy() - model.Ty()) / model.fy();
  double z = 1.0;
  Point3_<double> correct_point = Point3_<double>(x, y, z).normalized();

  EXPECT_TRUE(point.isApprox(correct_point, 1e-5));
}

TEST(PinholeCameraModel, projectPixelTo3dRay_Cloud2)
{
  PinholeCameraModel model = PinholeCameraModel(initCameraInfo());
  double u1 = 200.0, u2 = 300.0;
  double v1 = 300.0, v2 = 400.0;
  Cloud2_<double> pixels(Size2{2, 2});
  pixels(0, 0) = Point2_<double>(u1, v1);
  pixels(0, 1) = Point2_<double>(u1, v2);
  pixels(1, 0) = Point2_<double>(u2, v2);
  pixels(1, 1) = Point2_<double>(u2, v2);
  Cloud3_<double> points = model.projectPixelTo3dRay(pixels);

  Cloud3_<double> correct_points(Size2{2, 2});
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 2; ++j)
      correct_points(i, j) = model.projectPixelTo3dRay(Point2_<double>(pixels(i, j)));

  EXPECT_TRUE(points.container().isApprox(correct_points.container(), 1e-5));
}

TEST(PinholeCameraModel, project3dToPixel_Point3)
{
  PinholeCameraModel model = PinholeCameraModel(initCameraInfo());
  double x = 0.5;
  double y = 0.5;
  double z = 2.0;
  Point3_<double> point = Point3_<double>(x, y, z);
  Point2_<double> pixel = model.project3dToPixel(point);

  double u = (x * model.fx() + model.Tx()) / z + model.cx();
  double v = (y * model.fy() + model.Ty()) / z + model.cy();
  Point2_<double> correct_pixel = Point2_<double>(u, v);

  EXPECT_TRUE(pixel.isApprox(correct_pixel, 1e-5));
}


TEST(PinholeCameraModel, project3dToPixel_Cloud3)
{
  PinholeCameraModel model = PinholeCameraModel(initCameraInfo());
  double x1 = 0.5, x2 = -0.4;
  double y1 = 0.3, y2 = 0.7;
  double z1 = 2.0, z2 = 1.5;
  Cloud3_<double> points(Size2{2, 4});
  points(0, 0) = Point3_<double>(x1, y1, z1);
  points(0, 1) = Point3_<double>(x2, y1, z1);
  points(1, 0) = Point3_<double>(x1, y2, z1);
  points(1, 1) = Point3_<double>(x2, y2, z1);
  points(0, 2) = Point3_<double>(x1, y1, z2);
  points(0, 3) = Point3_<double>(x2, y1, z2);
  points(1, 2) = Point3_<double>(x1, y2, z2);
  points(1, 3) = Point3_<double>(x2, y2, z2);
  Cloud2_<double> pixels = model.project3dToPixel(points);

  Cloud2_<double> correct_pixels(Size2{2, 4});
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 4; ++j)
      correct_pixels(i, j) = model.project3dToPixel(Point3_<double>(points(i, j)));

  EXPECT_TRUE(pixels.container().isApprox(correct_pixels.container(), 1e-5));
}

TEST(PinholeCameraModel, distortPoint_Point2)
{
  PinholeCameraModel model = PinholeCameraModel(initCameraInfo());
  double u = 123.0;
  double v = 256.0;
  Point2_<double> pixel = Point2_<double>(u, v);
  Point2_<double> dist_pixel = model.distortPoint(pixel);

  cv::Point2d cv_pixel = cv::Point2d(u, v);
  cv::Point2d cv_dist_pixel = model.unrectifyPoint(cv_pixel);

  EXPECT_NEAR(dist_pixel.x(), cv_dist_pixel.x, 1e-5);
  EXPECT_NEAR(dist_pixel.y(), cv_dist_pixel.y, 1e-5);
}


TEST(PinholeCameraModel, distortPoints_Cloud2)
{
  PinholeCameraModel model = PinholeCameraModel(initCameraInfo());
  double u1 = 123.0, u2 = 234.4;
  double v1 = 256.0, v2 = 125.6;
  Cloud2_<double> pixels(Size2{1, 2});
  pixels(0, 0) = Point2_<double>(u1, v1);
  pixels(0, 1) = Point2_<double>(u2, v2);
  Cloud2_<double> dist_pixels = model.distortPoints(pixels);

  Cloud2_<double> correct_dist_pixels(pixels.size());
  for (int i = 0; i < 1; ++i)
    for (int j = 0; j < 2; ++j)
      correct_dist_pixels(i, j) = model.distortPoint(Point2_<double>(pixels(i, j)));

  EXPECT_TRUE(dist_pixels.container().isApprox(correct_dist_pixels.container(), 1e-5));
}
