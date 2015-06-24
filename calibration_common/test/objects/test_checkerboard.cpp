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
#include <calibration_common/objects/checkerboard.h>
#include <calibration_common/pinhole/pinhole.h>

using namespace unipd::calib;
using sensor_msgs::CameraInfo;

TEST(Checkerboard, Checkerboard_1)
{
  Checkerboard(2, 3, 0.1, 0.2);
  Checkerboard("cb", 2, 3, 0.1, 0.2);
  SUCCEED();
}

TEST(Checkerboard, Checkerboard_2)
{
  std::shared_ptr<Checkerboard> original = std::make_shared<Checkerboard>(2, 3, 0.1, 0.2);

  CameraInfo camera_info = CameraInfo();
  camera_info.width = 640;
  camera_info.height = 480;
  camera_info.distortion_model = "plumb_bob";
  camera_info.K = CameraInfo::_K_type{200.0, 0.0, 320.0, 0.0, 200.0, 240.0, 0.0, 0.0, 1.0};
  camera_info.P = CameraInfo::_P_type{200.0, 0.0, 320.0, 0.0, 0.0, 200.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  camera_info.D = CameraInfo::_D_type{0.0, 0.0, 0.0, 0.0, 0.0};

  std::shared_ptr<PinholeSensor> sensor = std::make_shared<PinholeSensor>();
  sensor->setCameraModel(PinholeCameraModel(camera_info));

  Checkerboard moved = *original;
  moved.transform(Transform3::Identity() * Translation3(Vector3::UnitZ()));

  Cloud2 corners_image = sensor->cameraModel().project3dToPixel(moved.corners());

  PinholePointsView<Checkerboard> view;
  view.setObject(original);
  view.setPoints(corners_image);
  view.setSensor(sensor);

  Checkerboard created = Checkerboard(view);

  EXPECT_TRUE(created.corners().container().isApprox(moved.corners().container(), 1e-5));
}

TEST(Checkerboard, pose)
{
 auto cb = Checkerboard(2, 3, 0.1, 0.2);
 EXPECT_TRUE((cb.pose().matrix().array() == Pose3::Identity().matrix().array()).all());
}

TEST(Checkerboard, frame_id)
{
 auto cb = Checkerboard("cb", 2, 3, 0.1, 0.2);
 EXPECT_EQ(cb.frameId(), "cb");
}

TEST(Checkerboard, parent)
{
 auto cb = Checkerboard(2, 3, 0.1, 0.2);
 EXPECT_FALSE(cb.parent());
}

TEST(Checkerboard, plane)
{
 auto cb = Checkerboard(2, 3, 0.1, 0.2);
 EXPECT_TRUE((cb.plane().coeffs().array() == Plane3(Vector3::UnitZ(), 0).coeffs().array()).all());
}

TEST(Checkerboard, cols)
{
 auto cb = Checkerboard(2, 3, 0.1, 0.2);
 EXPECT_EQ(cb.cols(), 2);
}

TEST(Checkerboard, rows)
{
 auto cb = Checkerboard(2, 3, 0.1, 0.2);
 EXPECT_EQ(cb.rows(), 3);
}
