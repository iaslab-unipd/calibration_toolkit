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

#include <eigen_conversions/eigen_msg.h>

#include <calibration_ros/visualization/objects.h>

namespace unipd
{
namespace calib
{

geometry_msgs::TransformStamped
toTransformStamped (const BaseObject & object)
{
  assert(object.parent());

  geometry_msgs::TransformStamped transform_msg;

  transform_msg.header.frame_id = object.parent()->frameId();
  transform_msg.header.stamp = ros::Time::now();
  transform_msg.child_frame_id = object.frameId();

  tf::transformEigenToMsg(object.pose(), transform_msg.transform);
  return transform_msg;
}

visualization_msgs::Marker
toMarker (const BaseObject & object)
{
  assert(object.parent());

  visualization_msgs::Marker marker_msg;
  marker_msg.header.stamp = ros::Time::now();
  marker_msg.header.frame_id = object.parent()->frameId();

  marker_msg.type = visualization_msgs::Marker::SPHERE;
  marker_msg.action = visualization_msgs::Marker::ADD;
  marker_msg.scale.x = 0.001;
  marker_msg.scale.y = 0.001;
  marker_msg.scale.z = 0.001;
  marker_msg.color.r = 1.0;
  marker_msg.color.a = 1.0;

  tf::poseEigenToMsg(object.pose(), marker_msg.pose);
  return marker_msg;
}

visualization_msgs::Marker
toMarker (const PlanarObject & object)
{
  assert(object.parent());

  visualization_msgs::Marker marker_msg;
  marker_msg.header.stamp = ros::Time::now();
  marker_msg.header.frame_id = object.parent()->frameId();

  marker_msg.type = visualization_msgs::Marker::CUBE;
  marker_msg.action = visualization_msgs::Marker::ADD;
  marker_msg.scale.x = 5;
  marker_msg.scale.y = 5;
  marker_msg.scale.z = 0.005;
  marker_msg.color.b = 1.0;
  marker_msg.color.a = 0.5;

  tf::poseEigenToMsg(object.pose(), marker_msg.pose);
  return marker_msg;
}

visualization_msgs::Marker
toMarker (const Checkerboard & checkerboard)
{
  assert(checkerboard.parent());

  visualization_msgs::Marker marker_msg;

  marker_msg.header.stamp = ros::Time::now();
  marker_msg.header.frame_id = checkerboard.parent()->frameId();

  marker_msg.type = visualization_msgs::Marker::CUBE_LIST;
  marker_msg.action = visualization_msgs::Marker::ADD;
  marker_msg.scale.z = 0.01;

  marker_msg.scale.x = checkerboard.cellWidth();
  marker_msg.scale.y = checkerboard.cellHeight();

  std_msgs::ColorRGBA black;
  black.a = 1.0;
  std_msgs::ColorRGBA white;
  white.b = white.g = white.r = white.a = 1.0;

  Vector3 dc = (checkerboard.corners()(1, 1) - checkerboard.corners()(0, 0)) / 2.0;
  Vector3 dx = (checkerboard.corners()(1, 0) - checkerboard.corners()(0, 0));
  Vector3 dy = (checkerboard.corners()(0, 1) - checkerboard.corners()(0, 0));

  Cloud3 centers(Size2{checkerboard.cols() + 1, checkerboard.rows() + 1});

  centers(0, 0) = checkerboard.corners()(0, 0) - dc;
  for (int x = 1; x < checkerboard.cols() + 1; ++x)
    centers(x, 0) = centers(x - 1, 0) + dx;

  for (int y = 1; y < checkerboard.rows() + 1; ++y)
  {
    centers(0, y) = centers(0, y - 1) + dy;
    for (int x = 1; x < checkerboard.cols() + 1; ++x)
      centers(x, y) = centers(x - 1, y) + dx;
  }

  centers.transform(checkerboard.pose().inverse());

  for (int y = 0; y < checkerboard.rows() + 1; ++y)
  {
    for (int x = 0; x < checkerboard.cols() + 1; ++x)
    {
      geometry_msgs::Point p;
      tf::pointEigenToMsg(centers(x, y), p);
      marker_msg.points.push_back(p);
      if (y % 2)
        marker_msg.colors.push_back(x % 2 ? black : white);
      else
        marker_msg.colors.push_back(x % 2 ? white : black);
    }
  }

  tf::poseEigenToMsg(checkerboard.pose(), marker_msg.pose);
  return marker_msg;
}

} // namespace calib
} // namespace unipd
