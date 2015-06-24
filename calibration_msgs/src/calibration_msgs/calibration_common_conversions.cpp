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

#include <calibration_msgs/calibration_common_conversions.h>
#include <eigen_conversions/eigen_msg.h>

namespace unipd
{
namespace calib
{

Checkerboard
fromMessage (const calibration_msgs::Checkerboard & msg)
{
  assert (msg.parent_frame_id == "");
  return Checkerboard(msg.header.frame_id, msg.cols, msg.rows, msg.cell_width, msg.cell_height);
}

Checkerboard
fromMessage (const calibration_msgs::Checkerboard & msg,
             const std::shared_ptr<BaseObject> & parent)
{
  assert (msg.parent_frame_id == parent->frameId());
  Checkerboard checkerboard = Checkerboard(msg.header.frame_id, msg.cols, msg.rows, msg.cell_width, msg.cell_height);
  checkerboard.setParent(parent);
  Pose3 pose;
  tf::poseMsgToEigen(msg.pose, pose);
  checkerboard.setPose(pose);
  return checkerboard;
}

calibration_msgs::Checkerboard
toMessage (const Checkerboard & checkerboard)
{
  calibration_msgs::Checkerboard msg;
  msg.header.frame_id = checkerboard.frameId();
  if (checkerboard.hasParent())
    msg.parent_frame_id = checkerboard.parent()->frameId();
  msg.rows = checkerboard.rows();
  msg.cols = checkerboard.cols();
  msg.cell_width = checkerboard.cellWidth();
  msg.cell_height = checkerboard.cellHeight();
  tf::poseEigenToMsg(checkerboard.pose(), msg.pose);
  return msg;
}

Cloud2
fromMessage (const calibration_msgs::Point2DArray & msg)
{
  Cloud2 points(Size2{msg.x_size, msg.y_size});
  for (int i = 0; i < msg.points.size(); ++i)
    points[i] << msg.points[i].x, msg.points[i].y;
  return points;
}

Cloud3
fromMessage (const calibration_msgs::PointArray & msg)
{
  Cloud3 points(Size2{msg.x_size, msg.y_size});
  for (int i = 0; i < msg.points.size(); ++i)
    points[i] << msg.points[i].x, msg.points[i].y, msg.points[i].z;
  return points;
}

const calibration_msgs::Point2DArray
toMessage (const Cloud2 & points)
{
  calibration_msgs::Point2DArray msg;
  msg.x_size = points.size().x;
  msg.y_size = points.size().y;
  msg.points.resize(msg.x_size * msg.y_size);
  for (int i = 0; i < msg.points.size(); ++i)
  {
    msg.points[i].x = points[i].x();
    msg.points[i].y = points[i].y();
  }
  return msg;
}

const calibration_msgs::PointArray
toMessage (const Cloud3 & points)
{
  calibration_msgs::PointArray msg;
  msg.x_size = points.size().x;
  msg.y_size = points.size().y;
  msg.points.resize(msg.x_size * msg.y_size);
  for (int i = 0; i < msg.points.size(); ++i)
  {
    msg.points[i].x = points[i].x();
    msg.points[i].y = points[i].y();
    msg.points[i].z = points[i].z();
  }
  return msg;
}

} // namespace calib
} // namespace unipd
