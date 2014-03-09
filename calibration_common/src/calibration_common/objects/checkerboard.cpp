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

#include <calibration_common/objects/checkerboard.h>
#include <calibration_common/color/view.h>
#include <eigen_conversions/eigen_msg.h>

namespace calibration
{

void Checkerboard::toMarker(visualization_msgs::Marker & marker) const
{
  if (not parent())
    return;

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = parent()->frameId();

  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.z = 0.01;

  marker.scale.x = cellWidth();
  marker.scale.y = cellHeight();

  std_msgs::ColorRGBA black;
  black.a = 1.0;
  std_msgs::ColorRGBA white;
  white.b = white.g = white.r = white.a = 1.0;

  Eigen::Vector3d dc = (corners_(1, 1) - corners_(0, 0)) / 2.0;
  Eigen::Vector3d dx = (corners_(1, 0) - corners_(0, 0));
  Eigen::Vector3d dy = (corners_(0, 1) - corners_(0, 0));

  Cloud3 centers(cols() + 1, rows() + 1);

  centers(0, 0) = corners_(0, 0) - dc;
  for (int x = 1; x < cols() + 1; ++x)
    centers(x, 0) = centers(x - 1, 0) + dx;

  for (int y = 1; y < rows() + 1; ++y)
  {
    centers(0, y) = centers(0, y - 1) + dy;
    for (int x = 1; x < cols() + 1; ++x)
      centers(x, y) = centers(x - 1, y) + dx;
  }

  centers.transform(pose().inverse());

  for (int y = 0; y < rows() + 1; ++y)
  {
    for (int x = 0; x < cols() + 1; ++x)
    {
      geometry_msgs::Point p;
      tf::pointEigenToMsg(centers(x, y), p);
      marker.points.push_back(p);
      if (y % 2)
        marker.colors.push_back(x % 2 ? black : white);
      else
        marker.colors.push_back(x % 2 ? white : black);
    }
  }

  tf::poseEigenToMsg(pose(), marker.pose);
}

} /* namespace calibration */

