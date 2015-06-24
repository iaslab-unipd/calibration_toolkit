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

#include <cv_bridge/cv_bridge.h>
#include <calibration_ros/sensors/pinhole_camera.h>

namespace unipd
{
namespace calib
{

ROSPinholeCamera::ROSPinholeCamera (const ros::NodeHandle & node_handle)
  : ROSSensor(),
    node_handle_(node_handle),
    image_transport_(node_handle),
    sensor_(std::make_shared<PinholeSensor>())
{
  std::string frame_id;
  node_handle_.param("frame_id", frame_id, std::string("camera"));
  sensor_->setFrameId(frame_id);
  log_ = "[" + frame_id + "] ";
}

void
ROSPinholeCamera::subscribe (int topic)
{
  assert (static_cast<bool>(topic & Topic::IMAGE) == static_cast<bool>(topic & Topic::CAMERA_INFO));
  if (topic & Topic::IMAGE)
  {
    image_sub_ = image_transport_.subscribeCamera("image", 1, &ROSPinholeCamera::imageCallback, this);
    ROS_DEBUG_STREAM(log_ << "Subscribed to \"" << image_sub_.getTopic() << "\" topic.");
    ROS_DEBUG_STREAM(log_ << "Subscribed to \"" << image_sub_.getInfoTopic() << "\" topic.");
  }
  else
  {
    ROS_DEBUG_STREAM(log_ << "No subscription.");
  }
  setTopicMask(topic);
}

void
ROSPinholeCamera::imageCallback (const sensor_msgs::Image::ConstPtr & image_msg,
                                 const sensor_msgs::CameraInfo::ConstPtr & camera_info_msg)
{
  last_messages_.image_msg = image_msg;
  last_messages_.camera_info_msg = camera_info_msg;

  if (not isSensorSet())
  {
    sensor_msgs::CameraInfo new_camera_info_msg = *camera_info_msg;
    new_camera_info_msg.header.frame_id = sensor_->frameId();
    sensor_->setCameraModel(PinholeCameraModel(new_camera_info_msg));
  }

  newMessageReceived(Topic::IMAGE | Topic::CAMERA_INFO);
}

auto
ROSPinholeCamera::convertMessages (const Messages & messages) const -> Data
{
  cv::Mat image;
  auto image_ptr = cv_bridge::toCvCopy(messages.image_msg);
  return Data{image_ptr->image};
}

} // namespace calib
} // namespace unipd
