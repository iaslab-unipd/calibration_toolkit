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
#include <sensor_msgs/image_encodings.h>
#include <pcl_conversions/pcl_conversions.h>
#include <calibration_ros/sensors/depth_sensor.h>

namespace unipd
{
namespace calib
{

ROSDepthSensor::ROSDepthSensor (const ros::NodeHandle & node_handle)
  : ROSSensor(),
    node_handle_(node_handle),
    image_transport_(node_handle),
    sensor_(std::make_shared<PinholeDepthSensor>())
{
  std::string frame_id;
  node_handle_.param("frame_id", frame_id, std::string("depth_sensor"));
  sensor_->setFrameId(frame_id);
  log_ = "[" + frame_id + "] ";

  bool error_ok = node_handle_.hasParam("error");
  if (error_ok)
  {
    std::vector<Scalar> coefficients;
    int max_degree, min_degree;
    error_ok = error_ok and node_handle_.getParam("error/coefficients", coefficients);
    error_ok = error_ok and node_handle_.getParam("error/max_degree", max_degree);
    error_ok = error_ok and node_handle_.getParam("error/min_degree", min_degree);
    if (max_degree - min_degree + 1 == coefficients.size())
      sensor_->setError(PolynomialX_<Scalar>(max_degree, min_degree), coefficients);
    else
      ROS_FATAL_STREAM(log_ << "[~error/max_degree - ~error/min_degree + 1] must be equal to [~error/coefficients].size()! E.g.:\n"
                            << "<rosparam param=\"error\">\n  max_degree: 2\n  min_degree: 0\n  coefficients: [0.0, 0.0, 0.0035]\n </rosparam>");
  }
  if (not error_ok)
    ROS_FATAL_STREAM(log_ << "Missing or incomplete parameter [~error]! "
                          << "[~error] must contain: [max_degree, min_degree, coefficients] fields. E.g.:\n"
                          << "<rosparam param=\"error\">\n  max_degree: 2\n  min_degree: 0\n  coefficients: [0.0, 0.0, 0.0035]\n </rosparam>");

}

void
ROSDepthSensor::subscribe (int topic)
{
  if ((topic & Topic::IMAGE) and (topic & Topic::CAMERA_INFO))
  {
    image_sub_ = image_transport_.subscribeCamera("image", 1, &ROSDepthSensor::imageCallback, this);
    ROS_DEBUG_STREAM(log_ << "Subscribed to [" << image_sub_.getTopic() << "] topic.");
    ROS_DEBUG_STREAM(log_ << "Subscribed to [" << image_sub_.getInfoTopic() << "] topic.");
  }
  else if (topic & Topic::CAMERA_INFO)
  {
    camera_info_sub_ = node_handle_.subscribe("camera_info", 1, &ROSDepthSensor::cameraInfoCallback, this);
    ROS_DEBUG_STREAM(log_ << "Subscribed to [" << camera_info_sub_.getTopic() << "] topic.");
  }

  if (topic & Topic::POINT_CLOUD)
  {
    cloud_sub_ = node_handle_.subscribe("cloud", 1, &ROSDepthSensor::cloudCallback, this);
    ROS_DEBUG_STREAM(log_ << "Subscribed to [" << cloud_sub_.getTopic() << "] topic.");
  }

  if (topic == Topic::NONE)
    ROS_DEBUG_STREAM(log_ << "No subscription.");

  setTopicMask(topic);
}

void
ROSDepthSensor::imageCallback (const sensor_msgs::Image::ConstPtr & image_msg,
                                  const sensor_msgs::CameraInfo::ConstPtr & camera_info_msg)
{
  last_messages_.image_msg = image_msg;
  newMessageReceived(Topic::IMAGE);
  cameraInfoCallback(camera_info_msg);
}

void
ROSDepthSensor::cameraInfoCallback (const sensor_msgs::CameraInfo::ConstPtr & camera_info_msg)
{
  last_messages_.camera_info_msg = camera_info_msg;
  newMessageReceived(Topic::CAMERA_INFO);
  if (not isSensorSet())
  {
    sensor_msgs::CameraInfo new_camera_info_msg = *camera_info_msg;
    new_camera_info_msg.header.frame_id = sensor_->frameId();
    sensor_->setCameraModel(PinholeCameraModel(new_camera_info_msg));
  }
}

void
ROSDepthSensor::cloudCallback (const sensor_msgs::PointCloud2::ConstPtr & cloud_msg)
{
  last_messages_.cloud_msg = cloud_msg;
  newMessageReceived(Topic::POINT_CLOUD);
}

auto
ROSDepthSensor::convertMessages (const Messages & messages) const -> Data
{
  cv::Mat image;
  if (topic_mask_ & Topic::IMAGE)
  {
    auto image_ptr = cv_bridge::toCvCopy(messages.image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    image = image_ptr->image;
  }
  auto data = Data{image, boost::make_shared<PCLCloud3>()};

  if (topic_mask_ & Topic::POINT_CLOUD)
  {
    pcl::fromROSMsg(*messages.cloud_msg, *data.cloud);
  }
  
  return data;
}

} // namespace calib
} // namespace unipd
