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

#ifndef UNIPD_CALIBRATION_MULTISENSOR_CALIBRATION_KINECT1_NODE_H_
#define UNIPD_CALIBRATION_MULTISENSOR_CALIBRATION_KINECT1_NODE_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <calibration_common/objects/checkerboard.h>
#include <calibration_algorithms/checkerboard_extraction.h>
#include <calibration_ros/devices/ros_device.h>

#include <actionlib/server/simple_action_server.h>
#include <calibration_msgs/CheckerboardExtractionAction.h>
#include <calibration_msgs/GetDeviceInfo.h>

namespace unipd
{
namespace calib
{

class DeviceNode
{
public:

  DeviceNode (const ros::NodeHandle & node_handle);

  bool
  getDeviceInfo (calibration_msgs::GetDeviceInfo::Request & request,
                 calibration_msgs::GetDeviceInfo::Response & response);

  bool
  initialize ();

  void
  extractCheckerboard (const calibration_msgs::CheckerboardExtractionGoal::ConstPtr & goal);

private:

  CheckerboardExtraction::ImageResult
  extractCheckerboardFromSensor (const std::shared_ptr<ROSPinholeCamera> & sensor,
                                 const Checkerboard & checkerboard,
                                 int extraction_number);

  CheckerboardExtraction::DepthResult
  extractCheckerboardFromSensor (const std::shared_ptr<ROSDepthSensor> & sensor,
                                 const CheckerboardExtraction::ImageResult & image_result,
                                 int extraction_number);

  ros::NodeHandle node_handle_;
  ros::ServiceServer get_sensor_info_srv_;

  using CheckerboardExtractionServer = actionlib::SimpleActionServer<calibration_msgs::CheckerboardExtractionAction>;
  CheckerboardExtractionServer cb_extraction_server_;

  ROSDevice device_;

  std::string log_;

};

} // namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_MULTISENSOR_CALIBRATION_KINECT1_NODE_H_
