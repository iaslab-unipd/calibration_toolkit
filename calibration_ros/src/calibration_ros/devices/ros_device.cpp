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

#include <calibration_ros/devices/ros_device.h>

namespace unipd
{
namespace calib
{

ROSDevice::ROSDevice (ros::NodeHandle node_handle)
{
  std::string name;
  node_handle.param("name", name, std::string("device"));
  log_ = "[" + name + "] ";

  if (not node_handle.hasParam("sensors"))
    ROS_FATAL_STREAM(log_ << "Missing \"sensors\" parameter!");

  std::map<std::string, std::shared_ptr<Sensor>> sensor_map;

  std::vector<std::string> intensity_sensors;
  node_handle.getParam("sensors/intensity", intensity_sensors);
  for (const std::string & sensor_name : intensity_sensors)
  {
    std::shared_ptr<ROSPinholeCamera> sensor = std::make_shared<ROSPinholeCamera>(ros::NodeHandle(node_handle, sensor_name));
    intensity_sensors_.push_back(sensor);
    sensor_map[sensor_name] = sensor->sensor();
  }

  std::vector<std::string> depth_sensors;
  node_handle.getParam("sensors/depth", depth_sensors);
  for (const std::string & sensor_name : depth_sensors)
  {
    std::shared_ptr<ROSDepthSensor> sensor = std::make_shared<ROSDepthSensor>(ros::NodeHandle(node_handle, sensor_name));
    depth_sensors_.push_back(sensor);
    sensor_map[sensor_name] = sensor->sensor();
  }

  std::vector<std::string> sensors = intensity_sensors;
  sensors.insert(sensors.end(), depth_sensors.begin(), depth_sensors.end());
  if (node_handle.hasParam("transforms"))
  {
    for (const std::string & sensor_name : sensors)
    {
      if (node_handle.hasParam("transforms/" + sensor_name))
      {
        std::string parent;
        node_handle.getParam("transforms/" + sensor_name + "/parent", parent);

        if (sensor_map.find(parent) == sensor_map.end())
          ROS_FATAL_STREAM(log_ << "Unknown sensor \"" << parent<< "\" in \"transforms\"!");

        std::map<std::string, Scalar> position;
        std::map<std::string, Scalar> orientation;
        node_handle.getParam("transforms/" + sensor_name + "/position", position);
        node_handle.getParam("transforms/" + sensor_name + "/orientation", orientation);

        Translation3 t = Translation3(position["x"], position["y"], position["z"]);
        Quaternion q = Quaternion(orientation["w"], orientation["x"], orientation["y"], orientation["z"]);

        sensor_map[sensor_name]->setParent(sensor_map[parent]);
        sensor_map[sensor_name]->setPose(t * q);
      }
    }
  }
}

} // namespace calib
} // namespace unipd
