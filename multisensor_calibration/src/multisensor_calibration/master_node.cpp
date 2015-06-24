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

#include <calibration_msgs/GetDeviceInfo.h>
#include <calibration_msgs/calibration_common_conversions.h>

#include <multisensor_calibration/master_node.h>

namespace fs = boost::filesystem;
namespace msgs = calibration_msgs;

namespace unipd
{
namespace calib
{

CalibrationMasterNode::CalibrationMasterNode (const ros::NodeHandle & node_handle)
  : node_handle_(node_handle),
    log_("[" + node_handle.getNamespace() + "] "),
    tf_pub_(),
    plane_flag_(false)
{
  action_sub_ = node_handle_.subscribe("action", 10, &CalibrationMasterNode::actionCallback, this);
  acquisition_sub_ = node_handle_.subscribe("acquisition", 1, &CalibrationMasterNode::actionCallback, this);
  marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("markers", 10);

  result_service_ = node_handle_.advertiseService("get_results", &CalibrationMasterNode::getResultsCallback, this);

  std::string network_file;
  if (not node_handle_.getParam("network_file", network_file))
    ROS_FATAL_STREAM(log_ << "[~network_file] parameter missing!");

  if (parseNetworkFile(network_file))
    ROS_INFO_STREAM(log_ << "File [" << network_file << "] parsed.");
  else
    ROS_FATAL_STREAM(log_ << "File [" << network_file << "] does not exist or is empty.");

  double cell_width, cell_height;
  int rows, cols;

  bool cb_ok = true;
  cb_ok = cb_ok and node_handle_.getParam("checkerboard/cell_width", cell_width);
  cb_ok = cb_ok and node_handle_.getParam("checkerboard/cell_height", cell_height);
  cb_ok = cb_ok and node_handle_.getParam("checkerboard/rows", rows);
  cb_ok = cb_ok and node_handle_.getParam("checkerboard/cols", cols);
  if (not cb_ok)
    ROS_FATAL_STREAM(log_ << "[~checkerboard] parameter missing! Please set "
                          << "[~checkerboard/{rows, cols, cell_width, cell_height}].");

  checkerboard_ = std::make_shared<Checkerboard>(cols, rows, cell_width, cell_height);
  checkerboard_->setFrameId("/checkerboard");
}

bool
CalibrationMasterNode::parseNetworkFile (const std::string & filename)
{
  auto file = fs::path(filename);
  if (fs::exists(file) and not file.empty())
  {
    YAML::Node file_node = YAML::LoadFile(filename);

    const YAML::Node & network_node = file_node["network"];
    for (const YAML::Node & pc_node : network_node)
    {
      PC pc{pc_node["pc"].as<std::string>()};
      for (const YAML::Node & sensor_node : pc_node["sensors"])
      {
        pc.device_vec.push_back(Device{sensor_node["id"].as<std::string>()});
      }
      pcs_.push_back(pc);
    }
    return true;
  }
  else
  {
    return false;
  }

}

bool
CalibrationMasterNode::callGetDeviceInfo (Device & device,
                                          std::vector<std::shared_ptr<PinholeSensor>> & pinhole_sensors,
                                          std::vector<std::shared_ptr<PinholeDepthSensor>> & depth_sensors)
{
  msgs::GetDeviceInfo msg;
  msg.request.requested_types = msgs::GetDeviceInfo::Request::ALL;
  if (device.get_device_info_client.call(msg))
  {
    ROS_DEBUG_STREAM(log_ << "Service [" << device.get_device_info_client.getService() << "] replied.");

    std::map<std::string, std::shared_ptr<Sensor>> sensor_map;

    // Parse camera_infos
    for (int i = 0; i < msg.response.sensor_types.size(); ++i)
    {
      if (msg.response.sensor_types[i] == msgs::GetDeviceInfo::Response::INTENSITY)
      {
        auto sensor = fromMessage<PinholeSensor>(msg.response.camera_infos[i]);
        sensor_map.emplace(sensor->frameId(), sensor);
        pinhole_sensors.push_back(sensor);
      }
      else if (msg.response.sensor_types[i] == msgs::GetDeviceInfo::Response::DEPTH)
      {
        auto sensor = fromMessage<PinholeDepthSensor>(msg.response.camera_infos[i]);
        sensor->setError(fromMessage<Scalar>(msg.response.error_polynomials[i]));
        sensor_map.emplace(sensor->frameId(), sensor);
        depth_sensors.push_back(sensor);
      }
      else // (msg.response.sensor_types[i] == msgs::GetDeviceInfo::Response::UNDEFINED)
      {
        ROS_ERROR_STREAM(log_ << "Service [" << device.get_device_info_client.getService()
                              << "] replied with an UNDEFINED sensor at index: " << i << ". Skipping.");
      }
    }

    // Parse initial_transforms
    for (const geometry_msgs::TransformStamped & transform_msg : msg.response.initial_transforms)
    {
      std::shared_ptr<Sensor> sensor;
      if (sensor_map.find(transform_msg.child_frame_id) != sensor_map.end())
        sensor = sensor_map[transform_msg.child_frame_id];
      else
        ROS_ERROR_STREAM(log_ << "Sensor [" << transform_msg.child_frame_id << "] unknown. Skipping transform ["
                         << transform_msg.header.frame_id << "] -> [" << transform_msg.child_frame_id << "].");

      std::shared_ptr<Sensor> parent;
      if (sensor_map.find(transform_msg.header.frame_id) != sensor_map.end())
        parent = sensor_map[transform_msg.header.frame_id];
      else
        ROS_ERROR_STREAM(log_ << "Sensor [" << transform_msg.header.frame_id << "] unknown. Skipping transform ["
                         << transform_msg.header.frame_id << "] -> [" << transform_msg.child_frame_id << "].");

      if (sensor and parent)
      {
        if (sensor->hasParent())
        {
          ROS_ERROR_STREAM(log_ << "Sensor [" << sensor->frameId() << "] already has a parent: [" << sensor->parent()->frameId() << "].");
        }
        else
        {
          sensor->setParent(parent);
          Transform3 transform;
          tf::transformMsgToEigen(transform_msg.transform, transform);
          sensor->transform(transform);
        }
      }
    }

    // Parse fixed_transforms
    if (not msg.response.fixed_transforms.empty())
      ROS_WARN_STREAM(log_ << "Fixed transforms are not supported yet. Message from ["
                           << device.get_device_info_client.getService() << "] service not fully parsed.");

  }
  else
  {
    ROS_ERROR_STREAM(log_ << "Service [" << device.get_device_info_client.getService() << "] did not reply.");
    return false;
  }

  return true;
}

bool
CalibrationMasterNode::initialize ()
{
  for (PC & pc : pcs_)
  {
    for (Device & device : pc.device_vec)
    {
      std::string node_name = "/" + pc.name + "/" + device.name + "_node";

      // Test GetDeviceInfo service
      std::string get_device_info_srv = node_name + "/get_device_info";
      device.get_device_info_client = node_handle_.serviceClient<msgs::GetDeviceInfo>(get_device_info_srv);

      if (device.get_device_info_client.waitForExistence())
      {
        ROS_INFO_STREAM(log_ << "Connected to [" << get_device_info_srv << "] service." );
      }
      else
      {
        ROS_ERROR_STREAM(log_ << "Error while trying to connect to [" << get_device_info_srv << "] service." );
        return false;
      }

      // Test CheckerboardExtraction action
      std::string cb_extraction_action = node_name + "/extract_checkerboard";
      device.cb_extraction_client = std::make_shared<CheckerboardExtractionClient>(cb_extraction_action);

      if (device.cb_extraction_client->waitForServer())
      {
        ROS_INFO_STREAM(log_ << "Connected to [" << cb_extraction_action << "] action server." );
      }
      else
      {
        ROS_ERROR_STREAM(log_ << "Error while trying to connect to [" << cb_extraction_action << "] action server." );
        return false;
      }
    }
  }

  ROS_INFO_STREAM(log_ << "Getting device infos...");

  std::vector<std::shared_ptr<PinholeSensor>> pinhole_sensors;
  std::vector<std::shared_ptr<PinholeDepthSensor>> depth_sensors;

  // Get device infos
  for (PC & pc : pcs_)
    for (Device & device : pc.device_vec)
      if (not callGetDeviceInfo(device, pinhole_sensors, depth_sensors))
        return false;

  // Add sensors
  for (const auto & sensor : pinhole_sensors)
  {
    frame_ids_[SENSORS].push_back(sensor->frameId());
    calibration_.addSensor(*sensor);
  }
  for (const auto & sensor : depth_sensors)
  {
    frame_ids_[SENSORS].push_back(sensor->frameId());
    calibration_.addSensor(*sensor);
  }

  ROS_INFO_STREAM(log_ << "Initialization complete.");

  return true;
}

void
CalibrationMasterNode::actionCallback (const std_msgs::String::ConstPtr & msg)
{
  if (msg->data == "begin plane")
  {
    if (not plane_flag_)
      frame_ids_[PLANES].push_back(calibration_.beginPlane());
    else
      ROS_ERROR_STREAM(log_ << "Action \"begin plane\" already called. Ignoring.");
  }
  else if (msg->data == "end plane")
  {
    if (plane_flag_)
      calibration_.endPlane();
    else
      ROS_ERROR_STREAM(log_ << "Action \"end plane\" already called or \"begin plane\" never called. Ignoring.");
  }
}

void
CalibrationMasterNode::acquisitionCallback (const std_msgs::Empty::ConstPtr & msg)
{
  frame_ids_[CHECKERBOARDS].push_back(calibration_.beginStep(*checkerboard_));
  performAcquisition();
  calibration_.endStep();
}

bool
CalibrationMasterNode::getResultsCallback (msgs::GetCalibrationResults::Request & request,
                                           msgs::GetCalibrationResults::Response & response)
{
  response.poses.reserve(frame_ids_[SENSORS].size());

  for (auto sensor_frame : frame_ids_[SENSORS])
  {
    if (calibration_.isPoseEstimated(sensor_frame))
    {
      const std::shared_ptr<const BaseObject> & sensor = calibration_.get(sensor_frame);
      msgs::CalibrationPose pose;
      pose.frame_id = sensor->parent()->frameId();
      pose.child_frame_id = sensor->frameId();
      tf::poseEigenToMsg(sensor->pose(), pose.pose);
      response.poses.push_back(pose);
    }
  }

  return true;
}

void
CalibrationMasterNode::spinOnce ()
{
  ros::spinOnce();

  publish(*std::static_pointer_cast<const Checkerboard>(calibration_.get(frame_ids_[CHECKERBOARDS].back())), "checkerboard", 0);

  if (plane_flag_)
    publish(*std::static_pointer_cast<const PlanarObject>(calibration_.get(frame_ids_[PLANES].back())), "plane", 0);

  int index = 0;
  for (auto sensor_frame : frame_ids_[SENSORS])
    publish(*calibration_.get(sensor_frame), "sensor", index++);
}

void
CalibrationMasterNode::performAcquisition ()
{
  msgs::CheckerboardExtractionGoal goal;
  goal.header.stamp = ros::Time::now();
//TODO  goal.header.seq = calibration_.currentStepIndex();
  goal.extract_from = msgs::CheckerboardExtractionGoal::EXTRACT_FROM_ALL;
  goal.max_depth_points = msgs::CheckerboardExtractionGoal::PLANE_ONLY;
  goal.checkerboard = toMessage(*checkerboard_);

  // Send goals
  for (PC & pc : pcs_)
  {
    for (Device & device : pc.device_vec)
    {
      ROS_DEBUG_STREAM(log_ << "Sending goal to [" << device.name << "]...");
      device.cb_extraction_client->sendGoal(goal);
    }
  }

  // Get results
  for (PC & pc : pcs_)
  {
    for (Device & device : pc.device_vec)
    {
      ROS_DEBUG_STREAM(log_ << "Waiting for [" << device.name << "] reply...");
      if (device.cb_extraction_client->waitForResult())
      {
        ROS_DEBUG_STREAM(log_ << "Result from [" << device.name << "] received.");
        const msgs::CheckerboardExtractionResult & result_msg = *device.cb_extraction_client->getResult();

        for (const msgs::Point2DArray & points_msg : result_msg.image_corners)
        {
          calibration_.addPinholeData(points_msg.header.frame_id, fromMessage(points_msg));
        }

        for (const msgs::PointArray & view_msg : result_msg.depth_points)
        {
          Point3 point_on_plane(view_msg.points[0].x, view_msg.points[0].y, view_msg.points[0].z);
          Vector3 normal(view_msg.points[1].x, view_msg.points[1].y, view_msg.points[1].z);
          calibration_.addDepthData(view_msg.header.frame_id, Plane3(normal, point_on_plane));
        }

      }
    }
  }
}

} // namespace calib
} // namespace unipd

int
main (int argc, char ** argv)
{
  ros::init(argc, argv, "master_node");
  ros::NodeHandle node_handle("~");

  unipd::calib::CalibrationMasterNode calib_node(node_handle);
  if (not calib_node.initialize())
    return 1;

  ros::spin();

  return 0;
}
