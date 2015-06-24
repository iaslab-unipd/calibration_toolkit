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

#include <pcl/common/centroid.h>
#include <eigen_conversions/eigen_msg.h>

#include <calibration_algorithms/plane_fit.h>
#include <calibration_msgs/calibration_common_conversions.h>
#include <calibration_pcl/depth/view.h>
#include <calibration_pcl/base/pcl_eigen_conversions.h>

#include <multisensor_calibration/device_node.h>

namespace unipd
{
namespace calib
{

namespace msgs = calibration_msgs;

DeviceNode::DeviceNode (const ros::NodeHandle & node_handle)
  : node_handle_(node_handle),
    cb_extraction_server_(node_handle,
                          "extract_checkerboard",
                          boost::bind(&DeviceNode::extractCheckerboard, this, _1),
                          false),
    device_(ros::NodeHandle(node_handle, "device")),
    log_("[" + node_handle.getNamespace() + "] ")
{
  for (const std::shared_ptr<ROSPinholeCamera> & sensor : device_.intensitySensors())
    sensor->subscribe(ROSPinholeCamera::Topic::ALL);
  for (const std::shared_ptr<ROSDepthSensor> & sensor : device_.depthSensors())
    sensor->subscribe(ROSDepthSensor::Topic::CAMERA_INFO | ROSDepthSensor::Topic::POINT_CLOUD);
}

bool
DeviceNode::initialize ()
{
  ROS_INFO_STREAM(log_ << "Waiting for messages...");
  if (device_.waitForAllMessages())
    ROS_INFO_STREAM(log_ << "All messages received.");
  else
    return false;

  get_sensor_info_srv_ = node_handle_.advertiseService("get_device_info", &DeviceNode::getDeviceInfo, this);
  ROS_INFO_STREAM(log_ << "[" << node_handle_.getNamespace() << "/get_device_info] service started.");

  cb_extraction_server_.start();
  ROS_INFO_STREAM(log_ << "[" << node_handle_.getNamespace() << "/extract_checkerboard] action server started.");

  return true;
}

bool
DeviceNode::getDeviceInfo (msgs::GetDeviceInfo::Request & request,
                           msgs::GetDeviceInfo::Response & response)
{
  if (request.requested_types & msgs::GetDeviceInfo::Request::INTENSITY)
  {
    response.sensor_types.resize(device_.intensitySensors().size(), msgs::GetDeviceInfo::Response::INTENSITY);
    response.error_polynomials.resize(device_.intensitySensors().size());
    for (const std::shared_ptr<ROSPinholeCamera> & sensor : device_.intensitySensors())
    {
      response.camera_infos.push_back(sensor->sensor()->cameraModel().cameraInfo());
      if (sensor->sensor()->hasParent())
      {
        geometry_msgs::TransformStamped transform_msg;
        transform_msg.child_frame_id = sensor->sensor()->frameId();
        transform_msg.header.frame_id = sensor->sensor()->parent()->frameId();
        tf::transformEigenToMsg(sensor->sensor()->pose(), transform_msg.transform);
        response.initial_transforms.push_back(transform_msg);
      }
    }
  }
  if (request.requested_types & msgs::GetDeviceInfo::Request::DEPTH)
  {
    response.sensor_types.resize(response.sensor_types.size() + device_.depthSensors().size(), msgs::GetDeviceInfo::Response::DEPTH);
    for (const std::shared_ptr<ROSDepthSensor> & sensor : device_.depthSensors())
    {
      response.camera_infos.push_back(sensor->sensor()->cameraModel().cameraInfo());
      response.error_polynomials.push_back(toMessage(sensor->sensor()->error(), sensor->sensor()->error().coefficients()));
      if (sensor->sensor()->hasParent())
      {
        geometry_msgs::TransformStamped transform_msg;
        transform_msg.child_frame_id = sensor->sensor()->frameId();
        transform_msg.header.frame_id = sensor->sensor()->parent()->frameId();
        tf::transformEigenToMsg(sensor->sensor()->pose(), transform_msg.transform);
        response.initial_transforms.push_back(transform_msg);
      }
    }
  }

  return true;
}

CheckerboardExtraction::ImageResult
DeviceNode::extractCheckerboardFromSensor (const std::shared_ptr<ROSPinholeCamera> & sensor,
                                           const Checkerboard & checkerboard,
                                           int extraction_number)
{
  ROSPinholeCamera::Data image_data = sensor->convertMessages(sensor->lastMessages());
  auto checkerboard_ptr = std::make_shared<Checkerboard>(checkerboard);

  CheckerboardExtraction ex;
  ex.setImage(image_data.image);
  ex.setColorSensor(sensor->sensor());
  ex.setCheckerboard(checkerboard_ptr);
  ex.setExtractionNumber(extraction_number);

  auto image_result = ex.performImage();
  ROS_DEBUG_STREAM_COND(image_result.extracted, log_ << "Checkerboard corners extracted.");

  return image_result;
}

CheckerboardExtraction::DepthResult
DeviceNode::extractCheckerboardFromSensor (const std::shared_ptr<ROSDepthSensor> & sensor,
                                           const CheckerboardExtraction::ImageResult & image_result,
                                           int extraction_number)
{
  assert(sensor->sensor()->hasParent() and image_result.view->sensor()->frameId() == sensor->sensor()->parent()->frameId());
  ROSDepthSensor::Data depth_data = sensor->convertMessages(sensor->lastMessages());

  CheckerboardExtraction ex;
  ex.setCloud(depth_data.cloud);
  ex.setDepthSensor(sensor->sensor());
  ex.setColorToDepthTransform(sensor->sensor()->pose());
  ex.setCheckerboard(image_result.view->object());
  ex.setExtractionNumber(extraction_number);

  auto depth_result = ex.performDepth(image_result);
  ROS_DEBUG_STREAM_COND(depth_result.extracted, log_ << "Checkerboard plane extracted.");

  return depth_result;
}

void
DeviceNode::extractCheckerboard (const msgs::CheckerboardExtractionGoal::ConstPtr & goal_msg)
{
  ROS_DEBUG_STREAM(log_ << "Extraction request received.");

  msgs::CheckerboardExtractionResult result_msg;
  std::map<std::shared_ptr<const BaseObject>, CheckerboardExtraction::ImageResult> result_map;

  if ((goal_msg->extract_from & msgs::CheckerboardExtractionGoal::EXTRACT_FROM_IMAGE) |
      (goal_msg->extract_from & msgs::CheckerboardExtractionGoal::EXTRACT_FROM_DEPTH))
  {
    for (const std::shared_ptr<ROSPinholeCamera> & sensor : device_.intensitySensors())
    {
      auto image_result = extractCheckerboardFromSensor(sensor, fromMessage(goal_msg->checkerboard), goal_msg->header.seq);
      if (image_result.extracted)
      {
        result_msg.image_corners.push_back(toMessage(image_result.view->points()));
        result_map[sensor->sensor()] = image_result;
      }
    }
  }

  if (goal_msg->extract_from & msgs::CheckerboardExtractionGoal::EXTRACT_FROM_DEPTH)
  {
    for (const std::shared_ptr<ROSDepthSensor> & sensor : device_.depthSensors())
    {
      assert(sensor->sensor()->hasParent());
      if (result_map.find(sensor->sensor()->parent()) == result_map.end())
        continue;

      auto depth_result = extractCheckerboardFromSensor(sensor, result_map[sensor->sensor()->parent()], goal_msg->header.seq);

      if (depth_result.extracted)
      {
        result_msg.depth_points.emplace_back();
        msgs::PointArray & points_msg = result_msg.depth_points.back();
        points_msg.header.frame_id = sensor->sensor()->frameId();

        if (goal_msg->max_depth_points == msgs::CheckerboardExtractionGoal::PLANE_ONLY)
        {
          auto eigen_cloud = Cloud3(pcl2eigen<Cloud3::Container>(*depth_result.view->data(), *depth_result.view->indices()));
          auto plane = plane_fit(eigen_cloud);

          Eigen::Vector4d centroid;
          pcl::compute3DCentroid(*depth_result.view->data(), *depth_result.view->indices(), centroid);

          Line3 line = Line3(Point3::Zero(), centroid.topRows<3>());
          centroid.topRows<3>() = line.intersectionPoint(plane);

          points_msg.points.resize(2);
          points_msg.points[0].x = centroid.x();
          points_msg.points[0].y = centroid.y();
          points_msg.points[0].z = centroid.z();
          points_msg.points[1].x = plane.normal().x();
          points_msg.points[1].y = plane.normal().y();
          points_msg.points[1].z = plane.normal().z();

        }
        else
        {
          auto point_indices = depth_result.view->indices();

          if (goal_msg->max_depth_points == 1 or goal_msg->max_depth_points == 2 or goal_msg->max_depth_points < -1)
          {
            std::stringstream error_ss;
            error_ss << "\"max_depth_points\" must be greater than 2 or ALL_POINTS or PLANE_ONLY! ("
                     << goal_msg->max_depth_points << " provided)";
            ROS_ERROR_STREAM(log_ << error_ss.str());
            cb_extraction_server_.setAborted(msgs::CheckerboardExtractionResult(), error_ss.str());
            return;
          }

          if (goal_msg->max_depth_points != msgs::CheckerboardExtractionGoal::ALL_POINTS and
              goal_msg->max_depth_points < point_indices->size())
          {
            std::random_device rd;
            std::mt19937 generator(rd());
            std::shuffle(point_indices->begin(), point_indices->end(), generator);
            point_indices->resize(goal_msg->max_depth_points);
          }

          points_msg.points.resize(depth_result.view->indices()->size());
          for (auto index : *point_indices)
          {
            const auto & pcl_point = depth_result.view->data()->points[index];
            auto & point = points_msg.points[index];
            point.x = pcl_point.x;
            point.y = pcl_point.y;
            point.z = pcl_point.z;
          }
        }
      }
    }
  }

  cb_extraction_server_.setSucceeded(result_msg);

}

} // namespace calib
} // namespace unipd

int
main (int argc,
      char ** argv)
{
  ros::init(argc, argv, "device_node");
  ros::NodeHandle node_handle("~");

  unipd::calib::DeviceNode device_node(node_handle);

  if (not device_node.initialize())
    return 1;
  ros::spin();

  return 0;
}
