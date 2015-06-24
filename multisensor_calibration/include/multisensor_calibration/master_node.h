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

#ifndef UNIPD_CALIBRATION_MULTISENSOR_CALIBRATION_CALIBRATION_MASTER_NODE_H_
#define UNIPD_CALIBRATION_MULTISENSOR_CALIBRATION_CALIBRATION_MASTER_NODE_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>

#include <calibration_ros/visualization/objects.h>
#include <calibration_common/objects/checkerboard.h>

#include <actionlib/client/simple_action_client.h>
#include <calibration_msgs/CheckerboardExtractionAction.h>
#include <calibration_msgs/GetCalibrationResults.h>

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
//#include <opt_msgs/CalibrationStatus.h>

#include <multisensor_calibration/calibration.h>

using namespace actionlib;

namespace unipd
{
namespace calib
{

using CheckerboardExtractionClient = SimpleActionClient<calibration_msgs::CheckerboardExtractionAction>;

class CalibrationMasterNode
{
public:

  struct Device
  {
    std::string name;
    ros::ServiceClient get_device_info_client;
    std::shared_ptr<CheckerboardExtractionClient> cb_extraction_client;
  };

  struct PC
  {
    std::string name;
    std::vector<Device> device_vec;
  };

  CalibrationMasterNode (const ros::NodeHandle & node_handle);

  void
  actionCallback (const std_msgs::String::ConstPtr & msg);

  void
  acquisitionCallback (const std_msgs::Empty::ConstPtr & msg);

  bool
  getResultsCallback (calibration_msgs::GetCalibrationResults::Request & request,
                      calibration_msgs::GetCalibrationResults::Response & response);

  bool
  initialize ();

  bool
  callGetDeviceInfo (Device & device,
                     std::vector<std::shared_ptr<PinholeSensor>> & pinhole_sensors,
                     std::vector<std::shared_ptr<PinholeDepthSensor>> & depth_sensors);

  void
  performAcquisition ();

  void
  spinOnce ();

  void
  spin ();

private:

  bool
  parseNetworkFile (const std::string & filename);

  template <typename ObjectT_>
    void
    publish (const ObjectT_ & object,
             const std::string & ns,
             int id)
    {
      if (not object.hasParent())
        return;
      auto marker = toMarker(object);
      marker.ns = ns;
      marker.id = id;
      marker_pub_.publish(marker);
      tf_pub_.sendTransform(toTransformStamped(object));
    }

  ros::NodeHandle node_handle_;

  std::string log_;

  ros::Subscriber acquisition_sub_;
  ros::Subscriber action_sub_;
  ros::Publisher marker_pub_;
  ros::ServiceServer result_service_;
  tf::TransformBroadcaster tf_pub_;

  std::shared_ptr<Checkerboard> checkerboard_;
  std::vector<PC> pcs_;

  enum : int {SENSORS = 0, CHECKERBOARDS = 1, PLANES = 2, N = 3};
  std::vector<std::string> frame_ids_[N];

  Calibration calibration_;
  bool plane_flag_;
};

} // namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_MULTISENSOR_CALIBRATION_CALIBRATION_MASTER_NODE_H_
