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

#ifndef UNIPD_CALIBRATION_MULTISENSOR_CALIBRATION_CALIBRATION_H_
#define UNIPD_CALIBRATION_MULTISENSOR_CALIBRATION_CALIBRATION_H_

#include <queue>

#include <ros/console.h>

#include <calibration_common/objects/checkerboard.h>
#include <calibration_common/pinhole/sensor.h>
#include <calibration_common/depth/sensor.h>

#include <multisensor_calibration/common.h>
#include <multisensor_calibration/optimization.h>

namespace unipd
{
namespace calib
{

struct SensorInfo
{
  enum State : int {NO_POSE = 0, ESTIMATING_POSE, YES_POSE, OPTIMIZING_POSE};
  enum class Type {INTENSITY, DEPTH};

  SensorInfo (int index, const std::shared_ptr<Sensor> & sensor, Type type) : index(index), sensor(sensor), type(type) {}
  SensorInfo (int index, const std::shared_ptr<DepthSensor> & sensor) : SensorInfo(index, sensor, Type::DEPTH) {}
  SensorInfo (int index, const std::shared_ptr<PinholeSensor> & sensor) : SensorInfo(index, sensor, Type::INTENSITY) {}

  const int index;
  const std::shared_ptr<Sensor> sensor;
  const Type type;
  State state = State::NO_POSE;
  std::vector<int> steps;
};

struct StepInfo
{
  enum State : int {NO_POSE = 0, ESTIMATING_POSE, YES_POSE, OPTIMIZING_POSE};
  enum class Type {NORMAL, PLANE};

  StepInfo (int index, const std::shared_ptr<Checkerboard> & checkerboard)
    : index(index), checkerboard(checkerboard), plane(), type(Type::NORMAL) {}
  StepInfo (int index, const std::shared_ptr<Checkerboard> & checkerboard, const std::shared_ptr<PlanarObject> & plane)
    : index(index), checkerboard(checkerboard), plane(plane), type(Type::PLANE) {}

  const int index;
  const std::shared_ptr<Checkerboard> checkerboard;
  const std::shared_ptr<PlanarObject> plane;
  const Type type;
  State state = State::NO_POSE;
  std::vector<int> sensors;

  Cloud3
  initialCorners () const
  {
    Cloud3 corners = checkerboard->corners();
    corners.transform(checkerboard->pose().inverse());
    return corners;
  }
};

class CalibrationMatrix
{
public:

  template <typename SensorT_>
  int
  addSensor (const std::shared_ptr<SensorT_> & sensor)
  {
    sensor_info_.push_back(std::make_shared<SensorInfo>(sensor_info_.size(), sensor));
    return sensor_info_.size() - 1;
  }

  int
  newStep (const std::shared_ptr<Checkerboard> & checkerboard,
           const std::shared_ptr<PlanarObject> & plane)
  {
    step_info_.push_back(std::make_shared<StepInfo>(step_info_.size(), checkerboard, plane));
    data_.resize(data_.size() + sensor_info_.size());
    return step_info_.size() - 1;
  }

  inline int index (int step, int sensor_index) const { return sensor_info_.size() * step + sensor_index; }

  template <typename DataT_>
    std::shared_ptr<const DataT_>
    data (int step,
          int sensor_index) const
    {
      return std::static_pointer_cast<const DataT_>(data_[index(step, sensor_index)]);
    }

  void
  addData (int step,
           int sensor_index,
           const std::shared_ptr<const Data> & data_ptr)
  {
    assert(not data<Data>(step, sensor_index));
    data_[index(step, sensor_index)] = data_ptr;
    stepInfo(step)->sensors.push_back(sensor_index);
    sensorInfo(sensor_index)->steps.push_back(step);
  }

  std::shared_ptr<StepInfo> &
  stepInfo (int step)
  {
    return step_info_[step];
  }

  std::shared_ptr<SensorInfo> &
  sensorInfo (int sensor_index)
  {
    return sensor_info_[sensor_index];
  }

private:

  std::vector<std::shared_ptr<const Data>> data_;
  std::vector<std::shared_ptr<StepInfo>> step_info_;
  std::vector<std::shared_ptr<SensorInfo>> sensor_info_;

};

class Calibration
{
public:

  virtual
  ~Calibration () {}

  template <typename SensorT_>
    void
    addSensor (const SensorT_ & sensor);

  template <typename Cloud2T_>
    void
    addPinholeData (const std::string & sensor_id,
                    Cloud2T_ && corners);

  template <typename Plane3T_>
    void
    addDepthData (const std::string & sensor_id,
                  Plane3T_ && plane);

  const std::string &
  beginStep (const Checkerboard & checkerboard);

  void
  endStep ();

  const std::string &
  beginPlane ();

  void
  endPlane ();

  const std::shared_ptr<const BaseObject> &
  get (const std::string & frame_id);

  bool
  isPoseEstimated (const std::string & frame_id);

protected:

  struct EstimatedPose
  {
    std::shared_ptr<const BaseObject> parent;
    Transform3 transform;
  };

  virtual EstimatedPose
  estimateSensorPose (const SensorInfo & sensor_info);

  virtual EstimatedPose
  estimateCheckerboardPose (const StepInfo & step_info);

private:

  // Methods

  void
  addOptimizationConstraint (int step,
                             int sensor_index);

  bool
  tryAddSensorToTree (int sensor_index);

  bool
  tryAddCheckerboardToTree (int step);

  bool
  inTree (const std::shared_ptr<const BaseObject> & object) const;

  void
  addToTree (const std::shared_ptr<BaseObject> & object,
             const std::shared_ptr<const BaseObject> & parent,
             const Pose3 & pose);

  void
  addToTree (const std::shared_ptr<BaseObject> & object);

  void
  initTree (const std::shared_ptr<Sensor> & sensor);

  // Variables

  Optimization optimization_;
  std::string log_ = "[/calibration] ";

  CalibrationMatrix matrix_;
  std::map<std::string, int> sensor_index_;

  std::shared_ptr<BaseObject> world_ = std::make_shared<BaseObject>("/world");
  std::set<std::shared_ptr<const BaseObject>> tree_;
  std::map<std::string, std::shared_ptr<const BaseObject>> all_objects_;

  int current_step_ = -1;
  std::shared_ptr<PlanarObject> current_plane_;
  std::vector<int> current_plane_steps_;

  std::queue<int> update_queue_;
  int fixed_sensor_index_;

};

template <typename SensorT_>
  void
  Calibration::addSensor (const SensorT_ & sensor)
  {
    assert(sensor_index_.count(sensor.frameId()) == 0);
    auto new_sensor = std::make_shared<SensorT_>(sensor);
    sensor_index_[sensor.frameId()] = matrix_.addSensor(new_sensor);
    all_objects_[new_sensor->frameId()] = new_sensor;

    optimization_.addObject(new_sensor);
    optimization_.set6DOFTransform(new_sensor);
    optimization_.setFixed(new_sensor);
    ROS_INFO_STREAM(log_ << "Sensor [" << sensor.frameId() << "] added.");
  }

template <typename Cloud2T_>
  void
  Calibration::addPinholeData (const std::string & sensor_id,
                               Cloud2T_ && corners)
  {
    assert(sensor_index_.count(sensor_id) > 0);
    int sensor_index = sensor_index_[sensor_id];
    auto sensor = matrix_.sensorInfo(sensor_index)->sensor;
    auto data = std::make_shared<IntensityData>(std::forward<Cloud2T_>(corners));

    matrix_.addData(current_step_, sensor_index, data);

    if (tree_.empty() and not sensor->hasParent())
    {
      fixed_sensor_index_ = sensor_index;
      initTree(sensor);
    }

    ROS_DEBUG_STREAM(log_ << "Data from sensor [" << sensor_id << "] added.");
  }

template <typename Plane3T_>
  void
  Calibration::addDepthData (const std::string & sensor_id,
                             Plane3T_ && plane)
  {
    assert(sensor_index_.count(sensor_id) > 0);
    auto data = std::make_shared<DepthData>(std::forward<Plane3T_>(plane));

    matrix_.addData(current_step_, sensor_index_[sensor_id], data);

    ROS_DEBUG_STREAM(log_ << "Data from sensor [" << sensor_id << "] added.");
  }

} // namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_MULTISENSOR_CALIBRATION_CALIBRATION_H_
