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

enum class State {NO, IN_QUEUE, YES};

struct SensorInfo
{
  enum class Type {INTENSITY, DEPTH};

  SensorInfo (int index, const std::shared_ptr<Sensor> & sensor, Type type) : index(index), sensor(sensor), type(type) {}
  SensorInfo (int index, const std::shared_ptr<DepthSensor> & sensor) : SensorInfo(index, sensor, Type::DEPTH) {}
  SensorInfo (int index, const std::shared_ptr<PinholeSensor> & sensor) : SensorInfo(index, sensor, Type::INTENSITY) {}

  const int index;
  const std::shared_ptr<Sensor> sensor;
  const Type type;

  std::vector<int> steps;
};

struct StepInfo
{
  enum class Type {NORMAL, ON_PLANE};

  StepInfo (int index, const std::shared_ptr<Checkerboard> & checkerboard)
    : index(index), checkerboard(checkerboard), plane(), type(Type::NORMAL) {}
  StepInfo (int index, const std::shared_ptr<Checkerboard> & checkerboard, const std::shared_ptr<PlanarObject> & plane)
    : index(index), checkerboard(checkerboard), plane(plane), type(Type::ON_PLANE) {assert(plane);}

  const int index;
  const std::shared_ptr<Checkerboard> checkerboard;
  const std::shared_ptr<PlanarObject> plane;
  const Type type;

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

  int
  newStep (const std::shared_ptr<Checkerboard> & checkerboard)
  {
    step_info_.push_back(std::make_shared<StepInfo>(step_info_.size(), checkerboard));
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

  void
  addSensorConstraint (const std::string & sensor_id,
                       const std::string & parent_id,
                       const Transform3 & transform);

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

  std::shared_ptr<const BaseObject>
  get (const std::string & frame_id) const;

  bool
  isPoseEstimated (const std::string & frame_id) const;

protected:

  struct EstimatedPose
  {
    bool estimated;
    std::shared_ptr<const BaseObject> parent;
    Transform3 transform;
  };

  virtual EstimatedPose
  estimateSensorPose (const SensorInfo & sensor_info);

  virtual EstimatedPose
  estimateCheckerboardPose (const StepInfo & step_info);

  void
  projectCheckerboardOnPlane (const std::shared_ptr<Checkerboard> checkerboard,
                              const std::shared_ptr<const PlanarObject> plane) const;

private:

  // Methods

  std::shared_ptr<Checkerboard>
  createCheckerboard_ (const Checkerboard & checkerboard,
                      int id) const;

  std::shared_ptr<PlanarObject>
  createPlane_ (int id) const;

  void
  addOptimizationConstraint (int step,
                             int sensor_index);

  bool
  tryEstimateSensorPose_ (int sensor_index);

  bool
  tryEstimateCheckerboardPose_ (int step);

  inline void
  addObject_ (const std::shared_ptr<BaseObject> & object)
  {
    assert(all_objects_.count(object->frameId()) == 0);
    all_objects_[object->frameId()] = object;
    assert(pose_estimated_.count(object) == 0);
    pose_estimated_[object] = State::NO;
  }

  inline bool
  isPoseEstimated_ (const std::shared_ptr<const BaseObject> & object) const
  {
    assert(pose_estimated_.count(object) > 0);
    return pose_estimated_.at(object) == State::YES;
  }

  inline void
  setPoseState_ (const std::shared_ptr<const BaseObject> & object,
                 State state)
  {
    assert(pose_estimated_.count(object) > 0);
    pose_estimated_[object] = state;
  }

  inline bool
  inQueue_ (const std::shared_ptr<const BaseObject> & object)
  {
    assert(pose_estimated_.count(object) > 0);
    return pose_estimated_.at(object) == State::IN_QUEUE;
  }

  void
  transformToWorldCoordinates_ (const std::shared_ptr<BaseObject> & object);

  // Variables

  Optimization optimization_;
  std::string log_ = "[/calibration] ";

  bool initialized_ = false;

  CalibrationMatrix matrix_;
  std::map<std::string, int> sensor_index_;

  std::shared_ptr<BaseObject> world_ = std::make_shared<BaseObject>("/world");
  std::map<std::shared_ptr<const BaseObject>, State> pose_estimated_;
  std::map<std::string, std::shared_ptr<BaseObject>> all_objects_;

  int current_step_ = -1;
  std::shared_ptr<PlanarObject> current_plane_;

  int fixed_sensor_index_;

  std::queue<int> step_update_queue_;
  std::queue<int> sensor_update_queue_;

};

template <typename SensorT_>
  void
  Calibration::addSensor (const SensorT_ & sensor)
  {
    assert(sensor_index_.count(sensor.frameId()) == 0);
    auto new_sensor = std::make_shared<SensorT_>(sensor);
    new_sensor->reset();
    sensor_index_[sensor.frameId()] = matrix_.addSensor(new_sensor);

    addObject_(new_sensor);
    setPoseState_(new_sensor, State::NO);

    ROS_INFO_STREAM(log_ << "Sensor [" << sensor << "] added.");
  }

template <typename Cloud2T_>
  void
  Calibration::addPinholeData (const std::string & sensor_id,
                               Cloud2T_ && corners)
  {
    assert(sensor_index_.count(sensor_id) > 0);
    int sensor_index = sensor_index_[sensor_id];
    auto & sensor = matrix_.sensorInfo(sensor_index)->sensor;
    auto data = std::make_shared<IntensityData>(std::forward<Cloud2T_>(corners));

    matrix_.addData(current_step_, sensor_index, data);

    if (not initialized_ and not sensor->hasParent() and sensor->pose().isApprox(Pose3::Identity()))
    {
      fixed_sensor_index_ = sensor_index;
      sensor->setParent(world_);
      addObject_(world_);
      setPoseState_(world_, State::YES);
      setPoseState_(sensor, State::YES);
      optimization_.addObject(sensor);
      optimization_.set6DOFTransform(sensor);
      optimization_.setFixed(sensor);
      initialized_ = true;
      ROS_INFO_STREAM(log_ << " +  Object [" << world_->frameId() << "] created.");
      ROS_INFO_STREAM(log_ << " +  Object [" << sensor->frameId() << "] pose estimated:\n"
                           << *sensor);
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
