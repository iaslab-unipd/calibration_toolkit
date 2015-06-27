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

#include <calibration_algorithms/plane_to_plane_calibration.h>
#include <multisensor_calibration/calibration.h>

namespace unipd
{
namespace calib
{
void
Calibration::addSensorConstraint (const std::string & sensor_id,
                                  const std::string & parent_id,
                                  const Transform3 & transform)
{
  assert(sensor_index_.count(sensor_id) > 0);
  int sensor_index = sensor_index_[sensor_id];
  auto sensor = matrix_.sensorInfo(sensor_index)->sensor;

  assert(sensor_index_.count(parent_id) > 0);
  int parent_index = sensor_index_[parent_id];
  auto parent = matrix_.sensorInfo(parent_index)->sensor;

  sensor->setParent(parent);
  sensor->transform(transform);
}

std::shared_ptr<Checkerboard>
Calibration::createCheckerboard_ (const Checkerboard & checkerboard,
                                  int id) const
{
  auto new_checkerboard = std::make_shared<Checkerboard>(checkerboard);
  std::stringstream ss;
  ss << checkerboard.frameId() << "_" << id;
  new_checkerboard->setFrameId(ss.str());
  return new_checkerboard;
}

const std::string &
Calibration::beginStep (const Checkerboard & checkerboard)
{
  auto new_checkerboard = createCheckerboard_(checkerboard, current_step_ + 1);
  addObject_(new_checkerboard);
  setPoseState_(new_checkerboard, State::NO);

  if (current_plane_)
    current_step_ = matrix_.newStep(new_checkerboard, current_plane_);
  else
    current_step_ = matrix_.newStep(new_checkerboard);

  ROS_DEBUG_STREAM(log_ << "==> Begin step " << current_step_ << " [" << new_checkerboard->frameId() << "]");
  return new_checkerboard->frameId();
}

void
Calibration::endStep ()
{
  step_update_queue_.push(current_step_);
  auto step_info = matrix_.stepInfo(current_step_);
  setPoseState_(step_info->checkerboard, State::IN_QUEUE);

  bool need_to_optimize = false;

  while (not step_update_queue_.empty())
  {
    while (not step_update_queue_.empty())
    {
      int step_index = step_update_queue_.front();
      step_update_queue_.pop();
      auto step_info = matrix_.stepInfo(step_index);

      if (tryEstimateCheckerboardPose_(current_step_))
      {
        need_to_optimize = true;
        setPoseState_(step_info->checkerboard, State::YES);
        optimization_.addObject(step_info->checkerboard);

        if (step_info->type == StepInfo::Type::ON_PLANE)
        {
          if (not isPoseEstimated_(step_info->plane))
          {
            setPoseState_(step_info->plane, State::YES);
            optimization_.addObject(step_info->plane);
            optimization_.setPlaneTransform(step_info->plane);
          }
          optimization_.set3DOFTransform(step_info->checkerboard);
        }
        else
        {
          optimization_.set6DOFTransform(step_info->checkerboard);
        }

        for (int sensor_index : step_info->sensors)
        {
          auto sensor_info = matrix_.sensorInfo(sensor_index);
          if (not isPoseEstimated_(sensor_info->sensor) and not inQueue_(sensor_info->sensor))
          {
            sensor_update_queue_.push(sensor_index);
            setPoseState_(sensor_info->sensor, State::IN_QUEUE);
          }
          else
          {
            addOptimizationConstraint(step_index, sensor_index);
          }
        }
      }
      else
      {
        setPoseState_(step_info->checkerboard, State::NO);
      }
    }

    while (not sensor_update_queue_.empty())
    {
      int sensor_index = sensor_update_queue_.front();
      sensor_update_queue_.pop();
      auto sensor_info = matrix_.sensorInfo(sensor_index);

      if (tryEstimateSensorPose_(sensor_index))
      {
        need_to_optimize = true;
        setPoseState_(sensor_info->sensor, State::YES);
        optimization_.addObject(sensor_info->sensor);
        optimization_.set6DOFTransform(sensor_info->sensor);
        ROS_INFO_STREAM(log_ << " +  Object [" << sensor_info->sensor->frameId() << "] pose estimated:\n"
                             << *sensor_info->sensor);

        for (int step_index : sensor_info->steps)
        {
          auto step_info = matrix_.stepInfo(step_index);
          if (not isPoseEstimated_(step_info->checkerboard) and not inQueue_(step_info->checkerboard))
          {
            step_update_queue_.push(step_index);
            setPoseState_(step_info->checkerboard, State::IN_QUEUE);
          }
          else
          {
            addOptimizationConstraint(step_index, sensor_index);
          }
        }
      }
      else
      {
        setPoseState_(sensor_info->sensor, State::NO);
      }
    }
  }

  if (need_to_optimize)
  {
    optimization_.perform();
    for (auto pair : all_objects_)
    {
      if (pair.second != world_ and isPoseEstimated_(pair.second))
      {
        Transform3 t = optimization_.estimatedTransform(pair.second);
        pair.second->transform(t);
      }
    }

    optimization_.reset();
  }

  ROS_DEBUG_STREAM(log_ << "<== End step " << current_step_ << " [" << matrix_.stepInfo(current_step_)->checkerboard->frameId() << "]");
}

std::shared_ptr<PlanarObject>
Calibration::createPlane_ (int id) const
{
  auto new_plane = std::make_shared<PlanarObject>(Plane3(Vector3::UnitZ(), 0));
  std::stringstream ss;
  ss << "/plane_" << id;
  new_plane->setFrameId(ss.str());
  return new_plane;
}

const std::string &
Calibration::beginPlane ()
{
  assert(not current_plane_);
  current_plane_ = createPlane_(current_step_);
  addObject_(current_plane_);
  setPoseState_(current_plane_, State::NO);

  ROS_DEBUG_STREAM(log_ << "--> Begin plane [" << current_plane_->frameId() << "]");
  return current_plane_->frameId();
}

void
Calibration::endPlane ()
{
  assert(current_plane_);
  ROS_DEBUG_STREAM(log_ << "<-- End plane [" << current_plane_->frameId() << "]");
  current_plane_.reset();
}

std::shared_ptr<const BaseObject>
Calibration::get (const std::string & frame_id) const
{
  assert(all_objects_.count(frame_id) > 0);
  return all_objects_.at(frame_id);
}

bool
Calibration::isPoseEstimated (const std::string & frame_id) const
{
  if (all_objects_.count(frame_id) == 0)
    return false;
  return pose_estimated_.at(all_objects_.at(frame_id)) == State::YES;
}

// protected:

Calibration::EstimatedPose
Calibration::estimateSensorPose (const SensorInfo & sensor_info)
{
  if (sensor_info.type == SensorInfo::Type::INTENSITY)
  {
    for (int step : sensor_info.steps)
    {
      std::shared_ptr<StepInfo> & step_info = matrix_.stepInfo(step);
      if (isPoseEstimated_(step_info->checkerboard))
      {
        std::shared_ptr<const IntensityData> data = matrix_.data<IntensityData>(step, sensor_info.index);
        std::shared_ptr<PinholeSensor> sensor = std::static_pointer_cast<PinholeSensor>(sensor_info.sensor);
        Transform3 inverse_transform = sensor->cameraModel().estimatePose(data->corners, step_info->initialCorners());
        return EstimatedPose{true, step_info->checkerboard, inverse_transform.inverse()};
      }
    }
  }
  else // (sensor_info.type == SensorInfo::Type::DEPTH)
  {
    PlaneToPlaneCalibration calibration_alg;
    for (int step : sensor_info.steps)
    {
      std::shared_ptr<StepInfo> & step_info = matrix_.stepInfo(step);
      if (isPoseEstimated_(step_info->checkerboard))
      {
        std::shared_ptr<const DepthData> data = matrix_.data<DepthData>(step, sensor_info.index);
        if (step_info->type == StepInfo::Type::NORMAL)
          calibration_alg.addPlanePair(std::make_pair(step_info->checkerboard->plane(), data->plane));
        else //(step_info->type == StepInfo::Type::ON_PLANE)
          calibration_alg.addPlanePair(std::make_pair(step_info->plane->plane(), data->plane));

      }
    }
    if (calibration_alg.canEstimateTransform())
      return EstimatedPose{true, world_, calibration_alg.estimateTransform()};

  }

  return EstimatedPose{false};
}

Calibration::EstimatedPose
Calibration::estimateCheckerboardPose (const StepInfo & step_info)
{
  for (int sensor_index : step_info.sensors)
  {
    std::shared_ptr<SensorInfo> & sensor_info = matrix_.sensorInfo(sensor_index);
    if (sensor_info->type == SensorInfo::Type::INTENSITY and isPoseEstimated_(sensor_info->sensor))
    {
      std::shared_ptr<const IntensityData> data = matrix_.data<IntensityData>(step_info.index, sensor_index);
      std::shared_ptr<PinholeSensor> sensor = std::static_pointer_cast<PinholeSensor>(sensor_info->sensor);
      Transform3 transform = sensor->cameraModel().estimatePose(data->corners, step_info.checkerboard->corners());
      return EstimatedPose{true, sensor, transform};
    }
  }

  return EstimatedPose{false};
}

void
Calibration::projectCheckerboardOnPlane (const std::shared_ptr<Checkerboard> checkerboard,
                                         const std::shared_ptr<const PlanarObject> plane) const
{
  checkerboard->transform(plane->pose().inverse());
  Quaternion q = Quaternion::FromTwoVectors(checkerboard->pose().rotation().col(2), Vector3::UnitZ());
  checkerboard->transform(Transform3::Identity() * q);
  Vector3 t = Plane3(Vector3::UnitZ(), 0).projection(checkerboard->pose().translation()) - checkerboard->pose().translation();
  checkerboard->transform(Transform3::Identity() * Translation3(t));
  checkerboard->setParent(plane);
}

// private:

bool
Calibration::tryEstimateSensorPose_ (int sensor_index)
{
  std::shared_ptr<SensorInfo> & sensor_info = matrix_.sensorInfo(sensor_index);
  assert(not isPoseEstimated_(sensor_info->sensor));

  if (sensor_info->sensor->hasParent() and not isPoseEstimated_(sensor_info->sensor->parent()))
    return false;

  bool pose_estimated = false;
  if (sensor_info->sensor->hasParent() and isPoseEstimated_(sensor_info->sensor->parent()))
  {
    transformToWorldCoordinates_(sensor_info->sensor);
    pose_estimated = true;
  }
  else
  {
    EstimatedPose pose = estimateSensorPose(*sensor_info);
    if (pose.estimated)
    {
      sensor_info->sensor->transform(pose.transform);
      sensor_info->sensor->setParent(pose.parent);
      transformToWorldCoordinates_(sensor_info->sensor);
      pose_estimated = true;
    }
  }

  return pose_estimated;
}

bool
Calibration::tryEstimateCheckerboardPose_ (int step)
{
  std::shared_ptr<StepInfo> & step_info = matrix_.stepInfo(step);
  assert(not isPoseEstimated_(step_info->checkerboard));

  bool pose_estimated = false;

  EstimatedPose pose = estimateCheckerboardPose(*step_info);
  if (pose.estimated)
  {
    step_info->checkerboard->transform(pose.transform);
    step_info->checkerboard->setParent(pose.parent);
    transformToWorldCoordinates_(step_info->checkerboard);
    pose_estimated = true;

    if (step_info->type == StepInfo::Type::ON_PLANE)
    {
      if (not isPoseEstimated_(step_info->plane))
      {
        step_info->plane->transform(pose.transform);
        step_info->plane->setParent(pose.parent);
        transformToWorldCoordinates_(step_info->plane);
      }
      projectCheckerboardOnPlane(step_info->checkerboard, step_info->plane);
    }

  }

  return pose_estimated;
}

void
Calibration::addOptimizationConstraint (int step_index,
                                        int sensor_index)
{
  assert (matrix_.data<Data>(step_index, sensor_index));

  ROS_DEBUG_STREAM(log_ << " +  addOptimizationConstraint(" << step_index << ", " << sensor_index << ")");

  const std::shared_ptr<SensorInfo> & sensor_info = matrix_.sensorInfo(sensor_index);
  const std::shared_ptr<StepInfo> & step_info = matrix_.stepInfo(step_index);
  if (sensor_info->type == SensorInfo::Type::INTENSITY)
  {
    if (step_info->type == StepInfo::Type::ON_PLANE)
    {
      optimization_.addConstraint(step_info->checkerboard,
                                  step_info->plane,
                                  std::static_pointer_cast<PinholeSensor>(sensor_info->sensor),
                                  matrix_.data<IntensityData>(step_index, sensor_index));
    }
    else
    {
      optimization_.addConstraint(step_info->checkerboard,
                                  std::static_pointer_cast<PinholeSensor>(sensor_info->sensor),
                                  matrix_.data<IntensityData>(step_index, sensor_index));
    }
  }
  else
  {
    if (step_info->type == StepInfo::Type::ON_PLANE)
    {
      optimization_.addConstraint(step_info->checkerboard,
                                  step_info->plane,
                                  std::static_pointer_cast<DepthSensor>(sensor_info->sensor),
                                  matrix_.data<DepthData>(step_index, sensor_index));
    }
    else
    {
      optimization_.addConstraint(step_info->checkerboard,
                                  std::static_pointer_cast<DepthSensor>(sensor_info->sensor),
                                  matrix_.data<DepthData>(step_index, sensor_index));
    }
  }
}

void
Calibration::transformToWorldCoordinates_ (const std::shared_ptr<BaseObject> & object)
{
  assert(object->hasParent() and not isPoseEstimated_(object) and isPoseEstimated_(object->parent()));

  while (object->parent()->hasParent())
  {
    object->transform(object->parent()->pose());
    object->setParent(object->parent()->parent());
  }
}

} // namespace calib
} // namespace unipd
