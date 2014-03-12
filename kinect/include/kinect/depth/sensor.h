/*
 *  Copyright (c) 2013-2014, Filippo Basso <bassofil@dei.unipd.it>
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

#ifndef KINECT_DEPTH_SENSOR_H_
#define KINECT_DEPTH_SENSOR_H_

#include <boost/make_shared.hpp>
#include <calibration_common/depth/sensor.h>
#include <kinect/depth/camera_model.h>

namespace calibration
{

template <typename ModelT_>
  class KinectDepthSensor : public DepthSensor
  {
  public:

    typedef boost::shared_ptr<KinectDepthSensor> Ptr;
    typedef boost::shared_ptr<const KinectDepthSensor> ConstPtr;

    KinectDepthSensor()
      : DepthSensor(),
        depth_error_function_(Vector3(0.0, 0.0, 0.0035))
    {
      // Do nothing
    }

    const boost::shared_ptr<const ModelT_> undistortionModel() const
    {
      return undistortion_model_;
    }

    void setUndistortionModel(const boost::shared_ptr<const ModelT_> & undistortion_model)
    {
      undistortion_model_ = undistortion_model;
    }

    const KinectDepthCameraModel::ConstPtr & cameraModel() const
    {
      return camera_model_;
    }

    void setCameraModel(const KinectDepthCameraModel::ConstPtr & camera_model)
    {
      camera_model_ = camera_model;
    }

    const Polynomial<Scalar, 2> depthErrorFunction() const
    {
      return depth_error_function_;
    }

    void setDepthErrorFunction(const Polynomial<Scalar, 2> & depth_error_function)
    {
      depth_error_function_ = depth_error_function;
    }

  private:

    typename KinectDepthCameraModel::ConstPtr camera_model_;
    boost::shared_ptr<const ModelT_> undistortion_model_;
    Polynomial<Scalar, 2> depth_error_function_;

  };

//class KinectDepthSensorEigen : public DepthSensorEigen
//{
//public:
//
//  typedef boost::shared_ptr<KinectDepthSensorEigen> Ptr;
//  typedef boost::shared_ptr<const KinectDepthSensorEigen> ConstPtr;
//
//  typedef DepthSensorEigen Base;
//
//  KinectDepthSensorEigen()
//    : Base()
//  {
//    // Do nothing
//  }
//
//private:
//
//  typename KinectDepthCameraModel::ConstPtr camera_model_;
//
//};
//
//template <typename PCLPointT_>
//class KinectDepthSensorPCL : public DepthSensorPCL<PCLPointT_>
//{
//public:
//
//  typedef boost::shared_ptr<KinectDepthSensorPCL> Ptr;
//  typedef boost::shared_ptr<const KinectDepthSensorPCL> ConstPtr;
//
//  typedef DepthSensorPCL<PCLPointT_> Base;
//
//  KinectDepthSensorPCL()
//    : Base()
//  {
//    // Do nothing
//  }
//
//private:
//
//  typename KinectDepthCameraModel::ConstPtr camera_model_;
//
//};

} /* namespace calibration */
#endif /* KINECT_DEPTH_SENSOR_H_ */
