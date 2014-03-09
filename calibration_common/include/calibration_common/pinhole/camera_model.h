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

#ifndef CALIBRATION_COMMON_PINHOLE_CAMERA_MODEL_H_
#define CALIBRATION_COMMON_PINHOLE_CAMERA_MODEL_H_

#include <image_geometry/pinhole_camera_model.h>
#include <calibration_common/color/camera_model.h>

namespace calibration
{

class PinholeCameraModel : public image_geometry::PinholeCameraModel,
                           virtual public ColorCameraModel
{
public:

  typedef boost::shared_ptr<PinholeCameraModel> Ptr;
  typedef boost::shared_ptr<const PinholeCameraModel> ConstPtr;

  typedef image_geometry::PinholeCameraModel Base;

  PinholeCameraModel()
    : Base()
  {

  }

  PinholeCameraModel(const sensor_msgs::CameraInfo & msg)
    : Base()
  {
    Base::fromCameraInfo(msg);
  }

  PinholeCameraModel(const PinholeCameraModel & other)
    : Base(other)
  {
    // Do nothing
  }

  virtual Point3 projectPixelTo3dRay(const Point2 & pixel_point) const;

  virtual Point2 project3dToPixel(const Point3 & world_point) const;

  virtual void projectPixelTo3dRay(const Point2Matrix & pixel_points,
                                   Point3Matrix & world_points) const;

  virtual Point3Matrix projectPixelTo3dRay(const Point2Matrix & pixel_points) const;

  virtual void project3dToPixel(const Point3Matrix & world_points,
                                Point2Matrix & pixel_points) const;

  virtual Point2Matrix project3dToPixel(const Point3Matrix & world_points) const;

  virtual Pose estimatePose(const Point2Matrix & points_image,
                            const Point3Matrix & points_object) const;

  template <typename Scalar>
    typename Types<Scalar>::Point3 projectPixelTo3dRay(const typename Types<Scalar>::Point2 & pixel_point) const;

  template <typename Scalar>
    typename Types<Scalar>::Point2 project3dToPixel(const typename Types<Scalar>::Point3 & world_point) const;

  template <typename Scalar>
    void projectPixelTo3dRay(const typename Types<Scalar>::Cloud2 & pixel_points,
                             typename Types<Scalar>::Cloud3 & world_points) const;

  template <typename Scalar>
    typename Types<Scalar>::Cloud3 projectPixelTo3dRay(const typename Types<Scalar>::Cloud2 & pixel_points) const;

  template <typename Scalar>
    void project3dToPixel(const typename Types<Scalar>::Cloud3 & world_points,
                          typename Types<Scalar>::Cloud2 & pixel_points) const;

  template <typename Scalar>
    typename Types<Scalar>::Cloud2 project3dToPixel(const typename Types<Scalar>::Cloud3 & world_points) const;

  template <typename Scalar>
    typename Types<Scalar>::Pose estimatePose(const typename Types<Scalar>::Cloud2 & points_image,
                                              const typename Types<Scalar>::Cloud3 & points_object) const;

};

} /* namespace calibration */

#include <impl/calibration_common/pinhole/camera_model.hpp>

#endif /* CALIBRATION_COMMON_PINHOLE_CAMERA_MODEL_H_ */
