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

#ifndef CALIBRATION_COMMON_COLOR_CAMERA_MODEL_H_
#define CALIBRATION_COMMON_COLOR_CAMERA_MODEL_H_

#include <calibration_common/objects/globals.h>

namespace calibration
{

template <typename Scalar>
  class ColorCameraModel_
  {
  public:

    typedef boost::shared_ptr<ColorCameraModel_> Ptr;
    typedef boost::shared_ptr<const ColorCameraModel_> ConstPtr;

    typedef typename Types_<Scalar>::Point2 Point2;
    typedef typename Types_<Scalar>::Point3 Point3;
    typedef typename Types_<Scalar>::Point2Matrix Point2Matrix;
    typedef typename Types_<Scalar>::Point3Matrix Point3Matrix;
    typedef typename Types_<Scalar>::Pose Pose;

    virtual ~ColorCameraModel_()
    {
      // Do nothing
    }

    virtual Point3 projectPixelTo3dRay(const Point2 & pixel_point) const = 0;

    virtual Point2 project3dToPixel(const Point3 & world_point) const = 0;

    virtual void projectPixelTo3dRay(const Point2Matrix & pixel_points,
                                     Point3Matrix & world_points) const = 0;

    virtual Point3Matrix projectPixelTo3dRay(const Point2Matrix & pixel_points) const = 0;

    virtual void project3dToPixel(const Point3Matrix & world_points,
                                  Point2Matrix & pixel_points) const = 0;

    virtual Point2Matrix project3dToPixel(const Point3Matrix & world_points) const = 0;

    virtual Pose estimatePose(const Point2Matrix & points_image,
                              const Point3Matrix & points_object) const = 0;
  };

class ColorCameraModel : virtual public ColorCameraModel_<Types::Scalar>
{
public:

  typedef boost::shared_ptr<ColorCameraModel> Ptr;
  typedef boost::shared_ptr<const ColorCameraModel> ConstPtr;

};

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_COLOR_CAMERA_MODEL_H_ */
