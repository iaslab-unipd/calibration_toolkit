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

/**
 * @brief The ColorCameraModel_ class
 * @param
 */
template <typename ScalarT_>
  class ColorCameraModel_
  {
  public:

    typedef boost::shared_ptr<ColorCameraModel_> Ptr;
    typedef boost::shared_ptr<const ColorCameraModel_> ConstPtr;

    typedef typename Types<ScalarT_>::Point2 Point2;
    typedef typename Types<ScalarT_>::Point3 Point3;
    typedef typename Types<ScalarT_>::Cloud2 Cloud2;
    typedef typename Types<ScalarT_>::Cloud3 Cloud3;
    typedef typename Types<ScalarT_>::Pose Pose;

    /**
     * @brief ~ColorCameraModel_
     */
    virtual ~ColorCameraModel_()
    {
      // Do nothing
    }

    /**
     * @brief projectPixelTo3dRay
     * @param pixel_point
     * @return
     */
    //virtual Point3 projectPixelTo3dRay(const Point2 & pixel_point) const = 0;

    /**
     * @brief project3dToPixel
     * @param world_point
     * @return
     */
    //virtual Point2 project3dToPixel(const Point3 & world_point) const = 0;

    /**
     * @brief projectPixelTo3dRay
     * @param pixel_points
     * @param world_points
     */
    //virtual void projectPixelTo3dRay(const Cloud2 & pixel_points,
    //                                 Cloud3 & world_points) const = 0;

    /**
     * @brief projectPixelTo3dRay
     * @param pixel_points
     * @return
     */
    //virtual Cloud3 projectPixelTo3dRay(const Cloud2 & pixel_points) const = 0;

    /**
     * @brief project3dToPixel
     * @param world_points
     * @param pixel_points
     */
    //virtual void project3dToPixel(const Cloud3 & world_points,
    //                              Cloud2 & pixel_points) const = 0;

    /**
     * @brief project3dToPixel
     * @param world_points
     * @return
     */
    //virtual Cloud2 project3dToPixel(const Cloud3 & world_points) const = 0;

    /**
     * @brief estimatePose
     * @param points_image
     * @param points_object
     * @return
     */
    virtual Pose estimatePose(const Cloud2 & points_image,
                              const Cloud3 & points_object) const = 0;
  };

/**
 * @brief The ColorCameraModel class
 */
class ColorCameraModel : virtual public ColorCameraModel_<Scalar>
{
public:

  typedef boost::shared_ptr<ColorCameraModel> Ptr;
  typedef boost::shared_ptr<const ColorCameraModel> ConstPtr;

};

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_COLOR_CAMERA_MODEL_H_ */
