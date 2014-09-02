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

#ifndef CALIBRATON_COMMON_DEPTH_VIEW_H_
#define CALIBRATON_COMMON_DEPTH_VIEW_H_

#include <calibration_common/depth/sensor.h>
#include <calibration_common/objects/view.h>

namespace visualization_msgs
{
template <class ContainerAllocator>
  struct Marker_;

typedef Marker_<std::allocator<void> > Marker;
}

namespace calibration
{

///**
// * @brief The DepthView_ struct
// * @param SensorT_
// * @param DataT_
// * @param ObjectT_
// */
//template <typename SensorT_, typename DataT_, typename ObjectT_>
//  struct DepthView_ : public View_<SensorT_, DataT_, ObjectT_, 3>
//  {
//    typedef boost::shared_ptr<DepthView_> Ptr;
//    typedef boost::shared_ptr<const DepthView_> ConstPtr;

//    typedef View_<SensorT_, DataT_, ObjectT_, 3> Base;

//    void toMarker(visualization_msgs::Marker & marker) const;
//  };

/**
 * @brief The DepthViewEigen struct
 * @param ObjectT_
 */
template <typename ObjectT_>
  struct DepthViewEigen : public View_<DepthSensor, Cloud3::ConstPtr, Indices, ObjectT_, 3>
  {
    typedef boost::shared_ptr<DepthViewEigen> Ptr;
    typedef boost::shared_ptr<const DepthViewEigen> ConstPtr;

    typedef View_<DepthSensor, Cloud3::ConstPtr, Indices, ObjectT_, 3> Base;
    typedef typename Base::Point Point;

  protected:

    inline virtual Point computeCentroid() const
    {
      const Indices & points = Base::points();
      const Cloud3 & cloud = *Base::data();
      assert(not points.empty());
      Point sum = Point::Zero();
      for (Size1 i = 0; i < points.size(); ++i)
        sum += cloud[points[i]];
      sum /= points.size();
      return sum;
    }

  };

/**
 * @brief The DepthViewPCL struct
 * @param ObjectT_
 */
template <typename ObjectT_>
  struct DepthViewPCL : public View_<DepthSensor, PCLCloud3::ConstPtr, Indices, ObjectT_, 3>
  {
    typedef boost::shared_ptr<DepthViewPCL> Ptr;
    typedef boost::shared_ptr<const DepthViewPCL> ConstPtr;

    typedef View_<DepthSensor, PCLCloud3::ConstPtr, Indices, ObjectT_, 3> Base;
    typedef typename Base::Point Point;

  protected:

    inline virtual Point computeCentroid() const
    {
      const Indices & points = Base::points();
      const PCLCloud3 & cloud = *Base::data();
      assert(not points.empty());
      Point sum = Point::Zero();
      for (Size1 i = 0; i < points.size(); ++i)
      {
        sum.x() += cloud[points[i]].x;
        sum.y() += cloud[points[i]].y;
        sum.z() += cloud[points[i]].z;
      }
      sum /= points.size();
      return sum;
    }
  };

} /* namespace calibration */

#include <impl/calibration_common/depth/view.hpp>

#endif /* CALIBRATON_COMMON_DEPTH_VIEW_H_ */
