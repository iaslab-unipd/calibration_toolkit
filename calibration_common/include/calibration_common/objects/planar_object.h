/*
 *  Copyright (C) 2013 - Filippo Basso <bassofil@dei.unipd.it>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CALIBRATION_COMMON_OBJECTS_PLANAR_OBJECT_H_
#define CALIBRATION_COMMON_OBJECTS_PLANAR_OBJECT_H_

#include <calibration_common/objects/base_object.h>
#include <calibration_common/depth/view.h>

namespace calibration
{

/**
 * @brief The PlanarObject class
 */
class PlanarObject : public BaseObject
{
public:

  typedef boost::shared_ptr<PlanarObject> Ptr;
  typedef boost::shared_ptr<const PlanarObject> ConstPtr;

  /**
   * @brief PlanarObject
   */
  PlanarObject()
    : BaseObject(),
      plane_(PLANE_XY)
  {
    // Do nothing
  }

  explicit PlanarObject(const std::string & frame_id)
    : BaseObject(frame_id),
      plane_(PLANE_XY)
  {
    // Do nothing
  }

  /**
   * @brief ~PlanarObject
   */
  virtual ~PlanarObject()
  {
    // Do nothing
  }

  /**
   * @brief transform
   * @param transform
   */
  inline virtual void transform(const Transform & transform)
  {
    plane_.transform(transform);
    BaseObject::transform(transform);
  }

  /**
   * @brief plane
   * @return
   */
  inline const Plane & plane() const
  {
    return plane_;
  }

  /**
   * @brief setPlane
   * @param plane
   */
  inline void setPlane(const Plane & plane)
  {
    BaseObject::setPose(Util::plane3dTransform(PLANE_XY, plane));
    plane_ = plane;
  }

  /**
   * @brief toMarker
   * @param marker
   */
  void toMarker(visualization_msgs::Marker & marker) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  Plane plane_;

};

//template <typename Derived>
//PlanarObject::PlanarObject(const ColorView<Derived> & view)
//  : BaseObject(view), plane_(view.object()->plane())
//{
//  plane_.transform(pose());
//}

//template <typename PointT>
//  PlanarObject::PlanarObject(const DepthView<PlanarObject, PointT> & view)
//    : BaseObject("", view.sensor())
//  {
//    std::stringstream ss;
//    ss << view.object()->frameId() << "_" << view.id();
//    BaseObject::frame_id_ = ss.str();
//
//    plane_ = fitPlane(view.interestPoints());
//    BaseObject::transform(Util::plane3dTransform(view.object()->plane(), plane_));
//  }

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_OBJECTS_PLANAR_OBJECT_H_ */
