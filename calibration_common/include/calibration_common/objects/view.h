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

#ifndef CALIBRATON_COMMON_OBJECTS_VIEW_H_
#define CALIBRATON_COMMON_OBJECTS_VIEW_H_

#include <calibration_common/base/point_matrix.h>
#include <calibration_common/objects/globals.h>

namespace calibration
{

/**
 * @brief The View class
 */
class View
{
public:

  typedef boost::shared_ptr<View> Ptr;
  typedef boost::shared_ptr<const View> ConstPtr;

  /**
   * @brief setId
   * @param id
   */
  inline void setId(const std::string & id)
  {
    id_ = id;
  }

  /**
   * @brief id
   * @return
   */
  inline const std::string & id() const
  {
    return id_;
  }

protected:

  std::string id_;

};

/**
 * @brief The View_ class
 * @param SensorT_
 * @param DataT_
 * @param ObjectT_
 * @param Dimension_
 */
template <typename SensorT_, typename DataT_, typename PointT_, typename ObjectT_, int Dimension_>
  class View_ : public View
  {
  public:

    typedef boost::shared_ptr<View_> Ptr;
    typedef boost::shared_ptr<const View_> ConstPtr;

    typedef Eigen::Matrix<Scalar, Dimension_, 1> Point;

    /**
     * @brief View_
     */
    View_()
      : centroid_(Point::Zero()),
        centroid_need_recompute_(false)
    {
      // Do nothing
    }

    /**
     * @brief setData
     * @param data
     */
    inline void setData(const DataT_ & data)
    {
      data_ = data;
    }

    /**
     * @brief data
     * @return
     */
    inline const DataT_ & data() const
    {
      assert(data_);
      return data_;
    }

    /**
     * @brief setPoints
     * @param points
     */
    inline void setPoints(const PointT_ & points)
    {
      points_ = points;
      centroid_need_recompute_ = true;
    }

    /**
     * @brief points
     * @return
     */
    inline const PointT_ & points() const
    {
      return points_;
    }

    /**
     * @brief setObject
     * @param object
     */
    inline void setObject(const typename ObjectT_::ConstPtr & object)
    {
      object_ = object;
    }

    /**
     * @brief object
     * @return
     */
    inline const typename ObjectT_::ConstPtr & object() const
    {
      assert(object_);
      return object_;
    }

    /**
     * @brief setSensor
     * @param sensor
     */
    inline void setSensor(const typename SensorT_::ConstPtr & sensor)
    {
      sensor_ = sensor;
    }

    /**
     * @brief sensor
     * @return
     */
    inline const typename SensorT_::ConstPtr & sensor() const
    {
      assert(sensor_);
      return sensor_;
    }

    /**
     * @brief centroid
     * @return
     */
    inline Point centroid() const
    {
      if (centroid_need_recompute_)
      {
        centroid_ = computeCentroid();
        centroid_need_recompute_ = false;
      }
      return centroid_;
    }

  protected:

    /**
     * @brief computeCentroid
     * @return
     */
    virtual Point computeCentroid() const = 0;

    typename SensorT_::ConstPtr sensor_;
    typename ObjectT_::ConstPtr object_;
    DataT_ data_;
    PointT_ points_;

    mutable Point centroid_;
    mutable bool centroid_need_recompute_;

  };

} /* namespace calibration */
#endif /* CALIBRATON_COMMON_OBJECTS_VIEW_H_ */
