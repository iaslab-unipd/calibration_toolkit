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

#ifndef UNIPD_CALIBRATION_CALIBRATON_COMMON_OBJECTS_VIEW_H_
#define UNIPD_CALIBRATION_CALIBRATON_COMMON_OBJECTS_VIEW_H_

#include <calibration_common/objects/with_points.h>

namespace unipd
{
namespace calib
{

/**
 * @brief The View class
 */
class View
{
public:

  View () = default;
  View (const View & other) = default;
  View (View && other) = default;
  View & operator = (const View & other) = default;
  View & operator = (View && other) = default;

  View (const std::string & id)
    : id_(id)
  {
    // Do nothing
  }

  void
  setId (const std::string & id)
  {
    id_ = id;
  }

  const std::string &
  id () const
  {
    return id_;
  }

protected:

  std::string id_;

};

template <typename SensorT_, typename ObjectT_>
  class ObjectView_ : public View
  {
  public:

    ObjectView_ () = default;
    ObjectView_ (const ObjectView_ & other) = default;
    ObjectView_ (ObjectView_ && other) = default;
    ObjectView_ & operator = (const ObjectView_ & other) = default;
    ObjectView_ & operator = (ObjectView_ && other) = default;

    using View::View;

    virtual
    ~ObjectView_ ()
    {
      // Do nothing
    }

    void
    setObject (const std::shared_ptr<const ObjectT_> & object)
    {
      object_ = object;
    }

    const std::shared_ptr<const ObjectT_> &
    object () const
    {
      assert(object_);
      return object_;
    }

    void
    setSensor (const std::shared_ptr<const SensorT_> & sensor)
    {
      sensor_ = sensor;
    }

    const std::shared_ptr<const SensorT_> &
    sensor () const
    {
      assert(sensor_);
      return sensor_;
    }

  protected:

    std::shared_ptr<const SensorT_> sensor_;
    std::shared_ptr<const ObjectT_> object_;

  };

template <typename SensorT_, typename ObjectT_, typename DataT_>
  class DataView_ : public ObjectView_<SensorT_, ObjectT_>
  {
  public:

    using Base = ObjectView_<SensorT_, ObjectT_>;

    DataView_ () = default;
    DataView_ (const DataView_ & other) = default;
    DataView_ (DataView_ && other) = default;
    DataView_ & operator = (const DataView_ & other) = default;
    DataView_ & operator = (DataView_ && other) = default;

    using Base::ObjectView_;

    virtual
    ~DataView_ ()
    {
      // Do nothing
    }

    void
    setData (const DataT_ & data)
    {
      data_ = data;
    }

    const DataT_ &
    data () const
    {
      assert(data_);
      return data_;
    }

  protected:

    DataT_ data_;

  };

template <typename SensorT_, typename ObjectT_, typename DataT_, typename IndicesT_>
  class IndicesView_ : public DataView_<SensorT_, ObjectT_, DataT_>
  {
  public:

    using Base = DataView_<SensorT_, ObjectT_, DataT_>;

    IndicesView_ () = default;
    IndicesView_ (const IndicesView_ & other) = default;
    IndicesView_ (IndicesView_ && other) = default;
    IndicesView_ & operator = (const IndicesView_ & other) = default;
    IndicesView_ & operator = (IndicesView_ && other) = default;

    using Base::ObjectView_;

    virtual
    ~IndicesView_ ()
    {
      // Do nothing
    }

    void
    setIndices (const std::shared_ptr<IndicesT_> & indices)
    {
      indices_ = indices;
    }

    const std::shared_ptr<IndicesT_> &
    indices () const
    {
      return indices_;
    }

  protected:

    std::shared_ptr<IndicesT_> indices_;

  };

template <typename SensorT_, typename ObjectWithPointsT_, typename PointsT_>
  class PointsView_ : public ObjectView_<SensorT_, ObjectWithPointsT_>
  {
  public:

    static_assert (std::is_base_of<WithPoints, ObjectWithPointsT_>::value, "ObjectWithPointsT_ must be a subclass of WithPoints");

    using Base = ObjectView_<SensorT_, ObjectWithPointsT_>;
    using Point = Eigen::Matrix<Scalar, Dimension<PointsT_>::value, 1>;

    PointsView_ () = default;
    PointsView_ (const PointsView_ & other) = default;
    PointsView_ (PointsView_ && other) = default;
    PointsView_ & operator = (const PointsView_ & other) = default;
    PointsView_ & operator = (PointsView_ && other) = default;

    using Base::ObjectView_;

    virtual
    ~PointsView_ ()
    {
      // Do nothing
    }

    void
    setPoints (const PointsT_ & points)
    {
      points_ = points;
      recompute_centroid_ = true;
    }

    void
    setPoints (PointsT_ && points)
    {
      points_ = points;
      recompute_centroid_ = true;
    }

    const PointsT_ &
    points () const
    {
      return points_;
    }

    const Point &
    centroid () const
    {
      if (recompute_centroid_)
      {
        centroid_ = computeCentroid();
        recompute_centroid_ = false;
      }
      return centroid_;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF_VECTORIZABLE_FIXED_SIZE(Scalar, Dimension<PointsT_>::value)

  protected:

    virtual Point
    computeCentroid () const = 0;

    PointsT_ points_;

    mutable Point centroid_ = Point(Point::Zero());
    mutable bool recompute_centroid_ = false;

  };

} // namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_CALIBRATON_COMMON_OBJECTS_VIEW_H_
