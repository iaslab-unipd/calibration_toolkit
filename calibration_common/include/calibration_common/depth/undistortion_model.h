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

#ifndef CALIBRATION_COMMON_DEPTH_UNDISTORTION_MODEL_H_
#define CALIBRATION_COMMON_DEPTH_UNDISTORTION_MODEL_H_

#include <pcl/point_cloud.h>

#include <calibration_common/base/matrix.h>
#include <calibration_common/objects/globals.h>

namespace calibration
{

template <typename Traits_>
  struct DepthUndistortionModel
  {
    typedef boost::shared_ptr<DepthUndistortionModel> Ptr;
    typedef boost::shared_ptr<const DepthUndistortionModel> ConstPtr;

    typedef typename Traits_::Point Point;
    typedef typename Traits_::Cloud Cloud;

    virtual ~DepthUndistortionModel()
    {
      // Do nothing
    }

    virtual void undistort(Point &) const = 0;

    virtual void undistort(Cloud &) const = 0;

    virtual Ptr clone() const = 0;

  };

template <typename Traits_>
  struct DepthUndistortionModelImpl
  {
    typedef boost::shared_ptr<DepthUndistortionModelImpl> Ptr;
    typedef boost::shared_ptr<const DepthUndistortionModelImpl> ConstPtr;

    typedef typename Traits_::Scalar Scalar;
    typedef typename Traits_::Data Data;

    virtual ~DepthUndistortionModelImpl()
    {
      // Do nothing
    }

    virtual void setData(const boost::shared_ptr<Data> &) = 0;

    virtual const boost::shared_ptr<Data> & data() const = 0;

    virtual Scalar * dataPtr() = 0;

    virtual const Scalar * dataPtr() const = 0;

  };

template <typename Traits_>
  class NoUndistortion : public DepthUndistortionModel<Traits_>
  {
  public:

    typedef boost::shared_ptr<NoUndistortion> Ptr;
    typedef boost::shared_ptr<const NoUndistortion> ConstPtr;

    typedef DepthUndistortionModel<Traits_> Interface;

    typedef typename Traits_::Point Point;
    typedef typename Traits_::Cloud Cloud;

    virtual ~NoUndistortion()
    {
      // Do nothing
    }

    virtual void undistort(Point & point) const
    {
      // Do nothing
    }

    virtual void undistort(Cloud & cloud) const
    {
      // Do nothing
    }

    virtual typename Interface::Ptr clone() const
    {
      return boost::make_shared<NoUndistortion>();
    }

  };

template <typename Scalar_>
  struct UndTraitsEigen
  {
    typedef Scalar_ Scalar;
    typedef typename Types_<Scalar_>::Point3 Point;
    typedef typename Types_<Scalar_>::Point3Matrix Cloud;
  };

template <typename Scalar_>
  struct NoUndistortionEigen : public NoUndistortion<UndTraitsEigen<Scalar_> >
  {
    typedef boost::shared_ptr<NoUndistortionEigen> Ptr;
    typedef boost::shared_ptr<const NoUndistortionEigen> ConstPtr;
  };

template <typename Scalar_, typename PCLPoint_>
  struct UndTraitsPCL
  {
    typedef Scalar_ Scalar;
    typedef PCLPoint_ Point;
    typedef pcl::PointCloud<PCLPoint_> Cloud;
  };

template <typename Scalar_, typename PCLPoint_>
  struct NoUndistortionPCL : public NoUndistortion<UndTraitsPCL<Scalar_, PCLPoint_> >
  {
    typedef boost::shared_ptr<NoUndistortionPCL> Ptr;
    typedef boost::shared_ptr<const NoUndistortionPCL> ConstPtr;
  };

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_DEPTH_UNDISTORTION_MODEL_H_ */
