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
#include <calibration_common/depth/traits.h>

namespace calibration
{

template <typename DepthT_>
  struct DepthUndistortionModel
  {
    typedef boost::shared_ptr<DepthUndistortionModel> Ptr;
    typedef boost::shared_ptr<const DepthUndistortionModel> ConstPtr;

    typedef typename DepthTraits<DepthT_>::Point Point;
    typedef typename DepthTraits<DepthT_>::Cloud Cloud;

    virtual ~DepthUndistortionModel()
    {
      // Do nothing
    }

    virtual void undistort(Point & point) const = 0;

    virtual void undistort(Cloud & cloud) const = 0;

    virtual Ptr clone() const = 0;

  };

template <typename ImplT_>
  struct ImplTraits
  {
  };

template <typename ImplT_>
  struct DepthUndistortionModelImpl
  {
    typedef boost::shared_ptr<DepthUndistortionModelImpl> Ptr;
    typedef boost::shared_ptr<const DepthUndistortionModelImpl> ConstPtr;

    typedef typename ImplTraits<ImplT_>::Scalar Scalar;
    typedef typename ImplTraits<ImplT_>::Data Data;

    virtual ~DepthUndistortionModelImpl()
    {
      // Do nothing
    }

    virtual void setData(const boost::shared_ptr<Data> & data) = 0;

    virtual const boost::shared_ptr<Data> & data() const = 0;

    virtual Scalar * dataPtr() = 0;

    virtual const Scalar * dataPtr() const = 0;

  };

template <typename DepthT_>
  class NoUndistortion_ : public DepthUndistortionModel<DepthT_>
  {
  public:

    typedef boost::shared_ptr<NoUndistortion_> Ptr;
    typedef boost::shared_ptr<const NoUndistortion_> ConstPtr;

    typedef DepthUndistortionModel<DepthT_> Interface;

    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;

    virtual ~NoUndistortion_()
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
      return boost::make_shared<NoUndistortion_>();
    }

  };

template <typename Scalar_>
  struct NoUndistortionEigen_ : public NoUndistortion_<DepthEigen_<Scalar_> >
  {
    typedef boost::shared_ptr<NoUndistortionEigen_> Ptr;
    typedef boost::shared_ptr<const NoUndistortionEigen_> ConstPtr;
  };

template <typename PCLPointT_>
  struct NoUndistortionPCL_ : public NoUndistortion_<DepthPCL_<PCLPointT_> >
  {
    typedef boost::shared_ptr<NoUndistortionPCL_> Ptr;
    typedef boost::shared_ptr<const NoUndistortionPCL_> ConstPtr;
  };

typedef NoUndistortionPCL_<PCLPoint3> NoUndistortionPCL;
typedef NoUndistortionEigen_<Scalar> NoUndistortionEigen;

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_DEPTH_UNDISTORTION_MODEL_H_ */
