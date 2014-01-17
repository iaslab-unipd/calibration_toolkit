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

template <typename Scalar_>
  class DepthUndistortionModelBase
  {
  public:

    typedef boost::shared_ptr<DepthUndistortionModelBase> Ptr;
    typedef boost::shared_ptr<const DepthUndistortionModelBase> ConstPtr;

    virtual ~DepthUndistortionModelBase()
    {
      // Do nothing
    }

    virtual Scalar_ * data() = 0;

    virtual const Scalar_ * data() const = 0;

  };

template <typename Scalar_>
  class DepthUndistortionModel
  {
  public:

    typedef boost::shared_ptr<DepthUndistortionModel> Ptr;
    typedef boost::shared_ptr<const DepthUndistortionModel> ConstPtr;

    virtual ~DepthUndistortionModel()
    {
      // Do nothing
    }

    virtual void undistort(typename Types_<Scalar_>::Point3 & point) const = 0;

    virtual void undistort(typename Types_<Scalar_>::Point3Matrix & cloud) const = 0;

  };

template <typename Scalar_, typename PCLPoint_>
  class DepthUndistortionModelPCL
  {
  public:

    typedef boost::shared_ptr<DepthUndistortionModelPCL> Ptr;
    typedef boost::shared_ptr<const DepthUndistortionModelPCL> ConstPtr;

    virtual ~DepthUndistortionModelPCL()
    {
      // Do nothing
    }

    virtual void undistort(PCLPoint_ & point) const = 0;

    virtual void undistort(pcl::PointCloud<PCLPoint_> & cloud) const = 0;

  };

template <typename Scalar_>
  class NoUndistortionBase : public DepthUndistortionModelBase<Scalar_>
  {
  public:

    typedef boost::shared_ptr<NoUndistortionBase> Ptr;
    typedef boost::shared_ptr<const NoUndistortionBase> ConstPtr;

    virtual ~NoUndistortionBase()
    {
      // Do nothing
    }

    virtual Scalar_ * data()
    {
      return NULL;
    }

    virtual const Scalar_ * data() const
    {
      return NULL;
    }

  };

template <typename Scalar_>
  class NoUndistortion : public NoUndistortionBase<Scalar_>,
                         public DepthUndistortionModel<Scalar_>
  {
  public:

    typedef boost::shared_ptr<NoUndistortion> Ptr;
    typedef boost::shared_ptr<const NoUndistortion> ConstPtr;

    virtual ~NoUndistortion()
    {
      // Do nothing
    }

    virtual void undistort(typename Types_<Scalar_>::Point3 & point) const
    {
      // Do nothing
    }

    virtual void undistort(typename Types_<Scalar_>::Point3Matrix & cloud) const
    {
      // Do nothing
    }

  };

template <typename Scalar_, typename PCLPoint_>
  class NoUndistortionPCL : public NoUndistortionBase<Scalar_>,
                            public DepthUndistortionModelPCL<Scalar_, PCLPoint_>
  {
  public:

    typedef boost::shared_ptr<NoUndistortionPCL> Ptr;
    typedef boost::shared_ptr<const NoUndistortionPCL> ConstPtr;

    virtual ~NoUndistortionPCL()
    {
      // Do nothing
    }

    virtual void undistort(PCLPoint_ & point) const
    {
      // Do nothing
    }

    virtual void undistort(pcl::PointCloud<PCLPoint_> & cloud) const
    {
      // Do nothing
    }

  };

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_DEPTH_UNDISTORTION_MODEL_H_ */
