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

#ifndef KINECT_DEPTH_POLYNOMIAL_UNDISTORTION_MAP_FIT_H_
#define KINECT_DEPTH_POLYNOMIAL_UNDISTORTION_MAP_FIT_H_

#include <Eigen/Dense>
#include <calibration_common/depth/undistortion_model_fit.h>
#include <kinect/depth/polynomial_undistortion_function.h>
#include <pcl/point_types.h>
#include <pcl/common/point_tests.h>

namespace calibration
{

template <typename Polynomial_, typename Function_>
  class PolynomialUndistortionFunctionFit : public DepthUndistortionModelFit<typename Traits<Polynomial_>::Scalar>
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionFunctionFit> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionFunctionFit> ConstPtr;

    typedef typename Traits<Polynomial_>::Scalar Scalar;

    typedef typename DepthUndistortionModelFit<Scalar>::PointDistorsionBin PointDistorsionBin;
    typedef typename DepthUndistortionModelFit<Scalar>::AccumulationBin AccumulationBin;

    static const int Size = Traits<Polynomial_>::Size;
    static const int MinDegree = Traits<Polynomial_>::MinDegree;
    static const int Degree = Traits<Polynomial_>::Degree;

    PolynomialUndistortionFunctionFit()
      : function_(boost::make_shared<Function_>())
    {
      // Do nothing
    }

    PolynomialUndistortionFunctionFit(const typename Function_::Ptr & function)
      : function_(function)
    {
      // Do nothing
    }

    void setFunction(const typename Function_::Ptr & function)
    {
      function_ = function;
    }

    const typename Function_::Ptr & function() const
    {
      return function_;
    }

    void addAccumulatedPoints(const typename Types_<Scalar>::Plane & plane);

    void update();

  protected:

    typename Function_::Ptr function_;
    PointDistorsionBin distorsion_bin_;
    AccumulationBin accumulation_bin_;

  };

template <typename Polynomial_>
  class PolynomialUndistortionFunctionFitEigen : public PolynomialUndistortionFunctionFit<Polynomial_,
                                                   PolynomialUndistortionFunctionEigen<Polynomial_> >,
                                                 public DepthUndistortionModelFitEigen<
                                                   typename Traits<Polynomial_>::Scalar>
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionFunctionFitEigen> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionFunctionFitEigen> ConstPtr;

    typedef typename Traits<Polynomial_>::Scalar Scalar;
    typedef PolynomialUndistortionFunctionEigen<Polynomial_> Function;
    typedef PolynomialUndistortionFunctionFit<Polynomial_, Function> Base;

    PolynomialUndistortionFunctionFitEigen()
      : Base()
    {
      // Do nothing
    }

    PolynomialUndistortionFunctionFitEigen(const typename Function::Ptr & function)
      : Base(function)
    {
      // Do nothing
    }

    virtual ~PolynomialUndistortionFunctionFitEigen()
    {
      // Do nothing
    }

    virtual void accumulateCloud(const typename Types_<Scalar>::Point3Matrix & cloud)
    {
      for (size_t i = 0; i < cloud.size(); ++i)
        accumulatePoint(cloud[i]);
    }

    virtual void accumulateCloud(const typename Types_<Scalar>::Point3Matrix & cloud,
                                 const std::vector<int> & indices)
    {
      for (size_t i = 0; i < indices.size(); ++i)
        accumulatePoint(cloud[indices[i]]);
    }

    virtual void accumulatePoint(const typename Types_<Scalar>::Point3 & point)
    {
      Base::accumulation_bin_ += point;
    }

    virtual void addPoint(const typename Types_<Scalar>::Point3 & point,
                          const typename Types_<Scalar>::Plane & plane);

    virtual void undistort(typename Types_<Scalar>::Point3 & point) const
    {
      Base::function_->undistort(point);
    }

    virtual void undistort(typename Types_<Scalar>::Point3Matrix & cloud) const
    {
      Base::function_->undistort(cloud);
    }

    virtual void addAccumulatedPoints(const typename Types_<Scalar>::Plane & plane)
    {
      Base::addAccumulatedPoints(plane);
    }

    virtual void update()
    {
      Base::update();
    }

    virtual typename Function::Interface::Ptr clone() const
    {
      PolynomialUndistortionFunctionFitEigen::Ptr clone = boost::make_shared<PolynomialUndistortionFunctionFitEigen>(*this);
      clone->setFunction(boost::shared_static_cast<Function>(Base::function_->clone()));
      return clone;
    }

  };

template <typename Polynomial_, typename PCLPoint_>
  class PolynomialUndistortionFunctionFitPCL : public PolynomialUndistortionFunctionFit<Polynomial_,
                                                 PolynomialUndistortionFunctionPCL<Polynomial_, PCLPoint_> >,
                                               public DepthUndistortionModelFitPCL<typename Traits<Polynomial_>::Scalar,
                                                 PCLPoint_>
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionFunctionFitPCL> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionFunctionFitPCL> ConstPtr;

    typedef typename Traits<Polynomial_>::Scalar Scalar;
    typedef PolynomialUndistortionFunctionPCL<Polynomial_, PCLPoint_> Function;
    typedef PolynomialUndistortionFunctionFit<Polynomial_, Function> Base;

    PolynomialUndistortionFunctionFitPCL()
      : Base()
    {
      // Do nothing
    }

    PolynomialUndistortionFunctionFitPCL(const typename Function::Ptr & function)
      : Base(function)
    {
      // Do nothing
    }

    virtual ~PolynomialUndistortionFunctionFitPCL()
    {
      // Do nothing
    }

    virtual void accumulateCloud(const pcl::PointCloud<PCLPoint_> & cloud)
    {
      for (size_t i = 0; i < cloud.points.size(); ++i)
        accumulatePoint(cloud.points[i]);
    }

    virtual void accumulateCloud(const pcl::PointCloud<PCLPoint_> & cloud,
                                 const std::vector<int> & indices)
    {
      for (size_t i = 0; i < indices.size(); ++i)
        accumulatePoint(cloud.points[indices[i]]);
    }

    virtual void accumulatePoint(const PCLPoint_ & point)
    {
      if (not pcl::isFinite(point))
        return;

      Base::accumulation_bin_ += typename Types_<Scalar>::Point3(point.x, point.y, point.z);
    }

    virtual void addPoint(const PCLPoint_ & point,
                          const typename Types_<Scalar>::Plane & plane);

    virtual void undistort(PCLPoint_ & point) const
    {
      Base::function_->undistort(point);
    }

    virtual void undistort(pcl::PointCloud<PCLPoint_> & cloud) const
    {
      Base::function_->undistort(cloud);
    }

    virtual void addAccumulatedPoints(const typename Types_<Scalar>::Plane & plane)
    {
      Base::addAccumulatedPoints(plane);
    }

    virtual void update()
    {
      Base::update();
    }

    virtual typename Function::Interface::Ptr clone() const
    {
      PolynomialUndistortionFunctionFitPCL::Ptr clone = boost::make_shared<PolynomialUndistortionFunctionFitPCL>(*this);
      clone->setFunction(boost::shared_static_cast<Function>(Base::function_->clone()));
      return clone;
    }

  };

} /* namespace calibration */

#include <impl/kinect/depth/polynomial_undistortion_function_fit.hpp>

#endif /* KINECT_DEPTH_POLYNOMIAL_UNDISTORTION_MAP_FIT_H_ */
