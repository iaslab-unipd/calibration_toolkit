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

#ifndef KINECT_DEPTH_POLYNOMIAL_MAP_FIT_H_
#define KINECT_DEPTH_POLYNOMIAL_MAP_FIT_H_

#include <Eigen/Dense>
#include <calibration_common/depth/undistortion_model_fit.h>
#include <kinect/depth/polynomial_function.h>
#include <pcl/point_types.h>
#include <pcl/common/point_tests.h>

namespace calibration
{

template <typename ModelImpl_>
  class PolynomialUndistortionFunctionFitImpl : public DepthUndistortionModelFitImpl<ModelImpl_>
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionFunctionFitImpl> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionFunctionFitImpl> ConstPtr;

    typedef DepthUndistortionModelFitImpl<ModelImpl_> Base;

    typedef typename ModelImpl_::Poly Poly;
    typedef typename ModelImpl_::Scalar Scalar;

    static const int Size = MathTraits<Poly>::Size;
    static const int MinDegree = MathTraits<Poly>::MinDegree;
    static const int Degree = MathTraits<Poly>::Degree;

    typedef typename Types<Scalar>::Plane Plane;
    typedef typename Base::PointDistorsionBin PointDistorsionBin;
    typedef typename Base::AccumulationBin AccumulationBin;

    PolynomialUndistortionFunctionFitImpl()
    {
      // Do nothing
    }

    explicit PolynomialUndistortionFunctionFitImpl(const typename ModelImpl_::Ptr & model_impl)
      : model_impl_(model_impl)
    {
      // Do nothing
    }

    virtual ~PolynomialUndistortionFunctionFitImpl()
    {
      // Do nothing
    }

    virtual void setModelImpl(const typename ModelImpl_::Ptr & model_impl)
    {
      model_impl_ = model_impl;
    }

    virtual const typename ModelImpl_::Ptr & modelImpl() const
    {
      return model_impl_;
    }

    void addAccumulatedPoints(const Plane & plane);

    void update();

  protected:

    typename ModelImpl_::Ptr model_impl_;
    PointDistorsionBin distorsion_bin_;
    AccumulationBin accumulation_bin_;

  };

template <typename PolynomialT_>
  class PolynomialUndistortionFunctionFitEigen : public PolynomialUndistortionFunctionFitImpl<PolynomialUndistortionFunctionEigen<PolynomialT_> >,
                                                 public DepthUndistortionModelFit<DepthEigen_<typename MathTraits<PolynomialT_>::Scalar> >
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionFunctionFitEigen> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionFunctionFitEigen> ConstPtr;

    typedef typename MathTraits<PolynomialT_>::Scalar Scalar;

    typedef PolynomialUndistortionFunctionEigen<PolynomialT_> UndistortionModel;

    typedef PolynomialUndistortionFunctionFitImpl<UndistortionModel> Base;
    typedef typename Base::PointDistorsionBin PointDistorsionBin;
    typedef typename Base::AccumulationBin AccumulationBin;

    typedef DepthUndistortionModelFit<DepthEigen_<Scalar> > Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;
    typedef typename Interface::Plane Plane;

    PolynomialUndistortionFunctionFitEigen()
      : Base()
    {
      // Do nothing
    }

    explicit PolynomialUndistortionFunctionFitEigen(const typename PolynomialUndistortionFunctionEigen<PolynomialT_>::Ptr & model)
      : Base(model),
        model_(model)
    {
      // Do nothing
    }

    virtual ~PolynomialUndistortionFunctionFitEigen()
    {
      // Do nothing
    }

    virtual void setModel(const typename PolynomialUndistortionFunctionEigen<PolynomialT_>::Ptr & model)
    {
      Base::setModelImpl(model);
      model_ = model;
    }

    virtual const typename DepthUndistortion<DepthEigen_<Scalar> >::Ptr & model() const
    {
      return model_;
    }

    virtual void accumulateCloud(const Cloud & cloud)
    {
      for (size_t i = 0; i < cloud.size(); ++i)
        accumulatePoint(cloud[i]);
    }

    virtual void accumulateCloud(const Cloud & cloud,
                                 const std::vector<int> & indices)
    {
      for (size_t i = 0; i < indices.size(); ++i)
        accumulatePoint(cloud[indices[i]]);
    }

    virtual void accumulatePoint(const Point & point)
    {
      Base::accumulation_bin_ += point;
    }

    virtual void addPoint(const Point & point,
                          const Plane & plane);

    virtual void addAccumulatedPoints(const Plane & plane)
    {
      Base::addAccumulatedPoints(plane);
    }

    virtual void update()
    {
      Base::update();
    }

    virtual typename Interface::Ptr clone() const
    {
      PolynomialUndistortionFunctionFitEigen::Ptr clone =
        boost::make_shared<PolynomialUndistortionFunctionFitEigen>(*this);
      clone->setModel(boost::static_pointer_cast<UndistortionModel>(Base::model_impl_->clone()));
      return clone;
    }

  protected:

    typename DepthUndistortion<DepthEigen_<Scalar> >::Ptr model_;

  };

template <typename PolynomialT_, typename PCLPointT_>
  class PolynomialUndistortionFunctionFitPCL : public PolynomialUndistortionFunctionFitImpl<PolynomialUndistortionFunctionPCL<PolynomialT_, PCLPointT_> >,
                                               public DepthUndistortionModelFit<DepthPCL_<PCLPointT_>, typename MathTraits<PolynomialT_>::Scalar>
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionFunctionFitPCL> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionFunctionFitPCL> ConstPtr;

    typedef typename MathTraits<PolynomialT_>::Scalar Scalar;

    typedef PolynomialUndistortionFunctionPCL<PolynomialT_, PCLPointT_> UndistortionModel;

    typedef PolynomialUndistortionFunctionFitImpl<UndistortionModel> Base;
    typedef typename Base::PointDistorsionBin PointDistorsionBin;
    typedef typename Base::AccumulationBin AccumulationBin;

    typedef DepthUndistortionModelFit<DepthPCL_<PCLPointT_>, Scalar> Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;
    typedef typename Interface::Plane Plane;

    PolynomialUndistortionFunctionFitPCL()
      : Base()
    {
      // Do nothing
    }

    explicit PolynomialUndistortionFunctionFitPCL(const typename UndistortionModel::Ptr & model)
      : Base(model),
        model_(model)
    {
      // Do nothing
    }

    virtual ~PolynomialUndistortionFunctionFitPCL()
    {
      // Do nothing
    }

    virtual void setModel(const typename PolynomialUndistortionFunctionPCL<PolynomialT_, PCLPointT_>::Ptr & model)
    {
      Base::setModelImpl(model);
      model_ = model;
    }

    virtual const typename DepthUndistortion<DepthPCL_<PCLPointT_> >::Ptr & model() const
    {
      return model_;
    }

    virtual void accumulateCloud(const Cloud & cloud)
    {
      for (size_t i = 0; i < cloud.points.size(); ++i)
        accumulatePoint(cloud.points[i]);
    }

    virtual void accumulateCloud(const Cloud & cloud,
                                 const std::vector<int> & indices)
    {
      for (size_t i = 0; i < indices.size(); ++i)
        accumulatePoint(cloud.points[indices[i]]);
    }

    virtual void accumulatePoint(const Point & point)
    {
      if (not pcl::isFinite(point))
        return;

      Base::accumulation_bin_ += typename Types<Scalar>::Point3(point.x, point.y, point.z);
    }

    virtual void addPoint(const Point & point,
                          const Plane & plane);

    virtual void addAccumulatedPoints(const Plane & plane)
    {
      Base::addAccumulatedPoints(plane);
    }

    virtual void update()
    {
      Base::update();
    }

    virtual typename Interface::Ptr clone() const
    {
      PolynomialUndistortionFunctionFitPCL::Ptr clone = boost::make_shared<PolynomialUndistortionFunctionFitPCL>(*this);
      clone->setModel(boost::static_pointer_cast<UndistortionModel>(Base::model_impl_->clone()));
      return clone;
    }

  protected:

    typename DepthUndistortion<DepthPCL_<Point> >::Ptr model_;

  };

} /* namespace calibration */

#include <impl/kinect/depth/polynomial_function_fit.hpp>

#endif /* KINECT_DEPTH_POLYNOMIAL_MAP_FIT_H_ */
