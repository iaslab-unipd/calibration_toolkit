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

template <typename ModelT_>
  class PolynomialFunctionFit_ : public DepthUndistortionModelFitImpl<ModelT_>
  {
  public:

    typedef boost::shared_ptr<PolynomialFunctionFit_> Ptr;
    typedef boost::shared_ptr<const PolynomialFunctionFit_> ConstPtr;

    typedef DepthUndistortionModelFitImpl<ModelT_> Base;

    typedef typename ModelT_::Poly Poly;
    typedef typename ModelT_::Scalar Scalar;

    static const int Size = MathTraits<Poly>::Size;
    static const int MinDegree = MathTraits<Poly>::MinDegree;
    static const int Degree = MathTraits<Poly>::Degree;

    typedef typename Types<Scalar>::Plane Plane;
    typedef typename Base::PointDistorsionBin PointDistorsionBin;
    typedef typename Base::AccumulationBin AccumulationBin;

    PolynomialFunctionFit_()
    {
      // Do nothing
    }

    explicit PolynomialFunctionFit_(const typename ModelT_::Ptr & model)
      : model_(model)
    {
      // Do nothing
    }

    virtual ~PolynomialFunctionFit_()
    {
      // Do nothing
    }

    inline virtual void setModel(const typename ModelT_::Ptr & model)
    {
      model_ = model;
    }

    inline virtual const typename ModelT_::Ptr & model() const
    {
      return model_;
    }

    void addAccumulatedPoints(const Plane & plane);

    void update();

  protected:

    typename ModelT_::Ptr model_;
    PointDistorsionBin distorsion_bin_;
    AccumulationBin accumulation_bin_;

  };

template <typename PolynomialT_, typename ScalarT_>
  class PolynomialFunctionFitEigen : public PolynomialFunctionFit_<PolynomialFunctionModel<PolynomialT_> >,
                                     public DepthUndistortionModelFit<DepthEigen_<ScalarT_>, ScalarT_>
  {
  public:

    typedef boost::shared_ptr<PolynomialFunctionFitEigen> Ptr;
    typedef boost::shared_ptr<const PolynomialFunctionFitEigen> ConstPtr;

    typedef typename MathTraits<PolynomialT_>::Scalar Scalar;

    typedef PolynomialFunctionModel<PolynomialT_> Model;

    typedef PolynomialFunctionFit_<Model> Base;
    typedef typename Base::PointDistorsionBin PointDistorsionBin;
    typedef typename Base::AccumulationBin AccumulationBin;

    typedef DepthUndistortionModelFit<DepthEigen_<Scalar>, Scalar> Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;
    typedef typename Interface::Plane Plane;

    PolynomialFunctionFitEigen()
      : Base()
    {
      // Do nothing
    }

    explicit PolynomialFunctionFitEigen(const typename Model::Ptr & model)
      : Base(model)
    {
      // Do nothing
    }

    virtual ~PolynomialFunctionFitEigen()
    {
      // Do nothing
    }

    inline virtual void accumulateCloud(const Cloud & cloud)
    {
      for (Size1 i = 0; i < cloud.elements(); ++i)
        accumulatePoint(cloud[i]);
    }

    inline virtual void accumulateCloud(const Cloud & cloud,
                                        const std::vector<int> & indices)
    {
      for (Size1 i = 0; i < indices.size(); ++i)
        accumulatePoint(cloud[indices[i]]);
    }

    inline virtual void accumulatePoint(const Point & point)
    {
      Base::accumulation_bin_ += point;
    }

    virtual void addPoint(const Point & point,
                          const Plane & plane);

    inline virtual void addAccumulatedPoints(const Plane & plane)
    {
      Base::addAccumulatedPoints(plane);
    }

    inline virtual void update()
    {
      Base::update();
    }

  };

template <typename PolynomialT_, typename ScalarT_, typename PCLPointT_>
  class PolynomialFunctionFitPCL : public PolynomialFunctionFit_<PolynomialFunctionModel<PolynomialT_> >,
                                   public DepthUndistortionModelFit<DepthPCL_<PCLPointT_>, ScalarT_>
  {
  public:

    typedef boost::shared_ptr<PolynomialFunctionFitPCL> Ptr;
    typedef boost::shared_ptr<const PolynomialFunctionFitPCL> ConstPtr;

    typedef typename MathTraits<PolynomialT_>::Scalar Scalar;

    typedef PolynomialFunctionModel<PolynomialT_> Model;

    typedef PolynomialFunctionFit_<Model> Base;
    typedef typename Base::PointDistorsionBin PointDistorsionBin;
    typedef typename Base::AccumulationBin AccumulationBin;

    typedef DepthUndistortionModelFit<DepthPCL_<PCLPointT_>, Scalar> Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;
    typedef typename Interface::Plane Plane;

    PolynomialFunctionFitPCL()
      : Base()
    {
      // Do nothing
    }

    explicit PolynomialFunctionFitPCL(const typename Model::Ptr & model)
      : Base(model)
    {
      // Do nothing
    }

    virtual ~PolynomialFunctionFitPCL()
    {
      // Do nothing
    }

    inline virtual void accumulateCloud(const Cloud & cloud)
    {
      for (size_t i = 0; i < cloud.points.size(); ++i)
        accumulatePoint(cloud.points[i]);
    }

    inline virtual void accumulateCloud(const Cloud & cloud,
                                        const std::vector<int> & indices)
    {
      for (size_t i = 0; i < indices.size(); ++i)
        accumulatePoint(cloud.points[indices[i]]);
    }

    inline virtual void accumulatePoint(const Point & point)
    {
      if (not pcl::isFinite(point))
        return;

      Base::accumulation_bin_ += typename Types<Scalar>::Point3(point.x, point.y, point.z);
    }

    virtual void addPoint(const Point & point,
                          const Plane & plane);

    inline virtual void addAccumulatedPoints(const Plane & plane)
    {
      Base::addAccumulatedPoints(plane);
    }

    inline virtual void update()
    {
      Base::update();
    }

  };

} /* namespace calibration */

#include <impl/kinect/depth/polynomial_function_fit.hpp>

#endif /* KINECT_DEPTH_POLYNOMIAL_MAP_FIT_H_ */
