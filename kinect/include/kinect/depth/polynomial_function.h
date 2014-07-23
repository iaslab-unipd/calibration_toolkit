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

#ifndef KINECT_DEPTH_POLYNOMIAL_FUNCTION_H_
#define KINECT_DEPTH_POLYNOMIAL_FUNCTION_H_

#include <calibration_common/base/math.h>
#include <calibration_common/depth/undistortion_model.h>

namespace calibration
{

template <typename Polynomial_>
  class PolynomialFunctionModel;

template <typename PolynomialT_>
  struct ModelTraits<PolynomialFunctionModel<PolynomialT_> >
  {
    typedef PolynomialT_ Poly;
    typedef typename MathTraits<PolynomialT_>::Scalar Scalar;
    typedef PolynomialT_ Data;
  };

template <typename PolynomialT_>
  class PolynomialFunctionModel
  {
  public:

    typedef boost::shared_ptr<PolynomialFunctionModel> Ptr;
    typedef boost::shared_ptr<const PolynomialFunctionModel> ConstPtr;

    typedef ModelTraits<PolynomialFunctionModel> Traits;

    typedef typename Traits::Poly Poly;
    typedef typename Traits::Scalar Scalar;
    typedef typename Traits::Data Data;

    PolynomialFunctionModel()
    {
      // Do nothing
    }

    explicit PolynomialFunctionModel(const boost::shared_ptr<Data> & polynomial)
      : polynomial_(polynomial)
    {
      // Do nothing
    }

    virtual ~PolynomialFunctionModel()
    {
      // Do nothing
    }

    inline virtual void setPolynomial(const boost::shared_ptr<Data> & polynomial)
    {
      polynomial_ = polynomial;
    }

    inline virtual const boost::shared_ptr<Data> & polynomial() const
    {
      return polynomial_;
    }

    inline virtual void undistort(Scalar & depth) const
    {
      assert(polynomial_);
      depth = polynomial_->evaluate(depth);
    }

    inline virtual Scalar * dataPtr()
    {
      assert(polynomial_);
      return polynomial_->dataPtr();
    }

    inline virtual const Scalar * dataPtr() const
    {
      assert(polynomial_);
      return polynomial_->dataPtr();
    }

  protected:

    boost::shared_ptr<Data> polynomial_;

  };

template <typename Polynomial_, typename ScalarT_>
  class PolynomialFunctionEigen : public DepthUndistortion<DepthEigen_<ScalarT_> >
  {
  public:

    typedef boost::shared_ptr<PolynomialFunctionEigen> Ptr;
    typedef boost::shared_ptr<const PolynomialFunctionEigen> ConstPtr;

    typedef PolynomialFunctionModel<Polynomial_> Model;

    typedef typename MathTraits<Polynomial_>::Scalar Scalar;
    typedef typename DepthTraits<DepthEigen_<Scalar> >::Point Point;
    typedef typename DepthTraits<DepthEigen_<Scalar> >::Cloud Cloud;
    typedef typename Model::Traits::Data Data;

    typedef DepthUndistortion<DepthEigen_<Scalar> > Interface;

    PolynomialFunctionEigen()
    {
      // Do nothing
    }

    explicit PolynomialFunctionEigen(const typename Model::Ptr & model)
      : model_(model)
    {
      // Do nothing
    }

    virtual ~PolynomialFunctionEigen()
    {
      // Do nothing
    }

    inline virtual void undistort(Point & point) const
    {
      assert(model_);
      if (point.isNaN())
        return;

      Scalar z = Scalar(point.z());
      model_->undistort(z);
      point *= z / point.z();
    }

    virtual void undistort(Cloud & cloud) const
    {
      assert(model_);
      for (Size1 i = 0; i < cloud.elements(); ++i)
      {
        if (cloud[i].isNaN())
          continue;

        Scalar z = Scalar(cloud[i].z());
        model_->undistort(z);
        cloud[i] *= z / cloud[i].z();
      }
    }

  protected:

    Model model_;

  };

template <typename Polynomial_, typename PCLPoint_>
  class PolynomialFunctionPCL : public DepthUndistortion<DepthPCL_<PCLPoint_> >
  {
  public:

    typedef boost::shared_ptr<PolynomialFunctionPCL> Ptr;
    typedef boost::shared_ptr<const PolynomialFunctionPCL> ConstPtr;

    typedef PolynomialFunctionModel<Polynomial_> Model;

    typedef typename MathTraits<Polynomial_>::Scalar Scalar;
    typedef typename DepthTraits<DepthPCL_<PCLPoint_> >::Point Point;
    typedef typename DepthTraits<DepthPCL_<PCLPoint_> >::Cloud Cloud;
    typedef typename Model::Traits::Data Data;

    typedef DepthUndistortion<DepthPCL_<PCLPoint_> > Interface;

    PolynomialFunctionPCL()
    {
      // Do nothing
    }

    explicit PolynomialFunctionPCL(const typename Model::Ptr & model)
      : model_(model)
    {
      // Do nothing
    }

    virtual ~PolynomialFunctionPCL()
    {
      // Do nothing
    }

    inline virtual void undistort(Point & point) const
    {
      assert(model_);
      if (not pcl::isFinite(point))
        return;

      Scalar z = Scalar(point.z);
      model_->undistort(z);
      float k = static_cast<float>(z) / point.z;
      point.x *= k;
      point.y *= k;
      point.z *= k;
    }

    inline virtual void undistort(Cloud & cloud) const
    {
      assert(model_);
      for (Size1 i = 0; i < cloud.size(); ++i)
      {
        if (not pcl::isFinite(cloud[i]))
          continue;

        Scalar z = Scalar(cloud[i].z);
        model_->undistort(z);
        float k = static_cast<float>(z) / cloud[i].z;
        cloud[i].x *= k;
        cloud[i].y *= k;
        cloud[i].z *= k;
      }
    }

  protected:

    Model model_;

  };

} /* namespace calibration */
#endif /* KINECT_DEPTH_POLYNOMIAL_FUNCTION_H_ */
