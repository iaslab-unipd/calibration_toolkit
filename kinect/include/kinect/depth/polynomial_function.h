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
  class PolynomialUndistortionFunctionModel;

template <typename PolynomialT_>
  struct ModelTraits<PolynomialUndistortionFunctionModel<PolynomialT_> >
  {
    typedef PolynomialT_ Poly;
    typedef typename MathTraits<PolynomialT_>::Scalar Scalar;
    typedef PolynomialT_ Data;
  };

template <typename Polynomial_>
  class PolynomialUndistortionFunctionModel : public DepthUndistortionModel<PolynomialUndistortionFunctionModel<Polynomial_> >
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionFunctionModel> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionFunctionModel> ConstPtr;

    typedef ModelTraits<PolynomialUndistortionFunctionModel> Traits;
    typedef DepthUndistortionModel<PolynomialUndistortionFunctionModel> Base;

    typedef typename Traits::Poly Poly;
    typedef typename Traits::Scalar Scalar;
    typedef typename Traits::Data Data;

    PolynomialUndistortionFunctionModel()
    {
      // Do nothing
    }

    explicit PolynomialUndistortionFunctionModel(const boost::shared_ptr<Data> & polynomial)
      : polynomial_(polynomial)
    {
      // Do nothing
    }

    virtual ~PolynomialUndistortionFunctionModel()
    {
      // Do nothing
    }

    virtual void setData(const boost::shared_ptr<Data> & polynomial)
    {
      polynomial_ = polynomial;
    }

    virtual const boost::shared_ptr<Data> & data() const
    {
      return polynomial_;
    }

    virtual void undistort(Scalar & z) const
    {
      z = polynomial_->evaluate(z);
    }

    virtual Scalar * dataPtr()
    {
      assert(polynomial_);
      return polynomial_->dataPtr();
    }

    virtual const Scalar * dataPtr() const
    {
      assert(polynomial_);
      return polynomial_->dataPtr();
    }

  protected:

    boost::shared_ptr<Data> polynomial_;

  };

template <typename Polynomial_>
  class PolynomialUndistortionFunctionEigen : public PolynomialUndistortionFunctionModel<Polynomial_>,
                                              public DepthUndistortion<DepthEigen_<typename MathTraits<Polynomial_>::Scalar> >
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionFunctionEigen> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionFunctionEigen> ConstPtr;

    typedef PolynomialUndistortionFunctionModel<Polynomial_> Base;

    typedef typename MathTraits<Polynomial_>::Scalar Scalar;
    typedef typename DepthTraits<DepthEigen_<Scalar> >::Point Point;
    typedef typename DepthTraits<DepthEigen_<Scalar> >::Cloud Cloud;
    typedef typename Base::Traits::Data Data;

    typedef DepthUndistortion<DepthEigen_<Scalar> > Interface;

    PolynomialUndistortionFunctionEigen()
      : Base()
    {
      // Do nothing
    }

    explicit PolynomialUndistortionFunctionEigen(const typename Data::Ptr & polynomial)
      : Base(polynomial)
    {
      // Do nothing
    }

    PolynomialUndistortionFunctionEigen(const Base & other)
      : Base(other)
    {
      // Do nothing
    }

    virtual ~PolynomialUndistortionFunctionEigen()
    {
      // Do nothing
    }

    virtual void undistort(Point & point) const
    {
      assert(Base::polynomial_);
      point *= Base::polynomial_->evaluate(point.z()) / point.z();
    }

    virtual void undistort(Cloud & cloud) const
    {
      assert(Base::polynomial_);
      for (size_t i = 0; i < cloud.size(); ++i)
        cloud[i] *= Base::polynomial_->evaluate(cloud[i].z()) / cloud[i].z();
    }

    virtual typename Interface::Ptr clone() const
    {
      return boost::make_shared<PolynomialUndistortionFunctionEigen>(boost::make_shared<Data>(*Base::polynomial_));
    }

  };

template <typename Polynomial_, typename PCLPoint_>
  class PolynomialUndistortionFunctionPCL : public PolynomialUndistortionFunctionModel<Polynomial_>,
                                            public DepthUndistortion<DepthPCL_<PCLPoint_> >
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionFunctionPCL> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionFunctionPCL> ConstPtr;

    typedef PolynomialUndistortionFunctionModel<Polynomial_> Base;

    typedef typename MathTraits<Polynomial_>::Scalar Scalar;
    typedef typename DepthTraits<DepthPCL_<PCLPoint_> >::Point Point;
    typedef typename DepthTraits<DepthPCL_<PCLPoint_> >::Cloud Cloud;
    typedef typename Base::Traits::Data Data;

    typedef DepthUndistortion<DepthPCL_<PCLPoint_> > Interface;

    PolynomialUndistortionFunctionPCL()
      : Base()
    {
      // Do nothing
    }

    explicit PolynomialUndistortionFunctionPCL(const typename Data::Ptr & polynomial)
      : Base(polynomial)
    {
      // Do nothing
    }

    PolynomialUndistortionFunctionPCL(const Base & other)
      : Base(other)
    {
      // Do nothing
    }

    virtual ~PolynomialUndistortionFunctionPCL()
    {
      // Do nothing
    }

    virtual void undistort(Point & point) const
    {
      assert(Base::polynomial_);
      float k = static_cast<float>(Base::polynomial_->evaluate(point.z)) / point.z;
      point.x *= k;
      point.y *= k;
      point.z *= k;
    }

    virtual void undistort(Cloud & cloud) const
    {
      for (size_t i = 0; i < cloud.size(); ++i)
        undistort(cloud.points[i]);
    }

    virtual typename Interface::Ptr clone() const
    {
      return boost::make_shared<PolynomialUndistortionFunctionPCL>(boost::make_shared<Data>(*Base::polynomial_));
    }

  };

} /* namespace calibration */
#endif /* KINECT_DEPTH_POLYNOMIAL_FUNCTION_H_ */
