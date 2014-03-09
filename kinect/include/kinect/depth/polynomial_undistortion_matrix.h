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

#ifndef KINECT_DEPTH_POLYNOMIAL_UNDISTORTION_MATRIX_H_
#define KINECT_DEPTH_POLYNOMIAL_UNDISTORTION_MATRIX_H_

#include <calibration_common/base/math.h>
#include <calibration_common/base/matrix.h>
#include <calibration_common/depth/undistortion_model.h>

namespace calibration
{

template <typename PolynomialT_>
  class PolynomialUndistortionMatrixImpl;

template <typename PolynomialT_>
  struct ImplTraits<PolynomialUndistortionMatrixImpl<PolynomialT_> >
  {
    typedef PolynomialT_ Poly;
    typedef typename MathTraits<PolynomialT_>::Scalar Scalar;
    typedef EigenMatrix<typename MathTraits<PolynomialT_>::Coefficients> Data;
  };

template <typename PolynomialT_>
  class PolynomialUndistortionMatrixImpl : public DepthUndistortionModelImpl<PolynomialUndistortionMatrixImpl<PolynomialT_> >
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionMatrixImpl> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionMatrixImpl> ConstPtr;

    typedef ImplTraits<PolynomialUndistortionMatrixImpl> Traits;
    typedef DepthUndistortionModelImpl<PolynomialUndistortionMatrixImpl> Base;

    typedef typename Traits::Poly Poly;
    typedef typename Traits::Scalar Scalar;
    typedef typename Traits::Data Data;

    typedef typename Data::Element Element;
    typedef typename Data::ConstElement ConstElement;

    PolynomialUndistortionMatrixImpl()
    {
      // Do nothing
    }

    explicit PolynomialUndistortionMatrixImpl(const typename Data::Ptr & data)
      : data_(data)
    {
      // Do nothing
    }

    virtual ~PolynomialUndistortionMatrixImpl()
    {
      // Do nothing
    }

    void setData(const typename Data::Ptr & data)
    {
      data_ = data;
    }

    const typename Data::Ptr & data() const
    {
      return data_;
    }

    Element polynomialAt(size_t x_index,
                         size_t y_index)
    {
      assert(data_);
      return (*data_)(x_index, y_index);
    }

    const ConstElement polynomialAt(size_t x_index,
                                    size_t y_index) const
    {
      assert(data_);
      return (*boost::shared_static_cast<const Data>(data_))(x_index, y_index);
    }

    virtual void undistort(size_t x_index,
                           size_t y_index,
                           Scalar & z) const
    {
      z = PolynomialT_::evaluate(polynomialAt(x_index, y_index), z);
    }

    virtual Scalar * dataPtr()
    {
      assert(data_);
      return data_->matrix().data();
    }

    virtual const Scalar * dataPtr() const
    {
      assert(data_);
      return data_->matrix().data();
    }

  protected:

    typename Data::Ptr data_;

  };

template <typename PolynomialT_>
  class PolynomialUndistortionMatrixSphere : public PolynomialUndistortionMatrixImpl<PolynomialT_>
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionMatrixSphere> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionMatrixSphere> ConstPtr;

    typedef PolynomialUndistortionMatrixImpl<PolynomialT_> Base;

    typedef typename Base::Scalar Scalar;
    typedef typename Base::Data Data;
    typedef typename Base::Poly Poly;
    typedef typename Base::Element Element;
    typedef typename Base::ConstElement ConstElement;

    PolynomialUndistortionMatrixSphere()
      : Base(),
        bin_x_size_(0),
        bin_y_size_(0)
    {
      // Do nothing
    }

    explicit PolynomialUndistortionMatrixSphere(const typename Data::Ptr & data)
      : Base(data),
        bin_x_size_(0),
        bin_y_size_(0)
    {
      // Do nothing
    }

    PolynomialUndistortionMatrixSphere(const Base & other)
      : Base(other),
        zero_(other.zero_),
        fov_x_(other.fov_x_),
        fov_y_(other.fov_y_),
        bin_x_size_(other.bin_x_size_),
        bin_y_size_(other.bin_y_size_)
    {
      // Do nothing
    }

    virtual ~PolynomialUndistortionMatrixSphere()
    {
      // Do nothing
    }

    void setFieldOfView(Scalar x,
                        Scalar y)
    {
      assert(Base::data_);
      fov_x_ = x;
      fov_y_ = y;
//      bin_x_size_ = fov_x_ / Base::data_->xSize();
//      bin_y_size_ = fov_y_ / Base::data_->ySize();
//      zero_ = typename Types_<Scalar>::Point2(M_PI - fov_x_ / Scalar(2), M_PI_2 - fov_y_ / Scalar(2));
      zero_ = typename Types<Scalar>::Point2(- std::tan(fov_x_ / 2), - std::tan(fov_y_ / 2));
      bin_x_size_ =  - 2 * zero_.x() / Base::data_->xSize();
      bin_y_size_ =  - 2 * zero_.y() / Base::data_->ySize();
    }

    Scalar fieldOfViewX() const
    {
      return fov_x_;
    }

    Scalar fieldOfViewY() const
    {
      return fov_y_;
    }

    void getIndex(const typename Types<Scalar>::Point2 & point_sphere,
                  size_t & x_index,
                  size_t & y_index) const
    {
      assert(Base::data_);
      assert(bin_x_size_ > 0 and bin_y_size_ > 0);
      typename Types<Scalar>::Point2 diff = point_sphere - zero_;
      x_index = diff.x() < 0 ? 0 : size_t(std::min(Base::data_->xSize() - 1.0, std::floor(diff.x() / bin_x_size_)));
      y_index = diff.y() < 0 ? 0 : size_t(std::min(Base::data_->ySize() - 1.0, std::floor(diff.y() / bin_y_size_)));
    }

    using Base::polynomialAt;

    Element polynomialAt(const typename Types<Scalar>::Point2 & point_sphere)
    {
      size_t x_index, y_index;
      getIndex(point_sphere, x_index, y_index);
      return Base::polynomialAt(x_index, y_index);
    }

    const ConstElement polynomialAt(const typename Types<Scalar>::Point2 & point_sphere) const
    {
      size_t x_index, y_index;
      getIndex(point_sphere, x_index, y_index);
      return Base::polynomialAt(x_index, y_index);
    }

    using Base::undistort;

    void undistort(const typename Types<Scalar>::Point2 & point_sphere,
                   Scalar & z) const
    {
      assert(Base::data_);
      assert(bin_x_size_ > 0 and bin_y_size_ > 0);
      typename Types<Scalar>::Point2 diff = point_sphere - zero_;
      diff -= typename Types<Scalar>::Point2(bin_x_size_ * 0.5, bin_y_size_ * 0.5);

      size_t x_index[2];
      size_t y_index[2];
      Scalar x_weight[2] = {0.5, 0.5};
      Scalar y_weight[2] = {0.5, 0.5};

      Scalar dx = diff.x() / bin_x_size_;

      if (diff.x() < 0)
        x_index[0] = x_index[1] = 0;
      else if (dx > Base::data_->xSize() - 1)
        x_index[0] = x_index[1] = Base::data_->xSize() - 1;
      else
      {
        x_index[0] = size_t(std::floor(dx));
        x_index[1] = x_index[0] + 1;
        x_weight[1] = dx - x_index[0];
        x_weight[0] = 1.0 - x_weight[1];
      }

      Scalar dy = diff.y() / bin_y_size_;

      if (diff.y() < 0)
        y_index[0] = y_index[1] = 0;
      else if (dy > Base::data_->ySize() - 1)
        y_index[0] = y_index[1] = Base::data_->ySize() - 1;
      else
      {
        y_index[0] = size_t(std::floor(dy));
        y_index[1] = y_index[0] + 1;
        y_weight[1] = dy - y_index[0];
        y_weight[0] = 1.0 - y_weight[1];
      }

      Scalar tmp_z = 0.0;
      for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
          tmp_z += x_weight[i] * y_weight[j]
                   * PolynomialT_::evaluate(Base::polynomialAt(x_index[i], y_index[j]), z);

      z = tmp_z;

    }

    static typename Types<Scalar>::Point2 toSphericalCoordinates(Scalar x,
                                                                  Scalar y,
                                                                  Scalar z)
    {
//      Scalar norm(std::sqrt(x * x + y * y + z * z));
//      return typename Types_<Scalar>::Point2(M_PI + std::atan2(x, z), M_PI_2 - std::asin(y / norm));
      return typename Types<Scalar>::Point2(x / z, y / z);
    }

  protected:

    typename Types<Scalar>::Point2 zero_;
    Scalar fov_x_;
    Scalar fov_y_;
    Scalar bin_x_size_;
    Scalar bin_y_size_;

  };

template <typename PolynomialT_>
  class PolynomialUndistortionMatrixEigen : public PolynomialUndistortionMatrixSphere<PolynomialT_>,
                                            public DepthUndistortionModel<DepthEigen_<typename MathTraits<PolynomialT_>::Scalar> >
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionMatrixEigen> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionMatrixEigen> ConstPtr;

    typedef typename MathTraits<PolynomialT_>::Scalar Scalar;

    typedef PolynomialUndistortionMatrixSphere<PolynomialT_> Base;
    typedef typename Base::Data Data;
    typedef typename Base::Poly Poly;

    typedef DepthUndistortionModel<DepthEigen_<Scalar> > Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;

    PolynomialUndistortionMatrixEigen()
      : Base()
    {
      // Do nothing
    }

    explicit PolynomialUndistortionMatrixEigen(const typename Data::Ptr & data)
      : Base(data)
    {
      // Do nothing
    }

    PolynomialUndistortionMatrixEigen(const Base & other)
      : Base(other)
    {
      // Do nothing
    }

    virtual ~PolynomialUndistortionMatrixEigen()
    {
      // Do nothing
    }

    using Base::undistort;

    virtual void undistort(Point & point) const
    {
      //point *= Polynomial_::evaluate(Base::polynomialAt(toSphericalCoordinates(point)), point.z()) / point.z();

      Scalar z = point.z();
      Base::undistort(toSphericalCoordinates(point), z);
      point *= z / point.z();

    }

    virtual void undistort(Cloud & cloud) const
    {
//      for (size_t i = 0; i < cloud.size(); ++i)
//        cloud[i] *= Polynomial_::evaluate(Base::polynomialAt(toSphericalCoordinates(cloud[i])), cloud[i].z())
//          / cloud[i].z();

      for (size_t i = 0; i < cloud.size(); ++i)
      {
        Scalar z = cloud[i].z();
        Base::undistort(toSphericalCoordinates(cloud[i]), z);
        cloud[i] *= z / cloud[i].z();
      }
    }

    static typename Types<Scalar>::Point2 toSphericalCoordinates(const Point & point)
    {
//      return typename Types_<Scalar>::Point2(M_PI + std::atan2(point.x(), point.z()),
//      M_PI_2 - std::asin(point.y() / point.norm()));
      return typename Types<Scalar>::Point2(point.x() / point.z(), point.y() / point.z());
    }

    virtual typename Interface::Ptr clone() const
    {
      PolynomialUndistortionMatrixEigen::Ptr clone = boost::make_shared<PolynomialUndistortionMatrixEigen>(*this);
      clone->setData(boost::make_shared<Data>(*Base::data_));
      return clone;
    }

  };

template <typename PolynomialT_, typename PCLPointT_>
  class PolynomialUndistortionMatrixPCL : public PolynomialUndistortionMatrixSphere<PolynomialT_>,
                                          public DepthUndistortionModel<DepthPCL_<PCLPointT_> >
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionMatrixPCL> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionMatrixPCL> ConstPtr;

    typedef typename MathTraits<PolynomialT_>::Scalar Scalar;

    typedef PolynomialUndistortionMatrixSphere<PolynomialT_> Base;
    typedef typename Base::Data Data;
    typedef typename Base::Poly Poly;

    typedef DepthUndistortionModel<DepthPCL_<PCLPointT_> > Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;

    PolynomialUndistortionMatrixPCL()
      : Base()
    {
      // Do nothing
    }

    explicit PolynomialUndistortionMatrixPCL(const typename Data::Ptr & data)
      : Base(data)
    {
      // Do nothing
    }

    PolynomialUndistortionMatrixPCL(const Base & other)
      : Base(other)
    {
      // Do nothing
    }

    virtual ~PolynomialUndistortionMatrixPCL()
    {
      // Do nothing
    }

    using Base::undistort;

    virtual void undistort(Point & point) const
    {
//      float k = static_cast<float>(Polynomial_::evaluate(Base::polynomialAt(toSphericalCoordinates(point)), point.z))
//        / point.z;

      if (not pcl::isFinite(point))
        return;

      Scalar z = Scalar(point.z);
      Base::undistort(toSphericalCoordinates(point), z);
      float k = static_cast<float>(z) / point.z;

      point.x *= k;
      point.y *= k;
      point.z *= k;
    }

    virtual void undistort(Cloud & cloud) const
    {
      for (size_t i = 0; i < cloud.size(); ++i)
        undistort(cloud.points[i]);
    }

    static typename Types<Scalar>::Point2 toSphericalCoordinates(const Point & point)
    {
      return Base::toSphericalCoordinates(Scalar(point.x), Scalar(point.y), Scalar(point.z));
    }

    virtual typename Interface::Ptr clone() const
    {
      PolynomialUndistortionMatrixPCL::Ptr clone = boost::make_shared<PolynomialUndistortionMatrixPCL>(*this);
      clone->setData(boost::make_shared<Data>(*Base::data_));
      return clone;
    }

  };

} /* namespace calibration */
#endif /* KINECT_DEPTH_POLYNOMIAL_UNDISTORTION_MATRIX_H_ */
