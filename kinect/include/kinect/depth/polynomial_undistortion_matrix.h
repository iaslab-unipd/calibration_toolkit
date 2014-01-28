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

template <typename Polynomial_>
  class PolynomialUndistortionMatrix : public DepthUndistortionModel<typename Traits<Polynomial_>::Scalar>
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionMatrix> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionMatrix> ConstPtr;

    typedef typename Traits<Polynomial_>::Scalar Scalar;
    typedef typename Traits<Polynomial_>::Coefficients Coefficients;
    typedef typename EigenMatrix<Coefficients>::Element Element;
    typedef typename EigenMatrix<Coefficients>::ConstElement ConstElement;

    typedef DepthUndistortionModel<Scalar> Interface;

    typedef EigenMatrix<Coefficients> Data;

    PolynomialUndistortionMatrix()
    {
      // Do nothing
    }

    explicit PolynomialUndistortionMatrix(const typename Data::Ptr & data)
      : data_(data)
    {
      // Do nothing
    }

    virtual ~PolynomialUndistortionMatrix()
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
      z = Polynomial_::evaluate(polynomialAt(x_index, y_index), z);
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

template <typename Polynomial_>
  class PolynomialUndistortionMatrixSphere : public PolynomialUndistortionMatrix<Polynomial_>
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionMatrixSphere> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionMatrixSphere> ConstPtr;

    typedef PolynomialUndistortionMatrix<Polynomial_> Base;

    typedef typename Base::Scalar Scalar;
    typedef typename Base::Coefficients Coefficients;
    typedef typename Base::Element Element;
    typedef typename Base::ConstElement ConstElement;

    typedef typename Base::Data Data;

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
      bin_x_size_ = fov_x_ / Base::data_->xSize();
      bin_y_size_ = fov_y_ / Base::data_->ySize();
      zero_ = typename Types_<Scalar>::Point2(M_PI - fov_x_ / Scalar(2), M_PI_2 - fov_y_ / Scalar(2));
    }

    Scalar fieldOfViewX() const
    {
      return fov_x_;
    }

    Scalar fieldOfViewY() const
    {
      return fov_y_;
    }

    void getIndex(const typename Types_<Scalar>::Point2 & point_sphere,
                  size_t & x_index,
                  size_t & y_index) const
    {
      assert(Base::data_);
      assert(bin_x_size_ > 0 and bin_y_size_ > 0);
      typename Types_<Scalar>::Point2 diff = point_sphere - zero_;
      x_index =
        diff.x() < 0 ? 0 : static_cast<size_t>(std::min(Base::data_->xSize() - 1.0, std::floor(diff.x() / bin_x_size_)));
      y_index =
        diff.y() < 0 ? 0 : static_cast<size_t>(std::min(Base::data_->ySize() - 1.0, std::floor(diff.y() / bin_y_size_)));
    }

    using Base::polynomialAt;

    Element polynomialAt(const typename Types_<Scalar>::Point2 & point_sphere)
    {
      size_t x_index, y_index;
      getIndex(point_sphere, x_index, y_index);
      return Base::polynomialAt(x_index, y_index);
    }

    const ConstElement polynomialAt(const typename Types_<Scalar>::Point2 & point_sphere) const
    {
      size_t x_index, y_index;
      getIndex(point_sphere, x_index, y_index);
      return Base::polynomialAt(x_index, y_index);
    }

    static typename Types_<Scalar>::Point2 toSphericalCoordinates(Scalar x,
                                                                  Scalar y,
                                                                  Scalar z)
    {
      Scalar norm(std::sqrt(x * x + y * y + z * z));
      return typename Types_<Scalar>::Point2(M_PI + std::atan2(x, z), M_PI_2 - std::asin(y / norm));
    }

  protected:

    typename Types_<Scalar>::Point2 zero_;
    Scalar fov_x_;
    Scalar fov_y_;
    Scalar bin_x_size_;
    Scalar bin_y_size_;

  };

template <typename Polynomial_>
  class PolynomialUndistortionMatrixEigen : public PolynomialUndistortionMatrixSphere<Polynomial_>,
                                            public DepthUndistortionModelEigen<typename Traits<Polynomial_>::Scalar>
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionMatrixEigen> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionMatrixEigen> ConstPtr;

    typedef typename Traits<Polynomial_>::Scalar Scalar;
    typedef PolynomialUndistortionMatrixSphere<Polynomial_> Base;
    typedef typename Base::Data Data;

    typedef DepthUndistortionModelEigen<Scalar> Interface;

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

    virtual void undistort(typename Types_<Scalar>::Point3 & point) const
    {
      point *= Polynomial_::evaluate(Base::polynomialAt(toSphericalCoordinates(point)), point.z()) / point.z();
    }

    virtual void undistort(typename Types_<Scalar>::Point3Matrix & cloud) const
    {
      for (size_t i = 0; i < cloud.size(); ++i)
        cloud[i] *= Polynomial_::evaluate(Base::polynomialAt(toSphericalCoordinates(cloud[i])), cloud[i].z())
          / cloud[i].z();
    }

    static typename Types_<Scalar>::Point2 toSphericalCoordinates(const typename Types_<Scalar>::Point3 & point)
    {
      return typename Types_<Scalar>::Point2(M_PI + std::atan2(point.x(), point.z()), M_PI_2 - std::asin(point.y() / point.norm()));
    }

    virtual typename Interface::Ptr clone() const
    {
      PolynomialUndistortionMatrixEigen::Ptr clone = boost::make_shared<PolynomialUndistortionMatrixEigen>(*this);
      clone->setData(boost::make_shared<Data>(*Base::data_));
      return clone;
    }

  };

template <typename Polynomial_, typename PCLPoint_>
  class PolynomialUndistortionMatrixPCL : public PolynomialUndistortionMatrixSphere<Polynomial_>,
                                          public DepthUndistortionModelPCL<typename Traits<Polynomial_>::Scalar, PCLPoint_>
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionMatrixPCL> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionMatrixPCL> ConstPtr;

    typedef typename Traits<Polynomial_>::Scalar Scalar;
    typedef PolynomialUndistortionMatrixSphere<Polynomial_> Base;
    typedef typename Base::Data Data;

    typedef DepthUndistortionModelPCL<Scalar, PCLPoint_> Interface;

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

    virtual void undistort(PCLPoint_ & point) const
    {
      float k = static_cast<float>(Polynomial_::evaluate(Base::polynomialAt(toSphericalCoordinates(point)), point.z))
        / point.z;
      point.x *= k;
      point.y *= k;
      point.z *= k;
    }

    virtual void undistort(pcl::PointCloud<PCLPoint_> & cloud) const
    {
      for (size_t i = 0; i < cloud.size(); ++i)
        undistort(cloud.points[i]);
    }

    static typename Types_<Scalar>::Point2 toSphericalCoordinates(const PCLPoint_ & point)
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
