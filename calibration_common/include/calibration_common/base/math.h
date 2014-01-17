/*
 *  Copyright (C) 2013 - Filippo Basso <bassofil@dei.unipd.it>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CALIBRATION_COMMON_BASE_MATH_H_
#define CALIBRATION_COMMON_BASE_MATH_H_

#include <calibration_common/base/traits.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <unsupported/Eigen/Polynomials>

namespace calibration
{

template <typename Derived>
  class MathFunction
  {
  public:

    typedef boost::shared_ptr<MathFunction> Ptr;
    typedef boost::shared_ptr<const MathFunction> ConstPtr;

    typedef typename Traits<Derived>::Scalar Scalar;
    typedef typename Traits<Derived>::FunctionX FunctionX;
    typedef typename Traits<Derived>::FunctionY FunctionY;

    FunctionY operator ()(const FunctionX & x) const
    {
      return Derived::evaluate(x);
    }

  };

template <typename Scalar_, int Degree, int MinDegree = 0>
  class Polynomial;

template <typename Scalar_, int Degree_, int MinDegree_>
  struct Traits<Polynomial<Scalar_, Degree_, MinDegree_> >
  {
    static const int Size = (Degree_ == Eigen::Dynamic ? Eigen::Dynamic : Degree_ + 1 - MinDegree_);
//    static const int Size = Degree_ + 1 - MinDegree_;
    static const int MinDegree = MinDegree_;
    static const int Degree = Degree_;

    typedef Scalar_ Scalar;
    typedef Scalar FunctionX;
    typedef Scalar FunctionY;
    typedef Eigen::Array<Scalar, Size, 1, Eigen::DontAlign> Coefficients;
  };

template <typename Scalar_, int Degree, int MinDegree>
  class Polynomial : public MathFunction<Polynomial<Scalar_, Degree, MinDegree> >
  {
  public:

    typedef boost::shared_ptr<Polynomial> Ptr;
    typedef boost::shared_ptr<const Polynomial> ConstPtr;

    typedef MathFunction<Polynomial> Base;

    typedef typename Traits<Polynomial>::Scalar Scalar;
    typedef typename Traits<Polynomial>::FunctionX FunctionX;
    typedef typename Traits<Polynomial>::FunctionY FunctionY;
    typedef typename Traits<Polynomial>::Coefficients Coefficients;

    Polynomial()
    {
      EIGEN_STATIC_ASSERT(MinDegree >= 0, INVALID_MATRIX_TEMPLATE_PARAMETERS);
      EIGEN_STATIC_ASSERT(Degree >= MinDegree, INVALID_MATRIX_TEMPLATE_PARAMETERS);
    }

    template <typename OtherDerived>
      explicit Polynomial(const Eigen::DenseBase<OtherDerived> & coefficients)
        : coefficients_(coefficients)
      {
        EIGEN_STATIC_ASSERT(MinDegree >= 0, INVALID_MATRIX_TEMPLATE_PARAMETERS);
        EIGEN_STATIC_ASSERT(Degree >= MinDegree, INVALID_MATRIX_TEMPLATE_PARAMETERS);
      }

    FunctionY evaluate(const FunctionX & x) const
    {
      return evaluate(coefficients_, x);
    }

    template <typename OtherDerived>
      void setCoefficients(const Eigen::DenseBase<OtherDerived> & coefficients)
      {
        coefficients_ = coefficients;
      }

    const Coefficients & coefficients() const
    {
      return coefficients_;
    }

    Scalar * data()
    {
      return coefficients_.data();
    }

    const Scalar * data() const
    {
      return coefficients_.data();
    }

    static const Polynomial Identity()
    {
      return Polynomial(IdentityCoefficients());
    }

    size_t size() const
    {
      return Traits<Polynomial>::SIZE;
    }

    static const Coefficients IdentityCoefficients()
    {
      EIGEN_STATIC_ASSERT(MinDegree <= 1 and Degree >= 1, INVALID_MATRIX_TEMPLATE_PARAMETERS);
      Coefficients coeffs(Coefficients::Zero());
      coeffs[1 - MinDegree] = Scalar(1);
      return coeffs;
    }

    template <typename OtherDerived>
      static FunctionY evaluate(const Eigen::DenseBase<OtherDerived> & coefficients,
                                const FunctionX & x)
      {
        FunctionY y = Eigen::poly_eval(coefficients, x);
        for (int i = 0; i < MinDegree; ++i)
          y *= x;
        return y;
      }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:

    Coefficients coefficients_;

  };

template <typename Scalar_>
  class Polynomial<Scalar_, Eigen::Dynamic, 0> : public MathFunction<Polynomial<Scalar_, Eigen::Dynamic, 0> >
  {
  public:

    typedef boost::shared_ptr<Polynomial> Ptr;
    typedef boost::shared_ptr<const Polynomial> ConstPtr;

    typedef MathFunction<Polynomial> Base;

    typedef typename Traits<Polynomial>::Scalar Scalar;
    typedef typename Traits<Polynomial>::FunctionX FunctionX;
    typedef typename Traits<Polynomial>::FunctionY FunctionY;
    typedef typename Traits<Polynomial>::Coefficients Coefficients;

    Polynomial(size_t degree)
      : coefficients_(degree + 1)
    {
      // Do nothing
    }

    template <typename OtherDerived>
      explicit Polynomial(const Eigen::DenseBase<OtherDerived> & coefficients)
        : coefficients_(coefficients)
      {
        // Do nothing
      }

    size_t degree() const
    {
      return coefficients_.size() - 1;
    }

    size_t minDegree() const
    {
      return 0;
    }

    size_t size() const
    {
      return coefficients_.size();
    }

    FunctionY evaluate(const FunctionX & x) const
    {
      return evaluate(coefficients_, x);
    }

    Scalar & operator [](size_t index)
    {
      return coefficients_.coeffRef(index);
    }

    const Scalar & operator [](size_t index) const
    {
      return coefficients_.coeffRef(index);
    }

    template <typename OtherDerived>
      void setCoefficients(const Eigen::DenseBase<OtherDerived> & coefficients)
      {
        coefficients_ = coefficients;
      }

    const Coefficients & coefficients() const
    {
      return coefficients_;
    }

    Scalar * data()
    {
      return coefficients_.data();
    }

    const Scalar * data() const
    {
      return coefficients_.data();
    }

    static const Polynomial Identity(int degree)
    {
      return Polynomial(IdentityCoefficients(degree));
    }

    static const Coefficients IdentityCoefficients(int degree)
    {
      assert(degree >= 1);
      Coefficients coeffs(Coefficients::Zero(degree + 1));
      coeffs[1] = Scalar(1);
      return coeffs;
    }

    template <typename OtherDerived>
      static FunctionY evaluate(const Eigen::DenseBase<OtherDerived> & coefficients,
                                const FunctionX & x)
      {
        return Eigen::poly_eval(coefficients, x);
      }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:

    Coefficients coefficients_;

  };

//template <typename Scalar_>
//  class Polynomial<Scalar_, Eigen::Dynamic, Eigen::Dynamic> : public MathFunction<
//    Polynomial<Scalar_, Eigen::Dynamic, Eigen::Dynamic> >
//  {
//  public:
//
//    typedef boost::shared_ptr<Polynomial> Ptr;
//    typedef boost::shared_ptr<const Polynomial> ConstPtr;
//
//    typedef MathFunction<Polynomial> Base;
//
//    typedef typename Traits<Polynomial>::Scalar Scalar;
//    typedef typename Traits<Polynomial>::FunctionX FunctionX;
//    typedef typename Traits<Polynomial>::FunctionY FunctionY;
//    typedef typename Traits<Polynomial>::Coefficients Coefficients;
//
//    Polynomial(size_t degree,
//               size_t min_degree = 0)
//      : coefficients_(degree - min_degree + 1),
//        degree_(degree),
//        min_degree_(min_degree)
//    {
//      // Do nothing
//    }
//
//    template <typename OtherDerived>
//      explicit Polynomial(const Eigen::DenseBase<OtherDerived> & coefficients,
//                          size_t min_degree = 0)
//        : coefficients_(coefficients),
//          degree_(min_degree + coefficients.size() - 1),
//          min_degree_(min_degree)
//      {
//        // Do nothing
//      }
//
//    size_t degree() const
//    {
//      return degree_;
//    }
//
//    size_t minDegree() const
//    {
//      return min_degree_;
//    }
//
//    size_t size() const
//    {
//      return degree_ - min_degree_ + 1;
//    }
//
//    FunctionY evaluate(const FunctionX & x) const
//    {
//      return evaluate(coefficients_, min_degree_, x);
//    }
//
//    Scalar & operator [](size_t index)
//    {
//      return coefficients_.coeffRef(index);
//    }
//
//    const Scalar & operator [](size_t index) const
//    {
//      return coefficients_.coeffRef(index);
//    }
//
//    template <typename OtherDerived>
//      void setCoefficients(const Eigen::DenseBase<OtherDerived> & coefficients)
//      {
//        assert(coefficients.size() == size());
//        coefficients_ = coefficients;
//      }
//
//    const Coefficients & coefficients() const
//    {
//      return coefficients_;
//    }
//
//    Scalar * data()
//    {
//      return coefficients_.data();
//    }
//
//    const Scalar * data() const
//    {
//      return coefficients_.data();
//    }
//
//    static const Polynomial Identity(int degree,
//                                     int min_degree = 0)
//    {
//      return Polynomial(IdentityCoefficients(degree, min_degree));
//    }
//
//    static const Coefficients IdentityCoefficients(int degree,
//                                                   int min_degree = 0)
//    {
//      assert(min_degree <= 1 and degree >= 1);
//      Coefficients coeffs(Coefficients::Zero());
//      coeffs[1 - MinDegree] = Scalar(1);
//      return coeffs;
//    }
//
//    template <typename OtherDerived>
//      static FunctionY evaluate(const Eigen::DenseBase<OtherDerived> & coefficients,
//                                int min_degree,
//                                const FunctionX & x)
//      {
//        FunctionY y = Eigen::poly_eval(coefficients, x);
//        for (int i = 0; i < min_degree; ++i)
//          y *= x;
//        return y;
//      }
//
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//
//  protected:
//
//    Coefficients coefficients_;
//    size_t degree_;
//    size_t min_degree_;
//
//  };

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_BASE_MATH_H_ */
