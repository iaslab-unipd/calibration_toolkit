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

#ifndef CALIBRATION_COMMON_BASE_MATH_H_
#define CALIBRATION_COMMON_BASE_MATH_H_

#include <boost/smart_ptr/shared_ptr.hpp>
#include <unsupported/Eigen/Polynomials>

namespace calibration
{

template <typename DerivedT_>
  struct MathTraits
  {

  };

template <typename DerivedT_>
  class MathFunction
  {
  public:

    typedef boost::shared_ptr<MathFunction> Ptr;
    typedef boost::shared_ptr<const MathFunction> ConstPtr;

    typedef typename MathTraits<DerivedT_>::Scalar Scalar;
    typedef typename MathTraits<DerivedT_>::FunctionX FunctionX;
    typedef typename MathTraits<DerivedT_>::FunctionY FunctionY;

    FunctionY operator ()(const FunctionX & x) const
    {
      return DerivedT_::evaluate(x);
    }

  };

template <typename ScalarT_, int Degree_, int MinDegree_ = 0>
  class Polynomial;

template <typename ScalarT_, int Degree_, int MinDegree_>
  struct MathTraits<Polynomial<ScalarT_, Degree_, MinDegree_> >
  {
    static const int Size = (Degree_ == Eigen::Dynamic ? Eigen::Dynamic : Degree_ + 1 - MinDegree_);
    //    static const int Size = Degree_ + 1 - MinDegree_;
    static const int MinDegree = MinDegree_;
    static const int Degree = Degree_;

    typedef ScalarT_ Scalar;
    typedef Scalar FunctionX;
    typedef Scalar FunctionY;
    typedef Eigen::Array<Scalar, Size, 1> Coefficients;
  };

template <typename ScalarT_, int Degree_, int MinDegree_>
  class Polynomial : public MathFunction<Polynomial<ScalarT_, Degree_, MinDegree_> >
  {
  public:

    typedef boost::shared_ptr<Polynomial> Ptr;
    typedef boost::shared_ptr<const Polynomial> ConstPtr;

    typedef MathFunction<Polynomial> Base;

    typedef typename MathTraits<Polynomial>::Scalar Scalar;
    typedef typename MathTraits<Polynomial>::FunctionX FunctionX;
    typedef typename MathTraits<Polynomial>::FunctionY FunctionY;
    typedef typename MathTraits<Polynomial>::Coefficients Coefficients;

    Polynomial()
    {
      EIGEN_STATIC_ASSERT(MinDegree_ >= 0, INVALID_MATRIX_TEMPLATE_PARAMETERS);
      EIGEN_STATIC_ASSERT(Degree_ >= MinDegree_, INVALID_MATRIX_TEMPLATE_PARAMETERS);
    }

    template <typename OtherDerived>
      explicit Polynomial(const Eigen::DenseBase<OtherDerived> & coefficients)
        : coefficients_(coefficients)
      {
        EIGEN_STATIC_ASSERT(MinDegree_ >= 0, INVALID_MATRIX_TEMPLATE_PARAMETERS);
        EIGEN_STATIC_ASSERT(Degree_ >= MinDegree_, INVALID_MATRIX_TEMPLATE_PARAMETERS);
      }

    FunctionY evaluate(const FunctionX & x) const
    {
      return evaluate(coefficients_, x);
    }

    const Coefficients & coefficients() const
    {
      return coefficients_;
    }

    Coefficients & coefficients()
    {
      return coefficients_;
    }

    Scalar * dataPtr()
    {
      return coefficients_.data();
    }

    const Scalar * dataPtr() const
    {
      return coefficients_.data();
    }

    static const Polynomial Identity()
    {
      return Polynomial(IdentityCoefficients());
    }

    size_t size() const
    {
      return MathTraits<Polynomial>::Size;
    }

    static const Coefficients IdentityCoefficients()
    {
      EIGEN_STATIC_ASSERT(MinDegree_ <= 1 and Degree_ >= 1, INVALID_MATRIX_TEMPLATE_PARAMETERS);
      Coefficients coeffs(Coefficients::Zero());
      coeffs[1 - MinDegree_] = Scalar(1);
      return coeffs;
    }

    template <typename OtherDerived>
      static FunctionY evaluate(const Eigen::DenseBase<OtherDerived> & coefficients,
                                const FunctionX & x)
      {
        FunctionY y = Eigen::poly_eval(coefficients, x);
        for (int i = 0; i < MinDegree_; ++i)
          y *= x;
        return y;
      }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:

    Coefficients coefficients_;

  };

template <typename ScalarT_>
  class Polynomial<ScalarT_, Eigen::Dynamic, 0> : public MathFunction<Polynomial<ScalarT_, Eigen::Dynamic, 0> >
  {
  public:

    typedef boost::shared_ptr<Polynomial> Ptr;
    typedef boost::shared_ptr<const Polynomial> ConstPtr;

    typedef MathFunction<Polynomial> Base;

    typedef typename MathTraits<Polynomial>::Scalar Scalar;
    typedef typename MathTraits<Polynomial>::FunctionX FunctionX;
    typedef typename MathTraits<Polynomial>::FunctionY FunctionY;
    typedef typename MathTraits<Polynomial>::Coefficients Coefficients;

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

//template <typename ScalarT_>
//class Polynomial<ScalarT_, Eigen::Dynamic, Eigen::Dynamic> : public MathFunction<Polynomial<ScalarT_, Eigen::Dynamic, Eigen::Dynamic> >
//{
//public:
//
//  typedef boost::shared_ptr<Polynomial> Ptr;
//  typedef boost::shared_ptr<const Polynomial> ConstPtr;
//
//  typedef MathFunction<Polynomial> Base;
//
//  typedef typename Traits<Polynomial>::Scalar Scalar;
//  typedef typename Traits<Polynomial>::FunctionX FunctionX;
//  typedef typename Traits<Polynomial>::FunctionY FunctionY;
//  typedef typename Traits<Polynomial>::Coefficients Coefficients;
//
//  Polynomial(size_t degree,
//             size_t min_degree = 0)
//    : coefficients_(degree - min_degree + 1),
//      degree_(degree),
//      min_degree_(min_degree)
//  {
//    // Do nothing
//  }
//
//  template <typename OtherDerived>
//  explicit Polynomial(const Eigen::DenseBase<OtherDerived> & coefficients,
//                      size_t min_degree = 0)
//    : coefficients_(coefficients),
//      degree_(min_degree + coefficients.size() - 1),
//      min_degree_(min_degree)
//  {
//    // Do nothing
//  }
//
//  size_t degree() const
//  {
//    return degree_;
//  }
//
//  size_t minDegree() const
//  {
//    return min_degree_;
//  }
//
//  size_t size() const
//  {
//    return degree_ - min_degree_ + 1;
//  }
//
//  FunctionY evaluate(const FunctionX & x) const
//  {
//    return evaluate(coefficients_, min_degree_, x);
//  }
//
//  Scalar & operator [](size_t index)
//  {
//    return coefficients_.coeffRef(index);
//  }
//
//  const Scalar & operator [](size_t index) const
//  {
//    return coefficients_.coeffRef(index);
//  }
//
//  template <typename OtherDerived>
//  void setCoefficients(const Eigen::DenseBase<OtherDerived> & coefficients)
//  {
//    assert(coefficients.size() == size());
//    coefficients_ = coefficients;
//  }
//
//  const Coefficients & coefficients() const
//  {
//    return coefficients_;
//  }
//
//  Scalar * data()
//  {
//    return coefficients_.data();
//  }
//
//  const Scalar * data() const
//  {
//    return coefficients_.data();
//  }
//
//  static const Polynomial Identity(int degree,
//                                   int min_degree = 0)
//  {
//    return Polynomial(IdentityCoefficients(degree, min_degree));
//  }
//
//  static const Coefficients IdentityCoefficients(int degree,
//                                                 int min_degree = 0)
//  {
//    assert(min_degree <= 1 and degree >= 1);
//    Coefficients coeffs(Coefficients::Zero());
//    coeffs[1 - MinDegree] = Scalar(1);
//    return coeffs;
//  }
//
//  template <typename OtherDerived>
//  static FunctionY evaluate(const Eigen::DenseBase<OtherDerived> & coefficients,
//                            int min_degree,
//                            const FunctionX & x)
//  {
//    FunctionY y = Eigen::poly_eval(coefficients, x);
//    for (int i = 0; i < min_degree; ++i)
//      y *= x;
//    return y;
//  }
//
//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//
//  protected:
//
//    Coefficients coefficients_;
//  size_t degree_;
//  size_t min_degree_;
//
//};

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_BASE_MATH_H_ */
