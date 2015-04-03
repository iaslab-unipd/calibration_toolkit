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

#ifndef CALIBRATION_COMMON_CERES_MATH_H_
#define CALIBRATION_COMMON_CERES_MATH_H_

#include <calibration_common/base/math.h>
#include <ceres/ceres.h>

namespace ceres
{

/**
 * @brief poly_eval
 * @param polynomial
 * @param x
 * @return
 */
template <typename DerivedT_>
  typename Eigen::DenseBase<DerivedT_>::Scalar poly_eval(const Eigen::DenseBase<DerivedT_> & polynomial,
                                                         const typename Eigen::DenseBase<DerivedT_>::Scalar & x)
  {
    typedef typename Eigen::DenseBase<DerivedT_>::Scalar Scalar;
    typedef typename Eigen::NumTraits<Scalar>::Real Real;

    if (Eigen::numext::abs2(x) <= Real(1))
      return Eigen::poly_eval_horner(polynomial, x);
    else
    {
      Scalar val = polynomial[0];
      Scalar inv_x = Scalar(1) / x;
      for (Eigen::DenseIndex i = 1; i < polynomial.size(); ++i)
        val = val * inv_x + polynomial[i];

      return Eigen::numext::pow(x, Scalar(polynomial.size() - 1)) * val;
    }
  }

/**
 * @brief The Polynomial class
 * @param ScalarT_
 * @param Degree_
 * @param MinDegree_
 */
template <typename ScalarT_, int Degree_, int MinDegree_ = 0>
  class Polynomial : public calibration::Polynomial<ScalarT_, Degree_, MinDegree_>
  {
  public:

    typedef boost::shared_ptr<Polynomial> Ptr;
    typedef boost::shared_ptr<const Polynomial> ConstPtr;

    typedef calibration::Polynomial<ScalarT_, Degree_, MinDegree_> Base;

    typedef typename Base::Scalar Scalar;
    typedef typename Base::FunctionX FunctionX;
    typedef typename Base::FunctionY FunctionY;
    typedef typename Base::Coefficients Coefficients;

    /**
     * @brief Polynomial
     */
    Polynomial()
    {
      EIGEN_STATIC_ASSERT(MinDegree_ >= 0, INVALID_MATRIX_TEMPLATE_PARAMETERS);
      EIGEN_STATIC_ASSERT(Degree_ >= MinDegree_, INVALID_MATRIX_TEMPLATE_PARAMETERS);
    }

    /**
     * @brief Polynomial
     * @param coefficients
     */
    template <typename OtherDerived>
      explicit Polynomial(const Eigen::DenseBase<OtherDerived> & coefficients)
        : Base(coefficients)
      {
        EIGEN_STATIC_ASSERT(MinDegree_ >= 0, INVALID_MATRIX_TEMPLATE_PARAMETERS);
        EIGEN_STATIC_ASSERT(Degree_ >= MinDegree_, INVALID_MATRIX_TEMPLATE_PARAMETERS);
      }

    /**
     * @brief evaluate
     * @param x
     * @return
     */
    inline FunctionY evaluate(const FunctionX & x) const
    {
      return evaluate(Base::coefficients_, x);
    }

    /**
     * @brief Identity
     * @return
     */
    inline static const Polynomial Identity()
    {
      return Polynomial(IdentityCoefficients());
    }

    /**
     * @brief IdentityCoefficients
     * @return
     */
    inline static const Coefficients IdentityCoefficients()
    {
      EIGEN_STATIC_ASSERT(MinDegree_ <= 1 and Degree_ >= 1, INVALID_MATRIX_TEMPLATE_PARAMETERS);
      Coefficients coeffs(Coefficients::Zero());
      coeffs[1 - MinDegree_] = Scalar(1);
      return coeffs;
    }

    /**
     * @brief evaluate
     * @param coefficients
     * @param x
     * @return
     */
    template <typename OtherDerived>
      inline static FunctionY evaluate(const Eigen::DenseBase<OtherDerived> & coefficients,
                                       const FunctionX & x)
      {
        FunctionY y = poly_eval(coefficients, x);
        for (int i = 0; i < MinDegree_; ++i)
          y *= x;
        return y;
      }

  };

} /* namespace ceres */

namespace calibration
{
template <typename ScalarT_, int Degree_, int MinDegree_>
  struct MathTraits<ceres::Polynomial<ScalarT_, Degree_, MinDegree_> >
  {
    static const int Size = (Degree_ == Eigen::Dynamic ? Eigen::Dynamic : Degree_ + 1 - MinDegree_);
    static const int MinDegree = MinDegree_;
    static const int Degree = Degree_;

    typedef ScalarT_ Scalar;
    typedef Scalar FunctionX;
    typedef Scalar FunctionY;
    typedef Eigen::Array<Scalar, Size, 1, Eigen::DontAlign> Coefficients;
  };
} /* namespace calibration */

#endif /* CALIBRATION_COMMON_CERES_MATH_H_ */
