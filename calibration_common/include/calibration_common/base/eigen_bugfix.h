/*
 * Copyright (c) 2010, Manuel Yguel <manuel.yguel@gmail.com>
 * Copyright (c) 2014, Filippo Basso <bassofil@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0
 */

#ifndef UNIPD_CALIBRATION_CALIBRATION_COMMON_BASE_EIGEN_BUGFIX_H_
#define UNIPD_CALIBRATION_CALIBRATION_COMMON_BASE_EIGEN_BUGFIX_H_

#include <unsupported/Eigen/Polynomials>

namespace Eigen
{

/**
 * @brief Bugfix for Eigen::poly_eval method
 * @returns The evaluation of the polynomial at x using stabilized Horner algorithm.
 *
 * @param[in] polynomial The vector of coefficients of the polynomial ordered
 *  by degrees i.e. polynomial[i] is the coefficient of degree i of the polynomial
 *  e.g. \f$ 1 + 3x^2 \f$ is stored as a vector \f$ [1, 0, 3] \f$.
 * @param[in] x The value to evaluate the polynomial at.
 */
template <typename DerivedT_>
  typename DenseBase<DerivedT_>::Scalar poly_eval_bugfix(const DenseBase<DerivedT_> & polynomial,
                                                         const typename DenseBase<DerivedT_>::Scalar & x)
  {
    typedef typename Eigen::DenseBase<DerivedT_>::Scalar Scalar;
    typedef typename Eigen::NumTraits<Scalar>::Real Real;

    if (numext::abs2(x) <= Real(1))
      return poly_eval_horner(polynomial, x);
    else
    {
      Scalar val = polynomial[0];
      Scalar inv_x = Scalar(1.0) / x;
      for (Eigen::DenseIndex i = 1; i < polynomial.size(); ++i)
        val = val * inv_x + polynomial[i];

      return numext::pow(x, static_cast<Scalar>(polynomial.size() - 1)) * val;
    }
  }

} // namespace Eigen

#endif // UNIPD_CALIBRATION_CALIBRATION_COMMON_BASE_EIGEN_BUGFIX_H_
