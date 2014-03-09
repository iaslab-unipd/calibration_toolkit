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

#ifndef CALIBRATION_COMMON_ALGORITHMS_POLYNOMIAL_FIT_H_
#define CALIBRATION_COMMON_ALGORITHMS_POLYNOMIAL_FIT_H_

#include <calibration_common/objects/globals.h>
#include <calibration_common/base/math.h>
#include <calibration_common/ceres/ceres.h>

namespace calibration
{

template <typename ScalarT_>
  class MathFunctionFit
  {
  public:

    typedef boost::shared_ptr<MathFunctionFit> Ptr;
    typedef boost::shared_ptr<const MathFunctionFit> ConstPtr;

    virtual ~MathFunctionFit()
    {
      // Do nothing
    }

    virtual void addData(const ScalarT_ & x,
                         const ScalarT_ & y) = 0;

    virtual bool update() = 0;

  };

template <typename PolynomialT_>
  class PolynomialResidual
  {
  public:

    typedef typename MathTraits<PolynomialT_>::Scalar Scalar;

    static const int Size = MathTraits<PolynomialT_>::Size;
    static const int MinDegree = MathTraits<PolynomialT_>::MinDegree;
    static const int Degree = MathTraits<PolynomialT_>::Degree;

    PolynomialResidual(Scalar x,
                       Scalar y)
      : x_(x),
        y_(y)
    {
      // Do nothing
    }

    //    template <typename T, int N>
    //      bool operator()(const ceres::Jet<T, N> * const coefficients,
    //                      ceres::Jet<T, N> * residual) const
    //      {
    //        typedef typename Traits<Polynomial<ceres::Jet<T, N>, Degree, MinDegree> >::Coefficients Coefficients;
    //        ceres::Jet<T, N> y = ceres::poly_eval(Eigen::Map<const Coefficients>(coefficients), ceres::Jet<T, N>(x_));
    //        for (int i = 0; i < MinDegree; ++i)
    //          y = y * x_;
    //        residual[0] = ceres::Jet<T, N>(y_) - y;
    //        return true;
    //      }

    template <typename T>
      bool operator()(const T * const coefficients,
                      T * residual) const
      {
        typedef ceres::Polynomial<T, Degree, MinDegree> Polynomial_;
        typedef typename MathTraits<Polynomial_>::Coefficients Coefficients;
        residual[0] = T(y_) - Polynomial_::evaluate(Eigen::Map<const Coefficients>(coefficients), T(x_));
        return true;
      }

  private:

    const Scalar x_;
    const Scalar y_;

  };

template <typename PolynomialT_>
  class PolynomialFit : public MathFunctionFit<typename MathTraits<PolynomialT_>::Scalar>
  {
  public:

    typedef boost::shared_ptr<PolynomialFit> Ptr;
    typedef boost::shared_ptr<const PolynomialFit> ConstPtr;

    typedef typename MathTraits<PolynomialT_>::Scalar Scalar;
    typedef typename boost::shared_ptr<PolynomialT_> PolynomialPtr;

    static const int Size = MathTraits<PolynomialT_>::Size;
    static const int MinDegree = MathTraits<PolynomialT_>::MinDegree;
    static const int Degree = MathTraits<PolynomialT_>::Degree;

    typedef std::vector<std::pair<Scalar, Scalar> > DataBin;

    PolynomialFit(const PolynomialPtr & polynomial)
      : polynomial_(polynomial)
    {
      // Do nothing
    }

    virtual ~PolynomialFit()
    {
      // Do nothing
    }

    virtual void addData(const Scalar & x,
                         const Scalar & y)
    {
      data_bin_.push_back(std::make_pair(x, y));
    }

    const PolynomialT_ & polynomial()
    {
      return *polynomial_;
    }

    virtual bool update();

  private:

    DataBin data_bin_;
    PolynomialPtr polynomial_;

  };

} /* namespace calibration */

#include <impl/calibration_common/ceres/polynomial_fit.hpp>

#endif /* CALIBRATION_COMMON_ALGORITHMS_POLYNOMIAL_FIT_H_ */
