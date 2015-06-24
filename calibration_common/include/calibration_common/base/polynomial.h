/*
 *  Copyright (c) 2015-, Filippo Basso <bassofil@gmail.com>
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

#ifndef UNIPD_CALIBRATION_CALIBRATION_COMMON_BASE_POLYNOMIAL_H_
#define UNIPD_CALIBRATION_CALIBRATION_COMMON_BASE_POLYNOMIAL_H_

#include <calibration_common/base/eigen_bugfix.h>

namespace unipd
{
namespace calib
{

constexpr int
computePolynomialSize (int max_degree,
                       int min_degree = 0)
{
  return max_degree - min_degree + 1;
}

template <typename ScalarT_, int MaxDegree_, int MinDegree_>
  class Polynomial_
  {
  public:

    static_assert(MaxDegree_ != Eigen::Dynamic, "MaxDegree_ cannot be Eigen::Dynamic: "
                                                "(a specialized class template <typename ScalarT_> class "
                                                "Polynomial_<ScalarT_, Eigen::Dynamic, Eigen::Dynamic> exists).");
    static_assert(MinDegree_ != Eigen::Dynamic, "MinDegree_ cannot be Eigen::Dynamic: "
                                                "(a specialized class template <typename ScalarT_> "
                                                "class Polynomial_<ScalarT_, Eigen::Dynamic, Eigen::Dynamic> exists).");
    static_assert(MaxDegree_ >= MinDegree_, "MaxDegree_ >= MinDegree_");

    enum : int
    {
      MinDegree = MinDegree_,
      MaxDegree = MaxDegree_,
      Size = computePolynomialSize(MaxDegree, MinDegree),
    };

    using Scalar = ScalarT_;
    using Coefficients = Eigen::Array<Scalar, Size, 1>;

    Polynomial_ ()
      : coefficients_(Coefficients::Zero())
    {
      // Do nothing
    }

    template <typename OtherScalarT_>
      Polynomial_<OtherScalarT_, MaxDegree_, MinDegree_>
      cast () const
      {
        Polynomial_<OtherScalarT_, MaxDegree_, MinDegree_> p;
        p.setCoefficients(coefficients_.template cast<OtherScalarT_>());
        return p;
      }

    bool
    initialized () const
    {
      return true;
    }

    constexpr int
    maxDegree () const
    {
      return MaxDegree;
    }

    constexpr int
    minDegree () const
    {
      return MinDegree;
    }

    constexpr int
    size () const
    {
      return Size;
    }

    template <typename DerivedT_>
      void
      setCoefficients (const Eigen::DenseBase<DerivedT_> & coefficients)
      {
        coefficients_ = coefficients;
      }

    const Coefficients &
    coefficients  () const
    {
      return coefficients_;
    }

    Coefficients
    identityCoefficients () const
    {
      static_assert(MinDegree <= 1 and MaxDegree >= 1, "MinDegree <= 1 and MaxDegree >= 1");
      Coefficients coeffs(Coefficients::Zero());
      coeffs[1 - MinDegree] = Scalar(1);
      return coeffs;
    }

    Coefficients
    createCoefficients () const
    {
      return Coefficients();
    }

    template <typename DerivedT_>
      ScalarT_
      evaluate (const Eigen::DenseBase<DerivedT_> & coefficients,
                const ScalarT_ & x)
      {
        ScalarT_ y = Eigen::poly_eval_bugfix(coefficients, x);
        for (int i = 0; i < MinDegree; ++i)
          y *= x;
        return y;
      }

    ScalarT_
    evaluate (const ScalarT_ & x)
    {
      return evaluate(coefficients_, x);
    }

  private:

      Coefficients coefficients_;

  };

template <typename ScalarT_>
  class Polynomial_<ScalarT_, Eigen::Dynamic, Eigen::Dynamic>
  {
  public:

    enum : int
    {
      MinDegree = Eigen::Dynamic,
      MaxDegree = Eigen::Dynamic,
      Size = Eigen::Dynamic,
    };

    using Scalar = ScalarT_;
    using Coefficients = Eigen::Array<Scalar, Size, 1>;

    Polynomial_ ()
      : max_degree_(0),
        min_degree_(0),
        size_(0),
        initialized_(false)
    {
      // Do nothing
    }

    template <typename DerivedT_>
      Polynomial_ (int max_degree,
                   const Eigen::DenseBase<DerivedT_> & coefficients)
        : max_degree_(max_degree),
          min_degree_(max_degree - coefficients.size() + 1),
          size_(coefficients.size()),
          initialized_(true),
          coefficients_(coefficients)
      {
        // Do nothing
      }

    Polynomial_ (int max_degree,
                 int min_degree)
      : max_degree_(max_degree),
        min_degree_(min_degree),
        size_(computePolynomialSize(max_degree, min_degree)),
        initialized_(true),
        coefficients_(Coefficients::Zero(size_))
    {
      // Do nothing
    }

    template <typename OtherScalarT_, int MaxDegree_, int MinDegree_>
      explicit
      Polynomial_ (const Polynomial_<OtherScalarT_, MaxDegree_, MinDegree_> & other)
        :  max_degree_(other.maxDegree()),
           min_degree_(other.minDegree()),
           size_(other.size()),
           initialized_(other.initialized())
      {
        // Do nothing
      }

    template <typename OtherScalarT_>
      Polynomial_<OtherScalarT_, Eigen::Dynamic, Eigen::Dynamic>
      cast () const
      {
        Polynomial_<OtherScalarT_, Eigen::Dynamic, Eigen::Dynamic> p(max_degree_, min_degree_);
        p.setCoefficients(coefficients_.template cast<OtherScalarT_>());
        return p;
      }

    bool
    initialized () const
    {
      return initialized_;
    }

    int
    maxDegree () const
    {
      assert(initialized_);
      return max_degree_;
    }

    int
    minDegree () const
    {
      assert(initialized_);
      return min_degree_;
    }

    int
    size () const
    {
      assert(initialized_);
      return size_;
    }

    void
    setDegrees (int max_degree,
                int min_degree)
    {
      max_degree_ = max_degree;
      min_degree_ = min_degree;
      size_ = computePolynomialSize(max_degree_, min_degree_);
      initialized_ = true;
    }

    template <typename DerivedT_>
      void
      setCoefficients (const Eigen::DenseBase<DerivedT_> & coefficients)
      {
        coefficients_ = coefficients;
      }

    const Coefficients &
    coefficients  () const
    {
      return coefficients_;
    }

    Coefficients
    identityCoefficients () const
    {
      assert(initialized_);
      assert(min_degree_ <= 1 and max_degree_ >= 1);
      Coefficients coeffs(Coefficients::Zero(size_));
      coeffs[1 - min_degree_] = Scalar(1);
      return coeffs;
    }

    Coefficients
    createCoefficients () const
    {
      assert(initialized_);
      return Coefficients(size_);
    }

    template <typename DerivedT_>
      ScalarT_
      evaluate (const Eigen::DenseBase<DerivedT_> & coefficients,
                const ScalarT_ & x)
      {
        assert(initialized_);
        ScalarT_ y = Eigen::poly_eval_bugfix(coefficients, x);
        for (int i = 0; i < min_degree_; ++i)
          y *= x;
        return y;
      }

      ScalarT_
      evaluate (const ScalarT_ & x)
      {
        return evaluate(coefficients_, x);
      }


  private:

    int max_degree_;
    int min_degree_;
    int size_;
    bool initialized_;
    Coefficients coefficients_;

  };

template <typename ScalarT_>
  using PolynomialX_ = Polynomial_<ScalarT_, Eigen::Dynamic, Eigen::Dynamic>;

} // namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_CALIBRATION_COMMON_BASE_POLYNOMIAL_H_
