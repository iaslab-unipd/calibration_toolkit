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

/**
 * @brief The MathFunctionFit class
 */
template <typename ScalarT_>
  class MathFunctionFit
  {
  public:

    typedef boost::shared_ptr<MathFunctionFit> Ptr;
    typedef boost::shared_ptr<const MathFunctionFit> ConstPtr;

    /**
     * @brief ~MathFunctionFit
     */
    virtual ~MathFunctionFit()
    {
      // Do nothing
    }

    /**
     * @brief addData
     * @param x
     * @param y
     */
    virtual void addData(const ScalarT_ & x,
                         const ScalarT_ & y) = 0;

    /**
     * @brief update
     * @return
     */
    virtual bool update() = 0;

  };

/**
 * @brief The PolynomialResidual class
 * @param PolynomialT_
 */
template <typename PolynomialT_>
  class PolynomialResidual
  {
  public:

    typedef typename MathTraits<PolynomialT_>::Scalar Scalar;

    static const int Size = MathTraits<PolynomialT_>::Size;
    static const int MinDegree = MathTraits<PolynomialT_>::MinDegree;
    static const int Degree = MathTraits<PolynomialT_>::Degree;

    /**
     * @brief PolynomialResidual
     * @param x
     * @param y
     */
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

    /**
     * @brief operator ()
     * @param coefficients
     * @param residual
     * @return
     */
    template <typename T>
      inline bool operator()(const T * const coefficients,
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

/**
 * @brief The PolynomialFit class
 * @param PolynomialT_
 */
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

    /**
     * @brief PolynomialFit
     * @param polynomial
     */
    PolynomialFit(const PolynomialPtr & polynomial)
      : polynomial_(polynomial)
    {
      // Do nothing
    }

    /**
     * @brief ~PolynomialFit
     */
    virtual ~PolynomialFit()
    {
      // Do nothing
    }

    /**
     * @brief addData
     * @param x
     * @param y
     */
    inline virtual void addData(const Scalar & x,
                                const Scalar & y)
    {
      data_bin_.push_back(std::make_pair(x, y));
    }

    /**
     * @brief polynomial
     * @return
     */
    inline const PolynomialT_ & polynomial()
    {
      return *polynomial_;
    }

    /**
     * @brief update
     * @return
     */
    virtual bool update();

  private:

    DataBin data_bin_;
    PolynomialPtr polynomial_;

  };

/**
 * @brief The WeightedPolynomialResidual class
 * @param PolynomialT_
 */
template <typename PolynomialT_>
  class WeightedPolynomialResidual
  {
  public:

    typedef typename MathTraits<PolynomialT_>::Scalar Scalar;

    static const int Size = MathTraits<PolynomialT_>::Size;
    static const int MinDegree = MathTraits<PolynomialT_>::MinDegree;
    static const int Degree = MathTraits<PolynomialT_>::Degree;

    /**
     * @brief WeightedPolynomialResidual
     * @param x
     * @param y
     * @param weight
     */
    WeightedPolynomialResidual(Scalar x,
                               Scalar y,
                               Scalar weight)
      : x_(x),
        y_(y),
        weight_(weight)
    {
      // Do nothing
    }

    /**
     * @brief operator ()
     * @param coefficients
     * @param residual
     * @return
     */
    template <typename T>
      inline bool operator()(const T * const coefficients,
                             T * residual) const
      {
        typedef ceres::Polynomial<T, Degree, MinDegree> Polynomial_;
        typedef typename MathTraits<Polynomial_>::Coefficients Coefficients;
        residual[0] = T(weight_) * (T(y_) - Polynomial_::evaluate(Eigen::Map<const Coefficients>(coefficients), T(x_)));
        return true;
      }

  private:

    const Scalar x_;
    const Scalar y_;
    const Scalar weight_;

  };

/**
 * @brief The WeightedPolynomialFit class
 * @param PolynomialT_
 */
template <typename PolynomialT_>
  class WeightedPolynomialFit : public MathFunctionFit<typename MathTraits<PolynomialT_>::Scalar>
  {
  public:

    typedef boost::shared_ptr<WeightedPolynomialFit> Ptr;
    typedef boost::shared_ptr<const WeightedPolynomialFit> ConstPtr;

    typedef typename MathTraits<PolynomialT_>::Scalar Scalar;
    typedef typename boost::shared_ptr<PolynomialT_> PolynomialPtr;

    static const int Size = MathTraits<PolynomialT_>::Size;
    static const int MinDegree = MathTraits<PolynomialT_>::MinDegree;
    static const int Degree = MathTraits<PolynomialT_>::Degree;

    struct Data
    {
      Data(Scalar x, Scalar y, Scalar weight) : x_(x), y_(y), weight_(weight) {}
      Scalar x_;
      Scalar y_;
      Scalar weight_;
    };

    typedef std::vector<Data> DataBin;

    /**
     * @brief WeightedPolynomialFit
     * @param polynomial
     */
    WeightedPolynomialFit(const PolynomialPtr & polynomial)
      : polynomial_(polynomial)
    {
      // Do nothing
    }

    /**
     * @brief ~WeightedPolynomialFit
     */
    virtual ~WeightedPolynomialFit()
    {
      // Do nothing
    }

    /**
     * @brief addData
     * @param x
     * @param y
     */
    inline virtual void addData(const Scalar & x,
                                const Scalar & y,
                                const Scalar & weight)
    {
      data_bin_.push_back(Data(x, y, weight));
    }

    /**
     * @brief polynomial
     * @return
     */
    inline const PolynomialT_ & polynomial()
    {
      return *polynomial_;
    }

    /**
     * @brief update
     * @return
     */
    virtual bool update();

  private:

    DataBin data_bin_;
    PolynomialPtr polynomial_;

  };

} /* namespace calibration */

#include <impl/calibration_common/ceres/polynomial_fit.hpp>

#endif /* CALIBRATION_COMMON_ALGORITHMS_POLYNOMIAL_FIT_H_ */
