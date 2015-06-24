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

#ifndef UNIPD_CALIBRATION_CALIBRATION_COMMON_BASE_DEFINITIONS_H_
#define UNIPD_CALIBRATION_CALIBRATION_COMMON_BASE_DEFINITIONS_H_

#include <memory>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace unipd
{
namespace calib
{

using Scalar = double;

template <typename T_>
  struct Dimension {};

using Size1 = size_t;

struct Size2
{
  Size1 x;
  Size1 y;
};

template <>
  struct Dimension<Size1>
  {
    static const int value = 1;
  };

template <>
  struct Dimension<Size2>
  {
    static const int value = 2;
  };

inline bool
operator == (const Size2 & lhs, const Size2 & rhs)
{
  return (lhs.x == rhs.x) and (lhs.y == rhs.y);
}

inline Size1
reduce (const Size2 & size)
{
  return size.x * size.y;
}

using Index1 = int;
struct Index2
{
  Index1 x;
  Index1 y;
};

inline bool
operator == (const Index2 & lhs, const Index2 & rhs)
{
  return (lhs.x == rhs.x) and (lhs.y == rhs.y);
}

using Indices1 = std::vector<Index1>;
using Indices2 = std::vector<Index2>;

template <>
  struct Dimension<Indices1>
  {
    static const int value = 1;
  };
template <>
  struct Dimension<Indices2>
  {
    static const int value = 2;
  };

template<typename T>
  boost::shared_ptr<T> std2boost (std::shared_ptr<T> & ptr)
  {
    return boost::shared_ptr<T>(ptr.get(), [ptr](T*) mutable {ptr.reset();});
  }

template<typename T>
  std::shared_ptr<T> boost2std (boost::shared_ptr<T> & ptr)
  {
    return std::shared_ptr<T>(ptr.get(), [ptr](T*) mutable {ptr.reset();});
  }

} // namespace calib
} // namespace unipd

#endif /* UNIPD_CALIBRATION_CALIBRATION_COMMON_BASE_DEFINITIONS_H_ */
