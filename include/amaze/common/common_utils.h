/**
 * Copyright (c) 2020 Tokoro
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef INCLUDE_AMAZE_COMMON_COMMON_UTILS_H_
#define INCLUDE_AMAZE_COMMON_COMMON_UTILS_H_

#include <cstdint>
#include <limits>
#include <ostream>
#include <type_traits>
#include <utility>

#include "amaze/common/common_types.h"

namespace amaze {

/// \~japanese
/// 型\p Tの最大値で飽和する和を返します．
/// \tparam T 数値型
/// \param a, b 和を取る数
/// \returns \p a + \p b もしくは \p std::numeric_limits<T>::max()
///
/// \~english
/// Returns sum saturated at the maximum value of the type \p T.
/// \tparam T Numeric type
/// \param a, b Numbers to be added
/// \returns \p a + \p b or \p std::numeric_limits<T>::max()
template <typename T>
std::enable_if_t<std::is_unsigned_v<T>, T> satSum(T a, T b) {
  if (std::numeric_limits<T>::max() - a <= b) {
    return std::numeric_limits<T>::max();
  } else {
    return T(a + b);
  }
}

/* */
inline std::ostream &operator<<(std::ostream &os, Direction d) {
  if (d.bits.north) {
    os << 'N';
  }
  if (d.bits.south) {
    os << 'S';
  }
  if (d.bits.east) {
    os << 'E';
  }
  if (d.bits.west) {
    os << 'W';
  }
  if (d.half == 0) {
    os << '0';
  }
  return os;
}
inline std::ostream &operator<<(std::ostream &os, Position p) {
  os << "(" << int(p.x) << ", " << int(p.y) << ")";
  return os;
}
inline std::ostream &operator<<(std::ostream &os, Difference d) {
  os << "(" << int(d.x) << ", " << int(d.y) << ")";
  return os;
}
inline std::ostream &operator<<(std::ostream &os, AgentState as) {
  os << "(" << as.pos << ", " << as.dir << ", " << int(as.attribute) << ")";
  return os;
}

}  // namespace amaze
#endif  // INCLUDE_AMAZE_COMMON_COMMON_UTILS_H_
