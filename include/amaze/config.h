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

#ifndef AMAZE_CONFIG_H_
#define AMAZE_CONFIG_H_

/* clang-format off */

#ifdef __has_cpp_attribute
#if __has_cpp_attribute(unlikely)
#define UNLIKELY [[unlikely]]
#endif
#endif

#ifndef UNLIKELY
#define UNLIKELY
#endif

/* #undef AMAZE_NO_STDIO */
/* #undef AMAZE_DEBUG */

namespace amaze {
namespace config {
  static constexpr int kVersionMajor = 0;
  static constexpr int kVersionMinor = 0;
  static constexpr int kVersionPatch = 0;
}  // namespace config
}  // namespace amaze

/* clang-format on */

#endif  // AMAZE_CONFIG_H_
