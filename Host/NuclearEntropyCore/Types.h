// Copyright (c) 2012 - 2013 by Bertolt Mildner
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the copyright holder nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL BERTOLT MILDNER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef NUCLEARENTROPY_TYPE_H
#define NUCLEARENTROPY_TYPE_H

#include "NuclearEntropyCore/Config.h"

#include <vector>

// old GCC std. lib. does not have <cstdint>
extern "C"
{
# include <stdint.h>
}

#if (__cplusplus <= 199711L) && !(defined(_MSC_VER) && (_MSC_VER >= 1600))  // MSVC 2010 has std::shared_ptr<> but signals old C++ standard
# define NUCENT_USE_BOOST_SHARED_PTR
#endif

#ifdef NUCENT_USE_BOOST_SHARED_PTR
# include <boost/shared_ptr.hpp>
#else
# include <memory>
#endif

namespace NuclearEntropy
{
  typedef unsigned char      Byte;
  typedef std::vector<Byte>  Buffer;

  template <typename T>
  struct HandleType
  {
#ifdef NUCENT_USE_BOOST_SHARED_PTR
    typedef boost::shared_ptr<T> Type;
#else
    typedef std::shared_ptr<T> Type;
#endif
  };
}  // namespace NuclearEntropy

#endif

