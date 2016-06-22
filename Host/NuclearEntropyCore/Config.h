// Copyright (c) 2011 - 2013 by Bertolt Mildner
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

#ifndef NUCLEARENTROPY_CONFIG_H
#define NUCLEARENTROPY_CONFIG_H

// detect system
# ifdef __cplusplus
extern "C"
{
# endif

# if defined(__FreeBSD__) || ((defined(__APPLE__) && defined(__MACH__)))
#  include <unistd.h>
# endif

# if defined(__linux__)
#  include <features.h>
# endif

# ifdef __cplusplus
}
# endif

# if defined(_WIN32)
#  define NUCENT_SYSTEM_WINDOWS
# elif defined(_POSIX_VERSION) || defined(_POSIX_SOURCE) || defined(_POSIX_C_SOURCE)
#  define NUCENT_SYSTEM_POSIX
# else
#  error "Unknown System"
# endif


// detect compiler and set configuration
# ifdef _MSC_VER
#  if defined NUCENT_BUILD_DLL
#    define NUCENT_API __declspec(dllexport)
#  elif !defined NUCENT_STATIC_LIB
#   define NUCENT_API __declspec(dllimport)
#  else
#   define NUCENT_API
#  endif

#  define NUCENT_UNUSED(x) /*__pragma(warning(suppress:4100))*/ x

#  ifndef _CRT_SECURE_NO_WARNINGS
#   define _CRT_SECURE_NO_WARNINGS
#  endif

#  ifndef _SCL_SECURE_NO_WARNINGS
#   define _SCL_SECURE_NO_WARNINGS
#  endif

#  define NUCENT_SUPPRESS_EXPORT_WARNING __pragma(warning(suppress:4251 4275))

# elif defined(__GNUC__)

#  define NUCENT_UNUSED(x) (void) x
#  define NUCENT_API  // TODO: set properly for GCC

#  define NUCENT_SUPPRESS_EXPORT_WARNING

# elif defined(__clang__)

#  define NUCENT_UNUSED(x) (void) x
#  define NUCENT_API  // TODO: set properly for Clang / LLVM

#  define NUCENT_SUPPRESS_EXPORT_WARNING

# else
#  error "Unknown compiler"
# endif

#endif
