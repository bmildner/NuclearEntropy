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

#ifndef AUTOMATEDTOKENTESTDEVICE_CONFIG_H
#define AUTOMATEDTOKENTESTDEVICE_CONFIG_H

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
#  define ATTD_SYSTEM_WINDOWS
# elif defined(_POSIX_VERSION) || defined(_POSIX_SOURCE) || defined(_POSIX_C_SOURCE)
#  define ATTD_SYSTEM_POSIX
# else
#  error "Unknown System"
# endif


// detect compiler and set configuration
# ifdef _MSC_VER
#  if defined ATTD_BUILD_DLL
#    define ATTD_API __declspec(dllexport)
#  elif !defined ATTD_STATIC_LIB
#   define ATTD_API __declspec(dllimport)
#  else
#   define ATTD_API
#  endif

#  define ATTD_UNUSED(x) /*__pragma(warning(suppress:4100))*/ x

#  ifndef _CRT_SECURE_NO_WARNINGS
#   define _CRT_SECURE_NO_WARNINGS
#  endif

#  ifndef _SCL_SECURE_NO_WARNINGS
#   define _SCL_SECURE_NO_WARNINGS
#  endif

# elif defined(__GNUC__)

#  define ATTD_UNUSED(x) (void) x
#  define ATTD_API  // TODO: set properly for GCC

# elif defined(__clang__)

#  define ATTD_UNUSED(x) (void) x
#  define ATTD_API  // TODO: set properly for Clang / LLVM

# else
#  error "Unknown compiler"
# endif


// "inclusion guard" macros for boost headers, dreaded MSVC 2010 code analysis causes lots of warnings in boost headers ...
// Warning: MSVC 2010 code analysis causes warnings even in std. library headers for some stuff .... not really useful!
# ifdef _MSC_VER
#  include <codeanalysis\warnings.h>

#  define ATTD_BOOST_INCL_GUARD_BEGIN  __pragma(warning(push))                                 \
                                       __pragma(warning(disable: ALL_CODE_ANALYSIS_WARNINGS))  \
                                       __pragma(warning(disable: 4512))  /* warning C4512: 'boost::io::detail::group5<T1,T2,T3,T4,T5>' : assignment operator could not be generated */ 

#  define ATTD_BOOST_INCL_GUARD_END    __pragma(warning(pop))

#  define ATTD_BOOST_NO_FOREACH_WARNINGS_BEGIN __pragma(warning(push))  \
                                               __pragma(warning(disable: 6246))  /* warning C6246: Local declaration of '_foreach_col' hides declaration of the same name in outer scope. For additional information, see previous declaration at line '521' of 'd:\exchange\test\automatedtokentestdevice\automatedtokentestdevice\automatedtokentestdevice.cpp' */

#  define ATTD_BOOST_NO_FOREACH_WARNINGS_END   __pragma(warning(pop))

# else
#  define ATTD_BOOST_INCL_GUARD_BEGIN
#  define ATTD_BOOST_INCL_GUARD_END
#  define ATTD_BOOST_NO_FOREACH_WARNINGS_BEGIN
#  define ATTD_BOOST_NO_FOREACH_WARNINGS_END
# endif

#endif

