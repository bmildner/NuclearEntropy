// Copyright (c) 2012 by Bertolt Mildner
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

#include "NuclearEntropyCore/Utility.h"

#include <locale>
#include <iomanip>

ATTD_BOOST_INCL_GUARD_BEGIN
#include <boost/format.hpp>
#if !defined(ATTD_SYSTEM_WINDOWS) && !defined(ATTD_SYSTEM_POSIX)
# include <boost/thread/thread.hpp>
#endif
ATTD_BOOST_INCL_GUARD_END

#ifdef ATTD_SYSTEM_WINDOWS
# define WIN32_LEAN_AND_MEAN
# include <Windows.h>
#endif

#ifdef ATTD_SYSTEM_POSIX
extern "C"
{
# include <time.h>
}
#endif

using namespace std;

namespace AutomatedTokenTestDevice
{
  namespace Detail
  {
#ifdef ATTD_SYSTEM_WINDOWS
    string Win32ErrorToString(uint32_t error)
    {
      string reason;
      LPSTR buffer = 0;

      if (FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER, 0, error, 0, (LPSTR) &buffer, 0, 0) > 0)
      {
        try
        {
          reason = buffer;
          LocalFree(buffer);
        }
        catch (...)
        {
          LocalFree(buffer);
          throw;
        }
      }
      else
      {
        reason = "<Unknown>";
      }

      return (boost::format("Win32 API error: %1% (%2%)") % reason % boost::io::group(hex, showbase, setfill('0'), setw(8), error)).str();
    }
#endif
  }  // namespace Detail

  string WideToNarrowString(const wstring& str)
  {
    locale loc;
    string narrowStr;

    narrowStr.resize(str.size());

    use_facet<std::ctype<wchar_t> >(loc).narrow(&str[0], (&str[0]) + str.size(), '?', &narrowStr[0]);

    return narrowStr;
  }

  wstring NarrowToWideString(const string& str)
  {
    locale loc;
    wstring wideStr;

    wideStr.resize(str.size());

    use_facet<std::ctype<wchar_t> >(loc).widen(&str[0], (&str[0]) + str.size(), &wideStr[0]);

    return wideStr;
  }

  void ThreadSleepMs(uint32_t ms)
  {
#if defined(ATTD_SYSTEM_WINDOWS)
    ::Sleep(ms);
#elif defined(ATTD_SYSTEM_POSIX)
    // we do not want to depend on built boost libraryies on Posix
    timespec delay;

    delay.tv_sec =  ms / 1000;
    delay.tv_nsec = (ms % 1000) * 1000000;

    nanosleep(&delay, 0);
#else
    boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
#endif
  }

}  // namespace AutomatedTokenTestDevice

