// Copyright (c) 2011 - 2016 by Bertolt Mildner
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

#ifndef NUCLEARENTROPY_MUTEX_H
#define NUCLEARENTROPY_MUTEX_H

#include "NuclearEntropyCore/Config.h"

#include <boost/utility.hpp>

#include "NuclearEntropyCore/Types.h"

namespace NuclearEntropy
{

  class Mutex;

  namespace Detail
  {
    struct MutexImpl;
    struct LockImpl;

    class NUCENT_API Lock : boost::noncopyable
    {
      public:
        Lock(const Mutex& mutex);

        ~Lock();

      protected:
         Detail::LockImpl* m_Implementation;

      private:
    };

  }

  class NUCENT_API Mutex
  {
    public:
      typedef Detail::Lock LockType;

      Mutex();

    protected:
      friend class Detail::Lock;

      typedef HandleType<Detail::MutexImpl>::Type Implementation;

      NUCENT_SUPPRESS_EXPORT_WARNING
      Implementation m_Implementation;

    private:
  };

}  // namespace NuclearEntropy

#endif

