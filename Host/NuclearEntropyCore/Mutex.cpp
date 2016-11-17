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

#include "NuclearEntropyCore/Mutex.h"

#include <cassert>

#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/locks.hpp>

namespace NuclearEntropy
{

  namespace Detail
  {
    struct MutexImpl
    {
      boost::recursive_mutex m_Mutex;
    };

    struct LockImpl
    {
      LockImpl(boost::recursive_mutex& mutex) : m_Lock(mutex) {};      

      boost::lock_guard<boost::recursive_mutex> m_Lock;
    };

    Lock::Lock(const Mutex& mutex)
    : m_Implementation(new Detail::LockImpl(mutex.m_Implementation->m_Mutex))
    {
      assert(m_Implementation != 0);
    }

    Lock::~Lock()
    {
      assert(m_Implementation != 0);
      delete m_Implementation;
    }

  }  // namespace Detail

  Mutex::Mutex()
  : m_Implementation(new Detail::MutexImpl())
  {
  }

}  // namespace NuclearEntropy

