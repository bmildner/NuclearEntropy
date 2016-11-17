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

#include "NuclearEntropyCore/Port.h"

#include <string>
#include <cassert>
#include <iterator>

#include "NuclearEntropyCore/Utility.h"

using namespace std;

namespace NuclearEntropy
{
  const unsigned Port::scm_DefaultRetries = 10;
  const unsigned Port::scm_DefaultDelay = 50;

  void Port::HandleAndClearPortError() const
  {
    PortError errors = ClearPortError();

    // there may be multiple error reasons set
    // we will only report the first found!
    if ((errors & BreakSignal) != 0)
    {
      throw PortBreakSignal(*this);
    }

    if ((errors & OverflowError) != 0)
    {
      throw PortOverflowError(*this);
    }

    if ((errors & FrameError) != 0)
    {
      throw PortFrameError(*this);
    }

    if ((errors & ParityError) != 0)
    {
      throw PortParityError(*this);
    }
  }

  bool Port::HasSideChannel() const 
  {
    return false;
  }

  Byte Port::GetSideChannelMask() const
  {
    throw PortNotSupportedError(*this, "GetSideChannelMask()");
  }

  void Port::SetSideChannelConfig(Byte)
  {
    throw PortNotSupportedError(*this, "SetSideChannelConfig()");
  }

  Byte Port::GetSideChannelConfig() const 
  {
    throw PortNotSupportedError(*this, "GetSideChannelConfig()");
  }

  void Port::SetSideChannelData(Byte)
  {
    throw PortNotSupportedError(*this, "SetSideChannelData()");
  }

  Byte Port::GetSideChannelData() const 
  {
    throw PortNotSupportedError(*this, "GetSideChannelData()");
  }

  void Port::Send(Byte chr) const
  {
    Buffer buffer;

    buffer.push_back(chr);

    Send(buffer);
  }

  void Port::Send(const Buffer& data, unsigned retries, unsigned delay) const
  {
    Buffer::const_iterator iter = data.begin();
    Buffer::const_iterator endIter = data.end();

    unsigned retriesDone = 0;

    while (iter != endIter)
    {
      Buffer::const_iterator oldPos = iter;

      if (!Send(iter, endIter))
      {
        HandleAndClearPortError();
        throw PortWriteFailed(*this, GetLastOSErrorText());
      }

      if (distance(oldPos, iter) == 0)
      {
        retriesDone++;

        if (retriesDone == retries)
        {
          throw PortWriteTimeout(*this);
        }

        ThreadSleepMs(delay);
      }
      else
      {
        // reset retry counter when data was sent
        retriesDone = 0;
      }
    }

    assert(iter == data.end());
    assert(retries > 0);
  }

  Buffer Port::Receive(std::size_t length, unsigned retries, unsigned delay)
  {
    Buffer buffer;
    Buffer data;

    unsigned retriesDone = 0;

    while (length > 0)
    {
      if (!Receive(buffer, length))
      {
        HandleAndClearPortError();
        throw PortReadFailed(*this, GetLastOSErrorText());
      }

      if (buffer.empty())
      {        
        retriesDone++;

        if (retriesDone == retries)
        {
          throw PortReadTimeout(*this);
        }

        ThreadSleepMs(delay);
      }
      else
      {
        data.insert(data.end(), buffer.begin(), buffer.end());

        length -= buffer.size();

        retriesDone = 0;
      }
    }

    assert(length == 0);
    assert(retriesDone < retries);

    return data;
  }
}

