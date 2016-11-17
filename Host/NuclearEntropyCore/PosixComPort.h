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

#ifndef NUCLEARENTROPY_POSIXCOMPORT_H
#define NUCLEARENTROPY_POSIXCOMPORT_H

#include "NuclearEntropyCore/Config.h"

#ifdef NUCENT_SYSTEM_POSIX  // only compile on POSIX

#include <map>
#include <string>
#include <list>

#include "NuclearEntropyCore/Port.h"

namespace NuclearEntropy
{

  class NUCENT_API PosixComPort: public Port
  {
    public:
      typedef HandleType<PosixComPort>::Type Handle;

      typedef std::string PortName;

      typedef std::list<PortName> PortNameList;

      virtual ~PosixComPort();

      static Handle CreateHandle(const PortName& portName);

      static PortNameList GetPortNameList();

      virtual void SetDataConfig(unsigned baud = BaudRate_9600, DataBits dataBits = DataBits_8, Parity parity = Parity_None, StopBits stopBits = StopBits_1);
      virtual void SetFlowControl(FlowControl flowControl = FlowControl_None, Byte xOn = Default_XOn, Byte xOff = Default_XOff);

      virtual void GetTimeout(unsigned& readTimeout, unsigned& writeTimeout) const;
      virtual void SetTimeout(unsigned readTimeout = DefaultReadTimeout, unsigned writeTimeout = DefaultWriteTimeout);

      virtual void Purge(PortBuffer buffer = AllBuffers) const;

      virtual bool Send(Buffer::const_iterator& first, Buffer::const_iterator last) const;

      virtual Byte Receive() const;
      virtual bool Receive(Buffer& data, std::size_t length) const;

      virtual void SendBreakSignal(unsigned ms) const;

      virtual PortName GetPortName() const;

      virtual PortError ClearPortError() const;

    protected:
      struct SystemHandle;

      SystemHandle* m_SystemHandle;

      PortName m_PortName;

      mutable Mutex m_Mutex;

      explicit PosixComPort(const PortName& portName);

      virtual std::string GetLastOSErrorText() const;

    private:
  };

}  // namespace NuclearEntropy

#endif // NUCENT_SYSTEM_POSIX

#endif

