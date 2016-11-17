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

#include "NuclearEntropyCore/ComPort.h"

#ifdef NUCENT_SYSTEM_WINDOWS  // only compile on Windows

#include <cassert>
#include <iomanip>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/numeric/conversion/cast.hpp>

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include "NuclearEntropyCore/Utility.h"

using namespace std;

namespace
{
  using namespace NuclearEntropy;

  string PortNumberToName(ComPort::PortNumber port)
  {
    return (boost::format("COM%1%") % port).str();
  }
}  // namespace

namespace NuclearEntropy
{

  struct ComPort::SystemHandle : boost::noncopyable
  {
    public:
      typedef HANDLE HandleType;

      const HandleType Invalid_Handle;

      inline explicit SystemHandle(HandleType handle)
      : m_Handle(handle), Invalid_Handle(INVALID_HANDLE_VALUE)
      {
      }

      inline ~SystemHandle()
      {
        if (this->m_Handle != Invalid_Handle)
        {
          CloseHandle(this->m_Handle);
        }
      }

      inline HandleType& operator*()
      {
        assert(this->m_Handle != Invalid_Handle);
        return this->m_Handle;
      }

      inline bool IsValidHandle() const
      {
        return (this->m_Handle != Invalid_Handle);
      }

    private:
      HandleType m_Handle;
  };

  ComPort::PortNumberList ComPort::GetPortNumberList()
  {
    PortNumberList portList;

    for (PortNumber port = COM1; port < 255; port++)
    {      
      COMMCONFIG config;
      DWORD configLength = sizeof(COMMCONFIG);

      bool result = !!GetDefaultCommConfigA(PortNumberToName(port).c_str(), &config, &configLength);

      if (result) 
      {
        portList.push_back(port);
      }
    }

    return portList;
  }

  ComPort::PortNameList ComPort::GetPortNameList()
  {
    PortNameList portList;

    PortNumberList portNumbers = GetPortNumberList();

    BOOST_FOREACH(const PortNumberList::value_type& portNumber, portNumbers)
    {
      portList.push_back(PortNumberToName(portNumber));
    }

    return portList;
  }

  ComPort::Handle ComPort::CreateHandle(PortNumber portNumber)
  {
    return Handle(new ComPort(portNumber));
  }

  ComPort::Handle ComPort::CreateHandle(const string& portName)
  {
    if ((portName.size() < 4) || (portName.size() > 6) || (portName.find("COM") != 0)) 
    {
      throw PortNameInvalid(portName);
    }

    static const string numbers("0123456789");

    PortNumber portNumber = 0;

    for (string::const_iterator iter = portName.begin() + 3; iter != portName.end(); iter++)
    {
      portNumber *= 10;

      string::size_type pos = numbers.find(*iter);
      if (pos == string::npos)
      {
        throw PortNameInvalid(portName);
      }

      portNumber += boost::numeric_cast<DWORD>(pos);
    }

    return CreateHandle(portNumber);
  }

  ComPort::ComPort(PortNumber portNumber)
  : m_PortNumber(portNumber), m_SystemHandle(0), m_Mutex()
  {
    try
    {
      // open port
      this->m_SystemHandle = new SystemHandle(CreateFile((boost::wformat(L"\\\\.\\COM%1%") % this->m_PortNumber).str().c_str(),
                                              GENERIC_READ | GENERIC_WRITE,
                                              0,                             // must be opened with exclusive-access
                                              0,                             // default security attributes
                                              OPEN_EXISTING,                 // must use OPEN_EXISTING
                                              0,                             // no overlapped I/O
                                              0));                           // hTemplate must be NULL for serial devices

      if (!this->m_SystemHandle->IsValidHandle())
      {
        DWORD err = GetLastError();

        switch (err)
        {
          case ERROR_FILE_NOT_FOUND:
            throw PortNotFound(GetPortName());
            break;

          default:
            throw PortOpenFailed(*this, Detail::Win32ErrorToString(err));
        }
      }

      assert(this->m_SystemHandle != 0);
      assert(this->m_SystemHandle->IsValidHandle());

      // set base configure port
      DCB dcb;
      
      memset(&dcb, 0x00, sizeof(DCB));
      dcb.DCBlength = sizeof(DCB);
  
      if (!GetCommState(**this->m_SystemHandle, &dcb))
      {
        throw PortAPIError("GetCommState() failed " + Detail::Win32ErrorToString(GetLastError()), GetPortName());
      }

      dcb.fBinary           = true;
      dcb.fParity           = true;
      dcb.fDsrSensitivity   = false;
      dcb.fErrorChar        = false;
      dcb.fNull             = false;
      dcb.fAbortOnError     = true;
      dcb.fTXContinueOnXoff = true;

      if (!SetCommState(**this->m_SystemHandle, &dcb))
      {
        throw PortConfigFailed(*this, "SetCommState() failed " + Detail::Win32ErrorToString(GetLastError()));
      }
    }

    catch (...)
    {
      delete m_SystemHandle;

      throw;
    }
  }

  ComPort::~ComPort()
  {
    Mutex::LockType lock(m_Mutex);

    assert (this->m_SystemHandle != 0);

    delete this->m_SystemHandle;
  }

  void ComPort::SetDataConfig(unsigned baud, DataBits dataBits, Parity parity, StopBits stopBits)
  {
    Mutex::LockType lock(m_Mutex);

    DCB dcb;
      
    memset(&dcb, 0x00, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);
  
    if (!GetCommState(**this->m_SystemHandle, &dcb))
    {
      throw PortAPIError("GetCommState() failed " + Detail::Win32ErrorToString(GetLastError()), GetPortName());
    }

    dcb.BaudRate = baud;
    dcb.ByteSize = static_cast<BYTE>(dataBits);

    switch (parity)
    {
      case Parity_None:
        dcb.Parity = NOPARITY;
        break;

      case Parity_Odd:
        dcb.Parity = ODDPARITY;
        break;

      case Parity_Even:
        dcb.Parity = EVENPARITY;
        break;

      case Parity_Mark:
        dcb.Parity = MARKPARITY;
        break;

      case Parity_Space:
        dcb.Parity = SPACEPARITY;
        break;

      default:
        throw PortInvalidParam((boost::format("Invalid parity mode (%1%)") % parity).str());
    }

    switch (stopBits)
    {
      case StopBits_1:
        dcb.StopBits = ONESTOPBIT;
        break;

      case StopBits_2:
        dcb.StopBits = TWOSTOPBITS;
        break;

      default:
        throw PortInvalidParam((boost::format("Invalid number of stop bits (%1%)") % stopBits).str());
    }

    if (!SetCommState(**this->m_SystemHandle, &dcb))
    {
      throw PortConfigFailed(*this, "SetCommState() failed " + Detail::Win32ErrorToString(GetLastError()));
    }
  }

  void ComPort::SetFlowControl(FlowControl flowControl, Byte xOn, Byte xOff)
  {
    Mutex::LockType lock(m_Mutex);

    if ((flowControl & ~(FlowControl_RTS_CTS | FlowControl_DTS_DSR | FlowControl_XOn_XOff)) != 0)
    {
      throw PortInvalidParam((boost::format("Invalid flow control (%1%)") % flowControl).str());
    }

    DCB dcb;
      
    memset(&dcb, 0x00, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);
  
    if (!GetCommState(**this->m_SystemHandle, &dcb))
    {
      throw PortAPIError("GetCommState() failed " + Detail::Win32ErrorToString(GetLastError()), GetPortName());
    }

    dcb.fOutxCtsFlow = false;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fOutxDsrFlow = false;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fOutX = false;
    dcb.fInX = false; 
    dcb.XonChar = xOn;
    dcb.XoffChar = xOff;

    if ((flowControl & FlowControl_RTS_CTS) != 0)
    {
      dcb.fOutxCtsFlow = true;
      dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
    }

    if ((flowControl & FlowControl_DTS_DSR) != 0)
    {
      dcb.fOutxDsrFlow = true;
      dcb.fDtrControl = DTR_CONTROL_HANDSHAKE;
    }

    if ((flowControl & FlowControl_XOn_XOff) != 0)
    {
      dcb.fOutX = true;
      dcb.fInX = true;
    }

    if (!SetCommState(**this->m_SystemHandle, &dcb))
    {
      throw PortConfigFailed(*this, "SetCommState() failed " + Detail::Win32ErrorToString(GetLastError()));
    }
  }

  void ComPort::GetTimeout(unsigned& readTimeout, unsigned& writeTimeout) const
  {
    Mutex::LockType lock(m_Mutex);

    // get timeout values
    COMMTIMEOUTS timeouts;

    if (!GetCommTimeouts(**this->m_SystemHandle, &timeouts))
    {
      throw PortConfigFailed(*this, "GetCommTimeouts() failed " + Detail::Win32ErrorToString(GetLastError()));
    }

    // (over?) simplified query of timeout values
    readTimeout  = timeouts.ReadIntervalTimeout;
    writeTimeout = timeouts.WriteTotalTimeoutMultiplier;
  }

  void ComPort::SetTimeout(unsigned readTimeout, unsigned writeTimeout)
  {
    Mutex::LockType lock(m_Mutex);

    // setup timeout values
    COMMTIMEOUTS timeouts;

    // set default timeout (total duration and between bytes)
    timeouts.ReadIntervalTimeout         = (readTimeout == 0 ? MAXDWORD : readTimeout);  // set to MAXDWORD for immediate return
    timeouts.ReadTotalTimeoutMultiplier  = readTimeout;
    timeouts.ReadTotalTimeoutConstant    = 0;
    timeouts.WriteTotalTimeoutMultiplier = writeTimeout;
    timeouts.WriteTotalTimeoutConstant   = 0;

    if (!SetCommTimeouts(**this->m_SystemHandle, &timeouts))
    {
      throw PortConfigFailed(*this, "SetCommTimeouts() failed " + Detail::Win32ErrorToString(GetLastError()));
    }
  }

  void ComPort::Purge(PortBuffer buffer) const
  {
    Mutex::LockType lock(m_Mutex);

    DWORD flags = 0;

    switch (buffer)
    {
      case ReceiveBuffer:
        flags = PURGE_RXABORT | PURGE_RXCLEAR;
        break;

      case TransmitBuffer:
        flags = PURGE_TXABORT | PURGE_TXCLEAR;
        break;

      case AllBuffers:
        flags = PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR;
        break;

      default:
        throw PortInvalidParam((boost::format("Calles Purge() with invalid buffers parameter (%1%)") % buffer).str());
    }

    if (!PurgeComm(**this->m_SystemHandle, flags))
    {
      throw PortAPIError("PurgeComm() failed " + Detail::Win32ErrorToString(GetLastError()), GetPortName());
    }
  }

  bool ComPort::Send(Buffer::const_iterator& first, Buffer::const_iterator last) const
  {
    Mutex::LockType lock(m_Mutex);

    DWORD sent = 0;

    bool result = !!WriteFile(**this->m_SystemHandle, &(*first), boost::numeric_cast<DWORD>(last - first), &sent, 0);

    assert(sent <= static_cast<DWORD>(last - first));

    first += sent;

    return result;
  }

  Byte ComPort::Receive() const
  {
    Mutex::LockType lock(m_Mutex);

    Byte chr;
    DWORD bytesRead = 0;

    if (!ReadFile(**this->m_SystemHandle, &chr, sizeof(chr), &bytesRead, 0))
    {
      HandleAndClearPortError();
      throw PortReadFailed(*this, Detail::Win32ErrorToString(GetLastError()));
    }

    if (bytesRead < sizeof(chr))
    {
      throw PortReadTimeout(*this);
    }

    return chr;
  }

  bool ComPort::Receive(Buffer& data, size_t length) const
  {
    Mutex::LockType lock(m_Mutex);

    if (length < 1)
    {
      data.clear();
      return true;
    }

    data.resize(length);

    DWORD bytesRead = 0;

    bool result = !!ReadFile(**this->m_SystemHandle, &data[0], boost::numeric_cast<DWORD>(length), &bytesRead, 0);

    assert(bytesRead <= length);

    data.resize(bytesRead);

    return result;
  }

  Port::PortError ComPort::ClearPortError() const
  {
    Mutex::LockType lock(m_Mutex);

    DWORD err;

    if (ClearCommError(**m_SystemHandle, &err, 0))
    {
      PortError errors = NoError;

      if ((err & CE_OVERRUN) != 0)
      {
        errors = static_cast<PortError>(errors | OverflowError);
      }

      if ((err & CE_RXOVER) != 0)
      {
        errors = static_cast<PortError>(errors | OverflowError);
      }

      if ((err & CE_FRAME) != 0)
      {
        errors = static_cast<PortError>(errors | FrameError);
      }

      if ((err & CE_RXPARITY) != 0)
      {
        errors = static_cast<PortError>(errors | ParityError);
      }

      if ((err & CE_BREAK) != 0)
      {
        errors = static_cast<PortError>(errors | BreakSignal);
      }

      return errors;
    }

    throw PortAPIError("ClearCommError() failed " + Detail::Win32ErrorToString(GetLastError()), GetPortName());
  }

  string ComPort::GetPortName() const
  {
    return (boost::format("COM%1%") % this->m_PortNumber).str();
  }

  void ComPort::SendBreakSignal(unsigned ms) const
  {
    Mutex::LockType lock(m_Mutex);

    if (!SetCommBreak(**m_SystemHandle))
    {
      throw PortAPIError("SetCommBreak() failed "  + Detail::Win32ErrorToString(GetLastError()), GetPortName());
    }

    ThreadSleepMs(ms);

    if (!ClearCommBreak(**m_SystemHandle))
    {
      throw PortAPIError("ClearCommBreak() failed "  + Detail::Win32ErrorToString(GetLastError()), GetPortName());
    }
  }

  string ComPort::GetLastOSErrorText() const
  {
    return Detail::Win32ErrorToString(GetLastError());
  }

}  // namespace NuclearEntropy

#endif  // NUCENT_SYSTEM_WINDOWS

