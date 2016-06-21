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

#include "NuclearEntropyCore/PosixComPort.h"

#ifdef ATTD_SYSTEM_POSIX  // only compile on POSIX systems

#include <cassert>
#include <cstring>

#include <sys/types.h>
#include <sys/stat.h>

#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

ATTD_BOOST_INCL_GUARD_BEGIN
#include <boost/format.hpp>
ATTD_BOOST_INCL_GUARD_END


using namespace std;

namespace
{
  string PosixErrorToString(int error)
  {
    return string(string(strerror(error)) + (boost::format(" (%1%)") % error).str());
  }
}

namespace AutomatedTokenTestDevice
{

  struct PosixComPort::SystemHandle
  {
    public:
      typedef int HandleType;

      static const HandleType Invalid_Handle = -1;

      inline explicit SystemHandle(HandleType handle)
      : m_Handle(handle)
      {
      }

      inline ~SystemHandle()
      {
        if (this->m_Handle != Invalid_Handle)
        {
          close(this->m_Handle);
        }
      }

      inline HandleType operator*()
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

  PosixComPort::PortNameList PosixComPort::GetPortNameList()
  {
    PortNameList portList;

    // probably too system specific!?!

    return portList;
  }

  PosixComPort::Handle PosixComPort::CreateHandle(const string& portName)
  {
    return Handle(new PosixComPort(portName));
  }

  PosixComPort::PosixComPort(const PortName& portName)
  : m_SystemHandle(0), m_PortName(portName), m_Mutex()
  {
    try
    {
      // open port
      this->m_SystemHandle = new SystemHandle(open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY));

      if (!this->m_SystemHandle->IsValidHandle())
      {
        int err = errno;

        switch (err)
        {
          case ENOENT:
            throw PortNotFound(GetPortName());
            break;

          default:
            throw PortOpenFailed(*this, PosixErrorToString(err));
        }
      }

      assert(this->m_SystemHandle != 0);
      assert(this->m_SystemHandle->IsValidHandle());

      // set basic port configuration
      termios options;

      if (tcgetattr(**this->m_SystemHandle, &options) == -1)
      {
        throw PortAPIError("tcgetattr() failed " + PosixErrorToString(errno), GetPortName());
      }

      options.c_iflag = (IGNPAR |  // ignore framing errors and parity errors
                         IGNBRK    // ignore break signals (= do not insert \0 into data stream)
                         );

      options.c_cflag = (CS8   |   // 8bit characters
                         CREAD |   // enable receiver
                         CLOCAL    // ignore modem control lines
                         );

      options.c_lflag = (NOFLSH);  // do not flush buffers on INTR, QUIT, and SUSP characters

      options.c_oflag = 0;  // reset output mode

      memset(options.c_cc, 0x00, NCCS);  // make sure all control characters are 0

      // flush buffers
      if (tcflush(**this->m_SystemHandle, TCIOFLUSH) == -1)
      {
        throw PortAPIError("tcflush() failed " + PosixErrorToString(errno), GetPortName());
      }

      if (tcsetattr(**this->m_SystemHandle, TCSANOW, &options) == -1)
      {
        throw PortAPIError("tcsetattr() failed " + PosixErrorToString(errno), GetPortName());
      }

      // disable NDELAY so we can use the read timeout
      if (fcntl(**this->m_SystemHandle, F_SETFL, O_NDELAY) == -1)
      {
        throw PortAPIError("fcntl() failed " + PosixErrorToString(errno), GetPortName());
      }
    }

    catch (...)
    {
      delete m_SystemHandle;

      throw;
    }
  }

  PosixComPort::~PosixComPort()
  {
    Mutex::LockType lock(m_Mutex);

    assert (this->m_SystemHandle != 0);

    delete this->m_SystemHandle;
  }

  void PosixComPort::SetDataConfig(unsigned baud, DataBits dataBits, Parity parity, StopBits stopBits)
  {
    Mutex::LockType lock(m_Mutex);

    termios options;

    if (tcgetattr(**this->m_SystemHandle, &options) == -1)
    {
      throw PortAPIError("tcgetattr() failed " + PosixErrorToString(errno), GetPortName());
    }

#define BAUD_CASE(x) case x:  \
                       cfsetispeed(&options, B##x);  \
                       break

    switch (baud)
    {
      BAUD_CASE(50);
      BAUD_CASE(75);
      BAUD_CASE(110);
      BAUD_CASE(134);
      BAUD_CASE(150);
      BAUD_CASE(200);
      BAUD_CASE(300);
      BAUD_CASE(600);
      BAUD_CASE(1200);
      BAUD_CASE(1800);
      BAUD_CASE(2400);
      BAUD_CASE(4800);
      BAUD_CASE(9600);
      BAUD_CASE(19200);
      BAUD_CASE(38400);
      BAUD_CASE(57600);
      BAUD_CASE(115200);
#ifdef B230400
      BAUD_CASE(230400);
#endif
#ifdef B460800
      BAUD_CASE(460800);
#endif
#ifdef B500000
      BAUD_CASE(500000);
#endif
#ifdef B576000
      BAUD_CASE(576000);
#endif
#ifdef B921600
      BAUD_CASE(921600);
#endif
#ifdef B1000000
      BAUD_CASE(1000000);
#endif
#ifdef B1152000
      BAUD_CASE(1152000);
#endif
#ifdef B1500000
      BAUD_CASE(1500000);
#endif
#ifdef B2000000
      BAUD_CASE(2000000);
#endif
#ifdef B2500000
      BAUD_CASE(2500000);
#endif
#ifdef B3000000
      BAUD_CASE(3000000);
#endif
#ifdef B3500000
      BAUD_CASE(3500000);
#endif
#ifdef B4000000
      BAUD_CASE(4000000);
#endif

      default:
        throw PortInvalidParam((boost::format("Invalid baud rate (%1%)") % baud).str());
    }

#undef BAUD_CASE

    options.c_cflag &= ~CSIZE;

    switch (dataBits)
    {
      case 5:
        options.c_cflag |= CS5;
        break;

      case 6:
        options.c_cflag |= CS6;
        break;

      case 7:
        options.c_cflag |= CS7;
        break;

      case 8:
        options.c_cflag |= CS8;
        break;

      default:
        throw PortInvalidParam((boost::format("Invalid number of dataBits (%1%)") % dataBits).str());
    }

    switch (parity)
    {
      case Parity_None:
        options.c_iflag &= ~INPCK;
        options.c_cflag &= ~PARENB;
        break;

      case Parity_Odd:
        options.c_iflag |= INPCK;
        options.c_cflag |= PARENB;
        options.c_cflag |= PARODD;
        break;

      case Parity_Even:
        options.c_iflag |= INPCK;
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        break;

      case Parity_Mark:
      case Parity_Space:

      default:
        throw PortInvalidParam((boost::format("Invalid parity mode (%1%)") % parity).str());
    }

    switch (stopBits)
    {
      case StopBits_1:
        options.c_cflag &= ~CSTOPB;
        break;

      case StopBits_2:
        options.c_cflag |= CSTOPB;
        break;

      default:
        throw PortInvalidParam((boost::format("Invalid number of stop bits (%1%)") % stopBits).str());
    }

    if (tcsetattr(**this->m_SystemHandle, TCSADRAIN, &options) == -1)
    {
      throw PortAPIError("tcsetattr() failed " + PosixErrorToString(errno), GetPortName());
    }
  }

  void PosixComPort::SetFlowControl(FlowControl flowControl, Byte xOn, Byte xOff)
  {
    Mutex::LockType lock(m_Mutex);

    if ((flowControl & ~(FlowControl_RTS_CTS | FlowControl_DTS_DSR | FlowControl_XOn_XOff)) != 0)
    {
      throw PortInvalidParam((boost::format("Invalid flow control (%1%)") % flowControl).str());
    }

    termios options;

    if (tcgetattr(**this->m_SystemHandle, &options) == -1)
    {
      throw PortAPIError("tcgetattr() failed " + PosixErrorToString(errno), GetPortName());
    }

#if defined(CRTSCTS)
# define ATTD_RTSCTS CRTSCTS
#elif defined(CNEW_RTSCTS)
# define ATTD_RTSCTS CNEW_RTSCTS
#endif

#ifdef ATTD_RTSCTS
    options.c_cflag &= ~ATTD_RTSCTS;  // disable RTS/CTS flow control
#endif

    // disable XON/XOFF flow control
    options.c_iflag &= ~IXON;
    options.c_iflag &= ~IXOFF;

    // reset XON/XOFF characters
    options.c_cc[VSTART] = 0;
    options.c_cc[VSTOP] = 0;


    // enable XON/XOFF flow control
    if ((flowControl & FlowControl_XOn_XOff) != 0)
    {
      options.c_iflag |= IXON | IXOFF;
      options.c_cc[VSTART] = xOn;
      options.c_cc[VSTOP] = xOff;
    }

    // enable RTS/CTS flow control
#ifdef ATTD_RTSCTS
    if ((flowControl & FlowControl_RTS_CTS) != 0)
    {
      options.c_cflag |= ATTD_RTSCTS;
    }
#else
    if ((flowControl & FlowControl_RTS_CTS) != 0)
    {
      throw PortInvalidParam((boost::format("DTS/DSR flow control not supported (%1%)") % flowControl).str());
    }
#endif

    if ((flowControl & FlowControl_DTS_DSR) != 0)
    {
      throw PortInvalidParam((boost::format("DTS/DSR flow control not supported (%1%)") % flowControl).str());
    }

    if (tcsetattr(**this->m_SystemHandle, TCSADRAIN, &options) == -1)
    {
      throw PortAPIError("tcsetattr() failed " + PosixErrorToString(errno), GetPortName());
    }
  }

#undef ATTD_RTSCTS

  void PosixComPort::GetTimeout(unsigned& readTimeout, unsigned& writeTimeout) const
  {
    Mutex::LockType lock(m_Mutex);

    termios options;

    if (tcgetattr(**this->m_SystemHandle, &options) == -1)
    {
      throw PortAPIError("tcgetattr() failed " + PosixErrorToString(errno), GetPortName());
    }

    readTimeout = options.c_cc[VTIME] * 100;

    writeTimeout = 0;  // write timeout is unused

    if (tcsetattr(**this->m_SystemHandle, TCSADRAIN, &options) == -1)
    {
      throw PortAPIError("tcsetattr() failed " + PosixErrorToString(errno), GetPortName());
    }
  }

  void PosixComPort::SetTimeout(unsigned readTimeout, unsigned writeTimeout)
  {
    Mutex::LockType lock(m_Mutex);

    termios options;

    if (tcgetattr(**this->m_SystemHandle, &options) == -1)
    {
      throw PortAPIError("tcgetattr() failed " + PosixErrorToString(errno), GetPortName());
    }

    options.c_cc[VTIME] = readTimeout / 100;
    ATTD_UNUSED(writeTimeout);  // write timeout is unused

    if (tcsetattr(**this->m_SystemHandle, TCSADRAIN, &options) == -1)
    {
      throw PortAPIError("tcsetattr() failed " + PosixErrorToString(errno), GetPortName());
    }
  }

  void PosixComPort::Purge(PortBuffer buffer) const
  {
    Mutex::LockType lock(m_Mutex);

    int flags = 0;

    switch (buffer)
    {
      case ReceiveBuffer:
        flags = TCIFLUSH;
        break;

      case TransmitBuffer:
        flags = TCOFLUSH;
        break;

      case AllBuffers:
        flags = TCIOFLUSH;
        break;

      default:
        throw PortInvalidParam((boost::format("Called Purge() with invalid buffers parameter (%1%)") % buffer).str());
    }

    if (tcflush(**this->m_SystemHandle, flags) == -1)
    {
      throw PortAPIError("tcflush() failed " + PosixErrorToString(errno), GetPortName());
    }
  }

  void PosixComPort::SendBreakSignal(unsigned ms) const
  {
    Mutex::LockType lock(m_Mutex);

    if (tcsendbreak(**this->m_SystemHandle, ms / 100) == -1)
    {
      throw PortAPIError("tcsendbreak() failed " + PosixErrorToString(errno), GetPortName());
    }
  }

  bool PosixComPort::Send(Buffer::const_iterator& first, Buffer::const_iterator last) const
  {
    Mutex::LockType lock(m_Mutex);

    ssize_t result = write(**this->m_SystemHandle, &(*first), last - first);

    if (result > 0)
    {
      assert(result <= static_cast<ssize_t>(last - first));

      first += result;
    }
    else
    {
      if ((result == -1) && (errno == EAGAIN))
      {
        result = 0;
      }
    }

    return (result != -1);
  }

  Byte PosixComPort::Receive() const
  {
    Mutex::LockType lock(m_Mutex);

    Byte chr;

    ssize_t result = read(**this->m_SystemHandle, &chr, 1);

    assert(result <= 1);

    if (result == -1)
    {
      if (errno == EAGAIN)
      {
        throw PortReadTimeout(*this);
      }

      HandleAndClearPortError();
      throw PortReadFailed(*this, PosixErrorToString(errno));
    }

    if (result == 0)
    {
      throw PortReadTimeout(*this);
    }

    return chr;
  }

  bool PosixComPort::Receive(Buffer& data, size_t length) const
  {
    Mutex::LockType lock(m_Mutex);

    if (length < 1)
    {
      data.clear();
      return true;
    }

    data.resize(length);

    ssize_t bytesRead = read(**this->m_SystemHandle, &data[0], length);

    assert((bytesRead == -1) || static_cast<size_t>(bytesRead) <= length);

    if (bytesRead >= 0)
    {
      data.resize(bytesRead);
    }
    else
    {
      data.clear();
    }

    if (bytesRead == -1)
    {
      if (errno == EAGAIN)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      return true;
    }
  }

  Port::PortErrors PosixComPort::ClearPortError() const
  {
    Mutex::LockType lock(m_Mutex);

    // TODO: how to detect break signal, parity, frame overflow and underflow errors !????

    return PortErrors(0);
  }

  string PosixComPort::GetPortName() const
  {
    return m_PortName;
  }

  string PosixComPort::GetLastOSErrorText() const
  {
    Mutex::LockType lock(m_Mutex);

    return PosixErrorToString(errno);
  }

}  // namespace AutomatedTokenTestDevice

#endif  // ATTD_SYSTEM_POSIX

