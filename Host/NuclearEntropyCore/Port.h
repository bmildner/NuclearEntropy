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

#ifndef NUCLEARENTROPY_PORT_H
#define NUCLEARENTROPY_PORT_H

#include "NuclearEntropyCore/Config.h"

#include <string>

#include <boost/utility.hpp>

#include "NuclearEntropyCore/Mutex.h"
#include "NuclearEntropyCore/Types.h"
#include "NuclearEntropyCore/Exceptions.h"

namespace NuclearEntropy
{

  class NUCENT_API Port : boost::noncopyable
  {
    public:
      enum PortNumbers {COM1 = 1, COM2, COM3, COM4, COM5, COM6, COM7, COM8, COM9, COM10, COM11, COM12, COM13, COM14, COM15, COM16, COM17, COM18, COM19, COM20,
                        COM21, COM22, COM23, COM24, COM25, COM26, COM27, COM28, COM29, COM30, COM31, COM32, COM33, COM34, COM35, COM36, COM37, COM38, COM39, COM40};

      enum BaudRate    {BaudRate_110 = 110, BaudRate_300 = 300, BaudRate_600 = 600, BaudRate_1200 = 1200, BaudRate_2400 = 2400, BaudRate_4800 = 4800, BaudRate_9600 = 9600, 
                        BaudRate_14400 = 14400, BaudRate_19200 = 19200, BaudRate_38400 = 38400, BaudRate_57600 = 57600, BaudRate_115200 = 115200, BaudRate_128000 = 128000, 
                        BaudRate_230400 = 230400, BaudRate_256000 = 256000, BaudRate_460800 = 460800, BaudRate_921600 = 921600};

      enum DataBits    {DataBits_5 = 5, DataBits_6 = 6, DataBits_7 = 7, DataBits_8 = 8};

      enum Parity      {Parity_None, Parity_Odd, Parity_Even, Parity_Mark, Parity_Space};

      enum StopBits    {StopBits_1 = 1, StopBits_2};

      enum FlowControl {FlowControl_None = 0, FlowControl_RTS_CTS = 1, FlowControl_DTS_DSR = 2, FlowControl_XOn_XOff = 4, Default_XOn = 0x11, Default_XOff = 0x13};

      enum Timeout     {DefaultReadTimeout = 500, DefaultWriteTimeout = 500};

      enum PortError   {NoError = 0x00, OverflowError = 0x01, FrameError = 0x02, ParityError = 0x04, BreakSignal = 0x08};

      enum PortBuffer  {ReceiveBuffer = 1, TransmitBuffer, AllBuffers = TransmitBuffer | ReceiveBuffer};

      typedef HandleType<Port>::Type Handle;

      static const unsigned scm_DefaultRetries;
      static const unsigned scm_DefaultDelay;

      Port() {};
      virtual ~Port() {};

      virtual void SetDataConfig(unsigned baud = BaudRate_9600, DataBits dataBits = DataBits_8, Parity parity = Parity_None, StopBits stopBits = StopBits_1) = 0;
      virtual void SetFlowControl(FlowControl flowControl = FlowControl_None, Byte xOn = Default_XOn, Byte xOff = Default_XOff) = 0;

      virtual void GetTimeout(unsigned& readTimeout, unsigned& writeTimeout) const = 0;
      virtual void SetTimeout(unsigned readTimeout = DefaultReadTimeout, unsigned writeTimeout = DefaultWriteTimeout) = 0;      

      virtual void Purge(PortBuffer buffer = AllBuffers) const = 0;

      virtual PortError ClearPortError() const = 0;
      virtual void HandleAndClearPortError() const;

      // TODO: probably change errorhandling strategy to not error out in Send() on port error (but what about a break signal!) !?!?!?!?!?
      virtual void Send(Byte chr) const;
      virtual bool Send(Buffer::const_iterator& first, Buffer::const_iterator last) const = 0;
      // max timeout between two bytes is ((delay + "port timeout") * retries) in ms
      virtual void Send(const Buffer& data, unsigned retries = scm_DefaultRetries, unsigned delay = scm_DefaultDelay) const;
      
      virtual Byte Receive() const = 0;
      // returns true on timeout and false on error, no exception is thrown on errors
      virtual bool Receive(Buffer& data, std::size_t length) const = 0;
      // max timeout between two bytes is ((delay + "port timeout") * retries) in ms
      virtual Buffer Receive(std::size_t length, unsigned retries = scm_DefaultRetries, unsigned delay = scm_DefaultDelay);

      virtual void SendBreakSignal(unsigned ms) const = 0;

      virtual std::string GetPortName() const = 0;

      virtual bool HasSideChannel() const;
     
      // 1 == bit is available as side channel
      virtual Byte GetSideChannelMask() const;

      // 1 == bit is in output mode, 0 == bit is in input mode
      virtual void SetSideChannelConfig(Byte outputs);
      virtual Byte GetSideChannelConfig() const;

      // un-used bits are 0, 1 == bit is high (input or output)
      virtual void SetSideChannelData(Byte data);
      virtual Byte GetSideChannelData() const;

    protected:
      virtual std::string GetLastOSErrorText() const = 0;

    private:
  };


  class NUCENT_API PortRuntimeError : public RuntimeError
  {
    protected:
      explicit PortRuntimeError(const std::string& message)
      : RuntimeError(message)
      {
      };
  };

  class NUCENT_API PortNotSupportedError : public PortRuntimeError
  {
    public:
      explicit PortNotSupportedError(const Port & port, const std::string& message)
      : PortRuntimeError(message + "  not supported for port " + port.GetPortName())
      {
        SetErrorCode(NUCENT_ERR_PORT_NOT_SUPPORTED);
      };
  };

  class NUCENT_API PortAPIError : public PortRuntimeError
  {
    public:
      explicit PortAPIError(const std::string& message, const std::string& portName = std::string())
      : PortRuntimeError("API error for port (" + (portName.empty() ? "<none>" : portName) + ": " + message)
      {
        SetErrorCode(NUCENT_ERR_PORT_API_ERROR);
      };
  };

  class NUCENT_API PortOpenError : public PortRuntimeError
  {
    protected:
      explicit PortOpenError(const std::string& message)
      : PortRuntimeError(message)
      {        
      };
  };

  class NUCENT_API PortOpenFailed : public PortOpenError
  {
    public:
      explicit PortOpenFailed(const Port& port, const std::string& reason)
      : PortOpenError("failed to open port (" + port.GetPortName() + ") due to \"" + reason + "\"")
      {
        SetErrorCode(NUCENT_ERR_PORT_OPEN_FAILED);
      };
  };

  class NUCENT_API PortNotFound : public PortOpenError
  {
    public:
      explicit PortNotFound(const std::string& portName)
      : PortOpenError("port not found (" + portName + ")")
      {
        SetErrorCode(NUCENT_ERR_PORT_NOT_FOUND);
      };
  };

  class NUCENT_API PortNameInvalid : public PortOpenError
  {
    public:
      explicit PortNameInvalid(const std::string& portName)
      : PortOpenError("invalid port name (" + portName + ")")
      {
        SetErrorCode(NUCENT_ERR_PORT_INVALID_NAME);
      };
  };

  class NUCENT_API PortConfigFailed : public PortRuntimeError
  {
    public:
      explicit PortConfigFailed(const Port& port, const std::string& message = "")
      : PortRuntimeError("failed to configure port (" + port.GetPortName() + ")" + (!message.empty() ? " " + message : ""))
      {
        SetErrorCode(NUCENT_ERR_PORT_CONFIG_ERROR);
      };
  };

  class NUCENT_API PortIOError : public PortRuntimeError
  {
    protected:
      explicit PortIOError(const std::string& message)
      : PortRuntimeError(message)
      {
      };
  };

  class NUCENT_API PortBreakSignal : public PortIOError
  {
    public:
      explicit PortBreakSignal(const Port& port)
      : PortIOError("Break detected on port (" + port.GetPortName() + ")")
      {
        SetErrorCode(NUCENT_ERR_PORT_BREAK_SIGNAL);
      };
  };

  class NUCENT_API PortFrameError : public PortIOError
  {
    public:
      explicit PortFrameError(const Port& port)
      : PortIOError("Frame error detected on port (" + port.GetPortName() + ")")
      {
        SetErrorCode(NUCENT_ERR_PORT_FRAME_ERROR);
      };
  };

  class NUCENT_API PortOverflowError : public PortIOError
  {
    public:
      explicit PortOverflowError(const Port& port)
      : PortIOError("Buffer overflow or overrun error detected on port (" + port.GetPortName() + ")")
      {
        SetErrorCode(NUCENT_ERR_PORT_OVERFLOW_ERROR);
      };
  };

  class NUCENT_API PortParityError: public PortIOError
  {
    public:
      explicit PortParityError(const Port& port)
      : PortIOError("Parity error detected on port (" + port.GetPortName() + ")")
      {
        SetErrorCode(NUCENT_ERR_PORT_PARITY_ERROR);
      };
  };

  class NUCENT_API PortWriteFailed : public PortIOError
  {
    public:
      explicit PortWriteFailed(const Port& port, const std::string& errorText)
      : PortIOError("write operation failed on port (" + port.GetPortName() + ") with error \"" + errorText + "\"")
      {
        SetErrorCode(NUCENT_ERR_PORT_WRITE_ERROR);
      };
  };

  class NUCENT_API PortReadFailed : public PortIOError
  {
    public:
      explicit PortReadFailed(const Port& port, const std::string& errorText)
      : PortIOError("read operation failed on port (" + port.GetPortName() + ") with error \"" + errorText + "\"")
      {
        SetErrorCode(NUCENT_ERR_PORT_READ_ERROR);
      };
  };

  class NUCENT_API PortTimeout : public PortIOError
  {
    protected:
      explicit PortTimeout(const std::string& message)
      : PortIOError(message)
      {
      };
  };

  class NUCENT_API PortWriteTimeout : public PortTimeout
  {
    public:
      explicit PortWriteTimeout(const Port& port)
      : PortTimeout("write operation on port (" + port.GetPortName() + ") timed out")
      {
        SetErrorCode(NUCENT_ERR_PORT_WRITE_TIMEOUT);
      };
  };

  class NUCENT_API PortReadTimeout : public PortTimeout
  {
    public:
      explicit PortReadTimeout(const Port& port)
      : PortTimeout("read operation on port (" + port.GetPortName() + ") timed out")
      {
        SetErrorCode(NUCENT_ERR_PORT_READ_TIMEOUT);
      };
  };

  class NUCENT_API PortLogicError : public LogicError
  {
    protected:
      explicit PortLogicError(const std::string& message)
      : LogicError(message)
      {
      };
  };

  class NUCENT_API PortInvalidParam : public PortLogicError
  {
    public:
      explicit PortInvalidParam(const std::string& message)
      : PortLogicError("Invalid parameter" + (message.empty() ? "" : " (" + message + ")"))
      {
        SetErrorCode(NUCENT_ERR_INVALID_PARAM);
      };
  };

}  // namespace NuclearEntropy

#endif

