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

#include "NuclearEntropyCore/D2XXPort.h"

#ifdef ATTD_SYSTEM_WINDOWS  // only compile on Windows

#include <cassert>
#include <iomanip>
#include <vector>

ATTD_BOOST_INCL_GUARD_BEGIN
#include <boost/foreach.hpp>
#include <boost/format.hpp>
ATTD_BOOST_INCL_GUARD_END

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include "D2xx/ftd2xx.h"

#include "NuclearEntropyCore/Utility.h"

using namespace std;

namespace
{
  using namespace AutomatedTokenTestDevice;

  typedef vector<FT_DEVICE_LIST_INFO_NODE> DeviceInfoList;

  // we better use a global mutex, FTDI does not document ANY type of thread safety for the D2xx API!
  Mutex& GetGlobalMutex()
  {
    static Mutex mutex;
    
    return mutex;
  }

  string D2XXErrorToString(FT_STATUS err)
  {
# define CASE(x) case x:                                                                                                                      \
                 return (boost::format("D2XX API error: %1% (%2%)") % #x % boost::io::group(hex, showbase, setw(8), setfill('0'), x)).str();  \
                 break

    switch (err)
    {
      CASE(FT_OK);
      CASE(FT_INVALID_HANDLE);
      CASE(FT_DEVICE_NOT_FOUND);
      CASE(FT_DEVICE_NOT_OPENED);
      CASE(FT_IO_ERROR);
      CASE(FT_INSUFFICIENT_RESOURCES);
      CASE(FT_INVALID_PARAMETER);
      CASE(FT_INVALID_BAUD_RATE);
      CASE(FT_DEVICE_NOT_OPENED_FOR_ERASE);
      CASE(FT_DEVICE_NOT_OPENED_FOR_WRITE);
      CASE(FT_FAILED_TO_WRITE_DEVICE);
      CASE(FT_EEPROM_READ_FAILED);
      CASE(FT_EEPROM_WRITE_FAILED);
      CASE(FT_EEPROM_ERASE_FAILED);
      CASE(FT_EEPROM_NOT_PRESENT);
      CASE(FT_EEPROM_NOT_PROGRAMMED);
      CASE(FT_INVALID_ARGS);
      CASE(FT_NOT_SUPPORTED);
      CASE(FT_OTHER_ERROR);
      CASE(FT_DEVICE_LIST_NOT_READY);

      default:
        return (boost::format("D2XX API error: <Unknown> (%1%)") % boost::io::group(hex, showbase, setw(8), setfill('0'), err)).str();
    }

# undef CASE
  }

  DeviceInfoList GetDeviceInfoList()
  {
    Mutex::LockType lock(GetGlobalMutex());

    DWORD numDevices;

    FT_STATUS err;
    
    err = FT_CreateDeviceInfoList(&numDevices);
    if (err != FT_OK)
    {
      throw PortAPIError("FT_CreateDeviceInfoList() failed " + D2XXErrorToString(err));
    }

    DeviceInfoList deviceInfoList;

    deviceInfoList.resize(numDevices);

    err = FT_GetDeviceInfoList(&deviceInfoList[0], &numDevices);
    if (err != FT_OK)
    {
      throw PortAPIError("FT_GetDeviceInfoList() failed " + D2XXErrorToString(err));
    }

    assert(deviceInfoList.size() >= numDevices);

    deviceInfoList.resize(numDevices);
  
    return deviceInfoList;
  }
}  // namespace

namespace AutomatedTokenTestDevice
{
  struct D2XXPort::SystemHandle
  {
    FT_HANDLE m_Handle;

    inline explicit SystemHandle()
    : m_Handle(0)
    {
    }

    inline ~SystemHandle()
    {
      if (this->m_Handle != 0)
      {
        Mutex::LockType lock(GetGlobalMutex());

        FT_Close(this->m_Handle);
      }
    }

    inline HANDLE& operator*()
    {
      return this->m_Handle;
    }
  };

  D2XXPort::PortList D2XXPort::GetPortDescriptionList()
  {
    PortList deviceList;

    DeviceInfoList deviceInfoList = GetDeviceInfoList();

    BOOST_FOREACH(const DeviceInfoList::value_type& deviceInfo, deviceInfoList)
    {
      if (((deviceInfo.Flags & FT_FLAGS_OPENED) == 0) && (strlen(deviceInfo.Description) > 0))
      {
        deviceList.push_back(deviceInfo.Description);
      }
    }

    return deviceList;
  }

  D2XXPort::PortList D2XXPort::GetPortSerialNumbersList()
  {
    PortList deviceList;

    DeviceInfoList deviceInfoList = GetDeviceInfoList();

    BOOST_FOREACH(const DeviceInfoList::value_type& deviceInfo, deviceInfoList)
    {
      if (((deviceInfo.Flags & FT_FLAGS_OPENED) == 0) && (strlen(deviceInfo.SerialNumber) > 0))
      {
        deviceList.push_back(deviceInfo.SerialNumber);
      }
    }

    return deviceList;
  }

  D2XXPort::PortList D2XXPort::GetPortNameList()
  {
    PortList deviceList;

    DeviceInfoList deviceInfoList = GetDeviceInfoList();

    BOOST_FOREACH(const DeviceInfoList::value_type& deviceInfo, deviceInfoList)
    {
      if (((deviceInfo.Flags & FT_FLAGS_OPENED) == 0) && ((strlen(deviceInfo.Description) > 0) || (strlen(deviceInfo.SerialNumber) > 0)))
      {
        deviceList.push_back(MakePortName(deviceInfo.Description, deviceInfo.SerialNumber));
      }
    }

    return deviceList;
  }

  void D2XXPort::FindPort(const string& portName, string& description, string& serial, bool& useDescription)
  {
    DeviceInfoList deviceInfoList = GetDeviceInfoList();

    DeviceInfoList::size_type descriptionCount = 0;
    DeviceInfoList::size_type serialCount = 0;
    DeviceInfoList::size_type portNameCount = 0;

    const DeviceInfoList::value_type* port = 0;
    //DeviceInfoList::const_iterator port;

    BOOST_FOREACH(const DeviceInfoList::value_type& deviceInfo, deviceInfoList)
    {
      if ((deviceInfo.Flags & FT_FLAGS_OPENED) == 0)
      {
        if (portName == deviceInfo.Description)
        {
          descriptionCount++;
          port = &deviceInfo;
        }

        if (portName == deviceInfo.SerialNumber)
        {
          serialCount++;
          port = &deviceInfo;
        }

        if (portName == MakePortName(deviceInfo.Description, deviceInfo.SerialNumber))
        {
          portNameCount++;
          port = &deviceInfo;
        }
      }
    }

    if ((descriptionCount + serialCount + portNameCount) == 0)
    {
      throw PortNotFound(portName);
    }

    if ((descriptionCount + serialCount + portNameCount) > 1)
    {
      throw PortNotFound(portName);  // TODO: probably throw something like a PortNameAmbiguous 
    }

    assert(port != 0);

    description = port->Description;
    serial = port->SerialNumber;

    if ((descriptionCount == 1) || ((portNameCount == 1) && (serial.empty())))
    {
      useDescription = true;
    }
    else
    {
      useDescription = false;
    }

    assert((descriptionCount == 1) ^ (serialCount == 1) ^ (portNameCount == 1));
  }

  string D2XXPort::MakePortName(const string& description, const string& serial)
  {
    return description + '/' + serial;
  }

  D2XXPort::Handle D2XXPort::CreateHandle(const string& portName)
  {
    string description;
    string serial;
    bool useDescription;

    FindPort(portName, description, serial, useDescription);
  
    return Handle(new D2XXPort(description, serial, useDescription));
  }


  D2XXPort::D2XXPort(const std::string& description, const std::string& serial, bool useDescription)
  : m_PortDescription(description), m_PortSerial(serial), m_SystemHandle(new SystemHandle), m_SideChannelIOMask(0), m_SideChannelConfig(0), m_SideChannelData(0)
  {
    Mutex::LockType lock(GetGlobalMutex());

    assert(this->m_SystemHandle != 0);

    try
    {
      FT_STATUS err;

      if (m_PortDescription.size() >= sizeof(FT_DEVICE_LIST_INFO_NODE().Description))
      {
        throw PortNameInvalid((boost::format("Port description too long (%1%)") % m_PortDescription).str());
      }

      if (m_PortSerial.size() >= sizeof(FT_DEVICE_LIST_INFO_NODE().SerialNumber))
      {
        throw PortNameInvalid((boost::format("Port serial number too long (%1%)") % m_PortSerial).str());
      }

      if (((m_PortDescription.size() < 1) && (m_PortSerial.size() < 1)) || 
          (useDescription ? (m_PortDescription.size() < 1) : (m_PortSerial.size() < 1)))
      {
        throw PortNameInvalid((boost::format("Invalid combination of port description and serial number (%1%) (%2%)") % m_PortDescription % m_PortSerial).str());
      }

      string tempName = useDescription ? m_PortDescription : m_PortSerial;

      **m_SystemHandle = FT_W32_CreateFile(reinterpret_cast<LPTSTR>(&tempName[0]), GENERIC_READ|GENERIC_WRITE, 0, 0, OPEN_EXISTING, 
                                           FILE_ATTRIBUTE_NORMAL | (useDescription ? FT_OPEN_BY_DESCRIPTION : FT_OPEN_BY_SERIAL_NUMBER), 0);
      if (**m_SystemHandle == INVALID_HANDLE_VALUE)
      {
        throw PortOpenFailed(*this, Detail::Win32ErrorToString(FT_W32_GetLastError(**m_SystemHandle)));
      }


      // set base configure port
      FTDCB dcb;
      
      memset(&dcb, 0x00, sizeof(FTDCB));
      dcb.DCBlength = sizeof(FTDCB);
  
      if (!FT_W32_GetCommState(**this->m_SystemHandle, &dcb))
      {
        throw PortAPIError("GetCommState() failed " + Detail::Win32ErrorToString(FT_W32_GetLastError(**m_SystemHandle)), GetPortName());
      }

      dcb.fBinary           = true;
      dcb.fParity           = true;
      dcb.fDsrSensitivity   = false;
      dcb.fErrorChar        = false;
      dcb.fNull             = false;
      dcb.fAbortOnError     = true;
      dcb.fTXContinueOnXoff = true;

      // D2XX library does not set default port config like Win32 does, we have to set some meaningful config!
      dcb.BaudRate = BaudRate_9600;
      dcb.ByteSize = DataBits_8;
      dcb.StopBits = StopBits_1;

      if (!FT_W32_SetCommState(**this->m_SystemHandle, &dcb))
      {
        throw PortConfigFailed(*this, "SetCommState() failed " + Detail::Win32ErrorToString(FT_W32_GetLastError(**m_SystemHandle)));
      }

      // set latency as low as possible (according to API docu 2-255 ms is possible, but 0-1 are also accepted for a FT232R !?)
      err = FT_SetLatencyTimer(**m_SystemHandle, 1);
      if (err != FT_OK)
      {
        err = FT_SetLatencyTimer(**m_SystemHandle, 2);
        if (err != FT_OK)
        {
          err = FT_SetLatencyTimer(**m_SystemHandle, 16);  // minimum for older devices
          if (err != FT_OK)
          {
            throw PortConfigFailed(*this);
          }
        }
      }

      // get side channel config from EEPROM
      FT_DEVICE device;

      err = FT_GetDeviceInfo(**m_SystemHandle, &device, 0, 0, 0, 0);
      if (err != FT_OK)
      {
        throw PortAPIError("FT_GetDeviceInfo() failed " + D2XXErrorToString(err), GetPortName());
      }

      // only FT232R has CBUS side channel that is available while using the UART!
      if (device == FT_DEVICE_232R)
      {
        FT_PROGRAM_DATA eePromData;

        eePromData.Signature1 = 0x00000000;
        eePromData.Signature2 = 0xffffffff;
        eePromData.Version    = 0x00000002; // EEPROM data structure version with FT232R extensions

        // we do not need any of the strings
        eePromData.Manufacturer   = 0;
        eePromData.ManufacturerId = 0;
        eePromData.Description    = 0;	
        eePromData.SerialNumber   = 0;

        err = FT_EE_Read(**m_SystemHandle, &eePromData);
        if (err != FT_OK)
        {
          throw PortAPIError("FT_EE_Read() failed " + D2XXErrorToString(err), GetPortName());
        }

        if (eePromData.Cbus0 == FT_232R_CBUS_IOMODE)
        {
          m_SideChannelIOMask |= (1 << 0);
        }

        if (eePromData.Cbus1 == FT_232R_CBUS_IOMODE)
        {
          m_SideChannelIOMask |= (1 << 1);
        }

        if (eePromData.Cbus2 == FT_232R_CBUS_IOMODE)
        {
          m_SideChannelIOMask |= (1 << 2);
        }

        if (eePromData.Cbus3 == FT_232R_CBUS_IOMODE)
        {
          m_SideChannelIOMask |= (1 << 3);
        }
        
        SetSideChannelConfig(0);
        SetSideChannelData(0);
      }
    }

    catch (...)
    {
      delete m_SystemHandle;

      throw;
    }
  }

  D2XXPort::~D2XXPort()
  {    
    assert (this->m_SystemHandle != 0);

    delete this->m_SystemHandle;
  }

  void D2XXPort::SetDataConfig(unsigned baud, DataBits dataBits, Parity parity, StopBits stopBits)
  {
    Mutex::LockType lock(GetGlobalMutex());

    FTDCB dcb;
      
    memset(&dcb, 0x00, sizeof(FTDCB));
    dcb.DCBlength = sizeof(FTDCB);
  
    if (!FT_W32_GetCommState(**this->m_SystemHandle, &dcb))
    {
      throw PortAPIError("GetCommState() failed " + Detail::Win32ErrorToString(FT_W32_GetLastError(**m_SystemHandle)), GetPortName());
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

    if (!FT_W32_SetCommState(**this->m_SystemHandle, &dcb))
    {
      throw PortConfigFailed(*this, "SetCommState() failed " + Detail::Win32ErrorToString(FT_W32_GetLastError(**m_SystemHandle)));
    }
  }

  void D2XXPort::SetFlowControl(FlowControl flowControl, Byte xOn, Byte xOff)
  {
    Mutex::LockType lock(GetGlobalMutex());

    if ((flowControl & ~(FlowControl_RTS_CTS | FlowControl_DTS_DSR | FlowControl_XOn_XOff)) != 0)
    {
      throw PortInvalidParam((boost::format("Invalid flow control (%1%)") % flowControl).str());
    }

    FTDCB dcb;
      
    memset(&dcb, 0x00, sizeof(FTDCB));
    dcb.DCBlength = sizeof(FTDCB);
  
    if (!FT_W32_GetCommState(**this->m_SystemHandle, &dcb))
    {
      throw PortAPIError("GetCommState() failed " + Detail::Win32ErrorToString(FT_W32_GetLastError(**m_SystemHandle)), GetPortName());
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

    if (!FT_W32_SetCommState(**this->m_SystemHandle, &dcb))
    {
      throw PortConfigFailed(*this, "SetCommState() failed " + Detail::Win32ErrorToString(FT_W32_GetLastError(**m_SystemHandle)));
    }
  }

  void D2XXPort::GetTimeout(unsigned& readTimeout, unsigned& writeTimeout) const
  {
    Mutex::LockType lock(GetGlobalMutex());

    // get timeout values
    FTTIMEOUTS timeouts;

    if (!FT_W32_GetCommTimeouts(**this->m_SystemHandle, &timeouts))
    {
      throw PortConfigFailed(*this, "SetCommTimeouts() failed " + Detail::Win32ErrorToString(FT_W32_GetLastError(**m_SystemHandle)));
    }

    // (over?) simplified query of timeout values
    readTimeout  = timeouts.ReadIntervalTimeout;
    writeTimeout = timeouts.WriteTotalTimeoutMultiplier;
  }

  void D2XXPort::SetTimeout(unsigned readTimeout, unsigned writeTimeout)
  {
    Mutex::LockType lock(GetGlobalMutex());

    // set default timeout for read and write

    FTTIMEOUTS timeouts;

    // set default timeout (total duration and between bytes)
    timeouts.ReadIntervalTimeout         = (readTimeout == 0 ? MAXDWORD : readTimeout);  // set to MAXDWORD for immediate return
    timeouts.ReadTotalTimeoutMultiplier  = readTimeout;
    timeouts.ReadTotalTimeoutConstant    = 0;
    timeouts.WriteTotalTimeoutMultiplier = writeTimeout;
    timeouts.WriteTotalTimeoutConstant   = 0;

    if (!FT_W32_SetCommTimeouts(**this->m_SystemHandle, &timeouts))
    {
      throw PortConfigFailed(*this, "FT_W32_SetCommTimeouts() failed " + Detail::Win32ErrorToString(FT_W32_GetLastError(**m_SystemHandle)));
    }
  }

  void D2XXPort::Purge(PortBuffer buffer) const
  {
    Mutex::LockType lock(GetGlobalMutex());

    DWORD flags;

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
        throw PortInvalidParam((boost::format("Called Purge() with invalid buffer parameter (%1%)") % buffer).str());
    }

    if (!FT_W32_PurgeComm(**this->m_SystemHandle, flags))
    {
      throw PortAPIError("PurgeComm() failed " + Detail::Win32ErrorToString(FT_W32_GetLastError(**m_SystemHandle)), GetPortName());
    }
  }

  bool D2XXPort::Send(Buffer::const_iterator& first, Buffer::const_iterator last) const
  {
    Mutex::LockType lock(GetGlobalMutex());

    DWORD sent = 0;

    // we need to check for errors our self, FTDI's WriteFile is broken because it does not respect the fAbortOnError flag
    ULONG status = 0;

    FT_STATUS err = FT_GetModemStatus(**m_SystemHandle, &status);  // FT_GetModemStatus does not reset port error state
    if (err != FT_OK)
    {
      throw PortAPIError("FT_GetModemStatus() failed " + D2XXErrorToString(err), GetPortName());
    }

    // check for overrun, parity, framing error and break signal
    if ((status & ((0x02 | 0x04 | 0x08 | 0x10) << 8)) != 0)
    {
      return false;
    }

    bool result = !!FT_W32_WriteFile(**this->m_SystemHandle, const_cast<Byte*>(&(*first)), last - first, &sent, 0);

    assert(sent <= static_cast<DWORD>(last - first));

    first += sent;

    return result;
  }

  Byte D2XXPort::Receive() const
  {
    Mutex::LockType lock(GetGlobalMutex());

    Byte chr;
    DWORD bytesRead = 0;

    HandleAndClearPortError();  // we have to check first for an error because FT_W32_WriteFile will not return with an error!

    if (!FT_W32_ReadFile(**this->m_SystemHandle, &chr, sizeof(chr), &bytesRead, 0))
    {
      throw PortReadFailed(*this, Detail::Win32ErrorToString(FT_W32_GetLastError(**m_SystemHandle)));
    }

    if (bytesRead < sizeof(chr))
    {
      throw PortReadTimeout(*this);
    }

    return chr;
  }

  bool D2XXPort::Receive(Buffer& data, size_t length) const
  {
    Mutex::LockType lock(GetGlobalMutex());

    if (length < 1)
    {
      data.clear();
      return true;
    }

    // we need to check for errors our self, FTDI's WriteFile is broken because it does not respect the fAbortOnError flag
    ULONG status = 0;

    FT_STATUS err = FT_GetModemStatus(**m_SystemHandle, &status);  // FT_GetModemStatus does not reset port error state
    if (err != FT_OK)
    {
      throw PortAPIError("FT_GetModemStatus() failed " + D2XXErrorToString(err), GetPortName());
    }

    // check for overrun, parity, framing error and break signal
    if ((status & ((0x02 | 0x04 | 0x08 | 0x10) << 8)) != 0)
    {
      data.clear();
      return false;
    }

    data.resize(length);

    DWORD bytesRead = 0;

    bool result = !!FT_W32_ReadFile(**this->m_SystemHandle, &data[0], length, &bytesRead, 0);

    assert(bytesRead <= length);

    data.resize(bytesRead);

    return result;
  }

  Port::PortErrors D2XXPort::ClearPortError() const
  {
    Mutex::LockType lock(GetGlobalMutex());

    DWORD err;

    FTCOMSTAT comStatus;

    if (FT_W32_ClearCommError(**m_SystemHandle, &err, &comStatus))
    {
      PortErrors errors = PortErrors(0);

      if ((err & CE_OVERRUN) != 0)
      {
        errors = static_cast<PortErrors>(errors | OverflowError);
      }

      if ((err & CE_RXOVER) != 0)
      {
        errors = static_cast<PortErrors>(errors | OverflowError);
      }

      if ((err & CE_FRAME) != 0)
      {
        errors = static_cast<PortErrors>(errors | FrameError);
      }

      if ((err & CE_RXPARITY) != 0)
      {
        errors = static_cast<PortErrors>(errors | ParityError);
      }

      if ((err & CE_BREAK) != 0)
      {
        errors = static_cast<PortErrors>(errors | BreakSignal);
      }

      return errors;
    }

    throw PortAPIError("ClearCommError() failed " + Detail::Win32ErrorToString(FT_W32_GetLastError(**m_SystemHandle)), GetPortName());
  }

  string D2XXPort::GetPortName() const
  {
    return MakePortName(m_PortDescription, m_PortSerial);
  }

  void D2XXPort::SendBreakSignal(unsigned ms) const
  {
    Mutex::LockType lock(GetGlobalMutex());

    FT_STATUS err;

    err = FT_SetBreakOn(**m_SystemHandle);
    if (err != FT_OK)
    {
      throw PortAPIError("FT_SetBreakOn() failed "  + D2XXErrorToString(err), GetPortName());
    }

    ThreadSleepMs(ms);

    err = FT_SetBreakOff(**m_SystemHandle);
    if (err != FT_OK)
    {
      throw PortAPIError("FT_SetBreakOff() failed "  + D2XXErrorToString(err), GetPortName());
    }
  }

  bool D2XXPort::HasSideChannel() const
  {
    return m_SideChannelIOMask != 0;
  }

  Byte D2XXPort::GetSideChannelMask() const
  {
    return m_SideChannelIOMask;
  }

  void D2XXPort::SetSideChannelConfig(Byte outputs)
  {
    Mutex::LockType lock(GetGlobalMutex());

    // mask non-CBus GPIO pins
    outputs &= m_SideChannelIOMask;

    // shift bits into position (upper nibble)
    outputs <<= scm_SideChannelConfigPosition;

    FT_STATUS err;

    err = FT_SetBitMode(**m_SystemHandle, outputs | GetSideChannelData(), FT_BITMODE_CBUS_BITBANG);
    if (err != FT_OK)
    {
      throw PortAPIError("FT_SetBitMode() failed "  + D2XXErrorToString(err), GetPortName());
    }

    m_SideChannelConfig = (outputs >> scm_SideChannelConfigPosition);
  }

  Byte D2XXPort::GetSideChannelConfig() const
  {
    Mutex::LockType lock(GetGlobalMutex());

    // something is wrong with reading side channel / CBUS config!! FT_GetBitMode return wrong config bit !?!?!?!?!?

    return m_SideChannelConfig;
  }

  void D2XXPort::SetSideChannelData(Byte data)
  {
    Mutex::LockType lock(GetGlobalMutex());

    // mask non-CBus GPIO pins
    data &= m_SideChannelIOMask;

    FT_STATUS err = FT_SetBitMode(**m_SystemHandle, data | (GetSideChannelConfig() << scm_SideChannelConfigPosition), FT_BITMODE_CBUS_BITBANG);
    if (err != FT_OK)
    {
      throw PortAPIError("FT_SetBitMode() failed "  + D2XXErrorToString(err), GetPortName());
    }

    m_SideChannelData = data;
  }

  Byte D2XXPort::GetSideChannelData() const
  {
    Mutex::LockType lock(GetGlobalMutex());

    // TODO: something is wrong with reading side channel / CBUS config!! FT_GetBitMode returns wrong config bits !?!?!?!?!?

    return m_SideChannelData;
  }

  std::string D2XXPort::GetLastOSErrorText() const
  {
    return Detail::Win32ErrorToString(FT_W32_GetLastError(**m_SystemHandle));
  }

}  // namespace AutomatedTokenTestDevice

#endif  // ATTD_SYSTEM_WINDOWS

