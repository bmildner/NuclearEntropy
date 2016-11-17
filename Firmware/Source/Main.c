/*
 * Test.c
 *
 * Created: 21.04.2016 00:48:59
 * Author : Berti
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#include "RingBuffer.h"
#include "Usart.h"
#include "GeigerTube.h"
#include "Configuration.h"
#include "Misc.h"
#include "CommandInterface.h"


void Initialize()
{
  // read config from EEPROM
  Configuration_Read();

  // initialize misc module
  MISC_Initialize();

  // initialize UART module
  USART_Initialize();

  // initialize geiger tube detectors module
  GeigerTube_Initialize();

  // initialize command interface module
  Command_Initialize();

  // enable interrupts
  sei();
}

Bool DoSelfTest()
{
  if (!RingBufferTest())
  {
    return FALSE;
  }

  // enable test signal
  // check event detection
  // disable test signal

  // enable high voltage
  // wait for event count to settle
  // check event rate

  return TRUE;
}


int main(void)
{
  

  Initialize();


  DoSelfTest();  // TODO: check selftest result

  MISC_WatchdogTimerReset();

  USART_SetDSR();

  GeigerTube_EnableTestMode();
  //GeigerTube_Disable();

  MISC_StatusLEDEnable();

  while (TRUE)
  {
    MISC_WatchdogTimerReset();
    Command_DoWork();
    GeigerTube_DoWork();
  }
  
}

