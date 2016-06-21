/*
 * Misc.c
 *
 * Created: 06.06.2016 21:29:10
 *  Author: Berti
 */ 

#include "Misc.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include "GeigerTube.h"

#define PORT_STATUSLED PORTB
#define DDR_STATUSLED  DDRB
#define PIN_STATUSLED  PINB
#define STATUSLED      PINB2

#define PORT_PWREN PORTD
#define DDR_PWREN  DDRD
#define PIN_PWREN  PIND
#define PWREN      PIND7


static State g_State        = Startup;
static Bool  g_StateChanged = FALSE;

const State* volatile g_pState        = &g_State;
Bool* volatile        g_pStateChanged = &g_StateChanged;



void MISC_Initialize()
{
  // disable not needed hardware
  PRR0 = (1 << PRTWI0) | (1 << PRUSART1) | (1 << PRSPI0) | (1 << PRADC);  // TWI0, USART1, SPI0, ADC
  PRR1 = (1 << PRTWI1) | (1 << PRPTC) | (1 << PRSPI1);                    // TWI1, PTC, SPI1

  // enable CFD interrupt
  XFDCSR |= (1 << XFDIE);

  // setup status LED pin (output, inverted)
  PORT_STATUSLED |= (1 << STATUSLED);
  DDR_STATUSLED  |= (1 << STATUSLED);

  // setup PWREN pin (input, inverted, interrupt on any change, PCINT23)
  PORT_PWREN &= ~(1 << PWREN);
  DDR_PWREN &= ~(1 << PWREN);
  PCMSK2 |= (1 << PCINT23);
  PCICR |= (1 << PCIE2);

  if (PIN_PWREN & (1 << PWREN))
  {
    g_State = Suspended;
  }

  // enable watchdog timer, timeout 1 sec
  wdt_enable(WDTO_1S);

  // check reset reason
  if (MCUSR & ((1 << WDRF) | (1 << BORF)))
  {
    g_State = StartupAfterError;
    g_StateChanged = TRUE;
  }
}

void MISC_StatusLEDEnable()
{
  PORT_STATUSLED &= ~(1 << STATUSLED);
}

void MISC_StatusLEDDisable()
{
  PORT_STATUSLED |= (1 << STATUSLED);
}

ResetReason MISC_GetResetReason()
{
  static ResetReason reason = 0;

  if (reason == 0)
  {
    Byte temp = MCUSR;

    MCUSR = 0x00;

    if (temp & (1 << PORF))
    {
      reason |= PowerOnReset;
    }

    if (temp & (1 << EXTRF))
    {
      reason |= ExternalReset;
    }
    
    if (temp & (1 << BORF))
    {
      reason |= BrownOutReset;
    }

    if (temp & (1 << WDRF))
    {
      reason |= WatchdogReset;
    }    
  }

  return reason;
}


ISR(CFD_vect)
{
  MISC_StatusLEDDisable();
  GeigerTube_Disable();

  // force watchdog reset!
  wdt_enable(WDTO_15MS);
  for (;;);
}

ISR(PCINT2_vect)
{
  static State savedState = Startup;

  if (PIN_PWREN & (1 << PWREN))
  {
    savedState = g_State;
    g_State = Suspended;
    g_StateChanged = TRUE;
  }
  else
  {
    g_State = savedState;
    g_StateChanged = TRUE;
  }
}

