/*
 * Misc.c
 *
 * Created: 06.06.2016 21:29:10
 *  Author: Berti
 */ 

#include "Misc.h"

#include <assert.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/atomic.h>

#include "GeigerTube.h"

#define PORT_STATUSLED PORTB
#define DDR_STATUSLED  DDRB
#define PIN_STATUSLED  PINB
#define STATUSLED      PINB2

#define PORT_PWREN PORTD
#define DDR_PWREN  DDRD
#define PIN_PWREN  PIND
#define PWREN      PIND7

#define PORT_RESET PORTE
#define DDR_RESET  DDRE
#define PIN_RESET  PINE
#define RESET      PINE1


static State g_State        = Startup;
static Bool  g_StateChanged = FALSE;

const State* volatile g_pState        = &g_State;
const Bool*  volatile g_pStateChanged = &g_StateChanged;

static uint32_t g_TimerValue = 0;  // simple timer


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

  // init TIMER2 for simple Timer
  TCCR2B = 0x00;                                       // stop clock
  TCCR2A = 0x00;                                       // normal mode
  ASSR   = 0x00;                                       // clock from CPU
  TIFR2  = (1 << OCF2B) | (1 << OCF2A) | (1 << TOV2);  // clear IRQ flags
  TIMSK2 = (1 << TOIE2);                               // enable overflow IRQ
  TCNT2  = 0x00;                                       // reset timer value

  // leave RESET (E1) in input-mode/high-Z

  // enable watchdog timer, timeout 1 sec
  wdt_enable(WDTO_1S);

  // check reset reason
  if ((MISC_GetResetReason() & !(PowerOnReset | ExternalReset)) != 0)
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

void MISC_ResetMCU()
{
  cli();

  // pull reset low  
  DDR_RESET  &= ~(1 << RESET);
  PORT_RESET |= (1 << RESET);

  for (;;);
}

void MISC_ResetMCUOnError()
{
  cli();

  // force watchdog reset!
  wdt_enable(WDTO_15MS);
  for (;;);
}


ResetReason MISC_GetResetReason()
{
  static ResetReason reason = 0;

  if (reason == 0)
  {
    Byte temp = MCUSR;

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

  assert(reason != 0);

  return reason;
}

void MISC_ClearResetReason()
{
  // make sure we do cache the last reset reason!
  MISC_GetResetReason();

  MCUSR = 0x00;
}

ISR(CFD_vect)
{
  MISC_StatusLEDDisable();
  GeigerTube_Disable();

  MISC_ResetMCUOnError();
}

// monitor PWREN from FTDI
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


void MISC_StartTimer(uint8_t seconds)
{
  TCCR2B = 0x00;  // stop timer clock

  g_TimerValue = (F_CPU / 1024) * seconds;;

  assert(((seconds == 0) && (g_TimerValue == 0)) || (g_TimerValue / seconds) == (F_CPU / 1024));

  TCNT2  = 0x00;                                       // reset timer value
  TCCR2B = (1 << CS20) | (1 << CS21) | (1 << CS22);    // set timer clock (F_CPU / 1024)
  GTCCR  = (1 << TSM) | (1 << PSRASY);                 // reset prescaler
  GTCCR  = 0x00;                                       // release timer clock
}

void MISC_ClearTimer()
{
  TCCR2B = 0x00;    // stop timer clock

  g_TimerValue = 0;
}

Bool MISC_HasTimerElapsed()
{
  uint32_t timerValue;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    timerValue = g_TimerValue;
  }

  return (timerValue == 0);
}


ISR(TIMER2_OVF_vect)
{
  if (g_TimerValue > 0)
  {
    g_TimerValue--;
  }
  else
  {
    TCCR2B = 0x00;  // stop timer clock
  }
}
