/*
 * Misc.h
 *
 * Created: 06.06.2016 21:29:22
 *  Author: Berti
 */ 

#ifndef MISC_H_
#define MISC_H_

#include <avr/wdt.h>

#include "Types.h"


typedef enum 
{
  Startup = 1,
  StartupAfterError,
  Selftest, SelftestFailed,
  Operational, Suspended, Failed
} State;

typedef enum {PowerOnReset = 1, ExternalReset = 2, BrownOutReset = 4, WatchdogReset = 8} ResetReason;


extern const State* volatile g_pState;
extern const Bool*  volatile g_pStateChanged;


void MISC_Initialize();

void MISC_StatusLEDEnable();
void MISC_StatusLEDDisable();

static ALWAYS_INLINE void MISC_WatchdogTimerReset();


NO_RETURN void MISC_ResetMCU();
NO_RETURN void MISC_ResetMCUOnError();

// get all reset reason flags from last reset, may indicate several accumulated reasons!
// does cache reason beyond an reset of the register in the MCU!
ResetReason MISC_GetResetReason();

// clears reset reason in the MCU register
// preserves (cached) reset reason reported by MISC_GetResetReason()!!
void MISC_ClearResetReason();

// inline implementations

void MISC_WatchdogTimerReset()
{
  wdt_reset();
}


// simple async timer

void MISC_StartTimer(uint8_t seconds);
void MISC_ClearTimer();
Bool MISC_HasTimerElapsed();

#endif /* MISC_H_ */