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
extern Bool* volatile        g_pStateChanged;


void MISC_Initialize();

void MISC_StatusLEDEnable();
void MISC_StatusLEDDisable();

static ALWAYS_INLINE void MISC_WatchdogReset();


ResetReason MISC_GetResetReason();


// inline implementations

void MISC_WatchdogReset()
{
  wdt_reset();
}

#endif /* MISC_H_ */