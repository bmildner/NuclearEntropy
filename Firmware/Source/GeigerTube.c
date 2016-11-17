/*
 * GeigerTube.c
 *
 * Created: 31.05.2016 11:43:21
 *  Author: Berti
 */ 

#include "GeigerTube.h"

#include <assert.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/atomic.h>

#include "Types.h"
#include "RingBuffer.h"
#include "Configuration.h"
#include "Usart.h"


#define PORT_HVEN PORTB
#define DDR_HVEN  DDRB
#define PIN_HVEN  PINB
#define HVEN      PINB1

#define PORT_TESTSIGNAL PORTD
#define DDR_TESTSIGNAL  DDRD
#define PIN_TESTSIGNAL  PIND
#define TESTSIGNAL      PIND6


typedef enum {Off, On, TestMode} State;

typedef enum {None = 0, Entropy = 1, Raw = 2, CPS = 4} OutputFormat;


typedef struct
{
  uint16_t m_InputCaptureValue;
  uint16_t m_CounterValue;
  uint16_t m_OverflowConter;
} RawTimeStamp;


#define TIME_STAMP_BUFFER_SIZE      8
#define TIME_STAMP_BUFFER_BYTE_SIZE (sizeof(RawTimeStamp) * TIME_STAMP_BUFFER_SIZE)

static NO_INIT Byte g_TimeStampBufferMemory1[sizeof(RingBuffer) + TIME_STAMP_BUFFER_BYTE_SIZE];
static NO_INIT Byte g_TimeStampBufferMemory2[sizeof(RingBuffer) + TIME_STAMP_BUFFER_BYTE_SIZE];
static NO_INIT Byte g_TimeStampBufferMemory3[sizeof(RingBuffer) + TIME_STAMP_BUFFER_BYTE_SIZE];


typedef struct
{
  const uint8_t m_TubeNumber;
  RingBuffer*   m_RawTimeStampBuffer;
  Bool          m_BufferOverflow;
  uint32_t      m_LastTimestamp;
} TubeState;


static uint16_t g_Tube1OverflowCount = 0;
static uint16_t g_Tube2OverflowCount = 0;
static uint16_t g_Tube3OverflowCount = 0;


static State        g_State        = Off;
static OutputFormat g_OutputFormat = Raw;

static TubeState g_TubeStates[MAX_NUMBER_OF_TUBES] = {{.m_TubeNumber = 1, .m_RawTimeStampBuffer = (RingBuffer*) g_TimeStampBufferMemory1, .m_BufferOverflow = FALSE, .m_LastTimestamp = 0}, 
                                                      {.m_TubeNumber = 2, .m_RawTimeStampBuffer = (RingBuffer*) g_TimeStampBufferMemory2, .m_BufferOverflow = FALSE, .m_LastTimestamp = 0}, 
                                                      {.m_TubeNumber = 3, .m_RawTimeStampBuffer = (RingBuffer*) g_TimeStampBufferMemory3, .m_BufferOverflow = FALSE, .m_LastTimestamp = 0}};


static void Enable(Bool testMode);

static void ProcessTimestamp(TubeState* pTubeState, const RawTimeStamp* pTimeStamp);
static PURE_FUNCTION size_t RequiredUARTBufferSize();


void GeigerTube_Initialize()
{
  // initialize time stamp buffers
  for (uint8_t count = 0; count < MAX_NUMBER_OF_TUBES; count++)
  {
    RingBuffer_Init(g_TubeStates[count].m_RawTimeStampBuffer, TIME_STAMP_BUFFER_BYTE_SIZE);
  }

  // initialize HV_EN pin (PB1, output, active low)
  PORT_HVEN |= (1 << HVEN);
  DDR_HVEN |= (1 << HVEN);

  // initialize TESTSIGNAL pin (PD6, OC0A, output, active high)
  PORT_TESTSIGNAL &= ~(1 << TESTSIGNAL);
  DDR_TESTSIGNAL |= (1 << TESTSIGNAL);

  // TC0, setup test signal on pin OC0A (166,66 Hz, duty-cycle 50:50)
  TCCR0A = (1 << WGM01) | (1 << COM0A0);  // mode 2/CTC, toggle OC0A on compare match
  TCCR0B = 0x00;                          // mode 2/CTC, clock off
  TIMSK0 = 0x00;                          // no interrupts
  OCR0A  = 53;                            // 18432000 / (2 * 1024 * (1 + OCR0A)) = 166,66 Hz

  // tube 1 / TC1
  TCCR1A = 0x00;  // normal mode, no OC
  TCCR1B = 0x00;  // normal mode, Input Capture Noise Canceler OFF, Input Capture Edge Select FALLING edge, clock off  
  TIMSK1 = 0x00;  // no interrupts

  // tube 2 / TC3  
  TCCR3A = 0x00;  // normal mode, no OC
  TCCR3B = 0x00;  // normal mode, Input Capture Noise Canceler OFF, Input Capture Edge Select FALLING edge, clock off  
  TIMSK3 = 0x00;  // no interrupts

  // tube 3 / TC4  
  TCCR4A = 0x00;  // normal mode, no OC
  TCCR4B = 0x00;  // normal mode, Input Capture Noise Canceler OFF, Input Capture Edge Select FALLING edge, clock off  
  TIMSK4 = 0x00;  // no interrupts
}


void GeigerTube_Enable()
{
  Enable(FALSE);
}

void GeigerTube_EnableTestMode()
{
  Enable(TRUE);
}

void Enable(Bool testMode)
{
  if (g_State != Off)
  {
    GeigerTube_Disable();
  }

  // keep prescalers in reset
  GTCCR = (1 << TSM) | (1 << PSRSYNC);

  // Tube 1 / Timer 1
  if (g_pConfiguration->m_IsTubeEnabled[0])
  {
    TCNT1 = 0x0000;                            // reset counter values
    g_Tube1OverflowCount = 0x0000;             // reset overflow counter
    g_TubeStates[0].m_BufferOverflow = FALSE;  // reset buffer overflow flag
    TIFR1 = 0xff;                              // reset interrupt flags

    TIMSK1 |= (1 << ICIE1) | (1 << TOIE1);  // enable Input Capture Interrupt and Input Capture Interrupt
    TCCR1B |= (1 << CS10);                  // enable timer clocks with no prescaler
  }

  // Tube 2 / Timer 3
  if (g_pConfiguration->m_IsTubeEnabled[1])
  {
    TCNT3 = 0x0000;                            // reset counter values
    g_Tube2OverflowCount = 0x0000;             // reset overflow counter
    g_TubeStates[1].m_BufferOverflow = FALSE;  // reset buffer overflow flag
    TIFR3 = 0xff;                              // reset interrupt flags

    TIMSK3 |= (1 << ICIE3) | (1 << TOIE3);  // enable Input Capture Interrupt and Input Capture Interrupt
    TCCR3B |= (1 << CS30);                  // enable timer clocks with no prescaler
  }

  // Tube 3 / Timer 4
  if (g_pConfiguration->m_IsTubeEnabled[2])
  {
    TCNT4 = 0x0000;                            // reset counter values
    g_Tube3OverflowCount = 0x0000;             // reset overflow counter
    g_TubeStates[2].m_BufferOverflow = FALSE;  // reset buffer overflow flag
    TIFR4 = 0xff;                              // reset interrupt flags

    TIMSK4 |= (1 << ICIE4) | (1 << TOIE4);  // enable Input Capture Interrupt and Input Capture Interrupt
    TCCR4B |= (1 << CS40);                  // enable timer clocks with no prescaler
  }

  if (testMode)
  {
    // disable high voltage generator
    PORT_HVEN |= (1 << HVEN);

    // enable test signal on pin OC0A
    TCNT0  = 0x00;                        // reset timer counter
    TCCR0B |= (1 << CS00) | (1 << CS02);  // enable clock with prescaler 1024
    TIMSK0 = 0x00;                        // no interrupts

    g_State = TestMode;
  }
  else
  {
    // enable high voltage generator
    PORT_HVEN &= ~(1 << HVEN);

    g_State = On;
  }

  // release prescalers / timer clocks
  GTCCR &= ~(1 << TSM);
}

void GeigerTube_Disable()
{
  // disable test signal timer clock
  TCCR0B &= ~((1 << CS00) | (1 << CS01) | (1 << CS02));

  // disable interrupts
  TIMSK1 = 0x00;
  TIMSK3 = 0x00;
  TIMSK4 = 0x00;

  // disable timer clocks
  TCCR1B &= ~((1 << CS10) | (1 << CS11) | (1 << CS12));
  TCCR3B &= ~((1 << CS30) | (1 << CS31) | (1 << CS32));
  TCCR4B &= ~((1 << CS40) | (1 << CS41) | (1 << CS42));

  // disable high voltage generator
  PORT_HVEN |= (1 << HVEN);

  g_State = Off;
}

void GeigerTube_DoWork()
{
  if (g_State == Off)
  {
    return;
  }

  RawTimeStamp timeStamp;

  // iterate over all tubes and check if there is a time stamp in the queue (+ check if we can send if needed)
  for (uint8_t count = 0; count < MAX_NUMBER_OF_TUBES; count++)
  {
    if ((g_pConfiguration->m_IsTubeEnabled[count]) &&
        (RingBuffer_GetUsedSize_Locked(g_TubeStates[count].m_RawTimeStampBuffer) >= sizeof(timeStamp)) &&
        (USART_GetFreeTXBufferSize() >= RequiredUARTBufferSize()))
    {
      RingBuffer_GetN_Locked(g_TubeStates[count].m_RawTimeStampBuffer, sizeof(timeStamp), &timeStamp);

      ProcessTimestamp(&g_TubeStates[count], &timeStamp);
    }
  }
}

// hold-off in CPU clock cycles for overflow counter fixup
#define OVERFLOW_FIXUP_HOLDOFF 10  // actually this currently does not account for entering the ISR ...

void ProcessTimestamp(TubeState* pTubeState, const RawTimeStamp* pTimeStamp)
{
  assert(pTubeState != NULL);
  assert(pTimeStamp != NULL);

  uint32_t overflowCount;

  // try to correct for timer overflow between event and ISR execution
  if ((pTimeStamp->m_CounterValue < pTimeStamp->m_InputCaptureValue) && 
      (pTimeStamp->m_CounterValue > OVERFLOW_FIXUP_HOLDOFF))
  {
    overflowCount = pTimeStamp->m_OverflowConter - 1;
  }
  else
  {
    overflowCount = pTimeStamp->m_OverflowConter;
  }

  pTubeState->m_LastTimestamp = (overflowCount * 0xffff) + pTimeStamp->m_InputCaptureValue;


  // send RAW data
  if (g_OutputFormat & Raw)
  {
    Bool overflow;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      overflow = pTubeState->m_BufferOverflow;
      pTubeState->m_BufferOverflow = FALSE;
    }

    USART_SendByte(Raw);
    USART_Send(sizeof(pTubeState->m_TubeNumber), &pTubeState->m_TubeNumber);
    USART_Send(sizeof(overflow), &overflow);
    USART_Send(sizeof(*pTimeStamp), pTimeStamp);
  }
}

size_t RequiredUARTBufferSize()
{
  size_t size = 0;

  if (g_OutputFormat & Raw)
  {
    // type tag + tube number + overflow + timestamp
    size += sizeof(uint8_t) + sizeof(((TubeState*) 0)->m_TubeNumber) + sizeof(Bool) + sizeof(RawTimeStamp);
  }

  return size;
}


ISR(TIMER1_CAPT_vect)
{
  RawTimeStamp timeStamp;

  timeStamp.m_InputCaptureValue = ICR1;
  timeStamp.m_CounterValue = TCNT1;
  timeStamp.m_OverflowConter = g_Tube1OverflowCount;

  if (!RingBuffer_PutN_Inl(g_TubeStates[0].m_RawTimeStampBuffer, sizeof(timeStamp), &timeStamp))
  {
    g_TubeStates[0].m_BufferOverflow = TRUE;
  }
}

ISR(TIMER1_OVF_vect)
{
  g_Tube1OverflowCount++;
}


ISR(TIMER3_CAPT_vect)
{
  RawTimeStamp timeStamp;

  timeStamp.m_InputCaptureValue = ICR3;
  timeStamp.m_CounterValue = TCNT3;
  timeStamp.m_OverflowConter = g_Tube2OverflowCount;

  if (!RingBuffer_PutN_Inl(g_TubeStates[1].m_RawTimeStampBuffer, sizeof(timeStamp), &timeStamp))
  {
    g_TubeStates[1].m_BufferOverflow = TRUE;
  }
}

ISR(TIMER3_OVF_vect)
{
  g_Tube2OverflowCount++;
}


ISR(TIMER4_CAPT_vect)
{
  RawTimeStamp timeStamp;

  timeStamp.m_InputCaptureValue = ICR4;
  timeStamp.m_CounterValue = TCNT4;
  timeStamp.m_OverflowConter = g_Tube3OverflowCount;

  if (!RingBuffer_PutN_Inl(g_TubeStates[2].m_RawTimeStampBuffer, sizeof(timeStamp), &timeStamp))
  {
    g_TubeStates[2].m_BufferOverflow = TRUE;
  }
}

ISR(TIMER4_OVF_vect)
{
  g_Tube3OverflowCount++;
}

