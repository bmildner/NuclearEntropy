/*
 * GeigerTube.c
 *
 * Created: 31.05.2016 11:43:21
 *  Author: Berti
 */ 

 #include "GeigerTube.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include "Types.h"
#include "RingBuffer.h"
#include "Configuration.h"
#include "Usart.h"


#ifndef ICIE3  // seems to be missing ...
# define ICIE3 ICIE1
#else
# error remove workaround!
#endif

#ifndef ICIE4  // seems to be missing ...
# define ICIE4 ICIE3
#else
# error remove workaround!
#endif

#ifndef CS40  // seems to be missing ...
# define  CS40 CS30
#else
# error remove workaround!
#endif

#ifndef CS41  // seems to be missing ...
# define  CS41 CS31
#else
# error remove workaround!
#endif

#ifndef CS42  // seems to be missing ...
# define  CS42 CS32
#else
# error remove workaround!
#endif


#define PORT_HVEN PORTB
#define DDR_HVEN  DDRB
#define PIN_HVEN  PINB
#define HVEN      PINB1

#define PORT_TESTSIGNAL PORTD
#define DDR_TESTSIGNAL  DDRD
#define PIN_TESTSIGNAL  PIND
#define TESTSIGNAL      PIND6


typedef enum {Off, On, TestMode} State;

typedef struct
{
  uint16_t m_InputCaptureValue;
  uint16_t m_CounterValue;
  uint16_t m_OverflowConter;
} TimeStamp;


#define TIME_STAMP_BUFFER_SIZE      5
#define TIME_STAMP_BUFFER_BYTE_SIZE (sizeof(TimeStamp) * TIME_STAMP_BUFFER_SIZE)

static NO_INIT Byte g_TimeStampBufferMemory1[sizeof(RingBuffer) + TIME_STAMP_BUFFER_BYTE_SIZE];
static RingBuffer* g_TimeStampBuffer1 = (RingBuffer*) g_TimeStampBufferMemory1;

static NO_INIT Byte g_TimeStampBufferMemory2[sizeof(RingBuffer) + TIME_STAMP_BUFFER_BYTE_SIZE];
static RingBuffer* g_TimeStampBuffer2 = (RingBuffer*) g_TimeStampBufferMemory2;

static NO_INIT Byte g_TimeStampBufferMemory3[sizeof(RingBuffer) + TIME_STAMP_BUFFER_BYTE_SIZE];
static RingBuffer* g_TimeStampBuffer3 = (RingBuffer*) g_TimeStampBufferMemory3;


static uint16_t g_Tube1OverflowCount = 0;
static uint16_t g_Tube2OverflowCount = 0;
static uint16_t g_Tube3OverflowCount = 0;

// TODO: add buffer overflow handling
static Bool g_Tube1BufferOverflow = FALSE;
static Bool g_Tube2BufferOverflow = FALSE;
static Bool g_Tube3BufferOverflow = FALSE;

static State g_State = Off;


static void Enable(Bool testMode);


void GeigerTube_Initialize()
{
  // initialize time stamp buffers
  RingBuffer_Init(g_TimeStampBuffer1, TIME_STAMP_BUFFER_BYTE_SIZE);
  RingBuffer_Init(g_TimeStampBuffer2, TIME_STAMP_BUFFER_BYTE_SIZE);
  RingBuffer_Init(g_TimeStampBuffer3, TIME_STAMP_BUFFER_BYTE_SIZE);

  // initialize HV_EN pin (PB1, output, active low)
  PORT_HVEN |= (1 << HVEN);
  DDR_HVEN |= (1 << HVEN);

  // initialize TESTSIGNAL pin (PD6, OC0A, output, active high)
  PORT_TESTSIGNAL &= ~(1 << TESTSIGNAL);
  DDR_TESTSIGNAL |= (1 << TESTSIGNAL);

  // setup test signal on pin OC0A (166,66 Hz, duty-cycle 50:50)
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
  TCNT1 = 0x0000;                 // reset counter values
  g_Tube1OverflowCount = 0x0000;  // reset overflow counter
  g_Tube1BufferOverflow = FALSE;  // reset buffer overflow flag
  TIFR1 = 0xff;                   // reset interrupt flags

  TIMSK1 |= (1 << ICIE1) | (1 << TOIE1);  // enable Input Capture Interrupt and Input Capture Interrupt
  TCCR1B |= (1 << CS10);                  // enable timer clocks with no prescaler

  // Tube 2 / Timer 3
  if (g_pConfiguration->m_Tube2Enabled)
  {
    TCNT3 = 0x0000;                 // reset counter values
    g_Tube2OverflowCount = 0x0000;  // reset overflow counter
    g_Tube2BufferOverflow = FALSE;  // reset buffer overflow flag
    TIFR3 = 0xff;                   // reset interrupt flags

    TIMSK3 |= (1 << ICIE3) | (1 << TOIE3);  // enable Input Capture Interrupt and Input Capture Interrupt
    TCCR3B |= (1 << CS30);                  // enable timer clocks with no prescaler
  }

  // Tube 3 / Timer 4
  if (g_pConfiguration->m_Tube3Enabled)
  {
    TCNT4 = 0x0000;                 // reset counter values
    g_Tube3OverflowCount = 0x0000;  // reset overflow counter
    g_Tube3BufferOverflow = FALSE;  // reset buffer overflow flag
    TIFR4 = 0xff;                   // reset interrupt flags

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

  typedef struct  
  {
    uint8_t   m_Tube;
    TimeStamp m_TimeStamp;
  } RawData;

  RawData rawData;

  if ((RingBuffer_GetUsedSize_Locked(g_TimeStampBuffer1) >= sizeof(TimeStamp)) &&
      (USART_GetFreeTXBufferSize() >= sizeof(RawData)))
  {
    rawData.m_Tube = 1;

    RingBuffer_GetN_Locked(g_TimeStampBuffer1, sizeof(rawData.m_TimeStamp), &rawData.m_TimeStamp);

    USART_Send(sizeof(rawData), &rawData);
  }

  if ((g_pConfiguration->m_Tube2Enabled) &&
      (RingBuffer_GetUsedSize_Locked(g_TimeStampBuffer2) >= sizeof(TimeStamp)) &&
      (USART_GetFreeTXBufferSize() >= sizeof(RawData)))
  {
    rawData.m_Tube = 2;

    RingBuffer_GetN_Locked(g_TimeStampBuffer2, sizeof(rawData.m_TimeStamp), &rawData.m_TimeStamp);

    USART_Send(sizeof(rawData), &rawData);
  }

  if ((g_pConfiguration->m_Tube3Enabled) &&
  (RingBuffer_GetUsedSize_Locked(g_TimeStampBuffer3) >= sizeof(TimeStamp)) &&
  (USART_GetFreeTXBufferSize() >= sizeof(RawData)))
  {
    rawData.m_Tube = 2;

    RingBuffer_GetN_Locked(g_TimeStampBuffer3, sizeof(rawData.m_TimeStamp), &rawData.m_TimeStamp);

    USART_Send(sizeof(rawData), &rawData);
  }
}


ISR(TIMER1_CAPT_vect)
{
  TimeStamp timeStamp;

  timeStamp.m_InputCaptureValue = ICR1;
  timeStamp.m_CounterValue = TCNT1;
  timeStamp.m_OverflowConter = g_Tube1OverflowCount;

  g_Tube1BufferOverflow = !RingBuffer_PutN_Inl(g_TimeStampBuffer1, sizeof(timeStamp), &timeStamp);
}

ISR(TIMER1_OVF_vect)
{
  g_Tube1OverflowCount++;
}


ISR(TIMER3_CAPT_vect)
{
  TimeStamp timeStamp;

  timeStamp.m_InputCaptureValue = ICR3;
  timeStamp.m_CounterValue = TCNT3;
  timeStamp.m_OverflowConter = g_Tube2OverflowCount;

  g_Tube2BufferOverflow = !RingBuffer_PutN_Inl(g_TimeStampBuffer2, sizeof(timeStamp), &timeStamp);
}

ISR(TIMER3_OVF_vect)
{
  g_Tube2OverflowCount++;
}


ISR(TIMER4_CAPT_vect)
{
  TimeStamp timeStamp;

  timeStamp.m_InputCaptureValue = ICR4;
  timeStamp.m_CounterValue = TCNT4;
  timeStamp.m_OverflowConter = g_Tube3OverflowCount;

  g_Tube3BufferOverflow = !RingBuffer_PutN_Inl(g_TimeStampBuffer3, sizeof(timeStamp), &timeStamp);
}

ISR(TIMER4_OVF_vect)
{
  g_Tube3OverflowCount++;
}

