/*
 * USART.c
 *
 * Created: 30.05.2016 23:05:00
 *  Author: Berti
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#define BAUD 1152000  // 1.152Mbps
#include <util/setbaud.h>

#include "RingBuffer.h"


#define PORT_RTS PORTD
#define DDR_RTS  DDRD
#define PIN_RTS  PIND
#define RTS      PIND3

#define PORT_CTS PORTD
#define DDR_CTS  DDRD
#define PIN_CTS  PIND
#define CTS      PIND4

#define PORT_DSR PORTB
#define DDR_DSR  DDRB
#define PIN_DSR  PINB
#define DSR      PINB4

#define PORT_DCD PORTB
#define DDR_DCD  DDRB
#define PIN_DCD  PINB
#define DCD      PINB3

#define PORT_RI PORTB
#define DDR_RI  DDRB
#define PIN_RI  PINB
#define RI      PINB5


#define MIN_FREE_RX_BUFFER_SIZE 2

#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 64


static NO_INIT Byte g_RXBufferMemory[sizeof(RingBuffer) + RX_BUFFER_SIZE];
static RingBuffer* g_pRXBuffer = (RingBuffer*) g_RXBufferMemory;

static NO_INIT Byte g_TXBufferMemory[sizeof(RingBuffer) + TX_BUFFER_SIZE];
static RingBuffer* g_pTXBuffer = (RingBuffer*) g_TXBufferMemory;

static volatile Bool g_DisableTX = FALSE;

// TODO: add RX overflow handling!
static volatile Bool g_RXOverflow = FALSE;


void USART_Initialize()
{
  // init buffers
  RingBuffer_Init(g_pRXBuffer, RX_BUFFER_SIZE);
  RingBuffer_Init(g_pTXBuffer, TX_BUFFER_SIZE);

  // setup DSR, DCD and RI (all are inverted output)
  PORT_DSR |= (1 << DSR);
  DDR_DSR  |= (1 << DSR);
  PORT_DCD |= (1 << DCD);
  DDR_DCD  |= (1 << DCD);
  PORT_RI  |= (1 << RI);
  DDR_RI   |= (1 << RI);

  // setup RTS/CTS handling (!RTS input PD3/INT0, !CTS output PD4)
  // setup io-pins
  PORT_RTS &= ~(1 << RTS);
  DDR_RTS &=  ~(1 << RTS);

  PORT_CTS &= ~(1 << CTS);
  DDR_CTS |= (1 << CTS);

  // setup INT0 (any logic change)
  EICRA |= (1 << ISC00);
  EICRA &= ~(1 << ISC01);
  EIMSK |= (1 << INT0);


  // 1.152Mbps @ 18.432 MHz
  UBRR0 = UBRR_VALUE;

  UCSR0A = 0x00;

  #if USE_2X
  UCSR0A |= (1 << U2X0);
  #else
  UCSR0A &= ~(1 << U2X0);
  #endif

  UCSR0D = 0x00;

  // Character Size 8-bit
  UCSR0C = (1 << UCSZ00) | (1 << UCSZ10);

  // RX Complete Interrupt Enable, USART Data Register Empty Interrupt Enable, Receiver Enable, Transmitter Enable
  UCSR0B = (1 << RXCIE0) | (1 << UDRIE0) | (1 << RXEN0) | (1 << TXEN0);
}


uint16_t USART_GetAvailableDataSize()
{
  return RingBuffer_GetUsedSize_Locked(g_pRXBuffer);
}

uint16_t USART_GetFreeTXBufferSize()
{
  return RingBuffer_GetFreeSize_Locked(g_pTXBuffer);
}

void USART_Send(uint16_t count, const void* pData)
{
  uint16_t done = 0;

  do 
  {
    done += RingBuffer_PutN_Locked(g_pTXBuffer, count - done, ((const Byte*)pData) + done);
  
    if (!g_DisableTX && !(UCSR0B & (1 << UDRIE0)))
    {
      // enable USART Data Register Empty Interrupt
      UCSR0B |= (1 << UDRIE0);
    }
  } while (done < count);
}

uint16_t USART_Receive(uint16_t count, void* pData)
{
  return RingBuffer_GetN_Locked(g_pRXBuffer, count, pData);
}

void USART_SetDSR()
{
  PORT_DSR &= ~(1 << DSR);
}

void USART_ClearDSR()
{
  PORT_DSR |= (1 << DSR);
}

void USART_SetDCD()
{
  PORT_DCD &= ~(1 << DCD);
}

void USART_ClearDCD()
{
  PORT_DCD |= (1 << DCD);
}

void USART_SetRI()
{
  PORT_RI &= ~(1 << RI);
}

void USART_ClearRI()
{
  PORT_RI |= (1 << RI);
}


ISR(INT0_vect)
{
  if (PIN_RTS & (1 << RTS))
  {
    // disable TX
    UCSR0B &= ~(1 << UDRIE0);
    g_DisableTX = TRUE;
  }
  else
  {
    // enable TX
    UCSR0B |= (1 << UDRIE0);
    g_DisableTX = FALSE;
  }
}

ISR(USART0_RX_vect)
{
  Byte byte = UDR0;

  uint16_t freeSize = RingBuffer_GetFreeSize_Inl(g_pRXBuffer);

  if (freeSize <= MIN_FREE_RX_BUFFER_SIZE)
  {
    PORT_CTS |= (1 << CTS);
  }

  if (freeSize > 0)
  {
    RingBuffer_Put_Inl(g_pRXBuffer, byte);
  }
  else
  {
    g_RXOverflow = TRUE;
  }
}

ISR(USART0_UDRE_vect)
{
  Byte byte;

  if (RingBuffer_CheckedGet_Inl(g_pTXBuffer, &byte))
  {
    UDR0 = byte;
  }
  else
  {
    UCSR0B &= ~(1 << UDRIE0);
  }
}

