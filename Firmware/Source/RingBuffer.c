/*
 * RingBuffer.c
 *
 * Created: 01.05.2016 22:57:15
 *  Author: Berti
 */ 

#include "RingBuffer.h"

#include <string.h>
#include <assert.h>

#include <util/atomic.h>


void RingBuffer_Init(RingBuffer* pRingBuffer, uint16_t size)
{
  assert(pRingBuffer != NULL);

  memset(pRingBuffer, 0x00, sizeof(RingBuffer));

  pRingBuffer->m_Size = size;
}

uint16_t RingBuffer_GetUsedSize_Locked(const RingBuffer* pRingBuffer)
{
  uint16_t size;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    size = RingBuffer_GetUsedSize_Inl(pRingBuffer);
  }

  return size;
}

uint16_t RingBuffer_GetFreeSize_Locked(const RingBuffer* pRingBuffer)
{
  uint16_t size;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    size = RingBuffer_GetFreeSize_Inl(pRingBuffer);
  }

  return size;
}

Bool RingBuffer_Put(RingBuffer* pRingBuffer, Byte byte)
{
  return RingBuffer_Put_Inl(pRingBuffer, byte);
}

uint16_t RingBuffer_PutN_Locked(RingBuffer* pRingBuffer, uint16_t count, const void* pBuffer)
{
  uint16_t done = 0;

  while (done < count)
  {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      if (!RingBuffer_Put_Inl(pRingBuffer, ((const Byte*)pBuffer)[done]))
      {
        break;
      }
    }
    
    done++;
  }

  return done;
}

Byte RingBuffer_Get(RingBuffer* pRingBuffer)
{
  return RingBuffer_Get_Inl(pRingBuffer);
}

Bool RingBuffer_CheckedGet(RingBuffer* pRingBuffer, Byte* pByte)
{
  return RingBuffer_CheckedGet_Inl(pRingBuffer, pByte);
}

uint16_t RingBuffer_GetN_Locked(RingBuffer* pRingBuffer, uint16_t count, void* pBuffer)
{
  uint16_t available;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    available = RingBuffer_GetUsedSize_Inl(pRingBuffer);
  }  

  if (available < count)
  {
    count = available;
  }

  for (uint16_t i = 0; i < count; i++)
  {
    Byte byte;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      byte = RingBuffer_Get_Inl(pRingBuffer);
    }

    ((Byte*)pBuffer)[i] = byte;
  }

  return count;
}