/*
 * RingBuffer.h
 *
 * Created: 01.05.2016 22:43:49
 *  Author: Berti
 */ 


#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#include <stdlib.h>

#include "Types.h"


typedef struct  
{
  uint16_t m_Size;
  uint16_t m_Head;
  uint16_t m_Tail;
  uint16_t m_UsedSize;
  Byte     m_Data[];
} RingBuffer;



void RingBuffer_Init(RingBuffer* pRingBuffer, uint16_t size);

static ALWAYS_INLINE uint16_t RingBuffer_GetSize_Inl(const RingBuffer* pRingBuffer);
static ALWAYS_INLINE uint16_t RingBuffer_GetUsedSize_Inl(const RingBuffer* pRingBuffer);
static ALWAYS_INLINE uint16_t RingBuffer_GetFreeSize_Inl(const RingBuffer* pRingBuffer);

uint16_t RingBuffer_GetUsedSize_Locked(const RingBuffer* pRingBuffer);
uint16_t RingBuffer_GetFreeSize_Locked(const RingBuffer* pRingBuffer);


static ALWAYS_INLINE Bool RingBuffer_Put_Inl(RingBuffer* pRingBuffer, Byte byte);
Bool RingBuffer_Put(RingBuffer* pRingBuffer, Byte byte);

static ALWAYS_INLINE Bool RingBuffer_PutN_Inl(RingBuffer* pRingBuffer, uint16_t count, const void* pBuffer);

uint16_t RingBuffer_PutN_Locked(RingBuffer* pRingBuffer, uint16_t count, const void* pBuffer);

static ALWAYS_INLINE Byte RingBuffer_Get_Inl(RingBuffer* pRingBuffer);
Byte RingBuffer_Get(RingBuffer* pRingBuffer);

static ALWAYS_INLINE uint16_t RingBuffer_GetN_Inl(RingBuffer* pRingBuffer, uint16_t count, void* pBuffer);
uint16_t RingBuffer_GetN_Locked(RingBuffer* pRingBuffer, uint16_t count, void* pBuffer);

static ALWAYS_INLINE Bool RingBuffer_CheckedGet_Inl(RingBuffer* pRingBuffer, Byte* pByte);
Bool RingBuffer_CheckedGet(RingBuffer* pRingBuffer, Byte* pByte);


Bool RingBufferTest();


/* inline implementations */

uint16_t RingBuffer_GetSize_Inl(const RingBuffer* pRingBuffer)
{
  return pRingBuffer->m_Size;
}

uint16_t RingBuffer_GetUsedSize_Inl(const RingBuffer* pRingBuffer)
{
  return pRingBuffer->m_UsedSize;
}

uint16_t RingBuffer_GetFreeSize_Inl(const RingBuffer* pRingBuffer)
{
  return pRingBuffer->m_Size - pRingBuffer->m_UsedSize;
}



Bool RingBuffer_Put_Inl(RingBuffer* pRingBuffer, Byte byte)
{
  if (pRingBuffer->m_UsedSize >= pRingBuffer->m_Size)
  {
    return FALSE;
  }

  pRingBuffer->m_Data[pRingBuffer->m_Head] = byte;

  pRingBuffer->m_Head++;

  if (pRingBuffer->m_Head >= pRingBuffer->m_Size)
  {
    pRingBuffer->m_Head = 0;
  }

  pRingBuffer->m_UsedSize++;

  return TRUE;
}

Bool RingBuffer_PutN_Inl(RingBuffer* pRingBuffer, uint16_t count, const void* pBuffer)
{
  if (RingBuffer_GetFreeSize_Inl(pRingBuffer) < count)
  {
    return FALSE;
  }

  for (uint16_t i = 0; i < count; i++)
  {
    pRingBuffer->m_Data[pRingBuffer->m_Head] = ((const Byte*)pBuffer)[i];

    pRingBuffer->m_Head++;

    if (pRingBuffer->m_Head >= pRingBuffer->m_Size)
    {
      pRingBuffer->m_Head = 0;
    }    
  }

  pRingBuffer->m_UsedSize += count;

  return TRUE;
}


Byte RingBuffer_Get_Inl(RingBuffer* pRingBuffer)
{
  Byte result = 0x00;

  RingBuffer_CheckedGet_Inl(pRingBuffer, &result);

  return result;
}

Bool RingBuffer_CheckedGet_Inl(RingBuffer* pRingBuffer, Byte* pByte)
{
  if ((pRingBuffer->m_UsedSize < 1) || (pByte == NULL))
  {
    return FALSE;
  }

  *pByte = pRingBuffer->m_Data[pRingBuffer->m_Tail];

  pRingBuffer->m_Tail++;

  if (pRingBuffer->m_Tail >= pRingBuffer->m_Size)
  {
    pRingBuffer->m_Tail = 0;
  }

  pRingBuffer->m_UsedSize--;

  return TRUE;
}

uint16_t RingBuffer_GetN_Inl(RingBuffer* pRingBuffer, uint16_t count, void* pBuffer)
{
  if (pBuffer != NULL)
  {
    count = (pRingBuffer->m_UsedSize < count) ? pRingBuffer->m_UsedSize : count;
  }
  else
  {
    return 0;
  }

  for (uint16_t i = 0; i < count; i++)
  {
    ((Byte*)pBuffer)[i] = pRingBuffer->m_Data[pRingBuffer->m_Tail];
    
    pRingBuffer->m_Tail++;

    if (pRingBuffer->m_Tail >= pRingBuffer->m_Size)
    {
      pRingBuffer->m_Tail = 0;
    }    
  }

  pRingBuffer->m_UsedSize -= count;

  return count;
}

#endif /* RINGBUFFER_H_ */
