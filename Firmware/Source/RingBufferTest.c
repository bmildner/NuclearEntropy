/*
 * RingBufferTest.c
 *
 * Created: 30.05.2016 21:53:57
 *  Author: Berti
 */ 

 #include "RingBuffer.h"


 Bool RingBufferTest()
 {
  {
 #define BUFFER_SIZE 8
    Byte bufferMemory[sizeof(RingBuffer) + BUFFER_SIZE];
    RingBuffer* pBuffer = (RingBuffer*) bufferMemory;

    RingBuffer_Init(pBuffer, BUFFER_SIZE);
 
    UNITTEST_ASSERT(RingBuffer_GetSize_Inl(pBuffer) == BUFFER_SIZE);
    UNITTEST_ASSERT(RingBuffer_GetUsedSize_Inl(pBuffer) == 0);
    UNITTEST_ASSERT(RingBuffer_GetFreeSize_Inl(pBuffer) == BUFFER_SIZE);

    RingBuffer_Put_Inl(pBuffer, 0xab);

    UNITTEST_ASSERT(RingBuffer_GetSize_Inl(pBuffer) == BUFFER_SIZE);
    UNITTEST_ASSERT(RingBuffer_GetUsedSize_Inl(pBuffer) == 1);
    UNITTEST_ASSERT(RingBuffer_GetFreeSize_Inl(pBuffer) == (BUFFER_SIZE - 1));

    Byte byte;

    UNITTEST_ASSERT(RingBuffer_CheckedGet(pBuffer, &byte));
    UNITTEST_ASSERT(byte == 0xab);

    UNITTEST_ASSERT(RingBuffer_GetSize_Inl(pBuffer) == BUFFER_SIZE);
    UNITTEST_ASSERT(RingBuffer_GetUsedSize_Inl(pBuffer) == 0);
    UNITTEST_ASSERT(RingBuffer_GetFreeSize_Inl(pBuffer) == BUFFER_SIZE);

    UNITTEST_ASSERT(RingBuffer_Put(pBuffer, 0xef));

    UNITTEST_ASSERT(RingBuffer_GetSize_Inl(pBuffer) == BUFFER_SIZE);
    UNITTEST_ASSERT(RingBuffer_GetUsedSize_Inl(pBuffer) == 1);
    UNITTEST_ASSERT(RingBuffer_GetFreeSize_Inl(pBuffer) == (BUFFER_SIZE - 1));

    UNITTEST_ASSERT(RingBuffer_GetN_Locked(pBuffer, 1, &byte) == 1);
    UNITTEST_ASSERT(byte == 0xef);

    UNITTEST_ASSERT(RingBuffer_GetSize_Inl(pBuffer) == BUFFER_SIZE);
    UNITTEST_ASSERT(RingBuffer_GetUsedSize_Inl(pBuffer) == 0);
    UNITTEST_ASSERT(RingBuffer_GetFreeSize_Inl(pBuffer) == BUFFER_SIZE);

    UNITTEST_ASSERT(RingBuffer_Put(pBuffer, 0xab));
  }

  {
#undef BUFFER_SIZE
#define BUFFER_SIZE 32

    Byte data[BUFFER_SIZE];

    Byte bufferMemory[sizeof(RingBuffer) + BUFFER_SIZE];
    RingBuffer* pBuffer = (RingBuffer*) bufferMemory;

    RingBuffer_Init(pBuffer, BUFFER_SIZE);
     
    UNITTEST_ASSERT(RingBuffer_GetSize_Inl(pBuffer) == BUFFER_SIZE);
    UNITTEST_ASSERT(RingBuffer_GetUsedSize_Inl(pBuffer) == 0);
    UNITTEST_ASSERT(RingBuffer_GetFreeSize_Inl(pBuffer) == BUFFER_SIZE);

    for (uint16_t i = 0; i < BUFFER_SIZE; i++)
    {
      UNITTEST_ASSERT(RingBuffer_Put(pBuffer, (Byte)i));
    }

    UNITTEST_ASSERT(RingBuffer_GetSize_Inl(pBuffer) == BUFFER_SIZE);
    UNITTEST_ASSERT(RingBuffer_GetUsedSize_Inl(pBuffer) == BUFFER_SIZE);
    UNITTEST_ASSERT(RingBuffer_GetFreeSize_Inl(pBuffer) == 0);

    UNITTEST_ASSERT(!RingBuffer_Put(pBuffer, 0xff));

    UNITTEST_ASSERT(RingBuffer_GetSize_Inl(pBuffer) == BUFFER_SIZE);
    UNITTEST_ASSERT(RingBuffer_GetUsedSize_Inl(pBuffer) == BUFFER_SIZE);
    UNITTEST_ASSERT(RingBuffer_GetFreeSize_Inl(pBuffer) == 0);

    UNITTEST_ASSERT(RingBuffer_GetN_Locked(pBuffer, BUFFER_SIZE, data) == BUFFER_SIZE);

    UNITTEST_ASSERT(RingBuffer_GetSize_Inl(pBuffer) == BUFFER_SIZE);
    UNITTEST_ASSERT(RingBuffer_GetUsedSize_Inl(pBuffer) == 0);
    UNITTEST_ASSERT(RingBuffer_GetFreeSize_Inl(pBuffer) == BUFFER_SIZE);

    for (uint16_t i = 0; i < BUFFER_SIZE; i++)
    {
      UNITTEST_ASSERT(data[i] == (Byte)i);
    }

    UNITTEST_ASSERT(RingBuffer_PutN_Locked(pBuffer, BUFFER_SIZE, data) == BUFFER_SIZE);

    UNITTEST_ASSERT(RingBuffer_GetSize_Inl(pBuffer) == BUFFER_SIZE);
    UNITTEST_ASSERT(RingBuffer_GetUsedSize_Inl(pBuffer) == BUFFER_SIZE);
    UNITTEST_ASSERT(RingBuffer_GetFreeSize_Inl(pBuffer) == 0);

    UNITTEST_ASSERT(!RingBuffer_Put_Inl(pBuffer, 0xff));

    UNITTEST_ASSERT(RingBuffer_GetSize_Inl(pBuffer) == BUFFER_SIZE);
    UNITTEST_ASSERT(RingBuffer_GetUsedSize_Inl(pBuffer) == BUFFER_SIZE);
    UNITTEST_ASSERT(RingBuffer_GetFreeSize_Inl(pBuffer) == 0);

    for (uint16_t i = 0; i < BUFFER_SIZE; i++)
    {
      UNITTEST_ASSERT(RingBuffer_Get_Inl(pBuffer) == data[i]);

      UNITTEST_ASSERT(RingBuffer_GetSize_Inl(pBuffer) == BUFFER_SIZE);
      UNITTEST_ASSERT(RingBuffer_GetUsedSize_Inl(pBuffer) == (BUFFER_SIZE - (i + 1)));
      UNITTEST_ASSERT(RingBuffer_GetFreeSize_Inl(pBuffer) == (i + 1));
    }

    UNITTEST_ASSERT(RingBuffer_GetSize_Inl(pBuffer) == BUFFER_SIZE);
    UNITTEST_ASSERT(RingBuffer_GetUsedSize_Inl(pBuffer) == 0);
    UNITTEST_ASSERT(RingBuffer_GetFreeSize_Inl(pBuffer) == BUFFER_SIZE);
  }

  return TRUE;
}