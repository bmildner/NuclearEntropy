/*
 * USART.h
 *
 * Created: 30.05.2016 23:05:10
 *  Author: Berti
 */ 

#ifndef USART_H_
#define USART_H_

#include "Types.h"


void USART_Initialize();

void USART_Send(uint16_t count, const void* pData);

uint16_t USART_Receive(uint16_t count, void* pData);

uint16_t USART_GetAvailableDataSize();

uint16_t USART_GetFreeTXBufferSize();

void USART_SetDSR();
void USART_ClearDSR();

void USART_SetDCD();
void USART_ClearDCD();

void USART_SetRI();
void USART_ClearRI();

#endif /* USART_H_ */
