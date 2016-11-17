/*
 * Configuration.h
 *
 * Created: 01.06.2016 11:38:36
 *  Author: Berti
 */ 

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include "Types.h"

#define MAX_NUMBER_OF_TUBES 3

typedef enum {Config_Empty = 0, Config_Read_OK, Config_Read_Fallback, Config_Read_Bad} ConfigurationState;

typedef struct  
{
  ConfigurationState m_State;

  Bool m_IsTubeEnabled[MAX_NUMBER_OF_TUBES];
} Configuration;


extern const Configuration* g_pConfiguration;


void Configuration_Read();

Bool Configuration_Set(const Configuration* pNewConfig);

#endif /* CONFIGURATION_H_ */
