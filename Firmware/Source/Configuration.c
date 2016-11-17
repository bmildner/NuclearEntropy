/*
 * Configuration.c
 *
 * Created: 01.06.2016 11:38:26
 *  Author: Berti
 */ 

#include "Configuration.h"

#include <string.h>
#include <assert.h>

#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/crc16.h>


#define BACKUP_CONFIG_OFFSET (uint8_t*)(E2SIZE / 2)


typedef uint8_t  Version;
typedef uint16_t CRC;

#define CONFIG_VERSION_1    1
#define MAX_NUMBER_OF_TUBES_VERSION_1 3

typedef struct  
{  
  Version m_Version;

  Bool m_IsTubeEnabled[MAX_NUMBER_OF_TUBES_VERSION_1];

  CRC     m_CRC;
} RawConfiguration_V1;


static NO_INIT Configuration g_Config;
const Configuration* g_pConfiguration = &g_Config;


static void SetDefaultValues(Configuration* pConfig);

static Configuration ReadConfigV1(const uint8_t* offset);
static Bool WriteConfigV1(uint8_t* offset, const Configuration* pNewConfig);

static CRC CalcCRCV1(const RawConfiguration_V1* pRawConfig);
static Bool VerifyCRCV1(const RawConfiguration_V1* pRawConfig);


void Configuration_Read()
{
  // read version of main and backup config
  Version mainVersion = eeprom_read_byte(0);
  Version bakVersion =  eeprom_read_byte(BACKUP_CONFIG_OFFSET);

  SetDefaultValues(&g_Config);

  if ((mainVersion == 0xff) && (bakVersion == 0xff))  // assume cleared EEPROM
  {
    g_Config.m_State = Config_Empty;

    return;
  }

  Configuration mainConfig;
  Configuration bakConfig;

  switch (mainVersion)
  {
    case CONFIG_VERSION_1:
        mainConfig = ReadConfigV1(0);
      break;

    default:
      mainConfig.m_State = Config_Read_Bad;
  }

  switch (bakVersion)
  {
    case CONFIG_VERSION_1:
      bakConfig = ReadConfigV1(BACKUP_CONFIG_OFFSET);
    break;

    default:
      bakConfig.m_State = Config_Read_Bad;
  }

  if (mainConfig.m_State == Config_Read_OK)
  {
    g_Config = mainConfig;
  }
  else if (bakConfig.m_State == Config_Read_OK)
  {
    g_Config = bakConfig;
    g_Config.m_State = Config_Read_Fallback;
  }
  else
  {
    g_Config.m_State = Config_Read_Bad;
  }
}

Bool Configuration_Set(const Configuration* pNewConfig)
{
  if (WriteConfigV1(0, pNewConfig))
  {
    WriteConfigV1(BACKUP_CONFIG_OFFSET, pNewConfig);
    return TRUE;
  }

  return FALSE;
}


void SetDefaultValues(Configuration* pConfig)
{
  for (uint8_t count = 0; count < MAX_NUMBER_OF_TUBES; count++)
  {
    pConfig->m_IsTubeEnabled[count] = TRUE;
  }
}


Configuration ReadConfigV1(const uint8_t* offset)
{
  Configuration       config;
  RawConfiguration_V1 rawConfig;

  // read
  eeprom_read_block(&rawConfig, offset, sizeof(RawConfiguration_V1));

  if (VerifyCRCV1(&rawConfig))
  {
    config.m_State = Config_Read_OK;

    assert(((sizeof(config.m_IsTubeEnabled) == sizeof(rawConfig.m_IsTubeEnabled)) == MAX_NUMBER_OF_TUBES) == MAX_NUMBER_OF_TUBES_VERSION_1);

    memcpy(config.m_IsTubeEnabled, rawConfig.m_IsTubeEnabled, sizeof(config.m_IsTubeEnabled));
  }
  else
  {
    config.m_State = Config_Read_Bad;
  }

  return config;
}

Bool WriteConfigV1(uint8_t* offset, const Configuration* pNewConfig)
{
  RawConfiguration_V1 rawConfig;

  rawConfig.m_Version = CONFIG_VERSION_1;

  assert(((sizeof(rawConfig.m_IsTubeEnabled) == sizeof(pNewConfig->m_IsTubeEnabled)) == MAX_NUMBER_OF_TUBES) == MAX_NUMBER_OF_TUBES_VERSION_1);
  memcpy(rawConfig.m_IsTubeEnabled, pNewConfig->m_IsTubeEnabled, sizeof(rawConfig.m_IsTubeEnabled));

  rawConfig.m_CRC = CalcCRCV1(&rawConfig);

  eeprom_write_block(&rawConfig, offset, sizeof(rawConfig));

  Byte buffer[sizeof(rawConfig)];

  eeprom_read_block(buffer, offset, sizeof(buffer));

  return (memcmp(&rawConfig, buffer, sizeof(rawConfig)) == 0);
}

CRC CalcCRCV1(const RawConfiguration_V1* pRawConfig)
{
  CRC crc = 0;
  const Byte* pRaw = (const Byte*) pRawConfig;

  for (uint16_t i = 0; i < (sizeof(RawConfiguration_V1) - sizeof(crc)); i++)
  {
    crc = _crc16_update(crc, pRaw[i]);
  }

  return crc;
}

Bool VerifyCRCV1(const RawConfiguration_V1* pRawConfig)
{  
  return (pRawConfig->m_CRC == CalcCRCV1(pRawConfig));
}

