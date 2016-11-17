/*
 * CommandInterface.c
 *
 * Created: 07.06.2016 08:04:44
 *  Author: Berti
 */ 

#include "CommandInterface.h"

#include <stdlib.h>
#include <assert.h>

#include "Types.h"
#include "Usart.h"
#include "Misc.h"
#include "Configuration.h"
#include "GeigerTube.h"


typedef Byte Ticket;

typedef enum {CmdRes_OK = 1, CmdRes_Failed, CmdRes_Unknown, CmdRes_InvalidTicket, CmdRes_InvalidParameter} CommandResult;

typedef enum {
               Cmd_RFU0 = 0, 

               Cmd_GetStatus = 1,     // returns device state and resets monotonic ticket counter
               Cmd_GetConfiguration,  // returns device configuration
               Cmd_GetResetReason,    // returns device accumulated set of reset reasons and resets them for AFTER the next reset

               Cmd_Reset,             // resets MCU, will NOT send any response!

               Cmd_EnableGeigerTube,
               Cmd_EnableGeigerTubeTestMode,
               Cmd_DisableGeigerTube,

               Cmd_RFU1 = 0xff
             } Command;

typedef struct  
{
  Command m_Command;
  Ticket  m_Ticket;  // monotonic counter from 1 to 0xff!, Cmd_GetStatus resets value!, invalid commands do not increment 
  Byte    m_Param1;  // must be 0 if unused!
  Byte    m_Param2;  // must be 0 if unused!
} CommandData;

typedef struct  
{
  Ticket        m_Ticket;
  CommandResult m_Result;
  uint16_t      m_ReturnedDataSize;
} CommandRespons;


static NO_INIT CommandData g_CurrentCommand;
static Ticket              g_LastTicket = 0;
static Bool                g_CommandPending = FALSE;


static Bool SendCommandResponse(Ticket ticket, CommandResult result);
static Bool SendCommandResponseWithData(Ticket ticket, CommandResult result, uint16_t size, const void* pData);

static Bool CmdGetStatus();
static Bool CmdGetConfiguration();

static Bool CmdGetResetReason();

static Bool CmdReset();

static Bool EnableGeigerTube();
static Bool EnableGeigerTubeTestMode();
static Bool DisableGeigerTube();


void Command_Initialize()
{
}

void Command_DoWork()
{
  // fetch new command if no pending command
  if (!g_CommandPending)
  {
    if (USART_GetAvailableDataSize() < sizeof(g_CurrentCommand))
    {
      return;  
    }

    // get command data
    USART_Receive(sizeof(g_CurrentCommand), &g_CurrentCommand);
  }  // if (!g_CommandPending)

  // validate command code
  switch (g_CurrentCommand.m_Command)
  {
    case Cmd_GetStatus:
    case Cmd_GetConfiguration:
    case Cmd_GetResetReason:

    case Cmd_Reset:

    case Cmd_EnableGeigerTube:
    case Cmd_EnableGeigerTubeTestMode:
    case Cmd_DisableGeigerTube:
      break;

    default:
      g_CommandPending = !SendCommandResponse(g_CurrentCommand.m_Ticket, CmdRes_Unknown);
      return;
  }

  // validate ticket
  if ((g_CurrentCommand.m_Ticket == 0) || 
      ((g_CurrentCommand.m_Ticket != (g_LastTicket + 1)) && (g_CurrentCommand.m_Command != Cmd_GetStatus)))
  {
    g_CommandPending = !SendCommandResponse(g_CurrentCommand.m_Ticket, CmdRes_InvalidTicket);
    return;
  }

  g_LastTicket = g_CurrentCommand.m_Ticket;

  switch (g_CurrentCommand.m_Command)
  {
    case Cmd_GetStatus:
      g_CommandPending = !CmdGetStatus();
      break;

    case Cmd_GetConfiguration:
      g_CommandPending = !CmdGetConfiguration();
      break;

    case Cmd_GetResetReason:
      g_CommandPending = !CmdGetResetReason();
      break;


    case Cmd_Reset:
      g_CommandPending = !CmdReset();
      break;


    case Cmd_EnableGeigerTube:
      g_CommandPending = !EnableGeigerTube();
      break;

    case Cmd_EnableGeigerTubeTestMode:
      g_CommandPending = !EnableGeigerTubeTestMode();
      break;

    case Cmd_DisableGeigerTube:
      g_CommandPending = !DisableGeigerTube();
      break;

    default:  // this should never happen!
      g_CommandPending = !SendCommandResponse(g_CurrentCommand.m_Ticket, CmdRes_Unknown);
      return;      
  }
}

Bool CmdGetStatus()
{
  if ((g_CurrentCommand.m_Param1 != 0) || (g_CurrentCommand.m_Param2 != 0))
  {
    return SendCommandResponse(g_CurrentCommand.m_Ticket, CmdRes_InvalidParameter);
  }
  else
  {    
    return SendCommandResponseWithData(g_CurrentCommand.m_Ticket, CmdRes_OK, sizeof(*g_pState), g_pState);
  }
}

Bool CmdGetConfiguration()
{
  if ((g_CurrentCommand.m_Param1 != 0) || (g_CurrentCommand.m_Param2 != 0))
  {
    return SendCommandResponse(g_CurrentCommand.m_Ticket, CmdRes_InvalidParameter);
  }
  else
  {
    return SendCommandResponseWithData(g_CurrentCommand.m_Ticket, CmdRes_OK, sizeof(*g_pConfiguration), g_pConfiguration);
  }
}

Bool CmdGetResetReason()
{
  if ((g_CurrentCommand.m_Param1 != 0) || (g_CurrentCommand.m_Param2 != 0))
  {
    return SendCommandResponse(g_CurrentCommand.m_Ticket, CmdRes_InvalidParameter);
  }
  else
  {
    ResetReason reason = MISC_GetResetReason();

    Bool result = SendCommandResponseWithData(g_CurrentCommand.m_Ticket, CmdRes_OK, sizeof(reason), &reason);

    if (result)
    {
      MISC_ClearResetReason();
    }

    return result;
  }
}

Bool CmdReset()
{
  if ((g_CurrentCommand.m_Param1 != 0) || (g_CurrentCommand.m_Param2 != 0))
  {
    return SendCommandResponse(g_CurrentCommand.m_Ticket, CmdRes_InvalidParameter);
  }
  else
  {
    if (SendCommandResponse(g_CurrentCommand.m_Ticket, CmdRes_OK))
    {
      // wait for all TX data to be sent out of the UART
      USART_PurgeTXBuffer();

      // do RESET, does not return!
      MISC_ResetMCU();

      return TRUE;
    }
    else
    {
      return FALSE;
    }
  }
}

Bool EnableGeigerTube()
{
  if ((g_CurrentCommand.m_Param1 != 0) || (g_CurrentCommand.m_Param2 != 0))
  {
    return SendCommandResponse(g_CurrentCommand.m_Ticket, CmdRes_InvalidParameter);
  }
  else
  {
    if (SendCommandResponse(g_CurrentCommand.m_Ticket, CmdRes_OK))
    {
      GeigerTube_Enable();
      return TRUE;
    }
    else
    {
      return FALSE;
    }
  }
}

Bool EnableGeigerTubeTestMode()
{
  if ((g_CurrentCommand.m_Param1 != 0) || (g_CurrentCommand.m_Param2 != 0))
  {
    return SendCommandResponse(g_CurrentCommand.m_Ticket, CmdRes_InvalidParameter);
  }
  else
  {
    if (SendCommandResponse(g_CurrentCommand.m_Ticket, CmdRes_OK))
    {
      GeigerTube_EnableTestMode();
      return TRUE;
    }
    else
    {
      return FALSE;
    }
  }
}

Bool DisableGeigerTube()
{
  if ((g_CurrentCommand.m_Param1 != 0) || (g_CurrentCommand.m_Param2 != 0))
  {
    return SendCommandResponse(g_CurrentCommand.m_Ticket, CmdRes_InvalidParameter);
  }
  else
  {
    if (SendCommandResponse(g_CurrentCommand.m_Ticket, CmdRes_OK))
    {
      GeigerTube_Disable();
      return TRUE;
    }
    else
    {
      return FALSE;
    }
  }
}


Bool SendCommandResponse(Ticket ticket, CommandResult result)
{
  return SendCommandResponseWithData(ticket, result, 0, NULL);
}

Bool SendCommandResponseWithData(Ticket ticket, CommandResult result, uint16_t size, const void* pData)
{
  CommandRespons respons = {.m_Ticket = ticket, .m_Result = result, .m_ReturnedDataSize = size};

  assert(((size == 0) && (pData == NULL)) || 
         ((size > 0)  && (pData != NULL)));

  if (USART_GetFreeTXBufferSize() < (sizeof(respons) + size))
  {
    return FALSE;
  }

  USART_Send(sizeof(respons), &respons);

  if (size > 0)
  {    
    USART_Send(size, pData);
  }

  return TRUE;
}

