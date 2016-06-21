// Copyright (c) 2012 by Bertolt Mildner
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the copyright holder nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL BERTOLT MILDNER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef AUTOMATEDTOKENTESTDEVICE_ERRORCODES_H
#define AUTOMATEDTOKENTESTDEVICE_ERRORCODES_H

#include "NuclearEntropyCore/Config.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#define ATTD_ERR_OK                               0

#define ATTD_ERR_GENERAL_ERROR                    1
#define ATTD_ERR_FATAL_ERROR                      2
#define ATTD_ERR_INVALID_PARAM                    3
#define ATTD_ERR_BUFFER_OVERFLOW                  4
#define ATTD_ERR_OUT_OF_MEMORY                    5
#define ATTD_ERR_INVALID_HANDLE                   6
#define ATTD_ERR_WRONG_HANDLE_TYPE                7

#define ATTD_ERR_DEVICE_NOT_FOUND               101
#define ATTD_ERR_DEVICE_VERSION_NOT_SUPPORTED   102
#define ATTD_ERR_DEVICE_NO_STATUS_MSG_AVAILABLE 103
#define ATTD_ERR_DEVICE_ALREADY_HAS_CALLBACK    104
#define ATTD_ERR_DEVICE_UNKNOWN_SMARTCARD_READ  105

#define ATTD_ERR_PORT_NOT_SUPPORTED             201
#define ATTD_ERR_PORT_API_ERROR                 202
#define ATTD_ERR_PORT_OPEN_FAILED               203
#define ATTD_ERR_PORT_NOT_FOUND                 204
#define ATTD_ERR_PORT_INVALID_NAME              205
#define ATTD_ERR_PORT_CONFIG_ERROR              206
#define ATTD_ERR_PORT_BREAK_SIGNAL              207
#define ATTD_ERR_PORT_FRAME_ERROR               208
#define ATTD_ERR_PORT_OVERFLOW_ERROR            209
#define ATTD_ERR_PORT_PARITY_ERROR              210
#define ATTD_ERR_PORT_WRITE_ERROR               211
#define ATTD_ERR_PORT_READ_ERROR                212
#define ATTD_ERR_PORT_READ_TIMEOUT              213
#define ATTD_ERR_PORT_WRITE_TIMEOUT             214

#define ATTD_ERR_VKBD_BAD_KEY_MAP               301
#define ATTD_ERR_VKBD_KEY_NOT_FOUND             302
#define ATTD_ERR_VKBD_NOT_SUPPORTED             303
#define ATTD_ERR_VKBD_READ_ERROR                304
#define ATTD_ERR_VKBD_WRITE_ERROR               305

#define ATTD_ERR_STATUSMSG_ENTRY_NOT_FOUND         401
#define ATTD_ERR_STATUSMSG_DATA_TOO_SHORT          402
#define ATTD_ERR_STATUSMSG_DESERIAL_ERROR          403
#define ATTD_ERR_STATUSMSG_BAD_FRAME_FORMAT        404
#define ATTD_ERR_STATUSMSG_BAD_FRAME_CHECKSUM      405
#define ATTD_ERR_STATUSMSG_BAD_FRAME_DATA_ENCODING 406
#define ATTD_ERR_STATUSMSG_WRONG_ENTRY_TYPE        407

typedef uint32_t  ATTD_Error;

#ifdef __cplusplus
}
#endif

#endif

