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

#ifndef NUCLEARENTROPY_ERRORCODES_H
#define NUCLEARENTROPY_ERRORCODES_H

#include "NuclearEntropyCore/Config.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#define NUCENT_ERR_OK                               0

#define NUCENT_ERR_GENERAL_ERROR                    1
#define NUCENT_ERR_FATAL_ERROR                      2
#define NUCENT_ERR_INVALID_PARAM                    3
#define NUCENT_ERR_BUFFER_OVERFLOW                  4
#define NUCENT_ERR_OUT_OF_MEMORY                    5
#define NUCENT_ERR_INVALID_HANDLE                   6
#define NUCENT_ERR_WRONG_HANDLE_TYPE                7

#define NUCENT_ERR_PORT_NOT_SUPPORTED             101
#define NUCENT_ERR_PORT_API_ERROR                 102
#define NUCENT_ERR_PORT_OPEN_FAILED               103
#define NUCENT_ERR_PORT_NOT_FOUND                 104
#define NUCENT_ERR_PORT_INVALID_NAME              105
#define NUCENT_ERR_PORT_CONFIG_ERROR              106
#define NUCENT_ERR_PORT_BREAK_SIGNAL              107
#define NUCENT_ERR_PORT_FRAME_ERROR               108
#define NUCENT_ERR_PORT_OVERFLOW_ERROR            109
#define NUCENT_ERR_PORT_PARITY_ERROR              110
#define NUCENT_ERR_PORT_WRITE_ERROR               111
#define NUCENT_ERR_PORT_READ_ERROR                112
#define NUCENT_ERR_PORT_READ_TIMEOUT              113
#define NUCENT_ERR_PORT_WRITE_TIMEOUT             114


typedef uint32_t  NUCENT_Error;

#ifdef __cplusplus
}
#endif

#endif

