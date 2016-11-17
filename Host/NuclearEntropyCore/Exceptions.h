// Copyright (c) 2012 - 2016 by Bertolt Mildner
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

#ifndef NUCLEARENTROPY_EXCEPTION_H
#define NUCLEARENTROPY_EXCEPTION_H

#include "NuclearEntropyCore/Config.h"

#include <stdexcept>
#include <string>

#include "NuclearEntropyCore/ErrorCodes.h"

namespace NuclearEntropy
{

  namespace Detail
  {
    class NUCENT_API Exception
    {
      public:
        Exception(NUCENT_Error error)
        : m_ErrorCode(error)
        {};

        virtual ~Exception()
        {};

        virtual void SetErrorCode(NUCENT_Error error)
        {
          m_ErrorCode = error;
        };

        virtual NUCENT_Error GetErrorCode() const
        {
          return m_ErrorCode;
        };

      protected:
        NUCENT_Error m_ErrorCode;

      private:
    };
  }  // namespace Detail

  NUCENT_SUPPRESS_EXPORT_WARNING
  class NUCENT_API RuntimeError : public std::runtime_error, public Detail::Exception
  {
    protected:
      explicit RuntimeError(const std::string& message)
      : std::runtime_error(message), Exception(NUCENT_ERR_GENERAL_ERROR)
      {
      };

      virtual ~RuntimeError() throw() {};  // looks like the std.lib. of GCC 4.6.3 uses "virtual ~exception() throw() {}"
  };

  NUCENT_SUPPRESS_EXPORT_WARNING
  class NUCENT_API LogicError : public std::logic_error, public Detail::Exception
  {
    protected:
      explicit LogicError(const std::string& message)
      : logic_error(message), Exception(NUCENT_ERR_GENERAL_ERROR)
      {
      };

      virtual ~LogicError() throw() {};  // looks like the std.lib. of GCC 4.6.3 uses "virtual ~exception() throw() {}"
  };

}  // namespace NuclearEntropy

#endif

