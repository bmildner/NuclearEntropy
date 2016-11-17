/*
 * Types.h
 *
 * Created: 30.05.2016 22:15:29
 *  Author: Berti
 */ 
  

#ifndef TYPES_H_
#define TYPES_H_

#include <stdint.h>


typedef uint8_t Byte;

typedef Byte Bool;

#ifndef FALSE
# define FALSE 0
#endif

#ifndef TRUE
# define TRUE 1
#endif

// TODO: find/make better place for these ...

#define ALWAYS_INLINE __attribute__((always_inline)) inline
#define NEVER_INLINE  __attribute__((noinline))

#define PURE_FUNCTION __attribute__((pure))

#define NO_INIT __attribute__ ((section (".noinit")))

#define NO_RETURN __attribute__ ((noreturn))

#define UNITTEST_ASSERT(x) if (!(x)) {asm volatile ("break"); return FALSE;}


#endif /* TYPES_H_ */