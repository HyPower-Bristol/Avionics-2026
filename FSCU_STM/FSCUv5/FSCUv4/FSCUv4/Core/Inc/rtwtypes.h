/*
 * rtwtypes.h  (copied from Simulink ERT generated code, ARM Cortex-M target)
 */
#ifndef RTWTYPES_H
#define RTWTYPES_H

#if (!defined(__cplusplus))
#ifndef false
#define false  (0U)
#endif
#ifndef true
#define true   (1U)
#endif
#endif

typedef signed char    int8_T;
typedef unsigned char  uint8_T;
typedef short          int16_T;
typedef unsigned short uint16_T;
typedef int            int32_T;
typedef unsigned int   uint32_T;
typedef long long      int64_T;
typedef unsigned long long uint64_T;
typedef float          real32_T;
typedef double         real_T;
typedef char           char_T;
typedef unsigned char  boolean_T;
typedef int            int_T;
typedef unsigned int   uint_T;
typedef unsigned long  ulong_T;
typedef signed long    slong_T;
typedef double         time_T;
typedef unsigned char  byte_T;

#define MAX_uint8_T   ((uint8_T)(255U))
#define MIN_uint8_T   ((uint8_T)(0U))
#define MAX_int8_T    ((int8_T)(127))
#define MIN_int8_T    ((int8_T)(-128))
#define MAX_uint16_T  ((uint16_T)(65535U))
#define MIN_uint16_T  ((uint16_T)(0U))
#define MAX_int16_T   ((int16_T)(32767))
#define MIN_int16_T   ((int16_T)(-32768))
#define MAX_uint32_T  ((uint32_T)(0xFFFFFFFFU))
#define MIN_uint32_T  ((uint32_T)(0U))
#define MAX_int32_T   ((int32_T)(0x7FFFFFFF))
#define MIN_int32_T   ((int32_T)(0x80000000))
#define MAX_uint64_T  ((uint64_T)(0xFFFFFFFFFFFFFFFFULL))
#define MIN_uint64_T  ((uint64_T)(0ULL))
#define MAX_int64_T   ((int64_T)(0x7FFFFFFFFFFFFFFFLL))
#define MIN_int64_T   ((int64_T)(0x8000000000000000LL))

#endif /* RTWTYPES_H */
