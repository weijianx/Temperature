# 1 "../APP/Src/modbus_ascii.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 359 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "../APP/Src/modbus_ascii.c" 2
# 1 "../APP/Inc\\type.h" 1


# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 110 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h"
typedef enum IRQn
{

    NonMaskableInt_IRQn = -14,
    HardFault_IRQn = -13,

    SVCall_IRQn = -5,

    PendSV_IRQn = -2,
    SysTick_IRQn = -1,

    WWDGT_IRQn = 0,
    LVD_IRQn = 1,
    RTC_IRQn = 2,
    FMC_IRQn = 3,
    RCU_IRQn = 4,
    EXTI0_1_IRQn = 5,
    EXTI2_3_IRQn = 6,
    EXTI4_15_IRQn = 7,
    DMA_Channel0_IRQn = 9,
    DMA_Channel1_2_IRQn = 10,
    DMA_Channel3_4_IRQn = 11,
    ADC_CMP_IRQn = 12,
    TIMER0_BRK_UP_TRG_COM_IRQn = 13,
    TIMER0_Channel_IRQn = 14,
    TIMER2_IRQn = 16,
    TIMER5_IRQn = 17,
    TIMER13_IRQn = 19,
    TIMER14_IRQn = 20,
    TIMER15_IRQn = 21,
    TIMER16_IRQn = 22,
    I2C0_EV_IRQn = 23,
    I2C1_EV_IRQn = 24,
    SPI0_IRQn = 25,
    SPI1_IRQn = 26,
    USART0_IRQn = 27,
    USART1_IRQn = 28,
    I2C0_ER_IRQn = 32,
    I2C1_ER_IRQn = 34,
} IRQn_Type;



# 1 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 1
# 29 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3







# 1 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdint.h" 1 3
# 56 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdint.h" 3
typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int int64_t;


typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long int uint64_t;





typedef signed char int_least8_t;
typedef signed short int int_least16_t;
typedef signed int int_least32_t;
typedef signed long long int int_least64_t;


typedef unsigned char uint_least8_t;
typedef unsigned short int uint_least16_t;
typedef unsigned int uint_least32_t;
typedef unsigned long long int uint_least64_t;




typedef signed int int_fast8_t;
typedef signed int int_fast16_t;
typedef signed int int_fast32_t;
typedef signed long long int int_fast64_t;


typedef unsigned int uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned int uint_fast32_t;
typedef unsigned long long int uint_fast64_t;






typedef signed int intptr_t;
typedef unsigned int uintptr_t;



typedef signed long long intmax_t;
typedef unsigned long long uintmax_t;
# 37 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 2 3
# 65 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
# 1 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_version.h" 1 3
# 29 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_version.h" 3
# 66 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 2 3
# 117 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
# 1 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_compiler.h" 1 3
# 47 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_compiler.h" 3
# 1 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 1 3
# 31 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3


# 1 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\arm_compat.h" 1 3







# 1 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\arm_acle.h" 1 3
# 35 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\arm_acle.h" 3
static __inline__ void __attribute__((__always_inline__, __nodebug__)) __wfi(void) {
  __builtin_arm_wfi();
}

static __inline__ void __attribute__((__always_inline__, __nodebug__)) __wfe(void) {
  __builtin_arm_wfe();
}

static __inline__ void __attribute__((__always_inline__, __nodebug__)) __sev(void) {
  __builtin_arm_sev();
}

static __inline__ void __attribute__((__always_inline__, __nodebug__)) __sevl(void) {
  __builtin_arm_sevl();
}

static __inline__ void __attribute__((__always_inline__, __nodebug__)) __yield(void) {
  __builtin_arm_yield();
}







static __inline__ uint32_t __attribute__((__always_inline__, __nodebug__))
__swp(uint32_t __x, volatile uint32_t *__p) {
  uint32_t v;
  do
    v = __builtin_arm_ldrex(__p);
  while (__builtin_arm_strex(__x, __p));
  return v;
}
# 95 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\arm_acle.h" 3
static __inline__ void __attribute__((__always_inline__, __nodebug__)) __nop(void) {
  __builtin_arm_nop();
}





static __inline__ uint32_t __attribute__((__always_inline__, __nodebug__))
__ror(uint32_t __x, uint32_t __y) {
  __y %= 32;
  if (__y == 0)
    return __x;
  return (__x >> __y) | (__x << (32 - __y));
}

static __inline__ uint64_t __attribute__((__always_inline__, __nodebug__))
__rorll(uint64_t __x, uint32_t __y) {
  __y %= 64;
  if (__y == 0)
    return __x;
  return (__x >> __y) | (__x << (64 - __y));
}

static __inline__ unsigned long __attribute__((__always_inline__, __nodebug__))
__rorl(unsigned long __x, uint32_t __y) {

  return __ror(__x, __y);



}



static __inline__ uint32_t __attribute__((__always_inline__, __nodebug__))
__clz(uint32_t __t) {
  return __builtin_clz(__t);
}

static __inline__ unsigned long __attribute__((__always_inline__, __nodebug__))
__clzl(unsigned long __t) {
  return __builtin_clzl(__t);
}

static __inline__ uint64_t __attribute__((__always_inline__, __nodebug__))
__clzll(uint64_t __t) {
  return __builtin_clzll(__t);
}


static __inline__ uint32_t __attribute__((__always_inline__, __nodebug__))
__cls(uint32_t __t) {
  return __builtin_arm_cls(__t);
}

static __inline__ uint32_t __attribute__((__always_inline__, __nodebug__))
__clsl(unsigned long __t) {

  return __builtin_arm_cls(__t);



}

static __inline__ uint32_t __attribute__((__always_inline__, __nodebug__))
__clsll(uint64_t __t) {
  return __builtin_arm_cls64(__t);
}


static __inline__ uint32_t __attribute__((__always_inline__, __nodebug__))
__rev(uint32_t __t) {
  return __builtin_bswap32(__t);
}

static __inline__ unsigned long __attribute__((__always_inline__, __nodebug__))
__revl(unsigned long __t) {

  return __builtin_bswap32(__t);



}

static __inline__ uint64_t __attribute__((__always_inline__, __nodebug__))
__revll(uint64_t __t) {
  return __builtin_bswap64(__t);
}


static __inline__ uint32_t __attribute__((__always_inline__, __nodebug__))
__rev16(uint32_t __t) {
  return __ror(__rev(__t), 16);
}

static __inline__ uint64_t __attribute__((__always_inline__, __nodebug__))
__rev16ll(uint64_t __t) {
  return (((uint64_t)__rev16(__t >> 32)) << 32) | __rev16(__t);
}

static __inline__ unsigned long __attribute__((__always_inline__, __nodebug__))
__rev16l(unsigned long __t) {

    return __rev16(__t);



}


static __inline__ int16_t __attribute__((__always_inline__, __nodebug__))
__revsh(int16_t __t) {
  return __builtin_bswap16(__t);
}


static __inline__ uint32_t __attribute__((__always_inline__, __nodebug__))
__rbit(uint32_t __t) {
  return __builtin_arm_rbit(__t);
}

static __inline__ uint64_t __attribute__((__always_inline__, __nodebug__))
__rbitll(uint64_t __t) {

  return (((uint64_t)__builtin_arm_rbit(__t)) << 32) |
         __builtin_arm_rbit(__t >> 32);



}

static __inline__ unsigned long __attribute__((__always_inline__, __nodebug__))
__rbitl(unsigned long __t) {

  return __rbit(__t);



}
# 9 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\arm_compat.h" 2 3
# 37 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\arm_compat.h" 3
static __inline__ unsigned int __attribute__((unavailable(
    "intrinsic not supported for this architecture"))) __disable_fiq(void);
# 66 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\arm_compat.h" 3
static __inline__ unsigned int __attribute__((__always_inline__, __nodebug__))
__disable_irq(void) {
  unsigned int cpsr;


  __asm__ __volatile__("mrs %[cpsr], primask\n"
                       "cpsid i\n"
                       : [cpsr] "=r"(cpsr));
  return cpsr & 0x1;
# 90 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\arm_compat.h" 3
}



static __inline__ void __attribute__((unavailable(
    "intrinsic not supported for this architecture"))) __enable_fiq(void);
# 113 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\arm_compat.h" 3
static __inline__ void __attribute__((__always_inline__, __nodebug__))
__enable_irq(void) {

  __asm__ __volatile__("cpsie i");
# 125 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\arm_compat.h" 3
}

static __inline__ void __attribute__((__always_inline__, __nodebug__)) __force_stores(void) {
    __asm__ __volatile__ ("" : : : "memory", "cc");
}

static void __attribute__((__always_inline__, __nodebug__)) __memory_changed(void) {
    __asm__ __volatile__ ("" : : : "memory", "cc");
}

static void __attribute__((__always_inline__, __nodebug__)) __schedule_barrier(void) {
    __asm__ __volatile__ ("" : : : "memory", "cc");
}

static __inline__ int __attribute__((__always_inline__, __nodebug__))
__semihost(int val, const void *ptr) {
  register int v __asm__("r0") = val;
  register const void *p __asm__("r1") = ptr;
  __asm__ __volatile__(


      "bkpt 0xab"
# 161 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\arm_compat.h" 3
      : "+r"(v), "+r"(p)
      :
      : "memory", "cc");
  return v;
}
# 182 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\arm_compat.h" 3
static __inline__ unsigned int __attribute__((
    unavailable("intrinsic not supported for targets without floating point")))
__vfp_status(unsigned int mask, unsigned int flags);
# 34 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 2 3
# 68 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpacked"

 struct __attribute__((packed)) T_UINT32 { uint32_t v; };
#pragma clang diagnostic pop



#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpacked"

 struct __attribute__((packed, aligned(1))) T_UINT16_WRITE { uint16_t v; };
#pragma clang diagnostic pop



#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpacked"

 struct __attribute__((packed, aligned(1))) T_UINT16_READ { uint16_t v; };
#pragma clang diagnostic pop



#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpacked"

 struct __attribute__((packed, aligned(1))) T_UINT32_WRITE { uint32_t v; };
#pragma clang diagnostic pop



#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpacked"

 struct __attribute__((packed, aligned(1))) T_UINT32_READ { uint32_t v; };
#pragma clang diagnostic pop
# 166 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline uint32_t __get_CONTROL(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, control" : "=r" (result) );
  return(result);
}
# 196 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline void __set_CONTROL(uint32_t control)
{
  __asm volatile ("MSR control, %0" : : "r" (control) : "memory");
}
# 220 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline uint32_t __get_IPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, ipsr" : "=r" (result) );
  return(result);
}







__attribute__((always_inline)) static __inline uint32_t __get_APSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, apsr" : "=r" (result) );
  return(result);
}







__attribute__((always_inline)) static __inline uint32_t __get_xPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, xpsr" : "=r" (result) );
  return(result);
}







__attribute__((always_inline)) static __inline uint32_t __get_PSP(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, psp" : "=r" (result) );
  return(result);
}
# 292 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline void __set_PSP(uint32_t topOfProcStack)
{
  __asm volatile ("MSR psp, %0" : : "r" (topOfProcStack) : );
}
# 316 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline uint32_t __get_MSP(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, msp" : "=r" (result) );
  return(result);
}
# 346 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline void __set_MSP(uint32_t topOfMainStack)
{
  __asm volatile ("MSR msp, %0" : : "r" (topOfMainStack) : );
}
# 397 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline uint32_t __get_PRIMASK(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, primask" : "=r" (result) );
  return(result);
}
# 427 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline void __set_PRIMASK(uint32_t priMask)
{
  __asm volatile ("MSR primask, %0" : : "r" (priMask) : "memory");
}
# 604 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline uint32_t __get_PSPLIM(void)
{




  return 0U;





}
# 652 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline void __set_PSPLIM(uint32_t ProcStackPtrLimit)
{




  (void)ProcStackPtrLimit;



}
# 696 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline uint32_t __get_MSPLIM(void)
{




  return 0U;





}
# 743 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline void __set_MSPLIM(uint32_t MainStackPtrLimit)
{




  (void)MainStackPtrLimit;



}
# 914 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline uint32_t __ROR(uint32_t op1, uint32_t op2)
{
  op2 %= 32U;
  if (op2 == 0U)
  {
    return op1;
  }
  return (op1 >> op2) | (op1 << (32U - op2));
}
# 949 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline uint8_t __CLZ(uint32_t value)
{
# 960 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
  if (value == 0U)
  {
    return 32U;
  }
  return __builtin_clz(value);
}
# 1180 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline int32_t __SSAT(int32_t val, uint32_t sat)
{
  if ((sat >= 1U) && (sat <= 32U))
  {
    const int32_t max = (int32_t)((1U << (sat - 1U)) - 1U);
    const int32_t min = -1 - max ;
    if (val > max)
    {
      return max;
    }
    else if (val < min)
    {
      return min;
    }
  }
  return val;
}
# 1205 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline uint32_t __USAT(int32_t val, uint32_t sat)
{
  if (sat <= 31U)
  {
    const uint32_t max = ((1U << sat) - 1U);
    if (val > (int32_t)max)
    {
      return max;
    }
    else if (val < 0)
    {
      return 0U;
    }
  }
  return (uint32_t)val;
}
# 1238 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline uint8_t __LDAB(volatile uint8_t *ptr)
{
  uint32_t result;

  __asm volatile ("ldab %0, %1" : "=r" (result) : "Q" (*ptr) : "memory" );
  return ((uint8_t) result);
}
# 1253 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline uint16_t __LDAH(volatile uint16_t *ptr)
{
  uint32_t result;

  __asm volatile ("ldah %0, %1" : "=r" (result) : "Q" (*ptr) : "memory" );
  return ((uint16_t) result);
}
# 1268 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline uint32_t __LDA(volatile uint32_t *ptr)
{
  uint32_t result;

  __asm volatile ("lda %0, %1" : "=r" (result) : "Q" (*ptr) : "memory" );
  return(result);
}
# 1283 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline void __STLB(uint8_t value, volatile uint8_t *ptr)
{
  __asm volatile ("stlb %1, %0" : "=Q" (*ptr) : "r" ((uint32_t)value) : "memory" );
}
# 1295 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline void __STLH(uint16_t value, volatile uint16_t *ptr)
{
  __asm volatile ("stlh %1, %0" : "=Q" (*ptr) : "r" ((uint32_t)value) : "memory" );
}
# 1307 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_armclang.h" 3
__attribute__((always_inline)) static __inline void __STL(uint32_t value, volatile uint32_t *ptr)
{
  __asm volatile ("stl %1, %0" : "=Q" (*ptr) : "r" ((uint32_t)value) : "memory" );
}
# 48 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include/cmsis_compiler.h" 2 3
# 118 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 2 3
# 235 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
typedef union
{
  struct
  {
    uint32_t _reserved0:28;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} APSR_Type;
# 265 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:23;
  } b;
  uint32_t w;
} IPSR_Type;
# 283 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:15;
    uint32_t T:1;
    uint32_t _reserved1:3;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} xPSR_Type;
# 322 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
typedef union
{
  struct
  {
    uint32_t nPRIV:1;
    uint32_t SPSEL:1;
    uint32_t _reserved1:30;
  } b;
  uint32_t w;
} CONTROL_Type;
# 353 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
typedef struct
{
  volatile uint32_t ISER[16U];
        uint32_t RESERVED0[16U];
  volatile uint32_t ICER[16U];
        uint32_t RSERVED1[16U];
  volatile uint32_t ISPR[16U];
        uint32_t RESERVED2[16U];
  volatile uint32_t ICPR[16U];
        uint32_t RESERVED3[16U];
  volatile uint32_t IABR[16U];
        uint32_t RESERVED4[16U];
  volatile uint32_t ITNS[16U];
        uint32_t RESERVED5[16U];
  volatile uint32_t IPR[124U];
} NVIC_Type;
# 383 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
typedef struct
{
  volatile const uint32_t CPUID;
  volatile uint32_t ICSR;

  volatile uint32_t VTOR;



  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
        uint32_t RESERVED1;
  volatile uint32_t SHPR[2U];
  volatile uint32_t SHCSR;
} SCB_Type;
# 560 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  volatile const uint32_t CALIB;
} SysTick_Type;
# 612 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
typedef struct
{
  volatile uint32_t CTRL;
        uint32_t RESERVED0[6U];
  volatile const uint32_t PCSR;
  volatile uint32_t COMP0;
        uint32_t RESERVED1[1U];
  volatile uint32_t FUNCTION0;
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP1;
        uint32_t RESERVED3[1U];
  volatile uint32_t FUNCTION1;
        uint32_t RESERVED4[1U];
  volatile uint32_t COMP2;
        uint32_t RESERVED5[1U];
  volatile uint32_t FUNCTION2;
        uint32_t RESERVED6[1U];
  volatile uint32_t COMP3;
        uint32_t RESERVED7[1U];
  volatile uint32_t FUNCTION3;
        uint32_t RESERVED8[1U];
  volatile uint32_t COMP4;
        uint32_t RESERVED9[1U];
  volatile uint32_t FUNCTION4;
        uint32_t RESERVED10[1U];
  volatile uint32_t COMP5;
        uint32_t RESERVED11[1U];
  volatile uint32_t FUNCTION5;
        uint32_t RESERVED12[1U];
  volatile uint32_t COMP6;
        uint32_t RESERVED13[1U];
  volatile uint32_t FUNCTION6;
        uint32_t RESERVED14[1U];
  volatile uint32_t COMP7;
        uint32_t RESERVED15[1U];
  volatile uint32_t FUNCTION7;
        uint32_t RESERVED16[1U];
  volatile uint32_t COMP8;
        uint32_t RESERVED17[1U];
  volatile uint32_t FUNCTION8;
        uint32_t RESERVED18[1U];
  volatile uint32_t COMP9;
        uint32_t RESERVED19[1U];
  volatile uint32_t FUNCTION9;
        uint32_t RESERVED20[1U];
  volatile uint32_t COMP10;
        uint32_t RESERVED21[1U];
  volatile uint32_t FUNCTION10;
        uint32_t RESERVED22[1U];
  volatile uint32_t COMP11;
        uint32_t RESERVED23[1U];
  volatile uint32_t FUNCTION11;
        uint32_t RESERVED24[1U];
  volatile uint32_t COMP12;
        uint32_t RESERVED25[1U];
  volatile uint32_t FUNCTION12;
        uint32_t RESERVED26[1U];
  volatile uint32_t COMP13;
        uint32_t RESERVED27[1U];
  volatile uint32_t FUNCTION13;
        uint32_t RESERVED28[1U];
  volatile uint32_t COMP14;
        uint32_t RESERVED29[1U];
  volatile uint32_t FUNCTION14;
        uint32_t RESERVED30[1U];
  volatile uint32_t COMP15;
        uint32_t RESERVED31[1U];
  volatile uint32_t FUNCTION15;
} DWT_Type;
# 727 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
typedef struct
{
  volatile const uint32_t SSPSR;
  volatile uint32_t CSPSR;
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;
        uint32_t RESERVED2[131U];
  volatile const uint32_t FFSR;
  volatile uint32_t FFCR;
  volatile uint32_t PSCR;
        uint32_t RESERVED3[759U];
  volatile const uint32_t TRIGGER;
  volatile const uint32_t ITFTTD0;
  volatile uint32_t ITATBCTR2;
        uint32_t RESERVED4[1U];
  volatile const uint32_t ITATBCTR0;
  volatile const uint32_t ITFTTD1;
  volatile uint32_t ITCTRL;
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;
  volatile uint32_t CLAIMCLR;
        uint32_t RESERVED7[8U];
  volatile const uint32_t DEVID;
  volatile const uint32_t DEVTYPE;
} TPI_Type;
# 1066 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
typedef struct
{
  volatile uint32_t DHCSR;
  volatile uint32_t DCRSR;
  volatile uint32_t DCRDR;
  volatile uint32_t DEMCR;
        uint32_t RESERVED0[1U];
  volatile uint32_t DAUTHCTRL;
  volatile uint32_t DSCSR;
} CoreDebug_Type;
# 1167 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
typedef struct
{
  volatile uint32_t DHCSR;
  volatile uint32_t DCRSR;
  volatile uint32_t DCRDR;
  volatile uint32_t DEMCR;
        uint32_t RESERVED0[1U];
  volatile uint32_t DAUTHCTRL;
  volatile uint32_t DSCSR;
} DCB_Type;
# 1279 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
typedef struct
{
  volatile uint32_t DLAR;
  volatile const uint32_t DLSR;
  volatile const uint32_t DAUTHSTATUS;
  volatile const uint32_t DDEVARCH;
  volatile const uint32_t DDEVTYPE;
} DIB_Type;
# 1524 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
static __inline void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    __asm volatile("":::"memory");
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __asm volatile("":::"memory");
  }
}
# 1543 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
static __inline uint32_t __NVIC_GetEnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}
# 1562 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
static __inline void __NVIC_DisableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __builtin_arm_dsb(0xF);
    __builtin_arm_isb(0xF);
  }
}
# 1581 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
static __inline uint32_t __NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}
# 1600 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
static __inline void __NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}
# 1615 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
static __inline void __NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}
# 1632 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
static __inline uint32_t __NVIC_GetActive(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}
# 1721 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
static __inline void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IPR[( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )] = ((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IPR[( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )] & ~(0xFFUL << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL))) |
       (((priority << (8U - 2U)) & (uint32_t)0xFFUL) << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL)));
  }
  else
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHPR[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] = ((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHPR[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] & ~(0xFFUL << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL))) |
       (((priority << (8U - 2U)) & (uint32_t)0xFFUL) << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL)));
  }
}
# 1745 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
static __inline uint32_t __NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IPR[ ( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )] >> ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL) ) & (uint32_t)0xFFUL) >> (8U - 2U)));
  }
  else
  {
    return((uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHPR[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] >> ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL) ) & (uint32_t)0xFFUL) >> (8U - 2U)));
  }
}
# 1770 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(2U)) ? (uint32_t)(2U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits = ((PriorityGroupTmp + (uint32_t)(2U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(2U));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority & (uint32_t)((1UL << (SubPriorityBits )) - 1UL)))
         );
}
# 1797 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(2U)) ? (uint32_t)(2U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits = ((PriorityGroupTmp + (uint32_t)(2U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(2U));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority = (Priority ) & (uint32_t)((1UL << (SubPriorityBits )) - 1UL);
}
# 1821 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
static __inline void __NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{

  uint32_t *vectors = (uint32_t *)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;



  vectors[(int32_t)IRQn + 16] = vector;
  __builtin_arm_dsb(0xF);
}
# 1841 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
static __inline uint32_t __NVIC_GetVector(IRQn_Type IRQn)
{

  uint32_t *vectors = (uint32_t *)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;



  return vectors[(int32_t)IRQn + 16];
}






__attribute__((__noreturn__)) static __inline void __NVIC_SystemReset(void)
{
  __builtin_arm_dsb(0xF);

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR = ((0x5FAUL << 16U) |
                 (1UL << 2U));
  __builtin_arm_dsb(0xF);

  for(;;)
  {
    __builtin_arm_nop();
  }
}
# 2066 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
static __inline uint32_t SCB_GetFPUType(void)
{
    return 0U;
}
# 2127 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
static __inline void DCB_SetAuthCtrl(uint32_t value)
{
    __builtin_arm_dsb(0xF);
    __builtin_arm_isb(0xF);
    ((DCB_Type *) (0xE000EDF0UL) )->DAUTHCTRL = value;
    __builtin_arm_dsb(0xF);
    __builtin_arm_isb(0xF);
}







static __inline uint32_t DCB_GetAuthCtrl(void)
{
    return (((DCB_Type *) (0xE000EDF0UL) )->DAUTHCTRL);
}
# 2194 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
static __inline uint32_t DIB_GetAuthStatus(void)
{
    return (((DIB_Type *) (0xE000EFB0UL) )->DAUTHSTATUS);
}
# 2238 "D:/Program Files/Packs/ARM/CMSIS/5.7.0/CMSIS/Core/Include\\core_cm23.h" 3
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = (uint32_t)(ticks - 1UL);
  __NVIC_SetPriority (SysTick_IRQn, (1UL << 2U) - 1UL);
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL = 0UL;
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2U) |
                   (1UL << 1U) |
                   (1UL );
  return (0UL);
}
# 153 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 2
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include/system_gd32e230.h" 1
# 46 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include/system_gd32e230.h"
extern uint32_t SystemCoreClock;



extern void SystemInit (void);

extern void SystemCoreClockUpdate (void);
# 154 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 2



typedef enum {DISABLE = 0, ENABLE = !DISABLE} EventStatus, ControlStatus;
typedef enum {RESET = 0, SET = !RESET} FlagStatus;
typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrStatus;
# 206 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h"
# 1 "../User\\gd32e230_libopt.h" 1
# 40 "../User\\gd32e230_libopt.h"
# 1 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_adc.h" 1
# 40 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_adc.h"
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 41 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_adc.h" 2
# 275 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_adc.h"
void adc_deinit(void);

void adc_enable(void);

void adc_disable(void);


void adc_calibration_enable(void);

void adc_dma_mode_enable(void);

void adc_dma_mode_disable(void);


void adc_tempsensor_vrefint_enable(void);

void adc_tempsensor_vrefint_disable(void);


void adc_discontinuous_mode_config(uint8_t channel_group, uint8_t length);

void adc_special_function_config(uint32_t function, ControlStatus newvalue);


void adc_data_alignment_config(uint32_t data_alignment);

void adc_channel_length_config(uint8_t channel_group, uint32_t length);

void adc_regular_channel_config(uint8_t rank, uint8_t channel, uint32_t sample_time);

void adc_inserted_channel_config(uint8_t rank, uint8_t channel, uint32_t sample_time);

void adc_inserted_channel_offset_config(uint8_t inserted_channel, uint16_t offset);

void adc_external_trigger_config(uint8_t channel_group, ControlStatus newvalue);

void adc_external_trigger_source_config(uint8_t channel_group, uint32_t external_trigger_source);

void adc_software_trigger_enable(uint8_t channel_group);


uint16_t adc_regular_data_read(void);

uint16_t adc_inserted_data_read(uint8_t inserted_channel);


FlagStatus adc_flag_get(uint32_t flag);

void adc_flag_clear(uint32_t flag);

FlagStatus adc_interrupt_flag_get(uint32_t flag);

void adc_interrupt_flag_clear(uint32_t flag);

void adc_interrupt_enable(uint32_t interrupt);

void adc_interrupt_disable(uint32_t interrupt);


void adc_watchdog_single_channel_enable(uint8_t channel);

void adc_watchdog_group_channel_enable(uint8_t channel_group);

void adc_watchdog_disable(void);

void adc_watchdog_threshold_config(uint16_t low_threshold, uint16_t high_threshold);


void adc_resolution_config(uint32_t resolution);

void adc_oversample_mode_config(uint8_t mode, uint16_t shift, uint8_t ratio);

void adc_oversample_mode_enable(void);

void adc_oversample_mode_disable(void);
# 41 "../User\\gd32e230_libopt.h" 2
# 1 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_crc.h" 1
# 40 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_crc.h"
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 41 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_crc.h" 2
# 88 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_crc.h"
void crc_deinit(void);


void crc_reverse_output_data_enable(void);

void crc_reverse_output_data_disable(void);


void crc_data_register_reset(void);

uint32_t crc_data_register_read(void);


uint8_t crc_free_data_register_read(void);

void crc_free_data_register_write(uint8_t free_data);


void crc_init_data_register_write(uint32_t init_data);

void crc_input_data_reverse_config(uint32_t data_reverse);


void crc_polynomial_size_set(uint32_t poly_size);

void crc_polynomial_set(uint32_t poly);


uint32_t crc_single_data_calculate(uint32_t sdata);

uint32_t crc_block_data_calculate(uint32_t array[], uint32_t size);
# 42 "../User\\gd32e230_libopt.h" 2
# 1 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_dbg.h" 1
# 40 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_dbg.h"
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 41 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_dbg.h" 2
# 84 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_dbg.h"
enum dbg_reg_idx
{
    DBG_IDX_CTL0 = 0x04U,
    DBG_IDX_CTL1 = 0x08U,
};


typedef enum
{
    DBG_FWDGT_HOLD = (((DBG_IDX_CTL0) << 6) | (8U)),
    DBG_WWDGT_HOLD = (((DBG_IDX_CTL0) << 6) | (9U)),
    DBG_TIMER0_HOLD = (((DBG_IDX_CTL0) << 6) | (10U)),
    DBG_TIMER2_HOLD = (((DBG_IDX_CTL0) << 6) | (12U)),
    DBG_TIMER5_HOLD = (((DBG_IDX_CTL0) << 6) | (19U)),
    DBG_TIMER13_HOLD = (((DBG_IDX_CTL0) << 6) | (27U)),
    DBG_TIMER14_HOLD = (((DBG_IDX_CTL1) << 6) | (16U)),
    DBG_TIMER15_HOLD = (((DBG_IDX_CTL1) << 6) | (17U)),
    DBG_TIMER16_HOLD = (((DBG_IDX_CTL1) << 6) | (18U)),
    DBG_I2C0_HOLD = (((DBG_IDX_CTL0) << 6) | (15U)),
    DBG_I2C1_HOLD = (((DBG_IDX_CTL0) << 6) | (16U)),
    DBG_RTC_HOLD = (((DBG_IDX_CTL1) << 6) | (10U)),
}dbg_periph_enum;



void dbg_deinit(void);

uint32_t dbg_id_get(void);


void dbg_low_power_enable(uint32_t dbg_low_power);

void dbg_low_power_disable(uint32_t dbg_low_power);


void dbg_periph_enable(dbg_periph_enum dbg_periph);

void dbg_periph_disable(dbg_periph_enum dbg_periph);
# 43 "../User\\gd32e230_libopt.h" 2
# 1 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_dma.h" 1
# 40 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_dma.h"
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 41 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_dma.h" 2
# 107 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_dma.h"
typedef enum
{
    DMA_CH0 = 0,
    DMA_CH1,
    DMA_CH2,
    DMA_CH3,
    DMA_CH4,
} dma_channel_enum;


typedef struct
{
    uint32_t periph_addr;
    uint32_t periph_width;
    uint32_t memory_addr;
    uint32_t memory_width;
    uint32_t number;
    uint32_t priority;
    uint8_t periph_inc;
    uint8_t memory_inc;
    uint8_t direction;
} dma_parameter_struct;
# 208 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_dma.h"
void dma_deinit(dma_channel_enum channelx);

void dma_struct_para_init(dma_parameter_struct* init_struct);

void dma_init(dma_channel_enum channelx, dma_parameter_struct* init_struct);

void dma_circulation_enable(dma_channel_enum channelx);

void dma_circulation_disable(dma_channel_enum channelx);

void dma_memory_to_memory_enable(dma_channel_enum channelx);

void dma_memory_to_memory_disable(dma_channel_enum channelx);

void dma_channel_enable(dma_channel_enum channelx);

void dma_channel_disable(dma_channel_enum channelx);


void dma_periph_address_config(dma_channel_enum channelx, uint32_t address);

void dma_memory_address_config(dma_channel_enum channelx, uint32_t address);

void dma_transfer_number_config(dma_channel_enum channelx, uint32_t number);

uint32_t dma_transfer_number_get(dma_channel_enum channelx);

void dma_priority_config(dma_channel_enum channelx, uint32_t priority);

void dma_memory_width_config (dma_channel_enum channelx, uint32_t mwidth);

void dma_periph_width_config (dma_channel_enum channelx, uint32_t pwidth);

void dma_memory_increase_enable(dma_channel_enum channelx);

void dma_memory_increase_disable(dma_channel_enum channelx);

void dma_periph_increase_enable(dma_channel_enum channelx);

void dma_periph_increase_disable(dma_channel_enum channelx);

void dma_transfer_direction_config(dma_channel_enum channelx, uint32_t direction);


FlagStatus dma_flag_get(dma_channel_enum channelx, uint32_t flag);

void dma_flag_clear(dma_channel_enum channelx, uint32_t flag);

FlagStatus dma_interrupt_flag_get(dma_channel_enum channelx, uint32_t flag);

void dma_interrupt_flag_clear(dma_channel_enum channelx, uint32_t flag);

void dma_interrupt_enable(dma_channel_enum channelx, uint32_t source);

void dma_interrupt_disable(dma_channel_enum channelx, uint32_t source);
# 44 "../User\\gd32e230_libopt.h" 2
# 1 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_exti.h" 1
# 40 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_exti.h"
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 41 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_exti.h" 2
# 204 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_exti.h"
typedef enum
{
    EXTI_0 = ((uint32_t)((uint32_t)0x01U<<(0))),
    EXTI_1 = ((uint32_t)((uint32_t)0x01U<<(1))),
    EXTI_2 = ((uint32_t)((uint32_t)0x01U<<(2))),
    EXTI_3 = ((uint32_t)((uint32_t)0x01U<<(3))),
    EXTI_4 = ((uint32_t)((uint32_t)0x01U<<(4))),
    EXTI_5 = ((uint32_t)((uint32_t)0x01U<<(5))),
    EXTI_6 = ((uint32_t)((uint32_t)0x01U<<(6))),
    EXTI_7 = ((uint32_t)((uint32_t)0x01U<<(7))),
    EXTI_8 = ((uint32_t)((uint32_t)0x01U<<(8))),
    EXTI_9 = ((uint32_t)((uint32_t)0x01U<<(9))),
    EXTI_10 = ((uint32_t)((uint32_t)0x01U<<(10))),
    EXTI_11 = ((uint32_t)((uint32_t)0x01U<<(11))),
    EXTI_12 = ((uint32_t)((uint32_t)0x01U<<(12))),
    EXTI_13 = ((uint32_t)((uint32_t)0x01U<<(13))),
    EXTI_14 = ((uint32_t)((uint32_t)0x01U<<(14))),
    EXTI_15 = ((uint32_t)((uint32_t)0x01U<<(15))),
    EXTI_16 = ((uint32_t)((uint32_t)0x01U<<(16))),
    EXTI_17 = ((uint32_t)((uint32_t)0x01U<<(17))),
    EXTI_18 = ((uint32_t)((uint32_t)0x01U<<(18))),
    EXTI_19 = ((uint32_t)((uint32_t)0x01U<<(19))),
    EXTI_20 = ((uint32_t)((uint32_t)0x01U<<(20))),
    EXTI_21 = ((uint32_t)((uint32_t)0x01U<<(21))),
    EXTI_22 = ((uint32_t)((uint32_t)0x01U<<(22))),
    EXTI_23 = ((uint32_t)((uint32_t)0x01U<<(23))),
    EXTI_24 = ((uint32_t)((uint32_t)0x01U<<(24))),
    EXTI_25 = ((uint32_t)((uint32_t)0x01U<<(25))),
    EXTI_26 = ((uint32_t)((uint32_t)0x01U<<(26))),
    EXTI_27 = ((uint32_t)((uint32_t)0x01U<<(27))),
}exti_line_enum;


typedef enum
{
    EXTI_INTERRUPT = 0,
    EXTI_EVENT
}exti_mode_enum;


typedef enum
{
    EXTI_TRIG_RISING = 0,
    EXTI_TRIG_FALLING,
    EXTI_TRIG_BOTH
}exti_trig_type_enum;



void exti_deinit(void);

void exti_init(exti_line_enum linex, exti_mode_enum mode, exti_trig_type_enum trig_type);

void exti_interrupt_enable(exti_line_enum linex);

void exti_event_enable(exti_line_enum linex);

void exti_interrupt_disable(exti_line_enum linex);

void exti_event_disable(exti_line_enum linex);


FlagStatus exti_flag_get(exti_line_enum linex);

void exti_flag_clear(exti_line_enum linex);

FlagStatus exti_interrupt_flag_get(exti_line_enum linex);

void exti_interrupt_flag_clear(exti_line_enum linex);

void exti_software_interrupt_enable(exti_line_enum linex);

void exti_software_interrupt_disable(exti_line_enum linex);
# 45 "../User\\gd32e230_libopt.h" 2
# 1 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_fmc.h" 1
# 40 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_fmc.h"
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 41 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_fmc.h" 2
# 111 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_fmc.h"
typedef enum
{
    FMC_READY,
    FMC_BUSY,
    FMC_PGERR,
    FMC_PGAERR,
    FMC_WPERR,
    FMC_TOERR,
    FMC_OB_HSPC
}fmc_state_enum;
# 207 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_fmc.h"
void fmc_unlock(void);

void fmc_lock(void);

void fmc_wscnt_set(uint8_t wscnt);


void fmc_prefetch_enable(void);

void fmc_prefetch_disable(void);

fmc_state_enum fmc_page_erase(uint32_t page_address);

fmc_state_enum fmc_mass_erase(void);

fmc_state_enum fmc_doubleword_program(uint32_t address, uint64_t data);

fmc_state_enum fmc_word_program(uint32_t address, uint32_t data);



void ob_unlock(void);

void ob_lock(void);

void ob_reset(void);

uint32_t option_byte_value_get(uint32_t addr);

fmc_state_enum ob_erase(void);

fmc_state_enum ob_write_protection_enable(uint16_t ob_wp);

fmc_state_enum ob_security_protection_config(uint16_t ob_spc);

fmc_state_enum ob_user_write(uint8_t ob_user);

fmc_state_enum ob_data_program(uint16_t data);

uint8_t ob_user_get(void);

uint16_t ob_data_get(void);

uint16_t ob_write_protection_get(void);

uint32_t ob_obstat_plevel_get(void);



void fmc_interrupt_enable(uint32_t interrupt);

void fmc_interrupt_disable(uint32_t interrupt);

FlagStatus fmc_flag_get(uint32_t flag);

void fmc_flag_clear(uint32_t flag);

FlagStatus fmc_interrupt_flag_get(uint32_t int_flag);

void fmc_interrupt_flag_clear(uint32_t int_flag);

fmc_state_enum fmc_state_get(void);

fmc_state_enum fmc_ready_wait(uint32_t timeout);
# 46 "../User\\gd32e230_libopt.h" 2
# 1 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_gpio.h" 1
# 40 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_gpio.h"
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 41 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_gpio.h" 2
# 281 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_gpio.h"
typedef FlagStatus bit_status;
# 354 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_gpio.h"
void gpio_deinit(uint32_t gpio_periph);

void gpio_mode_set(uint32_t gpio_periph, uint32_t mode, uint32_t pull_up_down, uint32_t pin);

void gpio_output_options_set(uint32_t gpio_periph, uint8_t otype, uint32_t speed, uint32_t pin);


void gpio_bit_set(uint32_t gpio_periph, uint32_t pin);

void gpio_bit_reset(uint32_t gpio_periph, uint32_t pin);

void gpio_bit_write(uint32_t gpio_periph, uint32_t pin, bit_status bit_value);

void gpio_port_write(uint32_t gpio_periph, uint16_t data);


FlagStatus gpio_input_bit_get(uint32_t gpio_periph, uint32_t pin);

uint16_t gpio_input_port_get(uint32_t gpio_periph);

FlagStatus gpio_output_bit_get(uint32_t gpio_periph, uint32_t pin);

uint16_t gpio_output_port_get(uint32_t gpio_periph);


void gpio_af_set(uint32_t gpio_periph,uint32_t alt_func_num, uint32_t pin);

void gpio_pin_lock(uint32_t gpio_periph, uint32_t pin);


void gpio_bit_toggle(uint32_t gpio_periph, uint32_t pin);

void gpio_port_toggle(uint32_t gpio_periph);
# 47 "../User\\gd32e230_libopt.h" 2
# 1 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_syscfg.h" 1
# 40 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_syscfg.h"
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 41 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_syscfg.h" 2
# 162 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_syscfg.h"
void syscfg_deinit(void);


void syscfg_dma_remap_enable(uint32_t syscfg_dma_remap);

void syscfg_dma_remap_disable(uint32_t syscfg_dma_remap);


void syscfg_high_current_enable(void);

void syscfg_high_current_disable(void);


void syscfg_exti_line_config(uint8_t exti_port, uint8_t exti_pin);

void syscfg_lock_config(uint32_t syscfg_lock);


void irq_latency_set(uint8_t irq_latency);


FlagStatus syscfg_flag_get(uint32_t syscfg_flag);

void syscfg_flag_clear(uint32_t syscfg_flag);
# 48 "../User\\gd32e230_libopt.h" 2
# 1 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_i2c.h" 1
# 40 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_i2c.h"
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 41 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_i2c.h" 2
# 165 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_i2c.h"
typedef enum
{

    I2C_FLAG_SBSEND = (((uint32_t)(0x14U) << 6) | (uint32_t)(0U)),
    I2C_FLAG_ADDSEND = (((uint32_t)(0x14U) << 6) | (uint32_t)(1U)),
    I2C_FLAG_BTC = (((uint32_t)(0x14U) << 6) | (uint32_t)(2U)),
    I2C_FLAG_ADD10SEND = (((uint32_t)(0x14U) << 6) | (uint32_t)(3U)),
    I2C_FLAG_STPDET = (((uint32_t)(0x14U) << 6) | (uint32_t)(4U)),
    I2C_FLAG_RBNE = (((uint32_t)(0x14U) << 6) | (uint32_t)(6U)),
    I2C_FLAG_TBE = (((uint32_t)(0x14U) << 6) | (uint32_t)(7U)),
    I2C_FLAG_BERR = (((uint32_t)(0x14U) << 6) | (uint32_t)(8U)),
    I2C_FLAG_LOSTARB = (((uint32_t)(0x14U) << 6) | (uint32_t)(9U)),
    I2C_FLAG_AERR = (((uint32_t)(0x14U) << 6) | (uint32_t)(10U)),
    I2C_FLAG_OUERR = (((uint32_t)(0x14U) << 6) | (uint32_t)(11U)),
    I2C_FLAG_PECERR = (((uint32_t)(0x14U) << 6) | (uint32_t)(12U)),
    I2C_FLAG_SMBTO = (((uint32_t)(0x14U) << 6) | (uint32_t)(14U)),
    I2C_FLAG_SMBALT = (((uint32_t)(0x14U) << 6) | (uint32_t)(15U)),

    I2C_FLAG_MASTER = (((uint32_t)(0x18U) << 6) | (uint32_t)(0U)),
    I2C_FLAG_I2CBSY = (((uint32_t)(0x18U) << 6) | (uint32_t)(1U)),
    I2C_FLAG_TR = (((uint32_t)(0x18U) << 6) | (uint32_t)(2U)),
    I2C_FLAG_RXGC = (((uint32_t)(0x18U) << 6) | (uint32_t)(4U)),
    I2C_FLAG_DEFSMB = (((uint32_t)(0x18U) << 6) | (uint32_t)(5U)),
    I2C_FLAG_HSTSMB = (((uint32_t)(0x18U) << 6) | (uint32_t)(6U)),
    I2C_FLAG_DUMOD = (((uint32_t)(0x18U) << 6) | (uint32_t)(7U)),

    I2C_FLAG_TFF = (((uint32_t)(0x80U) << 6) | (uint32_t)(12U)),
    I2C_FLAG_TFR = (((uint32_t)(0x80U) << 6) | (uint32_t)(13U)),
    I2C_FLAG_RFF = (((uint32_t)(0x80U) << 6) | (uint32_t)(14U)),
    I2C_FLAG_RFR = (((uint32_t)(0x80U) << 6) | (uint32_t)(15U))
}i2c_flag_enum;


typedef enum
{

    I2C_INT_FLAG_SBSEND = (((uint32_t)(0x14U) << 22) | (uint32_t)((0U) << 16) | (((uint32_t)(0x04U) << 6) | (uint32_t)(9U))),
    I2C_INT_FLAG_ADDSEND = (((uint32_t)(0x14U) << 22) | (uint32_t)((1U) << 16) | (((uint32_t)(0x04U) << 6) | (uint32_t)(9U))),
    I2C_INT_FLAG_BTC = (((uint32_t)(0x14U) << 22) | (uint32_t)((2U) << 16) | (((uint32_t)(0x04U) << 6) | (uint32_t)(9U))),
    I2C_INT_FLAG_ADD10SEND = (((uint32_t)(0x14U) << 22) | (uint32_t)((3U) << 16) | (((uint32_t)(0x04U) << 6) | (uint32_t)(9U))),
    I2C_INT_FLAG_STPDET = (((uint32_t)(0x14U) << 22) | (uint32_t)((4U) << 16) | (((uint32_t)(0x04U) << 6) | (uint32_t)(9U))),
    I2C_INT_FLAG_RBNE = (((uint32_t)(0x14U) << 22) | (uint32_t)((6U) << 16) | (((uint32_t)(0x04U) << 6) | (uint32_t)(9U))),
    I2C_INT_FLAG_TBE = (((uint32_t)(0x14U) << 22) | (uint32_t)((7U) << 16) | (((uint32_t)(0x04U) << 6) | (uint32_t)(9U))),
    I2C_INT_FLAG_BERR = (((uint32_t)(0x14U) << 22) | (uint32_t)((8U) << 16) | (((uint32_t)(0x04U) << 6) | (uint32_t)(8U))),
    I2C_INT_FLAG_LOSTARB = (((uint32_t)(0x14U) << 22) | (uint32_t)((9U) << 16) | (((uint32_t)(0x04U) << 6) | (uint32_t)(8U))),
    I2C_INT_FLAG_AERR = (((uint32_t)(0x14U) << 22) | (uint32_t)((10U) << 16) | (((uint32_t)(0x04U) << 6) | (uint32_t)(8U))),
    I2C_INT_FLAG_OUERR = (((uint32_t)(0x14U) << 22) | (uint32_t)((11U) << 16) | (((uint32_t)(0x04U) << 6) | (uint32_t)(8U))),
    I2C_INT_FLAG_PECERR = (((uint32_t)(0x14U) << 22) | (uint32_t)((12U) << 16) | (((uint32_t)(0x04U) << 6) | (uint32_t)(8U))),
    I2C_INT_FLAG_SMBTO = (((uint32_t)(0x14U) << 22) | (uint32_t)((14U) << 16) | (((uint32_t)(0x04U) << 6) | (uint32_t)(8U))),
    I2C_INT_FLAG_SMBALT = (((uint32_t)(0x14U) << 22) | (uint32_t)((15U) << 16) | (((uint32_t)(0x04U) << 6) | (uint32_t)(8U))),

    I2C_INT_FLAG_TFF = (((uint32_t)(0x80U) << 22) | (uint32_t)((12U) << 16) | (((uint32_t)(0x80U) << 6) | (uint32_t)(4U))),
    I2C_INT_FLAG_TFR = (((uint32_t)(0x80U) << 22) | (uint32_t)((13U) << 16) | (((uint32_t)(0x80U) << 6) | (uint32_t)(5U))),
    I2C_INT_FLAG_RFF = (((uint32_t)(0x80U) << 22) | (uint32_t)((14U) << 16) | (((uint32_t)(0x80U) << 6) | (uint32_t)(6U))),
    I2C_INT_FLAG_RFR = (((uint32_t)(0x80U) << 22) | (uint32_t)((15U) << 16) | (((uint32_t)(0x80U) << 6) | (uint32_t)(7U)))
}i2c_interrupt_flag_enum;


typedef enum
{

    I2C_INT_ERR = (((uint32_t)(0x04U) << 6) | (uint32_t)(8U)),
    I2C_INT_EV = (((uint32_t)(0x04U) << 6) | (uint32_t)(9U)),
    I2C_INT_BUF = (((uint32_t)(0x04U) << 6) | (uint32_t)(10U)),

    I2C_INT_TFF = (((uint32_t)(0x80U) << 6) | (uint32_t)(4U)),
    I2C_INT_TFR = (((uint32_t)(0x80U) << 6) | (uint32_t)(5U)),
    I2C_INT_RFF = (((uint32_t)(0x80U) << 6) | (uint32_t)(6U)),
    I2C_INT_RFR = (((uint32_t)(0x80U) << 6) | (uint32_t)(7U))
}i2c_interrupt_enum;
# 315 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_i2c.h"
void i2c_deinit(uint32_t i2c_periph);

void i2c_clock_config(uint32_t i2c_periph, uint32_t clkspeed, uint32_t dutycyc);

void i2c_mode_addr_config(uint32_t i2c_periph, uint32_t mode, uint32_t addformat, uint32_t addr);

void i2c_smbus_type_config(uint32_t i2c_periph, uint32_t type);

void i2c_ack_config(uint32_t i2c_periph, uint32_t ack);

void i2c_ackpos_config(uint32_t i2c_periph, uint32_t pos);

void i2c_master_addressing(uint32_t i2c_periph, uint32_t addr, uint32_t trandirection);

void i2c_dualaddr_enable(uint32_t i2c_periph, uint32_t addr);

void i2c_dualaddr_disable(uint32_t i2c_periph);

void i2c_enable(uint32_t i2c_periph);

void i2c_disable(uint32_t i2c_periph);


void i2c_start_on_bus(uint32_t i2c_periph);

void i2c_stop_on_bus(uint32_t i2c_periph);

void i2c_data_transmit(uint32_t i2c_periph, uint8_t data);

uint8_t i2c_data_receive(uint32_t i2c_periph);

void i2c_dma_enable(uint32_t i2c_periph, uint32_t dmastate);

void i2c_dma_last_transfer_config(uint32_t i2c_periph, uint32_t dmalast);

void i2c_stretch_scl_low_config(uint32_t i2c_periph, uint32_t stretchpara);

void i2c_slave_response_to_gcall_config(uint32_t i2c_periph, uint32_t gcallpara);

void i2c_software_reset_config(uint32_t i2c_periph, uint32_t sreset);


void i2c_pec_enable(uint32_t i2c_periph, uint32_t pecstate);

void i2c_pec_transfer_enable(uint32_t i2c_periph, uint32_t pecpara);

uint8_t i2c_pec_value_get(uint32_t i2c_periph);

void i2c_smbus_issue_alert(uint32_t i2c_periph, uint32_t smbuspara);

void i2c_smbus_arp_enable(uint32_t i2c_periph, uint32_t arpstate);


void i2c_sam_enable(uint32_t i2c_periph);

void i2c_sam_disable(uint32_t i2c_periph);

void i2c_sam_timeout_enable(uint32_t i2c_periph);

void i2c_sam_timeout_disable(uint32_t i2c_periph);


FlagStatus i2c_flag_get(uint32_t i2c_periph, i2c_flag_enum flag);

void i2c_flag_clear(uint32_t i2c_periph, i2c_flag_enum flag);

void i2c_interrupt_enable(uint32_t i2c_periph, i2c_interrupt_enum interrupt);

void i2c_interrupt_disable(uint32_t i2c_periph, i2c_interrupt_enum interrupt);

FlagStatus i2c_interrupt_flag_get(uint32_t i2c_periph, i2c_interrupt_flag_enum int_flag);

void i2c_interrupt_flag_clear(uint32_t i2c_periph, i2c_interrupt_flag_enum int_flag);
# 49 "../User\\gd32e230_libopt.h" 2
# 1 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_fwdgt.h" 1
# 40 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_fwdgt.h"
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 41 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_fwdgt.h" 2
# 99 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_fwdgt.h"
void fwdgt_write_enable(void);

void fwdgt_write_disable(void);

void fwdgt_enable(void);


ErrStatus fwdgt_prescaler_value_config(uint16_t prescaler_value);

ErrStatus fwdgt_reload_value_config(uint16_t reload_value);

ErrStatus fwdgt_window_value_config(uint16_t window_value);

void fwdgt_counter_reload(void);

ErrStatus fwdgt_config(uint16_t reload_value, uint8_t prescaler_div);


FlagStatus fwdgt_flag_get(uint16_t flag);
# 50 "../User\\gd32e230_libopt.h" 2
# 1 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_pmu.h" 1
# 40 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_pmu.h"
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 41 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_pmu.h" 2
# 113 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_pmu.h"
void pmu_deinit(void);


void pmu_lvd_select(uint32_t lvdt_n);

void pmu_ldo_output_select(uint32_t ldo_output);

void pmu_lvd_disable(void);



void pmu_to_sleepmode(uint8_t sleepmodecmd);

void pmu_to_deepsleepmode(uint32_t ldo, uint8_t deepsleepmodecmd);

void pmu_to_standbymode(uint8_t standbymodecmd);

void pmu_wakeup_pin_enable(uint32_t wakeup_pin);

void pmu_wakeup_pin_disable(uint32_t wakeup_pin);



void pmu_backup_write_enable(void);

void pmu_backup_write_disable(void);



void pmu_flag_clear(uint32_t flag_clear);

FlagStatus pmu_flag_get(uint32_t flag);
# 51 "../User\\gd32e230_libopt.h" 2
# 1 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_rcu.h" 1
# 40 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_rcu.h"
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 41 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_rcu.h" 2
# 224 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_rcu.h"
typedef enum
{

    IDX_AHBEN = 0x14U,
    IDX_APB2EN = 0x18U,
    IDX_APB1EN = 0x1CU,

    IDX_AHBRST = 0x28U,
    IDX_APB2RST = 0x0CU,
    IDX_APB1RST = 0x10U,

    IDX_CTL0 = 0x00U,
    IDX_BDCTL = 0x20U,
    IDX_CTL1 = 0x34U,

    IDX_RSTSCK = 0x24U,

    IDX_INT = 0x08U,

    IDX_CFG0 = 0x04U,
    IDX_CFG2 = 0x30U
}reg_idx;


typedef enum
{

    RCU_DMA = (((uint32_t)(IDX_AHBEN)<<6) | (uint32_t)(0U)),
    RCU_CRC = (((uint32_t)(IDX_AHBEN)<<6) | (uint32_t)(6U)),
    RCU_GPIOA = (((uint32_t)(IDX_AHBEN)<<6) | (uint32_t)(17U)),
    RCU_GPIOB = (((uint32_t)(IDX_AHBEN)<<6) | (uint32_t)(18U)),
    RCU_GPIOC = (((uint32_t)(IDX_AHBEN)<<6) | (uint32_t)(19U)),
    RCU_GPIOF = (((uint32_t)(IDX_AHBEN)<<6) | (uint32_t)(22U)),


    RCU_CFGCMP = (((uint32_t)(IDX_APB2EN)<<6) | (uint32_t)(0U)),
    RCU_ADC = (((uint32_t)(IDX_APB2EN)<<6) | (uint32_t)(9U)),
    RCU_TIMER0 = (((uint32_t)(IDX_APB2EN)<<6) | (uint32_t)(11U)),
    RCU_SPI0 = (((uint32_t)(IDX_APB2EN)<<6) | (uint32_t)(12U)),
    RCU_USART0 = (((uint32_t)(IDX_APB2EN)<<6) | (uint32_t)(14U)),
    RCU_TIMER14 = (((uint32_t)(IDX_APB2EN)<<6) | (uint32_t)(16U)),
    RCU_TIMER15 = (((uint32_t)(IDX_APB2EN)<<6) | (uint32_t)(17U)),
    RCU_TIMER16 = (((uint32_t)(IDX_APB2EN)<<6) | (uint32_t)(18U)),
    RCU_DBGMCU = (((uint32_t)(IDX_APB2EN)<<6) | (uint32_t)(22U)),


    RCU_TIMER2 = (((uint32_t)(IDX_APB1EN)<<6) | (uint32_t)(1U)),
    RCU_TIMER5 = (((uint32_t)(IDX_APB1EN)<<6) | (uint32_t)(4U)),
    RCU_TIMER13 = (((uint32_t)(IDX_APB1EN)<<6) | (uint32_t)(8U)),
    RCU_WWDGT = (((uint32_t)(IDX_APB1EN)<<6) | (uint32_t)(11U)),
    RCU_SPI1 = (((uint32_t)(IDX_APB1EN)<<6) | (uint32_t)(14U)),
    RCU_USART1 = (((uint32_t)(IDX_APB1EN)<<6) | (uint32_t)(17U)),
    RCU_I2C0 = (((uint32_t)(IDX_APB1EN)<<6) | (uint32_t)(21U)),
    RCU_I2C1 = (((uint32_t)(IDX_APB1EN)<<6) | (uint32_t)(22U)),
    RCU_PMU = (((uint32_t)(IDX_APB1EN)<<6) | (uint32_t)(28U)),


    RCU_RTC = (((uint32_t)(IDX_BDCTL)<<6) | (uint32_t)(15U))
}rcu_periph_enum;


typedef enum
{

    RCU_SRAM_SLP = (((uint32_t)(IDX_AHBEN)<<6) | (uint32_t)(2U)),
    RCU_FMC_SLP = (((uint32_t)(IDX_AHBEN)<<6) | (uint32_t)(4U)),
}rcu_periph_sleep_enum;


typedef enum
{

    RCU_GPIOARST = (((uint32_t)(IDX_AHBRST)<<6) | (uint32_t)(17U)),
    RCU_GPIOBRST = (((uint32_t)(IDX_AHBRST)<<6) | (uint32_t)(18U)),
    RCU_GPIOCRST = (((uint32_t)(IDX_AHBRST)<<6) | (uint32_t)(19U)),
    RCU_GPIOFRST = (((uint32_t)(IDX_AHBRST)<<6) | (uint32_t)(22U)),


    RCU_CFGCMPRST = (((uint32_t)(IDX_APB2RST)<<6) | (uint32_t)(0U)),
    RCU_ADCRST = (((uint32_t)(IDX_APB2RST)<<6) | (uint32_t)(9U)),
    RCU_TIMER0RST = (((uint32_t)(IDX_APB2RST)<<6) | (uint32_t)(11U)),
    RCU_SPI0RST = (((uint32_t)(IDX_APB2RST)<<6) | (uint32_t)(12U)),
    RCU_USART0RST = (((uint32_t)(IDX_APB2RST)<<6) | (uint32_t)(14U)),
    RCU_TIMER14RST = (((uint32_t)(IDX_APB2RST)<<6) | (uint32_t)(16U)),
    RCU_TIMER15RST = (((uint32_t)(IDX_APB2RST)<<6) | (uint32_t)(17U)),
    RCU_TIMER16RST = (((uint32_t)(IDX_APB2RST)<<6) | (uint32_t)(18U)),


    RCU_TIMER2RST = (((uint32_t)(IDX_APB1RST)<<6) | (uint32_t)(1U)),
    RCU_TIMER5RST = (((uint32_t)(IDX_APB1RST)<<6) | (uint32_t)(4U)),
    RCU_TIMER13RST = (((uint32_t)(IDX_APB1RST)<<6) | (uint32_t)(8U)),
    RCU_WWDGTRST = (((uint32_t)(IDX_APB1RST)<<6) | (uint32_t)(11U)),
    RCU_SPI1RST = (((uint32_t)(IDX_APB1RST)<<6) | (uint32_t)(14U)),
    RCU_USART1RST = (((uint32_t)(IDX_APB1RST)<<6) | (uint32_t)(17U)),
    RCU_I2C0RST = (((uint32_t)(IDX_APB1RST)<<6) | (uint32_t)(21U)),
    RCU_I2C1RST = (((uint32_t)(IDX_APB1RST)<<6) | (uint32_t)(22U)),
    RCU_PMURST = (((uint32_t)(IDX_APB1RST)<<6) | (uint32_t)(28U)),
}rcu_periph_reset_enum;


typedef enum
{
    RCU_FLAG_IRC40KSTB = (((uint32_t)(IDX_RSTSCK)<<6) | (uint32_t)(1U)),
    RCU_FLAG_LXTALSTB = (((uint32_t)(IDX_BDCTL)<<6) | (uint32_t)(1U)),
    RCU_FLAG_IRC8MSTB = (((uint32_t)(IDX_CTL0)<<6) | (uint32_t)(1U)),
    RCU_FLAG_HXTALSTB = (((uint32_t)(IDX_CTL0)<<6) | (uint32_t)(17U)),
    RCU_FLAG_PLLSTB = (((uint32_t)(IDX_CTL0)<<6) | (uint32_t)(25U)),
    RCU_FLAG_IRC28MSTB = (((uint32_t)(IDX_CTL1)<<6) | (uint32_t)(1U)),

    RCU_FLAG_V12RST = (((uint32_t)(IDX_RSTSCK)<<6) | (uint32_t)(23U)),
    RCU_FLAG_OBLRST = (((uint32_t)(IDX_RSTSCK)<<6) | (uint32_t)(25U)),
    RCU_FLAG_EPRST = (((uint32_t)(IDX_RSTSCK)<<6) | (uint32_t)(26U)),
    RCU_FLAG_PORRST = (((uint32_t)(IDX_RSTSCK)<<6) | (uint32_t)(27U)),
    RCU_FLAG_SWRST = (((uint32_t)(IDX_RSTSCK)<<6) | (uint32_t)(28U)),
    RCU_FLAG_FWDGTRST = (((uint32_t)(IDX_RSTSCK)<<6) | (uint32_t)(29U)),
    RCU_FLAG_WWDGTRST = (((uint32_t)(IDX_RSTSCK)<<6) | (uint32_t)(30U)),
    RCU_FLAG_LPRST = (((uint32_t)(IDX_RSTSCK)<<6) | (uint32_t)(31U))
}rcu_flag_enum;


typedef enum
{
    RCU_INT_FLAG_IRC40KSTB = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(0U)),
    RCU_INT_FLAG_LXTALSTB = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(1U)),
    RCU_INT_FLAG_IRC8MSTB = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(2U)),
    RCU_INT_FLAG_HXTALSTB = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(3U)),
    RCU_INT_FLAG_PLLSTB = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(4U)),
    RCU_INT_FLAG_IRC28MSTB = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(5U)),
    RCU_INT_FLAG_CKM = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(7U)),
}rcu_int_flag_enum;


typedef enum
{
    RCU_INT_FLAG_IRC40KSTB_CLR = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(16U)),
    RCU_INT_FLAG_LXTALSTB_CLR = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(17U)),
    RCU_INT_FLAG_IRC8MSTB_CLR = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(18U)),
    RCU_INT_FLAG_HXTALSTB_CLR = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(19U)),
    RCU_INT_FLAG_PLLSTB_CLR = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(20U)),
    RCU_INT_FLAG_IRC28MSTB_CLR = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(21U)),
    RCU_INT_FLAG_CKM_CLR = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(23U)),
}rcu_int_flag_clear_enum;


typedef enum
{
    RCU_INT_IRC40KSTB = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(8U)),
    RCU_INT_LXTALSTB = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(9U)),
    RCU_INT_IRC8MSTB = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(10U)),
    RCU_INT_HXTALSTB = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(11U)),
    RCU_INT_PLLSTB = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(12U)),
    RCU_INT_IRC28MSTB = (((uint32_t)(IDX_INT)<<6) | (uint32_t)(13U)),
}rcu_int_enum;


typedef enum
{
    RCU_ADCCK_IRC28M_DIV2 = 0U,
    RCU_ADCCK_IRC28M,
    RCU_ADCCK_APB2_DIV2,
    RCU_ADCCK_AHB_DIV3,
    RCU_ADCCK_APB2_DIV4,
    RCU_ADCCK_AHB_DIV5,
    RCU_ADCCK_APB2_DIV6,
    RCU_ADCCK_AHB_DIV7,
    RCU_ADCCK_APB2_DIV8,
    RCU_ADCCK_AHB_DIV9
}rcu_adc_clock_enum;


typedef enum
{
    RCU_HXTAL = (((uint32_t)(IDX_CTL0)<<6) | (uint32_t)(16U)),
    RCU_LXTAL = (((uint32_t)(IDX_BDCTL)<<6) | (uint32_t)(0U)),
    RCU_IRC8M = (((uint32_t)(IDX_CTL0)<<6) | (uint32_t)(0U)),
    RCU_IRC28M = (((uint32_t)(IDX_CTL1)<<6) | (uint32_t)(0U)),
    RCU_IRC40K = (((uint32_t)(IDX_RSTSCK)<<6) | (uint32_t)(0U)),
    RCU_PLL_CK = (((uint32_t)(IDX_CTL0)<<6) | (uint32_t)(24U))
}rcu_osci_type_enum;


typedef enum
{
    CK_SYS = 0U,
    CK_AHB,
    CK_APB1,
    CK_APB2,
    CK_ADC,
    CK_USART
}rcu_clock_freq_enum;
# 588 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_rcu.h"
void rcu_deinit(void);

void rcu_periph_clock_enable(rcu_periph_enum periph);

void rcu_periph_clock_disable(rcu_periph_enum periph);

void rcu_periph_clock_sleep_enable(rcu_periph_sleep_enum periph);

void rcu_periph_clock_sleep_disable(rcu_periph_sleep_enum periph);

void rcu_periph_reset_enable(rcu_periph_reset_enum periph_reset);

void rcu_periph_reset_disable(rcu_periph_reset_enum periph_reset);

void rcu_bkp_reset_enable(void);

void rcu_bkp_reset_disable(void);


void rcu_system_clock_source_config(uint32_t ck_sys);

uint32_t rcu_system_clock_source_get(void);

void rcu_ahb_clock_config(uint32_t ck_ahb);

void rcu_apb1_clock_config(uint32_t ck_apb1);

void rcu_apb2_clock_config(uint32_t ck_apb2);

void rcu_adc_clock_config(rcu_adc_clock_enum ck_adc);

void rcu_ckout_config(uint32_t ckout_src, uint32_t ckout_div);


void rcu_pll_config(uint32_t pll_src, uint32_t pll_mul);

void rcu_usart_clock_config(uint32_t ck_usart);

void rcu_rtc_clock_config(uint32_t rtc_clock_source);

void rcu_hxtal_prediv_config(uint32_t hxtal_prediv);

void rcu_lxtal_drive_capability_config(uint32_t lxtal_dricap);


FlagStatus rcu_flag_get(rcu_flag_enum flag);

void rcu_all_reset_flag_clear(void);

FlagStatus rcu_interrupt_flag_get(rcu_int_flag_enum int_flag);

void rcu_interrupt_flag_clear(rcu_int_flag_clear_enum int_flag_clear);

void rcu_interrupt_enable(rcu_int_enum stab_int);

void rcu_interrupt_disable(rcu_int_enum stab_int);


ErrStatus rcu_osci_stab_wait(rcu_osci_type_enum osci);

void rcu_osci_on(rcu_osci_type_enum osci);

void rcu_osci_off(rcu_osci_type_enum osci);

void rcu_osci_bypass_mode_enable(rcu_osci_type_enum osci);

void rcu_osci_bypass_mode_disable(rcu_osci_type_enum osci);

void rcu_hxtal_clock_monitor_enable(void);

void rcu_hxtal_clock_monitor_disable(void);


void rcu_irc8m_adjust_value_set(uint8_t irc8m_adjval);

void rcu_irc28m_adjust_value_set(uint8_t irc28m_adjval);

void rcu_voltage_key_unlock(void);

void rcu_deepsleep_voltage_set(uint32_t dsvol);


uint32_t rcu_clock_freq_get(rcu_clock_freq_enum clock);
# 52 "../User\\gd32e230_libopt.h" 2
# 1 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_rtc.h" 1
# 40 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_rtc.h"
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 41 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_rtc.h" 2
# 211 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_rtc.h"
typedef struct
{
    uint8_t rtc_year;
    uint8_t rtc_month;
    uint8_t rtc_date;
    uint8_t rtc_day_of_week;
    uint8_t rtc_hour;
    uint8_t rtc_minute;
    uint8_t rtc_second;
    uint16_t rtc_factor_asyn;
    uint16_t rtc_factor_syn;
    uint32_t rtc_am_pm;
    uint32_t rtc_display_format;
}rtc_parameter_struct;


typedef struct
{
    uint32_t rtc_alarm_mask;
    uint32_t rtc_weekday_or_date;
    uint8_t rtc_alarm_day;
    uint8_t rtc_alarm_hour;
    uint8_t rtc_alarm_minute;
    uint8_t rtc_alarm_second;
    uint32_t rtc_am_pm;
}rtc_alarm_struct;


typedef struct
{
    uint8_t rtc_timestamp_month;
    uint8_t rtc_timestamp_date;
    uint8_t rtc_timestamp_day;
    uint8_t rtc_timestamp_hour;
    uint8_t rtc_timestamp_minute;
    uint8_t rtc_timestamp_second;
    uint32_t rtc_am_pm;
}rtc_timestamp_struct;


typedef struct
{
    uint32_t rtc_tamper_source;
    uint32_t rtc_tamper_trigger;
    uint32_t rtc_tamper_filter;
    uint32_t rtc_tamper_sample_frequency;
    ControlStatus rtc_tamper_precharge_enable;
    uint32_t rtc_tamper_precharge_time;
    ControlStatus rtc_tamper_with_timestamp;
}rtc_tamper_struct;
# 493 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_rtc.h"
ErrStatus rtc_deinit(void);

ErrStatus rtc_init(rtc_parameter_struct* rtc_initpara_struct);

ErrStatus rtc_init_mode_enter(void);

void rtc_init_mode_exit(void);

ErrStatus rtc_register_sync_wait(void);


void rtc_current_time_get(rtc_parameter_struct* rtc_initpara_struct);

uint32_t rtc_subsecond_get(void);


void rtc_alarm_config(rtc_alarm_struct* rtc_alarm_time);

void rtc_alarm_subsecond_config(uint32_t mask_subsecond, uint32_t subsecond);

void rtc_alarm_get(rtc_alarm_struct* rtc_alarm_time);

uint32_t rtc_alarm_subsecond_get(void);

void rtc_alarm_enable(void);

ErrStatus rtc_alarm_disable(void);


void rtc_timestamp_enable(uint32_t edge);

void rtc_timestamp_disable(void);

void rtc_timestamp_get(rtc_timestamp_struct* rtc_timestamp);

uint32_t rtc_timestamp_subsecond_get(void);


void rtc_tamper_enable(rtc_tamper_struct* rtc_tamper);

void rtc_tamper_disable(uint32_t source);


void rtc_interrupt_enable(uint32_t interrupt);

void rtc_interrupt_disable(uint32_t interrupt);

FlagStatus rtc_flag_get(uint32_t flag);

void rtc_flag_clear(uint32_t flag);


void rtc_alter_output_config(uint32_t source, uint32_t mode);

ErrStatus rtc_calibration_config(uint32_t window, uint32_t plus, uint32_t minus);

void rtc_hour_adjust(uint32_t operation);

ErrStatus rtc_second_adjust(uint32_t add, uint32_t minus);

void rtc_bypass_shadow_enable(void);

void rtc_bypass_shadow_disable(void);

ErrStatus rtc_refclock_detection_enable(void);

ErrStatus rtc_refclock_detection_disable(void);
# 53 "../User\\gd32e230_libopt.h" 2
# 1 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_spi.h" 1
# 40 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_spi.h"
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 41 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_spi.h" 2
# 141 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_spi.h"
typedef struct
{
    uint32_t device_mode;
    uint32_t trans_mode;
    uint32_t frame_size;
    uint32_t nss;
    uint32_t endian;
    uint32_t clock_polarity_phase;
    uint32_t prescale;
}spi_parameter_struct;
# 319 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_spi.h"
void spi_i2s_deinit(uint32_t spi_periph);

void spi_struct_para_init(spi_parameter_struct* spi_struct);

ErrStatus spi_init(uint32_t spi_periph, spi_parameter_struct* spi_struct);

void spi_enable(uint32_t spi_periph);

void spi_disable(uint32_t spi_periph);


void i2s_init(uint32_t spi_periph, uint32_t mode, uint32_t standard, uint32_t ckpl);

void i2s_psc_config(uint32_t spi_periph, uint32_t audiosample, uint32_t frameformat, uint32_t mckout);

void i2s_enable(uint32_t spi_periph);

void i2s_disable(uint32_t spi_periph);



void spi_nss_output_enable(uint32_t spi_periph);

void spi_nss_output_disable(uint32_t spi_periph);

void spi_nss_internal_high(uint32_t spi_periph);

void spi_nss_internal_low(uint32_t spi_periph);


void spi_dma_enable(uint32_t spi_periph, uint8_t dma);

void spi_dma_disable(uint32_t spi_periph, uint8_t dma);


ErrStatus spi_i2s_data_frame_format_config(uint32_t spi_periph, uint16_t frame_format);

void spi_i2s_data_transmit(uint32_t spi_periph, uint16_t data);

uint16_t spi_i2s_data_receive(uint32_t spi_periph);

void spi_bidirectional_transfer_config(uint32_t spi_periph, uint32_t transfer_direction);



void spi_crc_polynomial_set(uint32_t spi_periph, uint16_t crc_poly);

uint16_t spi_crc_polynomial_get(uint32_t spi_periph);

void spi_crc_on(uint32_t spi_periph);

void spi_crc_off(uint32_t spi_periph);

void spi_crc_next(uint32_t spi_periph);

uint16_t spi_crc_get(uint32_t spi_periph, uint8_t crc);



void spi_ti_mode_enable(uint32_t spi_periph);

void spi_ti_mode_disable(uint32_t spi_periph);



void spi_nssp_mode_enable(uint32_t spi_periph);

void spi_nssp_mode_disable(uint32_t spi_periph);



void qspi_enable(uint32_t spi_periph);

void qspi_disable(uint32_t spi_periph);

void qspi_write_enable(uint32_t spi_periph);

void qspi_read_enable(uint32_t spi_periph);

void qspi_io23_output_enable(uint32_t spi_periph);

void qspi_io23_output_disable(uint32_t spi_periph);



void spi_i2s_interrupt_enable(uint32_t spi_periph, uint8_t interrupt);

void spi_i2s_interrupt_disable(uint32_t spi_periph, uint8_t interrupt);

FlagStatus spi_i2s_interrupt_flag_get(uint32_t spi_periph, uint8_t interrupt);

FlagStatus spi_i2s_flag_get(uint32_t spi_periph, uint32_t flag);

void spi_crc_error_clear(uint32_t spi_periph);



void spi_fifo_access_size_config(uint32_t spi_periph, uint16_t fifo_access_size);

void spi_transmit_odd_config(uint32_t spi_periph, uint16_t odd);

void spi_receive_odd_config(uint32_t spi_periph, uint16_t odd);

void spi_crc_length_set(uint32_t spi_periph, uint16_t crc_length);
# 54 "../User\\gd32e230_libopt.h" 2
# 1 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_timer.h" 1
# 40 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_timer.h"
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 41 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_timer.h" 2
# 254 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_timer.h"
typedef struct
{
    uint16_t prescaler;
    uint16_t alignedmode;
    uint16_t counterdirection;
    uint16_t clockdivision;
    uint32_t period;
    uint8_t repetitioncounter;
}timer_parameter_struct;


typedef struct
{
    uint16_t runoffstate;
    uint16_t ideloffstate;
    uint16_t deadtime;
    uint16_t breakpolarity;
    uint16_t outputautostate;
    uint16_t protectmode;
    uint16_t breakstate;
}timer_break_parameter_struct;


typedef struct
{
    uint16_t outputstate;
    uint16_t outputnstate;
    uint16_t ocpolarity;
    uint16_t ocnpolarity;
    uint16_t ocidlestate;
    uint16_t ocnidlestate;
}timer_oc_parameter_struct;


typedef struct
{
    uint16_t icpolarity;
    uint16_t icselection;
    uint16_t icprescaler;
    uint16_t icfilter;
}timer_ic_parameter_struct;
# 600 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_timer.h"
void timer_deinit(uint32_t timer_periph);

void timer_struct_para_init(timer_parameter_struct* initpara);

void timer_init(uint32_t timer_periph, timer_parameter_struct* initpara);

void timer_enable(uint32_t timer_periph);

void timer_disable(uint32_t timer_periph);

void timer_auto_reload_shadow_enable(uint32_t timer_periph);

void timer_auto_reload_shadow_disable(uint32_t timer_periph);

void timer_update_event_enable(uint32_t timer_periph);

void timer_update_event_disable(uint32_t timer_periph);

void timer_counter_alignment(uint32_t timer_periph, uint16_t aligned);

void timer_counter_up_direction(uint32_t timer_periph);

void timer_counter_down_direction(uint32_t timer_periph);

void timer_prescaler_config(uint32_t timer_periph, uint16_t prescaler, uint8_t pscreload);

void timer_repetition_value_config(uint32_t timer_periph, uint16_t repetition);

void timer_autoreload_value_config(uint32_t timer_periph, uint16_t autoreload);

void timer_counter_value_config(uint32_t timer_periph , uint16_t counter);

uint32_t timer_counter_read(uint32_t timer_periph);

uint16_t timer_prescaler_read(uint32_t timer_periph);

void timer_single_pulse_mode_config(uint32_t timer_periph, uint32_t spmode);

void timer_update_source_config(uint32_t timer_periph, uint32_t update);

void timer_ocpre_clear_source_config(uint32_t timer_periph, uint8_t ocpreclear);



void timer_interrupt_enable(uint32_t timer_periph, uint32_t interrupt);

void timer_interrupt_disable(uint32_t timer_periph, uint32_t interrupt);

FlagStatus timer_interrupt_flag_get(uint32_t timer_periph, uint32_t interrupt);

void timer_interrupt_flag_clear(uint32_t timer_periph, uint32_t interrupt);

FlagStatus timer_flag_get(uint32_t timer_periph, uint32_t flag);

void timer_flag_clear(uint32_t timer_periph, uint32_t flag);



void timer_dma_enable(uint32_t timer_periph, uint16_t dma);

void timer_dma_disable(uint32_t timer_periph, uint16_t dma);

void timer_channel_dma_request_source_select(uint32_t timer_periph, uint8_t dma_request);

void timer_dma_transfer_config(uint32_t timer_periph,uint32_t dma_baseaddr, uint32_t dma_lenth);

void timer_event_software_generate(uint32_t timer_periph, uint16_t event);



void timer_break_struct_para_init(timer_break_parameter_struct* breakpara);

void timer_break_config(uint32_t timer_periph, timer_break_parameter_struct* breakpara);

void timer_break_enable(uint32_t timer_periph);

void timer_break_disable(uint32_t timer_periph);

void timer_automatic_output_enable(uint32_t timer_periph);

void timer_automatic_output_disable(uint32_t timer_periph);

void timer_primary_output_config(uint32_t timer_periph, ControlStatus newvalue);

void timer_channel_control_shadow_config(uint32_t timer_periph, ControlStatus newvalue);

void timer_channel_control_shadow_update_config(uint32_t timer_periph, uint8_t ccuctl);



void timer_channel_output_struct_para_init(timer_oc_parameter_struct* ocpara);

void timer_channel_output_config(uint32_t timer_periph,uint16_t channel, timer_oc_parameter_struct* ocpara);

void timer_channel_output_mode_config(uint32_t timer_periph, uint16_t channel,uint16_t ocmode);

void timer_channel_output_pulse_value_config(uint32_t timer_periph, uint16_t channel, uint32_t pulse);

void timer_channel_output_shadow_config(uint32_t timer_periph, uint16_t channel, uint16_t ocshadow);

void timer_channel_output_fast_config(uint32_t timer_periph, uint16_t channel, uint16_t ocfast);

void timer_channel_output_clear_config(uint32_t timer_periph,uint16_t channel,uint16_t occlear);

void timer_channel_output_polarity_config(uint32_t timer_periph, uint16_t channel, uint16_t ocpolarity);

void timer_channel_complementary_output_polarity_config(uint32_t timer_periph, uint16_t channel, uint16_t ocnpolarity);

void timer_channel_output_state_config(uint32_t timer_periph, uint16_t channel, uint32_t state);

void timer_channel_complementary_output_state_config(uint32_t timer_periph, uint16_t channel, uint16_t ocnstate);



void timer_channel_input_struct_para_init(timer_ic_parameter_struct* icpara);

void timer_input_capture_config(uint32_t timer_periph, uint16_t channel, timer_ic_parameter_struct* icpara);

void timer_channel_input_capture_prescaler_config(uint32_t timer_periph, uint16_t channel, uint16_t prescaler);

uint32_t timer_channel_capture_value_register_read(uint32_t timer_periph, uint16_t channel);

void timer_input_pwm_capture_config(uint32_t timer_periph, uint16_t channel, timer_ic_parameter_struct* icpwm);

void timer_hall_mode_config(uint32_t timer_periph, uint32_t hallmode);



void timer_input_trigger_source_select(uint32_t timer_periph, uint32_t intrigger);

void timer_master_output_trigger_source_select(uint32_t timer_periph, uint32_t outrigger);

void timer_slave_mode_select(uint32_t timer_periph,uint32_t slavemode);

void timer_master_slave_mode_config(uint32_t timer_periph, uint32_t masterslave);

void timer_external_trigger_config(uint32_t timer_periph, uint32_t extprescaler, uint32_t extpolarity, uint32_t extfilter);

void timer_quadrature_decoder_mode_config(uint32_t timer_periph, uint32_t decomode, uint16_t ic0polarity, uint16_t ic1polarity);

void timer_internal_clock_config(uint32_t timer_periph);

void timer_internal_trigger_as_external_clock_config(uint32_t timer_periph, uint32_t intrigger);

void timer_external_trigger_as_external_clock_config(uint32_t timer_periph, uint32_t extrigger, uint16_t extpolarity,uint32_t extfilter);

void timer_external_clock_mode0_config(uint32_t timer_periph, uint32_t extprescaler, uint32_t extpolarity, uint32_t extfilter);

void timer_external_clock_mode1_config(uint32_t timer_periph, uint32_t extprescaler, uint32_t extpolarity, uint32_t extfilter);

void timer_external_clock_mode1_disable(uint32_t timer_periph);

void timer_channel_remap_config(uint32_t timer_periph,uint32_t remap);



void timer_write_chxval_register_config(uint32_t timer_periph, uint16_t ccsel);

void timer_output_value_selection_config(uint32_t timer_periph, uint16_t outsel);
# 55 "../User\\gd32e230_libopt.h" 2
# 1 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_usart.h" 1
# 40 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_usart.h"
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 41 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_usart.h" 2
# 221 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_usart.h"
typedef enum{

    USART_FLAG_REA = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(22U)),
    USART_FLAG_TEA = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(21U)),
    USART_FLAG_WU = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(20U)),
    USART_FLAG_RWU = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(19U)),
    USART_FLAG_SB = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(18U)),
    USART_FLAG_AM = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(17U)),
    USART_FLAG_BSY = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(16U)),
    USART_FLAG_ABD = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(15U)),
    USART_FLAG_ABDE = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(14U)),
    USART_FLAG_EB = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(12U)),
    USART_FLAG_RT = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(11U)),
    USART_FLAG_CTS = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(10U)),
    USART_FLAG_CTSF = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(9U)),
    USART_FLAG_LBD = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(8U)),
    USART_FLAG_TBE = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(7U)),
    USART_FLAG_TC = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(6U)),
    USART_FLAG_RBNE = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(5U)),
    USART_FLAG_IDLE = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(4U)),
    USART_FLAG_ORERR = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(3U)),
    USART_FLAG_NERR = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(2U)),
    USART_FLAG_FERR = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(1U)),
    USART_FLAG_PERR = (((uint32_t)(0x0000001CU) << 6) | (uint32_t)(0U)),

    USART_FLAG_EPERR = (((uint32_t)(0x000000C0U) << 6) | (uint32_t)(8U)),

    USART_FLAG_RFFINT = (((uint32_t)(0x000000D0U) << 6) | (uint32_t)(15U)),
    USART_FLAG_RFF = (((uint32_t)(0x000000D0U) << 6) | (uint32_t)(11U)),
    USART_FLAG_RFE = (((uint32_t)(0x000000D0U) << 6) | (uint32_t)(10U)),
}usart_flag_enum;


typedef enum
{

    USART_INT_FLAG_EB = (((uint32_t)(0x0000001CU) << 22) | (uint32_t)((12U) << 16) | (((uint32_t)(0x00000000U) << 6) | (uint32_t)(27U))),
    USART_INT_FLAG_RT = (((uint32_t)(0x0000001CU) << 22) | (uint32_t)((11U) << 16) | (((uint32_t)(0x00000000U) << 6) | (uint32_t)(26U))),
    USART_INT_FLAG_AM = (((uint32_t)(0x0000001CU) << 22) | (uint32_t)((17U) << 16) | (((uint32_t)(0x00000000U) << 6) | (uint32_t)(14U))),
    USART_INT_FLAG_PERR = (((uint32_t)(0x0000001CU) << 22) | (uint32_t)((0U) << 16) | (((uint32_t)(0x00000000U) << 6) | (uint32_t)(8U))),
    USART_INT_FLAG_TBE = (((uint32_t)(0x0000001CU) << 22) | (uint32_t)((7U) << 16) | (((uint32_t)(0x00000000U) << 6) | (uint32_t)(7U))),
    USART_INT_FLAG_TC = (((uint32_t)(0x0000001CU) << 22) | (uint32_t)((6U) << 16) | (((uint32_t)(0x00000000U) << 6) | (uint32_t)(6U))),
    USART_INT_FLAG_RBNE = (((uint32_t)(0x0000001CU) << 22) | (uint32_t)((5U) << 16) | (((uint32_t)(0x00000000U) << 6) | (uint32_t)(5U))),
    USART_INT_FLAG_RBNE_ORERR = (((uint32_t)(0x0000001CU) << 22) | (uint32_t)((3U) << 16) | (((uint32_t)(0x00000000U) << 6) | (uint32_t)(5U))),
    USART_INT_FLAG_IDLE = (((uint32_t)(0x0000001CU) << 22) | (uint32_t)((4U) << 16) | (((uint32_t)(0x00000000U) << 6) | (uint32_t)(4U))),

    USART_INT_FLAG_LBD = (((uint32_t)(0x0000001CU) << 22) | (uint32_t)((8U) << 16) | (((uint32_t)(0x00000004U) << 6) | (uint32_t)(6U))),

    USART_INT_FLAG_WU = (((uint32_t)(0x0000001CU) << 22) | (uint32_t)((20U) << 16) | (((uint32_t)(0x00000008U) << 6) | (uint32_t)(22U))),
    USART_INT_FLAG_CTS = (((uint32_t)(0x0000001CU) << 22) | (uint32_t)((9U) << 16) | (((uint32_t)(0x00000008U) << 6) | (uint32_t)(10U))),
    USART_INT_FLAG_ERR_NERR = (((uint32_t)(0x0000001CU) << 22) | (uint32_t)((2U) << 16) | (((uint32_t)(0x00000008U) << 6) | (uint32_t)(0U))),
    USART_INT_FLAG_ERR_ORERR = (((uint32_t)(0x0000001CU) << 22) | (uint32_t)((3U) << 16) | (((uint32_t)(0x00000008U) << 6) | (uint32_t)(0U))),
    USART_INT_FLAG_ERR_FERR = (((uint32_t)(0x0000001CU) << 22) | (uint32_t)((1U) << 16) | (((uint32_t)(0x00000008U) << 6) | (uint32_t)(0U))),

    USART_INT_FLAG_RFF = (((uint32_t)(0x000000D0U) << 22) | (uint32_t)((15U) << 16) | (((uint32_t)(0x000000D0U) << 6) | (uint32_t)(9U))),
}usart_interrupt_flag_enum;


typedef enum
{

    USART_INT_EB = (((uint32_t)(0x00000000U) << 6) | (uint32_t)(27U)),
    USART_INT_RT = (((uint32_t)(0x00000000U) << 6) | (uint32_t)(26U)),
    USART_INT_AM = (((uint32_t)(0x00000000U) << 6) | (uint32_t)(14U)),
    USART_INT_PERR = (((uint32_t)(0x00000000U) << 6) | (uint32_t)(8U)),
    USART_INT_TBE = (((uint32_t)(0x00000000U) << 6) | (uint32_t)(7U)),
    USART_INT_TC = (((uint32_t)(0x00000000U) << 6) | (uint32_t)(6U)),
    USART_INT_RBNE = (((uint32_t)(0x00000000U) << 6) | (uint32_t)(5U)),
    USART_INT_IDLE = (((uint32_t)(0x00000000U) << 6) | (uint32_t)(4U)),

    USART_INT_LBD = (((uint32_t)(0x00000004U) << 6) | (uint32_t)(6U)),

    USART_INT_WU = (((uint32_t)(0x00000008U) << 6) | (uint32_t)(22U)),
    USART_INT_CTS = (((uint32_t)(0x00000008U) << 6) | (uint32_t)(10U)),
    USART_INT_ERR = (((uint32_t)(0x00000008U) << 6) | (uint32_t)(0U)),

    USART_INT_RFF = (((uint32_t)(0x000000D0U) << 6) | (uint32_t)(9U)),
}usart_interrupt_enum;


typedef enum {

    USART_DINV_ENABLE,
    USART_DINV_DISABLE,

    USART_TXPIN_ENABLE,
    USART_TXPIN_DISABLE,

    USART_RXPIN_ENABLE,
    USART_RXPIN_DISABLE,

    USART_SWAP_ENABLE,
    USART_SWAP_DISABLE,
}usart_invert_enum;
# 438 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_usart.h"
void usart_deinit(uint32_t usart_periph);

void usart_baudrate_set(uint32_t usart_periph, uint32_t baudval);

void usart_parity_config(uint32_t usart_periph, uint32_t paritycfg);

void usart_word_length_set(uint32_t usart_periph, uint32_t wlen);

void usart_stop_bit_set(uint32_t usart_periph, uint32_t stblen);

void usart_enable(uint32_t usart_periph);

void usart_disable(uint32_t usart_periph);

void usart_transmit_config(uint32_t usart_periph, uint32_t txconfig);

void usart_receive_config(uint32_t usart_periph, uint32_t rxconfig);



void usart_data_first_config(uint32_t usart_periph, uint32_t msbf);

void usart_invert_config(uint32_t usart_periph, usart_invert_enum invertpara);

void usart_overrun_enable(uint32_t usart_periph);

void usart_overrun_disable(uint32_t usart_periph);

void usart_oversample_config(uint32_t usart_periph, uint32_t oversamp);

void usart_sample_bit_config(uint32_t usart_periph, uint32_t osb);

void usart_receiver_timeout_enable(uint32_t usart_periph);

void usart_receiver_timeout_disable(uint32_t usart_periph);

void usart_receiver_timeout_threshold_config(uint32_t usart_periph, uint32_t rtimeout);

void usart_data_transmit(uint32_t usart_periph, uint32_t data);

uint16_t usart_data_receive(uint32_t usart_periph);



void usart_autobaud_detection_enable(uint32_t usart_periph);

void usart_autobaud_detection_disable(uint32_t usart_periph);

void usart_autobaud_detection_mode_config(uint32_t usart_periph, uint32_t abdmod);



void usart_address_config(uint32_t usart_periph, uint8_t addr);

void usart_address_detection_mode_config(uint32_t usart_periph, uint32_t addmod);

void usart_mute_mode_enable(uint32_t usart_periph);

void usart_mute_mode_disable(uint32_t usart_periph);

void usart_mute_mode_wakeup_config(uint32_t usart_periph, uint32_t wmethod);



void usart_lin_mode_enable(uint32_t usart_periph);

void usart_lin_mode_disable(uint32_t usart_periph);

void usart_lin_break_detection_length_config(uint32_t usart_periph, uint32_t lblen);



void usart_halfduplex_enable(uint32_t usart_periph);

void usart_halfduplex_disable(uint32_t usart_periph);



void usart_clock_enable(uint32_t usart_periph);

void usart_clock_disable(uint32_t usart_periph);

void usart_synchronous_clock_config(uint32_t usart_periph, uint32_t clen, uint32_t cph, uint32_t cpl);



void usart_guard_time_config(uint32_t usart_periph, uint32_t guat);

void usart_smartcard_mode_enable(uint32_t usart_periph);

void usart_smartcard_mode_disable(uint32_t usart_periph);

void usart_smartcard_mode_nack_enable(uint32_t usart_periph);

void usart_smartcard_mode_nack_disable(uint32_t usart_periph);

void usart_smartcard_mode_early_nack_enable(uint32_t usart_periph);

void usart_smartcard_mode_early_nack_disable(uint32_t usart_periph);

void usart_smartcard_autoretry_config(uint32_t usart_periph, uint32_t scrtnum);

void usart_block_length_config(uint32_t usart_periph, uint32_t bl);



void usart_irda_mode_enable(uint32_t usart_periph);

void usart_irda_mode_disable(uint32_t usart_periph);

void usart_prescaler_config(uint32_t usart_periph, uint32_t psc);

void usart_irda_lowpower_config(uint32_t usart_periph, uint32_t irlp);



void usart_hardware_flow_rts_config(uint32_t usart_periph, uint32_t rtsconfig);

void usart_hardware_flow_cts_config(uint32_t usart_periph, uint32_t ctsconfig);



void usart_hardware_flow_coherence_config(uint32_t usart_periph, uint32_t hcm);


void usart_rs485_driver_enable(uint32_t usart_periph);

void usart_rs485_driver_disable(uint32_t usart_periph);

void usart_driver_assertime_config(uint32_t usart_periph, uint32_t deatime);

void usart_driver_deassertime_config(uint32_t usart_periph, uint32_t dedtime);

void usart_depolarity_config(uint32_t usart_periph, uint32_t dep);



void usart_dma_receive_config(uint32_t usart_periph, uint32_t dmacmd);

void usart_dma_transmit_config(uint32_t usart_periph, uint32_t dmacmd);

void usart_reception_error_dma_disable(uint32_t usart_periph);

void usart_reception_error_dma_enable(uint32_t usart_periph);


void usart_wakeup_enable(uint32_t usart_periph);

void usart_wakeup_disable(uint32_t usart_periph);

void usart_wakeup_mode_config(uint32_t usart_periph, uint32_t wum);



void usart_receive_fifo_enable(uint32_t usart_periph);

void usart_receive_fifo_disable(uint32_t usart_periph);

uint8_t usart_receive_fifo_counter_number(uint32_t usart_periph);



FlagStatus usart_flag_get(uint32_t usart_periph, usart_flag_enum flag);

void usart_flag_clear(uint32_t usart_periph, usart_flag_enum flag);

void usart_interrupt_enable(uint32_t usart_periph, usart_interrupt_enum inttype);

void usart_interrupt_disable(uint32_t usart_periph, usart_interrupt_enum inttype);

void usart_command_enable(uint32_t usart_periph, uint32_t cmdtype);

FlagStatus usart_interrupt_flag_get(uint32_t usart_periph, usart_interrupt_flag_enum int_flag);

void usart_interrupt_flag_clear(uint32_t usart_periph, usart_interrupt_flag_enum flag);
# 56 "../User\\gd32e230_libopt.h" 2
# 1 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_wwdgt.h" 1
# 40 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_wwdgt.h"
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 41 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_wwdgt.h" 2
# 72 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_wwdgt.h"
void wwdgt_deinit(void);

void wwdgt_enable(void);


void wwdgt_counter_update(uint16_t counter_value);

void wwdgt_config(uint16_t counter, uint16_t window, uint32_t prescaler);


void wwdgt_interrupt_enable(void);

FlagStatus wwdgt_flag_get(void);

void wwdgt_flag_clear(void);
# 57 "../User\\gd32e230_libopt.h" 2
# 1 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_misc.h" 1
# 40 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_misc.h"
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 41 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_misc.h" 2
# 69 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_misc.h"
void nvic_irq_enable(uint8_t nvic_irq, uint8_t nvic_irq_priority);

void nvic_irq_disable(uint8_t nvic_irq);

void nvic_system_reset(void);


void nvic_vector_table_set(uint32_t nvic_vict_tab, uint32_t offset);


void system_lowpower_set(uint8_t lowpower_mode);

void system_lowpower_reset(uint8_t lowpower_mode);


void systick_clksource_set(uint32_t systick_clksource);
# 58 "../User\\gd32e230_libopt.h" 2
# 1 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_cmp.h" 1
# 40 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_cmp.h"
# 1 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 1
# 41 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_cmp.h" 2
# 61 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_cmp.h"
typedef enum{
    CMP_HIGHSPEED = 0,
    CMP_MIDDLESPEED,
    CMP_LOWSPEED,
    CMP_VERYLOWSPEED
}operating_mode_enum;


typedef enum{
    CMP_1_4VREFINT = 0,
    CMP_1_2VREFINT,
    CMP_3_4VREFINT,
    CMP_VREFINT,
    CMP_PA4,
    CMP_PA5,
    CMP_PA0,
    CMP_PA2
}inverting_input_enum;


typedef enum{
    CMP_HYSTERESIS_NO = 0,
    CMP_HYSTERESIS_LOW,
    CMP_HYSTERESIS_MIDDLE,
    CMP_HYSTERESIS_HIGH
}cmp_hysteresis_enum;


typedef enum{
    CMP_OUTPUT_NONE = 0x0U,
    CMP_OUTPUT_TIMER0BKIN = 0x1U,
    CMP_OUTPUT_TIMER0IC0 = 0x2U,
    CMP_OUTPUT_TIMER0OCPRECLR = 0x3U,
    CMP_OUTPUT_TIMER2IC0 = 0x06U,
    CMP_OUTPUT_TIMER2OCPRECLR = 0x7U
}cmp_output_enum;
# 144 "../GD32E230_Firmware_Library/GD32E230_standard_peripheral/Include\\gd32e230_cmp.h"
void cmp_deinit(void);

void cmp_mode_init(operating_mode_enum operating_mode, inverting_input_enum inverting_input, cmp_hysteresis_enum output_hysteresis);

void cmp_output_init(cmp_output_enum output_slection, uint32_t output_polarity);

void cmp_enable(void);

void cmp_disable(void);

void cmp_switch_enable(void);

void cmp_switch_disable(void);

uint32_t cmp_output_level_get(void);

void cmp_lock_enable(void);
# 59 "../User\\gd32e230_libopt.h" 2
# 207 "../GD32E230_Firmware_Library/CMSIS/GD/GD32E230/Include\\gd32e230.h" 2
# 4 "../APP/Inc\\type.h" 2

# 1 "../Utilities\\gd32e230c_eval.h" 1
# 50 "../Utilities\\gd32e230c_eval.h"
typedef enum
{
    LED1 = 0,
    LED2 = 1,
    LED3 = 2,
    LED4 = 3
}led_typedef_enum;

typedef enum
{
    KEY_WAKEUP = 0,
    KEY_TAMPER = 1,
}key_typedef_enum;

typedef enum
{
    KEY_MODE_GPIO = 0,
    KEY_MODE_EXTI = 1
}keymode_typedef_enum;
# 136 "../Utilities\\gd32e230c_eval.h"
void gd_eval_led_init(led_typedef_enum lednum);

void gd_eval_led_on(led_typedef_enum lednum);

void gd_eval_led_off(led_typedef_enum lednum);

void gd_eval_led_toggle(led_typedef_enum lednum);

void gd_eval_key_init(key_typedef_enum keynum, keymode_typedef_enum keymode);

uint8_t gd_eval_key_state_get(key_typedef_enum keynum);

void gd_eval_com_init(uint32_t com);

void com_gpio_init(void);
void com_usart_init(void);

uint32_t UARTx_SendData(uint8_t* UARTx_SendBuff, uint32_t Len);
# 6 "../APP/Inc\\type.h" 2
# 1 "../BSP/Inc\\flash.h" 1
# 43 "../BSP/Inc\\flash.h"
# 1 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdbool.h" 1 3
# 44 "../BSP/Inc\\flash.h" 2
# 1 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 1 3
# 51 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
    typedef unsigned int size_t;






extern __attribute__((nothrow)) void *memcpy(void * __restrict ,
                    const void * __restrict , size_t ) __attribute__((__nonnull__(1,2)));






extern __attribute__((nothrow)) void *memmove(void * ,
                    const void * , size_t ) __attribute__((__nonnull__(1,2)));
# 77 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((nothrow)) char *strcpy(char * __restrict , const char * __restrict ) __attribute__((__nonnull__(1,2)));






extern __attribute__((nothrow)) char *strncpy(char * __restrict , const char * __restrict , size_t ) __attribute__((__nonnull__(1,2)));
# 93 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((nothrow)) char *strcat(char * __restrict , const char * __restrict ) __attribute__((__nonnull__(1,2)));






extern __attribute__((nothrow)) char *strncat(char * __restrict , const char * __restrict , size_t ) __attribute__((__nonnull__(1,2)));
# 117 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((nothrow)) int memcmp(const void * , const void * , size_t ) __attribute__((__nonnull__(1,2)));







extern __attribute__((nothrow)) int strcmp(const char * , const char * ) __attribute__((__nonnull__(1,2)));






extern __attribute__((nothrow)) int strncmp(const char * , const char * , size_t ) __attribute__((__nonnull__(1,2)));
# 141 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((nothrow)) int strcasecmp(const char * , const char * ) __attribute__((__nonnull__(1,2)));







extern __attribute__((nothrow)) int strncasecmp(const char * , const char * , size_t ) __attribute__((__nonnull__(1,2)));
# 158 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((nothrow)) int strcoll(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 169 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((nothrow)) size_t strxfrm(char * __restrict , const char * __restrict , size_t ) __attribute__((__nonnull__(2)));
# 193 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((nothrow)) void *memchr(const void * , int , size_t ) __attribute__((__nonnull__(1)));
# 209 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((nothrow)) char *strchr(const char * , int ) __attribute__((__nonnull__(1)));
# 218 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((nothrow)) size_t strcspn(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 232 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((nothrow)) char *strpbrk(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 247 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((nothrow)) char *strrchr(const char * , int ) __attribute__((__nonnull__(1)));
# 257 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((nothrow)) size_t strspn(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 270 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((nothrow)) char *strstr(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 280 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((nothrow)) char *strtok(char * __restrict , const char * __restrict ) __attribute__((__nonnull__(2)));
extern __attribute__((nothrow)) char *_strtok_r(char * , const char * , char ** ) __attribute__((__nonnull__(2,3)));
# 321 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((nothrow)) void *memset(void * , int , size_t ) __attribute__((__nonnull__(1)));





extern __attribute__((nothrow)) char *strerror(int );







extern __attribute__((nothrow)) size_t strlen(const char * ) __attribute__((__nonnull__(1)));






extern __attribute__((nothrow)) size_t strlcpy(char * , const char * , size_t ) __attribute__((__nonnull__(1,2)));
# 362 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((nothrow)) size_t strlcat(char * , const char * , size_t ) __attribute__((__nonnull__(1,2)));
# 388 "D:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((nothrow)) void _membitcpybl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((nothrow)) void _membitcpybb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((nothrow)) void _membitcpyhl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((nothrow)) void _membitcpyhb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((nothrow)) void _membitcpywl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((nothrow)) void _membitcpywb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((nothrow)) void _membitmovebl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((nothrow)) void _membitmovebb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((nothrow)) void _membitmovehl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((nothrow)) void _membitmovehb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((nothrow)) void _membitmovewl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((nothrow)) void _membitmovewb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
# 45 "../BSP/Inc\\flash.h" 2
# 90 "../BSP/Inc\\flash.h"
uint8_t Flash_Read_OneByte(uint32_t RWAddr);

void flash_read_multi(uint32_t readAdder, uint8_t *readBuf, uint16_t readLen);

uint32_t flash_read_MultiBytes(uint32_t read_addr, uint8_t *pBuf, uint16_t len);

uint8_t Flash_Write_OneByte(uint32_t RWAddr, uint8_t WrData);

uint8_t Flash_Write_MultiBytes(uint32_t RWAddr, uint8_t const *pWrbuf, uint16_t Wrlen);


uint32_t flash_write_word (uint32_t write_addr, uint32_t *pBuf, uint16_t wreLen);

uint8_t fmc_erase_for_app(uint32_t startAdder);

uint8_t fmc_erase_a_page(uint32_t startAdder);
uint8_t Flash_Write_32Bytes(uint32_t RWAddr, uint32_t data);

_Bool GDFLASHOP_EraseSpecifiedFlashPages( uint32_t startaddr, uint16_t npages );
# 7 "../APP/Inc\\type.h" 2
# 1 "../APP/Inc/modbus_asc.h" 1



# 1 "../APP/Inc/type.h" 1
# 5 "../APP/Inc/modbus_asc.h" 2
# 47 "../APP/Inc/modbus_asc.h"
void MBASC_Function(void);
void MBASC_AutoUpLoadFrame(void);
void MBASC_SendMsg(uint8_t *u8Msg, uint8_t u8MsgLen);

void MBASC_AutoUpLoadFrame(void);
# 8 "../APP/Inc/type.h" 2
# 1 "../APP/Inc/para.h" 1



# 1 "../APP/Inc/type.h" 1
# 5 "../APP/Inc/para.h" 2
# 28 "../APP/Inc/para.h"
void ReadPara(void);
# 9 "../APP/Inc/type.h" 2
# 1 "../User\\systick.h" 1
# 43 "../User\\systick.h"
void systick_config(void);

void delay_1ms(uint32_t count);

void delay_decrement(void);
# 10 "../APP/Inc/type.h" 2
# 1 "../APP/Inc/modbus_ascii.h" 1



# 1 "../APP/Inc/common.h" 1






void Delay_Ms(uint32_t cnt);
void Delay_Us(uint32_t cnt);

void Unshort2Array(uint16_t SourceData, uint8_t* Array);
void Unlong2Array(uint32_t SourceData, uint8_t* Array);

void long32Array(uint32_t SourceData, uint8_t* Array);
# 5 "../APP/Inc/modbus_ascii.h" 2







void MODBUS_ASCII_HexToAscii(uint8_t cyHexData, uint8_t *pCyAsciiBuf);
uint8_t MODBUS_ASCII_AsciiToHex(uint8_t *pCyAsciiBuf);
uint8_t MODBUS_ASCII_GetLrc(uint8_t *pCyAsciiBuf, uint16_t cyLen);
uint16_t MODBUS_ASCII_AsciiPacketToRtuPacket(uint8_t *pCyAsciiBuf, uint16_t cyAsciiLen, uint8_t *pCyRtuBuf);
uint16_t MODBUS_ASCII_RtuPacketToAsciiPacket(uint8_t *pCyRtuBuf, uint16_t cyRtuLen, uint8_t *pCyAsciiBuf);
void MODBUS_ASCII_HandlRevData(uint8_t cyRevData);
uint8_t MODBUS_ASCII_CheckAscii(uint8_t *pCyAsciiBuf, uint16_t cyLen);
uint8_t MODBUS_ASCII_RecvData(uint8_t* cyRecvBuff, uint16_t *pCyLen);
uint16_t MODBUS_ASCII_SendData(uint8_t *cySendBuff, uint16_t cyLen);
uint16_t MODBUS_ASCII_SendData1(uint8_t *cySendBuff, uint16_t cyLen);
# 11 "../APP/Inc/type.h" 2




void Led_Control(uint8_t color);


typedef enum
{
  Bit_RESET = 0,
  Bit_SET = 1
}BitAction;


typedef enum
{
    STA_STOP = 1,
    STA_WORK = 2,
}RotateStaTypeDef;


typedef enum
{
    Stall = 0,
    Foreward = 1,
    Reversal = 2,
}DirectionState_TypeDef;


typedef enum
{
    STA_NORMAL = 0,
    STA_TIMEOUT,
}StandbyTypeDef;

typedef struct
{
    int Temp;
    uint8_t SlaveAddr;
    uint8_t Baudrate;
    uint8_t Parity;

    uint16_t Up_Thr;
    uint16_t Do_Thr;
    uint16_t Du_Thr;
    uint32_t Duration;
    uint32_t AlarmSta;
 uint8_t Upload_persist;
}UserTypeDef;
# 2 "../APP/Src/modbus_ascii.c" 2


extern FlagStatus UartRecvFlag;
extern uint8_t receive;
extern BitAction UartRecvNewData;
 BitAction UartRecvFrameOK;
extern uint8_t UARTx_RXBuff[600];
extern uint8_t UARTx_TXBUFF[600];

static uint8_t g_cyRevState = 2;
static uint16_t g_cyRevBufffLen = 0;

uint8_t cyAsciiBuff[600];
# 41 "../APP/Src/modbus_ascii.c"
void MODBUS_ASCII_HexToAscii(uint8_t cyHexData, uint8_t *pCyAsciiBuf)
{
    uint8_t cyTemp;

    cyTemp = cyHexData / 16;
    if (10 > cyTemp)
    {
        *(pCyAsciiBuf + 0) = cyTemp + '0';
    }
    else
    {
        *(pCyAsciiBuf + 0) = (cyTemp - 10) + 'A';
    }

    cyTemp = cyHexData % 16;
    if (10 > cyTemp)
    {
        *(pCyAsciiBuf + 1) = cyTemp + '0';
    }
    else
    {
        *(pCyAsciiBuf + 1) = (cyTemp - 10) + 'A';
    }
}
# 78 "../APP/Src/modbus_ascii.c"
uint8_t MODBUS_ASCII_AsciiToHex(uint8_t *pCyAsciiBuf)
{
    uint8_t cyHexData;

    cyHexData = 0;
    if ('A' > *(pCyAsciiBuf + 0) )
    {
        cyHexData += *(pCyAsciiBuf + 0) - '0';
    }
    else if ('a' > *(pCyAsciiBuf + 0) )
    {
        cyHexData += *(pCyAsciiBuf + 0) - 'A' + 10;
    }
    else
    {
        cyHexData += *(pCyAsciiBuf + 0) - 'a' + 10;
    }

    cyHexData *= 16;

    if ('A' > *(pCyAsciiBuf + 1) )
    {
        cyHexData += *(pCyAsciiBuf + 1) - '0';
    }
    else if ('a' > *(pCyAsciiBuf + 1) )
    {
        cyHexData += *(pCyAsciiBuf + 1) - 'A' + 10;
    }
    else
    {
        cyHexData += *(pCyAsciiBuf + 1) - 'a' + 10;
    }

    return (cyHexData);
}
# 125 "../APP/Src/modbus_ascii.c"
uint8_t MODBUS_ASCII_GetLrc(uint8_t *pCyAsciiBuf, uint16_t cyLen)
{
    uint8_t cyLrcVal;
    uint16_t i;

    if (1 == (cyLen % 2) )
    {
        return 0;
    }

    cyLen /= 2;
    cyLrcVal = 0;
    for (i = 0; i < cyLen; i++)
    {
        cyLrcVal += MODBUS_ASCII_AsciiToHex(pCyAsciiBuf + i * 2);
    }

    cyLrcVal = ~cyLrcVal;
    cyLrcVal += 1;

    return (cyLrcVal);
}
# 160 "../APP/Src/modbus_ascii.c"
uint16_t MODBUS_ASCII_AsciiPacketToRtuPacket(uint8_t *pCyAsciiBuf, uint16_t cyAsciiLen, uint8_t *pCyRtuBuf)
{
    uint16_t i;
    uint16_t cyRtuLen;

    if (1 == (cyAsciiLen % 2))
    {
        return 0;
    }

    cyRtuLen = cyAsciiLen / 2;
    for (i = 0; i < cyRtuLen; i++)
    {
        *(pCyRtuBuf + i) = MODBUS_ASCII_AsciiToHex(pCyAsciiBuf + i * 2);
    }

    return (cyRtuLen);
}
# 191 "../APP/Src/modbus_ascii.c"
uint16_t MODBUS_ASCII_RtuPacketToAsciiPacket(uint8_t *pCyRtuBuf, uint16_t cyRtuLen, uint8_t *pCyAsciiBuf)
{
    uint16_t i;
    uint16_t cyAsciiLen;

    cyAsciiLen = cyRtuLen * 2;
    for (i = 0; i < cyRtuLen; i++)
    {
        MODBUS_ASCII_HexToAscii( *(pCyRtuBuf + i), pCyAsciiBuf + i * 2);
    }

    return (cyAsciiLen);
}
# 217 "../APP/Src/modbus_ascii.c"
void MODBUS_ASCII_HandlRevData(uint8_t cyRevData)
{


    switch(g_cyRevState)
    {
        case 0:
            if (':' == cyRevData)
            {
                g_cyRevBufffLen = 0;
            }
            else if (0x0D == cyRevData)
            {
                g_cyRevState = 1;
            }
            UARTx_RXBuff[g_cyRevBufffLen] = cyRevData;
            g_cyRevBufffLen++;
            if (600 <= g_cyRevBufffLen)
            {
                g_cyRevState = 2;
            }
            break;

      case 1:
            if (':' == cyRevData)
            {
                  g_cyRevBufffLen = 0;
                  g_cyRevState = 0;
                  UARTx_RXBuff[g_cyRevBufffLen] = cyRevData;
                  g_cyRevBufffLen++;
            }
            else if(0x0A == cyRevData)
            {
                    g_cyRevState = 2;

                    UARTx_RXBuff[g_cyRevBufffLen] = cyRevData;
                    g_cyRevBufffLen++;


                    UartRecvFrameOK = Bit_SET;
     UartRecvFlag = RESET;

            }
            else
            {
                    g_cyRevState = 2;
            }
            break;

        case 2:
            if (':' == cyRevData)
            {
                g_cyRevBufffLen = 0;
                g_cyRevState = 0;
                UARTx_RXBuff[g_cyRevBufffLen] = cyRevData;
                g_cyRevBufffLen++;

            }
            break;

        default:
           g_cyRevState = 2;
           break;
    }
}
# 294 "../APP/Src/modbus_ascii.c"
uint8_t MODBUS_ASCII_CheckAscii(uint8_t *pCyAsciiBuf, uint16_t cyLen)
{
    uint16_t i;

    for (i = 0; i < cyLen; i++)
    {
        if ('0' > *(pCyAsciiBuf + i) )
        {
            break;
        }

        if ( ('9' < *(pCyAsciiBuf + i) ) && ( *(pCyAsciiBuf + i) < 'A' ) )
        {
            break;
        }

        if ( ('F' < *(pCyAsciiBuf + i) ) && ( *(pCyAsciiBuf + i) < 'a' ) )
        {
            break;
        }

        if ('f' < *(pCyAsciiBuf + i) )
        {
            break;
        }
    }

    if (i == cyLen)
    {
        return (1);
    }

    return (0);
}
# 341 "../APP/Src/modbus_ascii.c"
uint8_t MODBUS_ASCII_RecvData(uint8_t* cyRecvBuff, uint16_t *pCyLen)
{
    uint8_t cyLrc;
    uint8_t eres;

    if (((uint8_t*)0) == cyRecvBuff)
    {
        return 0;
    }

    if ((Bit_RESET == UartRecvFrameOK) || (0 == g_cyRevBufffLen))
    {
        return 0;
    }

    UartRecvFrameOK = Bit_RESET;

    if (0 == MODBUS_ASCII_CheckAscii(&UARTx_RXBuff[1], g_cyRevBufffLen - 3) )
    {

     return 1;
    }

    cyLrc = MODBUS_ASCII_GetLrc(&UARTx_RXBuff[1], g_cyRevBufffLen - 5);
    if (cyLrc != MODBUS_ASCII_AsciiToHex(&UARTx_RXBuff[g_cyRevBufffLen - 4]) )
    {

     eres = 2;
    }
    else
    {
      eres = 3;
    }
    *pCyLen = MODBUS_ASCII_AsciiPacketToRtuPacket(&UARTx_RXBuff[1], g_cyRevBufffLen - 5, cyRecvBuff);

    return eres;
}
# 391 "../APP/Src/modbus_ascii.c"
uint16_t MODBUS_ASCII_SendData(uint8_t *cySendBuff, uint16_t cyLen)
{
    uint8_t cyLrc;
    uint16_t cyAsciiLen;

    if ( (0 == cyLen) || ( ((uint8_t*)0) == cySendBuff))
    {
        return 0;
    }

    if ( (cyLen * 2 + 5) > 600)
    {
     return 0;
    }

    cyAsciiBuff[0] = ':';
    cyAsciiLen = 1;

    cyAsciiLen += MODBUS_ASCII_RtuPacketToAsciiPacket(cySendBuff, cyLen, &cyAsciiBuff[1]);
    cyLrc = MODBUS_ASCII_GetLrc(&cyAsciiBuff[1], cyAsciiLen - 1);
    MODBUS_ASCII_HexToAscii(cyLrc, &cyAsciiBuff[cyAsciiLen]);
    cyAsciiLen += 2;
    cyAsciiBuff[cyAsciiLen] = 0x0D;
    cyAsciiLen++;
    cyAsciiBuff[cyAsciiLen] = 0x0A;



 cyAsciiLen+=1;
    return (UARTx_SendData(cyAsciiBuff, cyAsciiLen) );
# 442 "../APP/Src/modbus_ascii.c"
}

uint16_t MODBUS_ASCII_SendData1(uint8_t *cySendBuff, uint16_t cyLen)
{
    uint8_t cyLrc;
    uint16_t cyAsciiLen;

    if ( (0 == cyLen) || ( ((uint8_t*)0) == cySendBuff))
    {
        return 0;
    }

    if ( (cyLen * 2 + 5) > 600)
    {
     return 0;
    }

    cyAsciiBuff[0] = ':';
    cyAsciiLen = 1;

    cyAsciiLen += MODBUS_ASCII_RtuPacketToAsciiPacket(cySendBuff, cyLen, &cyAsciiBuff[1]);
    cyLrc = MODBUS_ASCII_GetLrc(&cyAsciiBuff[1], cyAsciiLen - 1);
    MODBUS_ASCII_HexToAscii(cyLrc, &cyAsciiBuff[cyAsciiLen]);
    cyAsciiLen += 2;
    cyAsciiBuff[cyAsciiLen] = 0x0D;
    cyAsciiLen++;
    cyAsciiBuff[cyAsciiLen] = 0x0A;
 cyAsciiLen+=1;
    return (UARTx_SendData(cyAsciiBuff, cyAsciiLen) );
# 486 "../APP/Src/modbus_ascii.c"
}
