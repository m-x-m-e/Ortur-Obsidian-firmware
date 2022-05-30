/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#ifdef ARDUINO_ARCH_STM32

#include "../../core/macros.h"
#include "../../core/serial.h"


#define CONNECT_(a,b) a##b
#define CONNECT(a,b) CONNECT_(a,b)

//选择使用不同的错误追踪库
//#define USE_UNWINDER
#define USE_CM_BACKTRUCK

#if defined(USE_UNWINDER)

#include "../shared/backtrace/unwinder.h"
#include "../shared/backtrace/unwmemaccess.h"

#include <stdarg.h>

// Debug monitor that dumps to the Programming port all status when
// an exception or WDT timeout happens - And then resets the board

// All the Monitor routines must run with interrupts disabled and
// under an ISR execution context. That is why we cannot reuse the
// Serial interrupt routines or any C runtime, as we don't know the
// state we are when running them

// A SW memory barrier, to ensure GCC does not overoptimize loops
#define sw_barrier() __asm__ volatile("": : :"memory");

// (re)initialize UART0 as a monitor output to 250000,n,8,1
static void TXBegin() {
// NOTE: needless
}

// Send character through UART with no interrupts
static void TX(char c) {
	while((CONNECT(USART,SERIAL_PORT) ->SR & 0X40 ) == 0)
		sw_barrier();//循环发送,直到发送完毕
	CONNECT(USART,SERIAL_PORT)->DR = (uint8_t) c;
}

// Send String through UART
static void TX(const char* s) {
  while (*s) TX(*s++);
}

static void TXDigit(uint32_t d) {
  if (d < 10) TX((char)(d+'0'));
  else if (d < 16) TX((char)(d+'A'-10));
  else TX('?');
}

// Send Hex number thru UART
static void TXHex(uint32_t v) {
  TX("0x");
  for (uint8_t i = 0; i < 8; i++, v <<= 4)
    TXDigit((v >> 28) & 0xF);
}

// Send Decimal number thru UART
static void TXDec(uint32_t v) {
  if (!v) {
    TX('0');
    return;
  }

  char nbrs[14];
  char *p = &nbrs[0];
  while (v != 0) {
    *p++ = '0' + (v % 10);
    v /= 10;
  }
  do {
    p--;
    TX(*p);
  } while (p != &nbrs[0]);
}

//NOTE: 避免与backtrace.cpp中已经存在的实现重名
namespace DM
{

// Dump a backtrace entry
static bool UnwReportOut(void* ctx, const UnwReport* bte) {
  int* p = (int*)ctx;

  (*p)++;
  TX('#'); TXDec(*p); TX(" : ");
  TX(bte->name?bte->name:"unknown"); TX('@'); TXHex(bte->function);
  TX('+'); TXDec(bte->address - bte->function);
  TX(" PC:");TXHex(bte->address); TX('\n');
  return true;
}

#ifdef UNW_DEBUG
  void UnwPrintf(const char* format, ...) {
    char dest[256];
    va_list argptr;
    va_start(argptr, format);
    vsprintf(dest, format, argptr);
    va_end(argptr);
    TX(&dest[0]);
  }
#endif



/* Table of function pointers for passing to the unwinder */
static const UnwindCallbacks UnwCallbacks = {
  UnwReportOut,
  UnwReadW,
  UnwReadH,
  UnwReadB
  #ifdef UNW_DEBUG
   , UnwPrintf
  #endif
};
}

/**
 * HardFaultHandler_C:
 * This is called from the HardFault_HandlerAsm with a pointer the Fault stack
 * as the parameter. We can then read the values from the stack and place them
 * into local variables for ease of reading.
 * We then read the various Fault Status and Address Registers to help decode
 * cause of the fault.
 * The function ends with a BKPT instruction to force control back into the debugger
 */
extern "C" {
void HardFault_HandlerC(unsigned long *sp, unsigned long lr, unsigned long cause) {

  static const char* causestr[] = {
    "NMI","Hard","Mem","Bus","Usage","Debug","WDT","RSTC"
  };

  UnwindFrame btf;

  // Dump report to the Programming port (interrupts are DISABLED)
  TXBegin();
  TX("\n\n## Software Fault detected ##\n");
  TX("Cause: "); TX(causestr[cause]); TX('\n');

  TX("R0   : "); TXHex(((unsigned long)sp[0])); TX('\n');
  TX("R1   : "); TXHex(((unsigned long)sp[1])); TX('\n');
  TX("R2   : "); TXHex(((unsigned long)sp[2])); TX('\n');
  TX("R3   : "); TXHex(((unsigned long)sp[3])); TX('\n');
  TX("R12  : "); TXHex(((unsigned long)sp[4])); TX('\n');
  TX("LR   : "); TXHex(((unsigned long)sp[5])); TX('\n');
  TX("PC   : "); TXHex(((unsigned long)sp[6])); TX('\n');
  TX("PSR  : "); TXHex(((unsigned long)sp[7])); TX('\n');

  // Configurable Fault Status Register
  // Consists of MMSR, BFSR and UFSR
  TX("CFSR : "); TXHex((*((volatile unsigned long *)(0xE000ED28)))); TX('\n');

  // Hard Fault Status Register
  TX("HFSR : "); TXHex((*((volatile unsigned long *)(0xE000ED2C)))); TX('\n');

  // Debug Fault Status Register
  TX("DFSR : "); TXHex((*((volatile unsigned long *)(0xE000ED30)))); TX('\n');

  // Auxiliary Fault Status Register
  TX("AFSR : "); TXHex((*((volatile unsigned long *)(0xE000ED3C)))); TX('\n');

  // Read the Fault Address Registers. These may not contain valid values.
  // Check BFARVALID/MMARVALID to see if they are valid values
  // MemManage Fault Address Register
  TX("MMAR : "); TXHex((*((volatile unsigned long *)(0xE000ED34)))); TX('\n');

  // Bus Fault Address Register
  TX("BFAR : "); TXHex((*((volatile unsigned long *)(0xE000ED38)))); TX('\n');

  TX("ExcLR: "); TXHex(lr); TX('\n');
  TX("ExcSP: "); TXHex((unsigned long)sp); TX('\n');

  btf.sp = ((unsigned long)sp) + 8*4; // The original stack pointer
  btf.fp = btf.sp;
  btf.lr = ((unsigned long)sp[5]);
  btf.pc = ((unsigned long)sp[6]) | 1; // Force Thumb, as CORTEX only support it

  // Perform a backtrace
  TX("\nBacktrace:\n\n");
  int ctr = 0;
  UnwindStart(&btf, &DM::UnwCallbacks, &ctr);

  // Disable all NVIC interrupts
  NVIC->ICER[0] = 0xFFFFFFFF;
  NVIC->ICER[1] = 0xFFFFFFFF;

  // Relocate VTOR table to default position
//  SCB->VTOR = 0;

  // Disable USB
//  otg_disable();

  // Restart watchdog
//  WDT_Restart(WDT);

  // Reset controller
//  NVIC_SystemReset();
//  for (;;) WDT_Restart(WDT);

#ifndef USE_FREERTOS
  for (;;) HAL_watchdog_refresh(WDT);
#else
  extern void errorBlink(int n);

  switch(cause)
  {
  // catch exceptions

  /** Hard fault - blink four short flash every two seconds */
  case 1 : // HardFault_Handler
    errorBlink(4);
    break;

  /** Bus fault - blink five short flashes every two seconds */
  case 3 : //BusFault_Handler
    errorBlink(5);
    break;

  /** Usage fault - blink six short flashes every two seconds */
  case 4 : //UsageFault_Handler
    errorBlink(6);
    break;

  default:
	errorBlink(7);
  }
#endif
}


#define HANDLER_ATTR __attribute__((naked))

HANDLER_ATTR void NMI_Handler() {
  __asm__ __volatile__ (
    ".syntax unified" "\n\t"
    A("tst lr, #4")
    A("ite eq")
    A("mrseq r0, msp")
    A("mrsne r0, psp")
    A("mov r1,lr")
    A("mov r2,#0")
    A("b HardFault_HandlerC")
  );
}

HANDLER_ATTR void HardFault_Handler() {
  __asm__ __volatile__ (
    ".syntax unified" "\n\t"
    A("tst lr, #4")
    A("ite eq")
    A("mrseq r0, msp")
    A("mrsne r0, psp")
    A("mov r1,lr")
    A("mov r2,#1")
    A("b HardFault_HandlerC")
  );
}

HANDLER_ATTR void MemManage_Handler() {
  __asm__ __volatile__ (
    ".syntax unified" "\n\t"
    A("tst lr, #4")
    A("ite eq")
    A("mrseq r0, msp")
    A("mrsne r0, psp")
    A("mov r1,lr")
    A("mov r2,#2")
    A("b HardFault_HandlerC")
  );
}

HANDLER_ATTR void BusFault_Handler() {
  __asm__ __volatile__ (
    ".syntax unified" "\n\t"
    A("tst lr, #4")
    A("ite eq")
    A("mrseq r0, msp")
    A("mrsne r0, psp")
    A("mov r1,lr")
    A("mov r2,#3")
    A("b HardFault_HandlerC")
  );
}

HANDLER_ATTR void UsageFault_Handler() {
  __asm__ __volatile__ (
    ".syntax unified" "\n\t"
    A("tst lr, #4")
    A("ite eq")
    A("mrseq r0, msp")
    A("mrsne r0, psp")
    A("mov r1,lr")
    A("mov r2,#4")
    A("b HardFault_HandlerC")
  );
}

HANDLER_ATTR void DebugMon_Handler() {
  __asm__ __volatile__ (
    ".syntax unified" "\n\t"
    A("tst lr, #4")
    A("ite eq")
    A("mrseq r0, msp")
    A("mrsne r0, psp")
    A("mov r1,lr")
    A("mov r2,#5")
    A("b HardFault_HandlerC")
  );
}

/* This is NOT an exception, it is an interrupt handler - Nevertheless, the framing is the same */
HANDLER_ATTR void WDT_Handler() {
  __asm__ __volatile__ (
    ".syntax unified" "\n\t"
    A("tst lr, #4")
    A("ite eq")
    A("mrseq r0, msp")
    A("mrsne r0, psp")
    A("mov r1,lr")
    A("mov r2,#6")
    A("b HardFault_HandlerC")
  );
}

HANDLER_ATTR void RSTC_Handler() {
  __asm__ __volatile__ (
    ".syntax unified" "\n\t"
    A("tst lr, #4")
    A("ite eq")
    A("mrseq r0, msp")
    A("mrsne r0, psp")
    A("mov r1,lr")
    A("mov r2,#7")
    A("b HardFault_HandlerC")
  );
}

} // extern "C"

#elif defined(USE_CM_BACKTRUCK)

#include <cm_backtrace.h>

#if defined(EXTENSIBLE_UI)
#include <TFT_eSPI.h>
extern TFT_eSPI tft;
#endif

// A SW memory barrier, to ensure GCC does not overoptimize loops
#define sw_barrier() __asm__ volatile("": : :"memory");

void dm_print(char c) {
	while((CONNECT(USART,SERIAL_PORT) ->SR & 0X40 ) == 0)
		sw_barrier();//直到发送完毕
	CONNECT(USART,SERIAL_PORT)->DR = (uint8_t) c;
}

void dm_print(char * s)
{
	while (*s)
		dm_print(*s++);
}

extern "C" void dm_print(const char* format, ...)
{
    char dest[256];
    va_list argptr;
    va_start(argptr, format);
    vsprintf(dest, format, argptr);
    va_end(argptr);
    dm_print(&dest[0]);

#ifndef DEBUG

#if defined(EXTENSIBLE_UI)

    static bool tft_ready = false;
    if(!tft_ready)
    {
    	//NOTE: 添加错误时的界面提示
    	//重新初始化屏幕
#ifdef STM32_DMA
    	tft.initDMA(); // Initialise the DMA engine
#endif
	    //tft.init(0);
		tft.setRotation(1);
		// Draw initial framebuffer contents:
		tft.fillScreen(TFT_BLUE);
		tft.setBackLight(255); //调节背光亮度

		tft.setCursor(20, 0);
		tft.setTextFont(1);
		tft.setTextSize(1);
		tft.setTextColor(TFT_WHITE, TFT_BLUE);
		tft_ready = true;
    }
    tft.print(&dest[0]);
#endif

#endif
  }

void dm_println(char c)
{
	dm_print(c);
	dm_print('\n');

}

void dm_println(char * s)
{
	while (*s)
		dm_print(*s++);
	dm_print('\n');
}

extern "C" void dm_HardFault_Handler(uint32_t fault_handler_lr, uint32_t fault_handler_sp ,struct cmb_hard_fault_regs * hard_fault_regs)
{
#ifndef USE_FREERTOS
  for (;;) HAL_watchdog_refresh(WDT);
#else
  extern void errorBlink(int n);

    errorBlink(4);
#endif
}

#endif


#endif // ARDUINO_ARCH_SAM
