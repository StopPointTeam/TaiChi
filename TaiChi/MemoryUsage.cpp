/*
MemoryUsage.c - MemoryUsage library
Copyright (c) 2015 Thierry Paris.  All right reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*! \file MemoryUsage.cpp

Main library code file.
*/

#include "Arduino.h"
#include "MemoryUsage.h"

/// Thanks to adafruit : https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
int mu_freeRam()
{
	//extern int __heap_start, *__brkval;
	int v;
	return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

/// Copy / adaptation of the library StackPaint available here : https://github.com/WickedDevice/StackPaint

#define STACK_CANARY	0xc5

void mu_StackPaint(void) __attribute__((naked)) __attribute__((section(".init1")));


/// Function called before any other function.
void mu_StackPaint(void)
{
#if 1
	uint8_t *p = &_end;

	while (p <= &__stack)
	{
		*p = STACK_CANARY;
		p++;
	}
#else
	__asm volatile (
		"    ldi r30,lo8(_end)\n"
		"    ldi r31,hi8(_end)\n"
		"    ldi r24,lo8(0xc5)\n" // STACK_CANARY = 0xc5
		"    ldi r25,hi8(__stack)\n"
		"    rjmp .cmp\n"
		".loop:\n"
		"    st Z+,r24\n"
		".cmp:\n"
		"    cpi r30,lo8(__stack)\n"
		"    cpc r31,r25\n"
		"    brlo .loop\n"
		"    breq .loop"::);
#endif
}

/// Checks the first undecorated byte.
uint16_t mu_StackCount(void)
{
	uint8_t *p = (__brkval == 0 ? (uint8_t *) &__heap_start : __brkval);

	while (*p == STACK_CANARY && (int) p <= SP)
		p++;

	return (uint16_t)RAMEND - (uint16_t)p;
}

/// Modified function from http://www.avr-developers.com/mm/memoryusage.html
void SRamDisplay(void)
{
	int	data_size	=	(int)&__data_end - (int)&__data_start;
	int	bss_size	=	(int)&__bss_end - (int)&__data_end;
	int heap_end 	= 	(int) (__brkval == 0 ? (uint8_t *) &__heap_start : __brkval);
	//int	heap_end	=	(int)SP - (int)&__malloc_margin;
	int	heap_size	=	heap_end - (int)&__bss_end;
	int	stack_size	=	RAMEND - (int)SP + 1;
	int	available	=	(RAMEND - (int)&__data_start + 1);
	
	available	-=	data_size + bss_size + heap_size + stack_size;

	NeoSerial.print(F("+----------------+ "));			NeoSerial.print((int)&__data_start);	NeoSerial.println(" (__data_start)");
	NeoSerial.print(F("+      data      +"));			NeoSerial.println();
	NeoSerial.print(F("+    variables   + size = "));	NeoSerial.println(data_size);
	NeoSerial.print(F("+----------------+ "));			NeoSerial.print((int)&__data_end);		NeoSerial.println(" (__data_end / __bss_start)");
	NeoSerial.print(F("+      bss       +"));			NeoSerial.println();
	NeoSerial.print(F("+    variables   + size = "));	NeoSerial.println(bss_size);
	NeoSerial.print(F("+----------------+ "));			NeoSerial.print((int)&__bss_end);		NeoSerial.println(" (__bss_end / __heap_start)");
	NeoSerial.print(F("+      heap      + size = "));	NeoSerial.println(heap_size);
	NeoSerial.print(F("+----------------+ "));			NeoSerial.print((int)heap_end);		NeoSerial.println(" (__brkval if not 0, or __heap_start)");
	NeoSerial.print(F("+                +"));			NeoSerial.println();
	NeoSerial.print(F("+                +"));			NeoSerial.println();
	NeoSerial.print(F("+   FREE RAM     + size = "));	NeoSerial.println(available);
	NeoSerial.print(F("+                +"));			NeoSerial.println();
	NeoSerial.print(F("+                +"));			NeoSerial.println();
	NeoSerial.print(F("+----------------+ "));			NeoSerial.print((int)SP);		NeoSerial.println(" (SP)");
	NeoSerial.print(F("+     stack      + size = "));	NeoSerial.println(stack_size);
	NeoSerial.print(F("+----------------+ "));			NeoSerial.print((int)RAMEND);		NeoSerial.println(" (RAMEND / __stack)");

	NeoSerial.println();
	NeoSerial.println();
}