/*
 *  arch/arm/plat-mxc/include/mach/uncompress.h
 *
 *  Copyright (C) 1999 ARM Limited
 *  Copyright (C) Shane Nay (shane@minirl.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __ASM_ARCH_MXC_UNCOMPRESS_H__
#define __ASM_ARCH_MXC_UNCOMPRESS_H__

#include "../imx-uart.h"

unsigned long uart_base;

#define UART(x) (*(volatile unsigned long *)(uart_base + (x)))

#define USR2 0x98
#define USR2_TXFE (1<<14)
#define TXR  0x40
#define UCR1 0x80
#define UCR1_UARTEN 1

/*
 * The following code assumes the serial port has already been
 * initialized by the bootloader.  We search for the first enabled
 * port in the most probable order.  If you didn't setup a port in
 * your bootloader then nothing will appear (which might be desired).
 *
 * This does not append a newline
 */

static void putc(int ch)
{
	if (!uart_base)
		return;
	if (!(UART(UCR1) & UCR1_UARTEN))
		return;

	while (!(UART(USR2) & USR2_TXFE))
		barrier();

	UART(TXR) = ch;
}

static inline void flush(void)
{
}

static inline void arch_decomp_setup(void)
{
	uart_base = UART_PADDR;
}

#endif				/* __ASM_ARCH_MXC_UNCOMPRESS_H__ */
