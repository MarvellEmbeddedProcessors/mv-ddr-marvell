/*******************************************************************************
Copyright (C) 2016 Marvell International Ltd.

This software file (the "File") is owned and distributed by Marvell
International Ltd. and/or its affiliates ("Marvell") under the following
alternative licensing terms.  Once you have made an election to distribute the
File under one of the following license alternatives, please (i) delete this
introductory statement regarding license alternatives, (ii) delete the three
license alternatives that you have not elected to use and (iii) preserve the
Marvell copyright notice above.

********************************************************************************
Marvell Commercial License Option

If you received this File from Marvell and you have entered into a commercial
license agreement (a "Commercial License") with Marvell, the File is licensed
to you under the terms of the applicable Commercial License.

********************************************************************************
Marvell GPL License Option

This program is free software: you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the Free
Software Foundation, either version 2 of the License, or any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

********************************************************************************
Marvell GNU General Public License FreeRTOS Exception

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File in accordance with the terms and conditions of the Lesser
General Public License Version 2.1 plus the following FreeRTOS exception.
An independent module is a module which is not derived from or based on
FreeRTOS.
Clause 1:
Linking FreeRTOS statically or dynamically with other modules is making a
combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
General Public License cover the whole combination.
As a special exception, the copyright holder of FreeRTOS gives you permission
to link FreeRTOS with independent modules that communicate with FreeRTOS solely
through the FreeRTOS API interface, regardless of the license terms of these
independent modules, and to copy and distribute the resulting combined work
under terms of your choice, provided that:
1. Every copy of the combined work is accompanied by a written statement that
details to the recipient the version of FreeRTOS used and an offer by yourself
to provide the FreeRTOS source code (including any modifications you may have
made) should the recipient request it.
2. The combined work is not itself an RTOS, scheduler, kernel or related
product.
3. The independent modules add significant and primary functionality to
FreeRTOS and do not merely extend the existing functionality already present in
FreeRTOS.
Clause 2:
FreeRTOS may not be used for any competitive or comparative purpose, including
the publication of any form of run time or compile time metric, without the
express permission of Real Time Engineers Ltd. (this is the norm within the
industry and is intended to ensure information accuracy).

********************************************************************************
Marvell BSD License Option

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File under the following licensing terms.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

	* Redistributions of source code must retain the above copyright notice,
	  this list of conditions and the following disclaimer.

	* Redistributions in binary form must reproduce the above copyright
	  notice, this list of conditions and the following disclaimer in the
	  documentation and/or other materials provided with the distribution.

	* Neither the name of Marvell nor the names of its contributors may be
	  used to endorse or promote products derived from this software without
	  specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef _MV_DDR_A38X_MC_STATIC_H
#define _MV_DDR_A38X_MC_STATIC_H

#include "mv_ddr_a38x.h"

#ifdef SUPPORT_STATIC_DUNIT_CONFIG

#ifdef CONFIG_CUSTOMER_BOARD_SUPPORT
static struct reg_data ddr3_customer_800[] = {
	/* parameters for customer board (based on 800MHZ) */
	{0x1400,	0x7b00cc30, 0xffffffff},
	{0x1404,	0x36301820, 0xffffffff},
	{0x1408,	0x5415baab, 0xffffffff},
	{0x140c,	0x38411def, 0xffffffff},
	{0x1410,	0x18300000, 0xffffffff},
	{0x1414,	0x00000700, 0xffffffff},
	{0x1424,	0x0060f3ff, 0xffffffff},
	{0x1428,	0x0011a940, 0xffffffff},
	{0x142c,	0x28c5134,  0xffffffff},
	{0x1474,	0x00000000, 0xffffffff},
	{0x147c,	0x0000d771, 0xffffffff},
	{0x1494,	0x00030000, 0xffffffff},
	{0x149c,	0x00000300, 0xffffffff},
	{0x14a8,	0x00000000, 0xffffffff},
	{0x14cc,	0xbd09000d, 0xffffffff},
	{0x1504,	0xfffffff1, 0xffffffff},
	{0x150c,	0xffffffe5, 0xffffffff},
	{0x1514,	0x00000000, 0xffffffff},
	{0x151c,	0x00000000, 0xffffffff},
	{0x1538,	0x00000b0b, 0xffffffff},
	{0x153c,	0x00000c0c, 0xffffffff},
	{0x15d0,	0x00000670, 0xffffffff},
	{0x15d4,	0x00000046, 0xffffffff},
	{0x15d8,	0x00000010, 0xffffffff},
	{0x15dc,	0x00000000, 0xffffffff},
	{0x15e0,	0x00000023, 0xffffffff},
	{0x15e4,	0x00203c18, 0xffffffff},
	{0x15ec,	0xf8000019, 0xffffffff},
	{0x16a0,	0xcc000006, 0xffffffff},	/* Clock Delay */
	{0xe4124,	0x08008073, 0xffffffff},	/* AVS BG default */
	{0, 0, 0}
};

#else /* CONFIG_CUSTOMER_BOARD_SUPPORT */

struct reg_data ddr3_a38x_933[MV_MAX_DDR3_STATIC_SIZE] = {
	/* parameters for 933MHZ */
	{0x1400,	0x7b00ce3a, 0xffffffff},
	{0x1404,	0x36301820, 0xffffffff},
	{0x1408,	0x7417eccf, 0xffffffff},
	{0x140c,	0x3e421f98, 0xffffffff},
	{0x1410,	0x1a300000, 0xffffffff},
	{0x1414,	0x00000700, 0xffffffff},
	{0x1424,	0x0060f3ff, 0xffffffff},
	{0x1428,	0x0013ca50, 0xffffffff},
	{0x142c,	0x028c5165, 0xffffffff},
	{0x1474,	0x00000000, 0xffffffff},
	{0x147c,	0x0000e871, 0xffffffff},
	{0x1494,	0x00010000, 0xffffffff},
	{0x149c,	0x00000001, 0xffffffff},
	{0x14a8,	0x00000000, 0xffffffff},
	{0x14cc,	0xbd09000d, 0xffffffff},
	{0x1504,	0xffffffe1, 0xffffffff},
	{0x150c,	0xffffffe5, 0xffffffff},
	{0x1514,	0x00000000, 0xffffffff},
	{0x151c,	0x00000000, 0xffffffff},
	{0x1538,	0x00000d0d, 0xffffffff},
	{0x153c,	0x00000d0d, 0xffffffff},
	{0x15d0,	0x00000608, 0xffffffff},
	{0x15d4,	0x00000044, 0xffffffff},
	{0x15d8,	0x00000020, 0xffffffff},
	{0x15dc,	0x00000000, 0xffffffff},
	{0x15e0,	0x00000021, 0xffffffff},
	{0x15e4,	0x00203c18, 0xffffffff},
	{0x15ec,	0xf8000019, 0xffffffff},
	{0x16a0,	0xcc000006, 0xffffffff},	/* Clock Delay */
	{0xe4124,	0x08008073, 0xffffffff},	/* AVS BG default */
	{0, 0, 0}
};

static struct reg_data ddr3_a38x_800[] = {
	/* parameters for 800MHZ */
	{0x1400,	0x7b00cc30, 0xffffffff},
	{0x1404,	0x36301820, 0xffffffff},
	{0x1408,	0x5415baab, 0xffffffff},
	{0x140c,	0x38411def, 0xffffffff},
	{0x1410,	0x18300000, 0xffffffff},
	{0x1414,	0x00000700, 0xffffffff},
	{0x1424,	0x0060f3ff, 0xffffffff},
	{0x1428,	0x0011a940, 0xffffffff},
	{0x142c,	0x28c5134,  0xffffffff},
	{0x1474,	0x00000000, 0xffffffff},
	{0x147c,	0x0000d771, 0xffffffff},
	{0x1494,	0x00030000, 0xffffffff},
	{0x149c,	0x00000300, 0xffffffff},
	{0x14a8,	0x00000000, 0xffffffff},
	{0x14cc,	0xbd09000d, 0xffffffff},
	{0x1504,	0xfffffff1, 0xffffffff},
	{0x150c,	0xffffffe5, 0xffffffff},
	{0x1514,	0x00000000, 0xffffffff},
	{0x151c,	0x00000000, 0xffffffff},
	{0x1538,	0x00000b0b, 0xffffffff},
	{0x153c,	0x00000c0c, 0xffffffff},
	{0x15d0,	0x00000670, 0xffffffff},
	{0x15d4,	0x00000046, 0xffffffff},
	{0x15d8,	0x00000010, 0xffffffff},
	{0x15dc,	0x00000000, 0xffffffff},
	{0x15e0,	0x00000023, 0xffffffff},
	{0x15e4,	0x00203c18, 0xffffffff},
	{0x15ec,	0xf8000019, 0xffffffff},
	{0x16a0,	0xcc000006, 0xffffffff},	/* Clock Delay */
	{0xe4124,	0x08008073, 0xffffffff},	/* AVS BG default */
	{0,   0, 0}
};

static struct reg_data ddr3_a38x_667[] = {
	/* parameters for 667MHZ */
	/* DDR SDRAM Configuration Register */
	{0x1400,    0x7b00ca28, 0xffffffff},
	/* Dunit Control Low Register - kw28 bit12 low (disable CLK1) */
	{0x1404,    0x36301820, 0xffffffff},
	/* DDR SDRAM Timing (Low) Register */
	{0x1408,    0x43149997, 0xffffffff},
	/* DDR SDRAM Timing (High) Register */
	{0x140c,    0x38411bc7, 0xffffffff},
	/* DDR SDRAM Address Control Register */
	{0x1410,    0x14330000, 0xffffffff},
	/* DDR SDRAM Open Pages Control Register */
	{0x1414,    0x00000700, 0xffffffff},
	/* Dunit Control High Register (2 :1 - bits 15:12 = 0xd) */
	{0x1424,    0x0060f3ff, 0xffffffff},
	/* Dunit Control High Register */
	{0x1428,    0x000f8830, 0xffffffff},
	/* Dunit Control High Register  (2:1 -  bit 29 = '1') */
	{0x142c,    0x28c50f8,  0xffffffff},
	{0x147c,    0x0000c671, 0xffffffff},
	/* DDR SDRAM ODT Control (Low) Register */
	{0x1494,    0x00030000, 0xffffffff},
	/* DDR SDRAM ODT Control (High) Register, will be configured at WL */
	{0x1498,    0x00000000, 0xffffffff},
	/* DDR Dunit ODT Control Register */
	{0x149c,    0x00000300, 0xffffffff},
	{0x14a8,    0x00000000, 0xffffffff}, /*  */
	{0x14cc,    0xbd09000d, 0xffffffff}, /*  */
	{0x1474,    0x00000000, 0xffffffff},
	/* Read Data Sample Delays Register */
	{0x1538,    0x00000009, 0xffffffff},
	/* Read Data Ready Delay Register */
	{0x153c,    0x0000000c, 0xffffffff},
	{0x1504,    0xfffffff1, 0xffffffff}, /*  */
	{0x150c,    0xffffffe5, 0xffffffff}, /*  */
	{0x1514,    0x00000000, 0xffffffff}, /*  */
	{0x151c,    0x0,	0xffffffff}, /*  */
	{0x15d0,    0x00000650, 0xffffffff}, /* MR0 */
	{0x15d4,    0x00000046, 0xffffffff}, /* MR1 */
	{0x15d8,    0x00000010, 0xffffffff}, /* MR2 */
	{0x15dc,    0x00000000, 0xffffffff}, /* MR3 */
	{0x15e0,    0x23,	0xffffffff}, /*  */
	{0x15e4,    0x00203c18, 0xffffffff}, /* ZQC Configuration Register */
	{0x15ec,    0xf8000019, 0xffffffff}, /* DDR PHY */
	{0x16a0,    0xcc000006, 0xffffffff}, /* Clock Delay */
	{0xe4124,   0x08008073, 0xffffffff}, /* AVS BG default */
	{0, 0, 0}
};

static struct reg_data ddr3_a38x_533[] = {
	/* parameters for 533MHZ */
	/* DDR SDRAM Configuration Register */
	{0x1400,    0x7b00d040, 0xffffffff},
	/* Dunit Control Low Register - kw28 bit12 low (disable CLK1) */
	{0x1404,    0x36301820, 0xffffffff},
	/* DDR SDRAM Timing (Low) Register */
	{0x1408,    0x33137772, 0xffffffff},
	/* DDR SDRAM Timing (High) Register */
	{0x140c,    0x3841199f, 0xffffffff},
	/* DDR SDRAM Address Control Register */
	{0x1410,    0x10330000, 0xffffffff},
	/* DDR SDRAM Open Pages Control Register */
	{0x1414,    0x00000700, 0xffffffff},
	/* Dunit Control High Register (2 :1 - bits 15:12 = 0xd) */
	{0x1424,    0x0060f3ff, 0xffffffff},
	/* Dunit Control High Register */
	{0x1428,    0x000d6720, 0xffffffff},
	/* Dunit Control High Register  (2:1 -  bit 29 = '1') */
	{0x142c,    0x028c50c3, 0xffffffff},
	{0x147c,    0x0000b571, 0xffffffff},
	/* DDR SDRAM ODT Control (Low) Register */
	{0x1494,    0x00030000, 0xffffffff},
	/* DDR SDRAM ODT Control (High) Register, will be configured at WL */
	{0x1498,    0x00000000, 0xffffffff},
	/* DDR Dunit ODT Control Register */
	{0x149c,    0x00000003, 0xffffffff},
	{0x14a8,    0x00000000, 0xffffffff}, /*  */
	{0x14cc,    0xbd09000d, 0xffffffff}, /*  */
	{0x1474,    0x00000000, 0xffffffff},
	/* Read Data Sample Delays Register */
	{0x1538,    0x00000707, 0xffffffff},
	/* Read Data Ready Delay Register */
	{0x153c,    0x00000707, 0xffffffff},
	{0x1504,    0xffffffe1, 0xffffffff}, /*  */
	{0x150c,    0xffffffe5, 0xffffffff}, /*  */
	{0x1514,    0x00000000, 0xffffffff}, /*  */
	{0x151c,    0x00000000,	0xffffffff}, /*  */
	{0x15d0,    0x00000630, 0xffffffff}, /* MR0 */
	{0x15d4,    0x00000046, 0xffffffff}, /* MR1 */
	{0x15d8,    0x00000008, 0xffffffff}, /* MR2 */
	{0x15dc,    0x00000000, 0xffffffff}, /* MR3 */
	{0x15e0,    0x00000023,	0xffffffff}, /*  */
	{0x15e4,    0x00203c18, 0xffffffff}, /* ZQC Configuration Register */
	{0x15ec,    0xf8000019, 0xffffffff}, /* DDR PHY */
	{0x16a0,    0xcc000006, 0xffffffff}, /* Clock Delay */
	{0xe4124,   0x08008073, 0xffffffff}, /* AVS BG default */
	{0, 0, 0}
};

#endif /* CONFIG_CUSTOMER_BOARD_SUPPORT */

#endif /* SUPPORT_STATIC_DUNIT_CONFIG */

#endif /* _MV_DDR_A38X_MC_STATIC_H */
