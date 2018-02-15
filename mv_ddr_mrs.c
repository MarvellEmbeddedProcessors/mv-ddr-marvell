/*******************************************************************************
Copyright (C) 2018 Marvell International Ltd.

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

#if defined(MV_DDR) /* U-BOOT MARVELL 2013.01 */
#include "ddr_mv_wrapper.h"
#include "mv_ddr_plat.h"
#elif defined(MV_DDR_ATF) /* MARVELL ATF */
#include "mv_ddr_atf_wrapper.h"
#include "mv_ddr_plat.h"
#elif defined(CONFIG_A3700)
#include "mv_ddr_a3700_wrapper.h"
#include "mv_ddr_plat.h"
#else /* U-BOOT SPL */
#include "ddr_ml_wrapper.h"
#include "a38x/mv_ddr_plat.h"
#endif

/*
 * Based on Proposed DDR4 Full spec update (79-4B), Item No. 1716.78C
 */

/* MR2 CWL, [5:3] bits */
#define MV_DDR_MR2_CWL9		0x0	/* 0b0000_0000 */
#define MV_DDR_MR2_CWL10	0x8	/* 0b0000_1000 */
#define MV_DDR_MR2_CWL11	0x10	/* 0b0001_0000 */
#define MV_DDR_MR2_CWL12	0x18	/* 0b0001_1000 */
#define MV_DDR_MR2_CWL14	0x20	/* 0b0010_0000 */
#define MV_DDR_MR2_CWL16	0x28	/* 0b0010_1000 */
#define MV_DDR_MR2_CWL18	0x30	/* 0b0011_0000 */
#define MV_DDR_MR2_CWL20	0x38	/* 0b0011_1000 */

int mv_ddr_mr2_cwl_get(unsigned int cwl, unsigned int *mr2_cwl)
{
	switch (cwl) {
	case 9:
		*mr2_cwl = MV_DDR_MR2_CWL9;
		break;
	case 10:
		*mr2_cwl = MV_DDR_MR2_CWL10;
		break;
	case 11:
		*mr2_cwl = MV_DDR_MR2_CWL11;
		break;
	case 12:
		*mr2_cwl = MV_DDR_MR2_CWL12;
		break;
	case 14:
		*mr2_cwl = MV_DDR_MR2_CWL14;
		break;
	case 16:
		*mr2_cwl = MV_DDR_MR2_CWL16;
		break;
	case 18:
		*mr2_cwl = MV_DDR_MR2_CWL18;
		break;
	case 20:
		*mr2_cwl = MV_DDR_MR2_CWL20;
		break;
	default:
		printf("error: %s: unsupported cwl value found\n", __func__);
		return -1;
	}

	return 0;
}
