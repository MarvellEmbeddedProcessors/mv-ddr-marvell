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

#ifndef _MV_DDR_MC6_DRV_H
#define _MV_DDR_MC6_DRV_H

/* includes */
#include "ddr3_topology_def.h"

/* fclk definition is used for trefi */
#define MV_DDR_MC6_FCLK_200MHZ_IN_KILO	200000

/* mc6 default timing parameters */
#define MV_DDR_MC6_TIMING_T_RES		100000
#define MV_DDR_MC6_TIMING_T_RESINIT	200000000
#define MV_DDR_MC6_TIMING_T_RESTCKE	500000000
#define MV_DDR_MC6_TIMING_T_ACTPDEN	2
#define MV_DDR_MC6_TIMING_T_ZQOPER	512
#define MV_DDR_MC6_TIMING_T_ZQINIT	1024
#define MV_DDR_MC6_TIMING_T_ZQCR	0
#define MV_DDR_MC6_TIMING_T_ZQCS	128
#define MV_DDR_MC6_TIMING_T_CCD		4
#define MV_DDR_MC6_TIMING_T_MRD		8
#define MV_DDR_MC6_TIMING_T_CAEXT	0
#define MV_DDR_MC6_TIMING_T_CACKEL	0
#define MV_DDR_MC6_TIMING_T_MPX_LH	0
#define MV_DDR_MC6_TIMING_T_MPX_S	0
#define MV_DDR_MC6_TIMING_T_XMP		0
#define MV_DDR_MC6_TIMING_T_MRD_PDA	0
#define MV_DDR_MC6_TIMING_T_XSDLL	768 /* worst case */
#define MV_DDR_MC6_TIMING_T_XP		6000
#define MV_DDR_MC6_TIMING_T_RRDL	4900
#define MV_DDR_MC6_TIMING_T_CKSRX	10000
#define MV_DDR_MC6_TIMING_T_CKE		3000
#define MV_DDR_MC6_TIMING_T_XS		170000
#define MV_DDR_MC6_TIMING_T_XS_FAST	170000
#define	MV_DDR_MC6_TIMING_T_RWD_EXT_DLY		1
#define MV_DDR_MC6_TIMMING_T_CCD_CCS_WR_EXT_DLY	1
#define MV_DDR_MC6_TIMING_T_CCD_CCS_EXT_DLY	0
#define	MV_DDR_MC6_TIMING_READ_GAP_EXTEND	0

/* registers definition */
#define MC6_BASE_ADDR			0x20000

/* timing registers */
/* dram timing */
#define MC6_REG_DRAM_CFG1		(MC6_BASE_ADDR + 0x300)
#define MC6_CAP_LATENCY_OFFS		28
#define MC6_CAP_LATENCY_MASK		0xf
#define MC6_CA_LATENCY_OFFS		24
#define MC6_CA_LATENCY_MASK		0xf
#define MC6_WL_SELECT_OFFS		15
#define MC6_WL_SELECT_MASK		0x1
#define MC6_CWL_OFFS			8
#define MC6_CWL_MASK			0x3f
#define MC6_CL_OFFS			0
#define MC6_CL_MASK			0x3f

/* mc6 timing */
#define MC6_REG_INIT_TIMING_CTRL_0	(MC6_BASE_ADDR + 0x380)
#define MC6_INIT_COUNT_NOP_OFFS		0
#define MC6_INIT_COUNT_NOP_MASK		0x3ffffff

#define MC6_REG_INIT_TIMING_CTRL_1	(MC6_BASE_ADDR + 0x384)
#define MC6_INIT_COUNT_OFFS		0
#define MC6_INIT_COUNT_MASK		0x7ffff

#define MC6_REG_INIT_TIMING_CTRL_2	(MC6_BASE_ADDR + 0x388)
#define MC6_RESET_COUNT_OFFS		0
#define MC6_RESET_COUNT_MASK		0x3fff

#define MC6_REG_ZQC_TIMING_0		(MC6_BASE_ADDR + 0x38C)
#define MC6_TZQINIT_OFFS		0
#define MC6_TZQINIT_MASK		0x7ff

#define MC6_REG_ZQC_TIMING_1		(MC6_BASE_ADDR + 0x390)
#define MC6_TZQCL_OFFS			0
#define MC6_TZQCL_MASK			0x3ff
#define MC6_TZQCS_OFFS			16
#define MC6_TZQCS_MASK			0xfff

#define MC6_REG_REFRESH_TIMING		(MC6_BASE_ADDR + 0x394)
#define MC6_TREFI_OFFS			0
#define MC6_TREFI_MASK			0x3fff
#define MC6_TRFC_OFFS			16
#define MC6_TRFC_MASK			0x7ff

#define MC6_REG_SELF_REFRESH_TIMING_0	(MC6_BASE_ADDR + 0x398)
#define MC6_TXSRD_OFFS			0
#define MC6_TXSRD_MASK			0x7ff
#define MC6_TXSNR_OFFS			16
#define MC6_TXSNR_MASK			0x7ff

#define MC6_REG_SELF_REFRESH_TIMING_1	(MC6_BASE_ADDR + 0x39C)
#define MC6_TCKSRX_OFFS			0
#define MC6_TCKSRX_MASK			0x1f
#define MC6_TCKSRE_OFFS			8
#define MC6_TCKSRE_MASK			0x1f

#define MC6_REG_POWER_DOWN_TIMING_0	(MC6_BASE_ADDR + 0x3A0)
#define MC6_TXARDS_OFFS			0
#define MC6_TXARDS_MASK			0x1f
enum {
	TXARDS = 0
};
#define MC6_TXP_OFFS			8
#define MC6_TXP_MASK			0x1f
#define MC6_TCKESR_OFFS			16
#define MC6_TCKESR_MASK			0x1f

#define MC6_REG_POWER_DOWN_TIMING_1	(MC6_BASE_ADDR + 0x3A4)
#define MC6_TPDEN_OFFS			0
#define MC6_TPDEN_MASK			0x7

#define MC6_REG_MRS_TIMING		(MC6_BASE_ADDR + 0x3A8)
#define MC6_TMRD_OFFS			0
#define MC6_TMRD_MASK			0x1f
#define MC6_TMOD_OFFS			8
#define MC6_TMOD_MASK			0x1f

#define MC6_REG_ACT_TIMING		(MC6_BASE_ADDR + 0x3AC)
#define MC6_TRAS_OFFS			0
#define MC6_TRAS_MASK			0x7f
#define MC6_TRCD_OFFS			8
#define MC6_TRCD_MASK			0x3f
#define MC6_TRC_OFFS			16
#define MC6_TRC_MASK			0xff
#define MC6_TFAW_OFFS			24
#define MC6_TFAW_MASK			0x3f

#define MC6_REG_PRE_CHARGE_TIMING	(MC6_BASE_ADDR + 0x3B0)
#define MC6_TRP_OFFS			0
#define MC6_TRP_MASK			0x3f
#define MC6_TRTP_OFFS			8
#define MC6_TRTP_MASK			0x1f
#define MC6_TWR_OFFS			16
#define MC6_TWR_MASK			0x3f
#define MC6_TRPA_OFFS			24
#define MC6_TRPA_MASK			0x3f

#define MC6_REG_CAS_RAS_TIMING_0	(MC6_BASE_ADDR + 0x3B4)
#define MC6_TWTR_S_OFFS			0
#define MC6_TWTR_S_MASK			0xf
#define MC6_TWTR_OFFS			8
#define MC6_TWTR_MASK			0x1f
#define MC6_TCCD_S_OFFS			16
#define MC6_TCCD_S_MASK			0x3
#define MC6_TCCD_OFFS			24
#define MC6_TCCD_MASK			0xf

#define MC6_REG_CAS_RAS_TIMING_1	(MC6_BASE_ADDR + 0x3B8)
#define MC6_TRRD_S_OFFS			0
#define MC6_TRRD_S_MASK			0xf
#define MC6_TRRD_OFFS			8
#define MC6_TRRD_MASK			0x1f
#define MC6_TDQS2DQ_OFFS		16
#define MC6_TDQS2DQ_MASK		0x3
enum {
	TDQS2DQ = 0
};

#define MC6_REG_OFF_SPEC_TIMING_0	(MC6_BASE_ADDR + 0x3BC)
#define MC6_TCCD_CCS_EXT_DLY_OFFS	0
#define MC6_TCCD_CCS_EXT_DLY_MASK	0xf
#define MC6_TCCD_CCS_WR_EXT_DLY_OFFS	8
#define MC6_TCCD_CCS_WR_EXT_DLY_MASK	0x7
#define MC6_TRWD_EXT_DLY_OFFS		16
#define MC6_TRWD_EXT_DLY_MASK		0x7

#define MC6_REG_OFF_SPEC_TIMING_1	(MC6_BASE_ADDR + 0x3C0)
#define MC6_READ_GAP_EXTEND_OFFS	0
#define MC6_READ_GAP_EXTEND_MASK	0x7
#define MC6_TCCD_CCS_EXT_DLY_MIN_OFFS	8
#define MC6_TCCD_CCS_EXT_DLY_MIN_MASK	0xf
#define MC6_TCCD_CCS_WR_EXT_DLY_MIN_OFFS	16
#define MC6_TCCD_CCS_WR_EXT_DLY_MIN_MASK	0x7

#define MC6_REG_DRAM_READ_TIMING	(MC6_BASE_ADDR + 0x3C4)
#define MC6_TDQSCK_OFFS			0
#define MC6_TDQSCK_MASK			0xf
enum {
	TDQSCK = 0
};

#define MC6_REG_CA_TRAIN_TIMING		(MC6_BASE_ADDR + 0x3C8)
#define MC6_TCACKEL_OFFS		0
#define MC6_TCACKEL_MASK		0x1f
#define MC6_TCAEXT_OFFS			8
#define MC6_TCAEXT_MASK			0x1f

#define MC6_REG_MPD_TIMING		(MC6_BASE_ADDR + 0x3CC)
#define MC6_TXMP_OFFS			0
#define MC6_TXMP_MASK			0x7ff
#define MC6_TMPX_S_OFFS			16
#define MC6_TMPX_S_MASK			0x7
#define MC6_TMPX_LH_OFFS		24
#define MC6_TMPX_LH_MASK		0xf

#define MC6_REG_PDA_TIMING		(MC6_BASE_ADDR + 0x3D0)
#define MC6_TMRD_PDA_OFFS		0
#define MC6_TMRD_PDA_MASK		0x1f

/* structures definitions */
/* struct used for DLB configuration array */
struct mv_ddr_mc6_timing {
	unsigned int cl;
	unsigned int cwl;
	unsigned int t_ckclk;
	unsigned int t_refi;
	unsigned int t_wr;
	unsigned int t_faw;
	unsigned int t_rrd;
	unsigned int t_rtp;
	unsigned int t_mod;
	unsigned int t_xp;
	unsigned int t_xs;
	unsigned int t_xs_fast;
	unsigned int t_ckesr;
	unsigned int t_cksrx;
	unsigned int t_cksre;
	unsigned int t_cke;
	unsigned int t_ras;
	unsigned int t_rcd;
	unsigned int t_rp;
	unsigned int t_rfc;
	unsigned int t_rrd_l;
	unsigned int t_wtr;
	unsigned int t_wtr_l;
	unsigned int t_rc;
	unsigned int t_ccd;
	unsigned int t_ccd_l;
	unsigned int t_mrd;
	unsigned int t_xsdll;
	unsigned int t_zqcr;
	unsigned int t_zqcs;
	unsigned int t_zqoper;
	unsigned int t_zqinit;
	unsigned int t_actpden;
	unsigned int t_resinit;
	unsigned int t_res;
	unsigned int t_restcke;
	unsigned int t_rwd_ext_dly;
	unsigned int t_ccd_ccs_wr_ext_dly;
	unsigned int t_ccd_ccs_ext_dly;
	unsigned int read_gap_extend;
	unsigned int t_caext;
	unsigned int t_cackel;
	unsigned int t_mpx_lh;
	unsigned int t_mpx_s;
	unsigned int t_xmp;
	unsigned int t_mrd_pda;
};

/* function definitions */
void mv_ddr_mc6_and_dram_timing_set(void);
void mv_ddr_mc6_timing_regs_cfg(unsigned int freq_mhz);
#endif	/* _MV_DDR_MC6_DRV_H */
