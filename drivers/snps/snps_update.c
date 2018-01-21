/*******************************************************************************
Copyright (C) 2017 Marvell International Ltd.

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

#include "snps.h"
#include "snps_regs.h"
#include "ddr_topology_def.h"

/* example of an update routine that returns runtime depndant value */
#if 0 /* the routine below is just an example - not to be used */
u16 snps_get_some_runtime_value(void)
{
	u16 val = 0, result;
	u32 state, sequence_ctrl;

	debug_enter();

	/* Get current state */
	state = snps_get_state();

	/* Get next training step */
	sequence_ctrl = snps_sequence_ctrl_get();

	switch (state) {
	case TRAINING_1D:
		switch (sequence_ctrl) {
		case SEQUENCE_CTRL_1D_WR_LVL:
			val = value_for_1d_wr_lv;
			break;
		case SEQUENCE_CTRL_1D_RX_EN:
			/* check some result of previous step  (write leveling) */
			result = snps_get_result(MSG_BLK_1D_CDD_RW_1_0, 1, SEQUENCE_CTRL_1D_WR_LVL);
			if (result < threshold_for_cdd_rw_1_0)
				val = value1_for_1d_rx_en_dq;
			else
				val = value2_for_1d_rx_en_dq;
			break;
		default:
			printf("%s: Error: invalid sequence_ctrl for 1D state (0x%x)\n", __func__, sequence_ctrl);
			break;
		}
	case TRAINING_2D:
		switch (sequence_ctrl) {
		case SEQUENCE_CTRL_2D_READ_DQS:
			val = value_for_2d_read_dqs;
			break;
		case SEQUENCE_CTRL_2D_WRITE_DQ:
			val = value_for_2d_write_dq;
			break;
		default:
			printf("%s: Error: invalid sequence_ctrl for 2D state (0x%x)\n", __func__, sequence_ctrl);
			break;
		}
		break;
	}

	pr_debug("%s: updated value for register XXX is 0x%x\n", __func__, val);
	debug_exit();
	return val;
}
#endif

/* example of an update routine that returns runtime topology dependant value */
u16 sar_get_ddr_freq(void)
{
	debug_enter();
	debug_exit();
	return 0x320; /* 800Mhz */
}

/* To control the total number of debug messages, a verbosity subfield
 * (HdtCtrl, Hardware Debug Trace Control) exists in the message block.
 * Every message has a verbosity level associated with it, and as the
 * HdtCtrl value is increased, less important s messages stop being sent
 * through the mailboxes. The meanings of several major HdtCtrl thresholds are explained below:
 * 0x05 = Detailed debug messages (e.g. Eye delays)
 * 0x0A = Coarse debug messages (e.g. rank information)
 * 0xC8 = Stage completion
 * 0xC9 = Assertion messages
 * 0xFF = Firmware completion messages only
 */
u16 snps_get_hdtctrl(void)
{
	debug_enter();
	u16 hdtctrl, sequence_ctrl = snps_sequence_ctrl_get();


	/* for write leveling stage, use assertion messages */
	if (sequence_ctrl & SEQUENCE_CTRL_1D_WR_LVL)
		hdtctrl = HDT_CTRL_COARSE_DEBUG;
	else
		hdtctrl = HDT_CTRL_FW_COMPLETION;

	debug_exit();
	return hdtctrl;

	/* TODO: add support for setting hdtctrl according to log_level */
	/* TODO: Do we need a dynamic hdtctrl? (e.g verbose 1D, and more quiet 2D) */
}

/* TODO: for all functions below get freq as an api */
u16 init_phy_pllctrl2_get(void)
{
	debug_enter();

	u16 ret_val = 0;
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	enum mv_ddr_freq freq = tm->interface_params[0].memory_freq;

	if (freq == MV_DDR_FREQ_800)
		ret_val = PLL_FREQ_SEL_800MHZ;
	else if (freq == MV_DDR_FREQ_1200)
		ret_val = PLL_FREQ_SEL_1200MHZ;
	else
		printf("error: %s: unsupported frequency found\n", __func__);

	debug_exit();
	return ret_val;
}

u16 init_phy_ardptrinitval_get(void)
{
	debug_enter();

	u16 ret_val = 0;
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	enum mv_ddr_freq freq = tm->interface_params[0].memory_freq;

	if (freq == MV_DDR_FREQ_800)
		ret_val = ARD_PTR_INIT_800MHZ;
	else if (freq == MV_DDR_FREQ_1200)
		ret_val = ARD_PTR_INIT_1200MHZ;
	else
		printf("error: %s: unsupported frequency found\n", __func__);

	debug_exit();
	return ret_val;
}

u16 init_phy_procodttimectl_get(void)
{
	debug_enter();

	u16 ret_val = 0;
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	enum mv_ddr_freq freq = tm->interface_params[0].memory_freq;

	if (freq == MV_DDR_FREQ_800)
		ret_val = PROCODTTIMECTL_800MHZ;
	else if (freq == MV_DDR_FREQ_1200)
		ret_val = PROCODTTIMECTL_1200MHZ;
	else
		printf("error: %s: unsupported frequency found\n", __func__);

	debug_exit();
	return ret_val;
}

u16 init_phy_caluclkinfo_get(void)
{
	debug_enter();

	u16 ret_val = 0;
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	enum mv_ddr_freq freq = tm->interface_params[0].memory_freq;

	if (freq == MV_DDR_FREQ_800)
		ret_val = ATXDLY_800MHZ;
	else if (freq == MV_DDR_FREQ_1200)
		ret_val = ATXDLY_1200MHZ;
	else
		printf("error: %s: unsupported frequency found\n", __func__);

	debug_exit();
	return ret_val;
}

u16 init_phy_seq0bdly0_get(void)
{
	debug_enter();

	u16 ret_val = 0;
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	enum mv_ddr_freq freq = tm->interface_params[0].memory_freq;

	if (freq == MV_DDR_FREQ_800)
		ret_val = SEQ0BDLY0_800MHZ;
	else if (freq == MV_DDR_FREQ_1200)
		ret_val = SEQ0BDLY0_1200MHZ;
	else
		printf("error: %s: unsupported frequency found\n", __func__);

	debug_exit();
	return ret_val;
}

u16 init_phy_seq0bdly1_get(void)
{
	debug_enter();

	u16 ret_val = 0;
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	enum mv_ddr_freq freq = tm->interface_params[0].memory_freq;

	if (freq == MV_DDR_FREQ_800)
		ret_val = SEQ0BDLY1_800MHZ;
	else if (freq == MV_DDR_FREQ_1200)
		ret_val = SEQ0BDLY1_1200MHZ;
	else
		printf("error: %s: unsupported frequency found\n", __func__);

	debug_exit();
	return ret_val;
}

u16 init_phy_seq0bdly2_get(void)
{
	debug_enter();

	u16 ret_val = 0;
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	enum mv_ddr_freq freq = tm->interface_params[0].memory_freq;

	if (freq == MV_DDR_FREQ_800)
		ret_val = SEQ0BDLY2_800MHZ;
	else if (freq == MV_DDR_FREQ_1200)
		ret_val = SEQ0BDLY2_1200MHZ;
	else
		printf("error: %s: unsupported frequency found\n", __func__);

	debug_exit();
	return ret_val;
}

u16 dmem_1d_2d_dram_freq_get(void)
{
	debug_enter();

	u16 ret_val = 0;
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	enum mv_ddr_freq freq = tm->interface_params[0].memory_freq;

	if (freq == MV_DDR_FREQ_800)
		ret_val = DATA_RATE_1600_MT_S_800MHZ;
	else if (freq == MV_DDR_FREQ_1200)
		ret_val = DATA_RATE_1600_MT_S_1200MHZ;
	else
		printf("error: %s: unsupported frequency found\n", __func__);

	debug_exit();
	return ret_val;
}

u16 dmem_1d_2d_mr0_get(void)
{
	debug_enter();

	u16 ret_val = 0;
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	enum mv_ddr_freq freq = tm->interface_params[0].memory_freq;

	if (freq == MV_DDR_FREQ_800)
		ret_val = MR0_800MHZ;
	else if (freq == MV_DDR_FREQ_1200)
		ret_val = MR0_1200MHZ;
	else
		printf("error: %s: unsupported frequency found\n", __func__);

	debug_exit();
	return ret_val;
}

u16 dmem_1d_2d_mr2_get(void)
{
	debug_enter();

	u16 ret_val = 0;
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	enum mv_ddr_freq freq = tm->interface_params[0].memory_freq;

	if (freq == MV_DDR_FREQ_800)
		ret_val = MR2_800MHZ;
	else if (freq == MV_DDR_FREQ_1200)
		ret_val = MR2_1200MHZ;
	else
		printf("error: %s: unsupported frequency found\n", __func__);

	debug_exit();
	return ret_val;
}

u16 dmem_1d_2d_mr6_get(void)
{
	debug_enter();

	u16 ret_val = 0;
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	enum mv_ddr_freq freq = tm->interface_params[0].memory_freq;

	if (freq == MV_DDR_FREQ_800)
		ret_val = MR6_800MHZ;
	else if (freq == MV_DDR_FREQ_1200)
		ret_val = MR6_1200MHZ;
	else
		printf("error: %s: unsupported frequency found\n", __func__);

	debug_exit();
	return ret_val;
}
