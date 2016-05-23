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

#include "ddr3_init.h"

#if defined(CONFIG_PHY_STATIC) || defined(CONFIG_MC_STATIC)
#include "mv_ddr_apn806_static.h"
#endif

#define DDR_INTERFACES_NUM		1
#define DDR_INTERFACE_OCTETS_NUM	9

#if defined(CONFIG_DDR4)
u16 apn806_odt_slope[] = {
	21443,
	1452,
	482,
	240,
	141,
	90,
	67,
	52
};

u16 apn806_odt_intercept[] = {
	1517,
	328,
	186,
	131,
	100,
	80,
	69,
	61
};
#endif /* CONFIG_DDR4 */

static u32 dq_bit_map_2_phy_pin[LAST_DDR_CFG_OPT][MAX_DQ_NUM] = {
	{/* LPDDR4_DIMM */},
	{/* LPDDR4_BRD */},
	{/* DDR4_DIMM */
	 /*DQ0	DQ1	DQ2	DQ3	DQ4	DQ5	DQ6	DQ7	DM*/
	 0,	7,	2,	6,	8,	10,	9,	1,/* 3,phy 0 */
	 6,	2,	1,	0,	8,	9,	10,	7,/* 3,phy 1 */
	 1,	2,	0,	6,	8,	9,	10,	7,/* 3,phy 2 */
	 1,	6,	0,	2,	8,	9,	10,	7,/* 3,phy 3 */
	 0,	2,	1,	6,	7,	8,	9,	10,/* 3,phy 4 */
	 0,	2,	1,	6,	7,	8,	9,	10,/* 3,phy 5 */
	 0,	6,	1,	2,	7,	8,	9,	10,/* 3,phy 6 */
	 0,	1,	2,	6,	7,	8,	9,	10,/* 3,phy 7 */
	 9,	0,	8,	7,	6,	10,	2,	1/* 3 phy 8 - ECC */},
	{/* DDR4_BRD */
	 0,	1,	2,	7,	10,	9,	8,	6,/* 3,phy 0 */
	 8,	10,	1,	2,	0,	7,	9,	6,/* 3,phy 1 */
	 9,	10,	2,	7,	0,	1,	6,	8,/* 3,phy 2 */
	 2,	1,	6,	0,	8,	10,	7,	9,/* 3,phy 3 */
	 0,	0,	0,	0,	0,	0,	0,	0,/* 0,phy 4 */
	 0,	0,	0,	0,	0,	0,	0,	0,/* 0,phy 5 */
	 0,	0,	0,	0,	0,	0,	0,	0,/* 0,phy 6 */
	 0,	0,	0,	0,	0,	0,	0,	0,/* 0,phy 7 */
	 2,	8,	0,	9,	6,	7,	10,	1/* 3 phy 8 - ECC */},
	{/* DDR3_DIMM */},
	{/* DDR3_BRD */}
};

static u8 mv_ddr_tip_clk_ratio_get(u32 freq)
{
	if ((freq == DDR_FREQ_LOW_FREQ) || (freq_val[freq] <= 400))
		return 1;

	return 2;
}


/*
 * Name:	mv_ddr_tip_freq_config_get
 * Desc:
 * Args:
 * Notes:
 * Returns:	MV_OK if success, other error code if fail
 */
static int mv_ddr_tip_freq_config_get(u8 dev_num, enum hws_ddr_freq freq,
				      struct hws_tip_freq_config_info
					*freq_config_info)
{
	if (freq_config_info == NULL)
		return MV_BAD_PARAM;

	freq_config_info->bw_per_freq = 0xff; /* TODO: TBD */
	freq_config_info->rate_per_freq = 0xff; /* TODO: TBD */
	freq_config_info->is_supported = 1;

	return MV_OK;
}

/*
 * Name:	ddr3_tip_apn806_if_read.
 * Desc:	this function reads from the tip and dunit in the ap806
 * Args:
 * Notes:
 * Returns:  MV_OK if success, other error code if fail.
 */
static int ddr3_tip_apn806_if_read(u8 dev_num, enum hws_access_type interface_access,
			  u32 if_id, u32 reg_addr, u32 *data, u32 mask)
{
	reg_addr += DUNIT_BASE_ADDR;
	*data = reg_read(reg_addr) & mask;

	return MV_OK;
}

/*
 * Name:	ddr3_tip_apn806_if_write.
 * Desc:	this function writes to the tip and dunit in the ap806
 * Args:
 * Notes:
 * Returns:  MV_OK if success, other error code if fail.
 */
static int ddr3_tip_apn806_if_write(u8 dev_num, enum hws_access_type interface_access,
			   u32 if_id, u32 reg_addr, u32 data_value,
			   u32 mask)
{
	u32 ui_data_read;

	if (mask != MASK_ALL_BITS) {
		CHECK_STATUS(ddr3_tip_apn806_if_read
			     (dev_num, ACCESS_TYPE_UNICAST, if_id, reg_addr,
			      &ui_data_read, MASK_ALL_BITS));
		data_value = (ui_data_read & (~mask)) | (data_value & mask);
	}

	reg_addr += DUNIT_BASE_ADDR;

	reg_write(reg_addr, data_value);

	return MV_OK;
}
/*
 * Name:	mv_ddr_init_ddr_freq_get
 * Desc:	gets ddr freq per ddr3 or ddr4 arrays
 * Args:
 * Notes:
 * Returns:	MV_OK if success, other error code if fail.
 */
static int mv_ddr_init_ddr_freq_get(int dev_num, enum hws_ddr_freq *freq)
{
	u32 ddr_clk_config;

	/* Read ddr clk config from sar */
	ddr_clk_config = (reg_read(SAR_REG_ADDR) >>
		RST2_CLOCK_FREQ_MODE_OFFS) &
		RST2_CLOCK_FREQ_MODE_MASK;

	switch (ddr_clk_config) {
#ifdef NO_EFUSE
#ifdef CONFIG_DDR4
	case SAR_CPU1800_0X2:
			*freq = DDR_FREQ_1200;
			break;
		case SAR_CPU1800_0X3:
			*freq = DDR_FREQ_1050;
			break;
		case SAR_CPU1600_0X4:
			*freq = DDR_FREQ_1050;
			break;
#endif
		case SAR_CPU1600_0X5:
			*freq = DDR_FREQ_900;
			break;
		case SAR_CPU1300_0X6:
			*freq = DDR_FREQ_800;
			break;
#ifdef CONFIG_DDR4
		case SAR_CPU1300_0X7:
			*freq = DDR_FREQ_650;
			break;
#endif
		case SAR_CPU1200_TBD:
			*freq = DDR_FREQ_800;
			break;
#else /* EFUSE */
		case SAR_CPU1600_0X0:
			*freq = DDR_FREQ_800;
			break;
		case SAR_CPU1600_0X1:
			*freq = DDR_FREQ_900;
			break;
#ifdef CONFIG_DDR4
		case SAR_CPU1000_0X2:
			*freq = DDR_FREQ_650;
			break;
#endif
		case SAR_CPU1200_0X3:
			*freq = DDR_FREQ_800;
			break;
		case SAR_CPU1400_0X4:
			*freq = DDR_FREQ_800;
			break;
		case SAR_CPU600_0X5:
			*freq = DDR_FREQ_800;
			break;
		case SAR_CPU800_0X6:
			*freq = DDR_FREQ_800;
			break;
		case SAR_CPU1000_0X7:
			*freq = DDR_FREQ_800;
			break;
#endif /* NO_EFUSE */
		default:
			*freq = 0;
			return MV_NOT_SUPPORTED;
	}

	return MV_OK;
}

/*
 * Name:	mv_ddr_target_div_calc
 * Desc:	calculates and returns target mc clk divider value
 * Args:
		curr_div -  current mc clk divider value
		curr_freq  - current frequency configured to
		target_freq - target frequency to step to
 * Notes:
 * Returns:	target mc clk divider value
 */
static u32 mv_ddr_target_div_calc(u32 curr_div, u32 curr_freq, u32 target_freq)
{
	u32 target_div;

	target_div = (curr_freq * curr_div) / target_freq;

	return target_div;
}

/*
 * Name:	mv_ddr_clk_dividers_set
 * Desc:	sets target frequency
		- changes clk dividers for mc6 and dunit
		  and executes DFS procedure.
		- dunit and mc6 freqs to be configured;
		  other freqs (e.g., hclk, fclk) are derivatives.
		- the DFS flow is as follows:
		  -> save specific SoC configurations (e.g., ODT),
		  -> turn DLL off (no need at low freq)
		  -> put memory in self-refresh mode
 * Args:
		ddr target frequency
 * Notes:
 * Returns:	status MV_OK or fail
 */
static int mv_ddr_clk_dividers_set(u8 dev_num, u32 if_id, enum hws_ddr_freq target_ddr_freq)
{
	u32 mc_div, ddr_div;
	u32 mc_target_div, ddr_target_div;
	u32 init_ddr_freq_val, target_ddr_freq_val;
	u32 reg;
	enum hws_ddr_freq init_ddr_freq;

	if (if_id != 0) {
		DEBUG_TRAINING_ACCESS(DEBUG_LEVEL_ERROR,
			              ("mv_ddr: a70x0: interface 0x%x not supported\n",
				       if_id));
		return MV_BAD_PARAM;
	}

	/* get ddr freq from sar */
	mv_ddr_init_ddr_freq_get(DEV_NUM_0, &init_ddr_freq);

	/* get mc & ddr clk dividers values */
	reg = reg_read(DEV_GEN_CTRL1_REG_ADDR);
	ddr_div = (reg >> MISC_CLKDIV_RATIO_1_OFFS) & MISC_CLKDIV_RATIO_1_MASK;
	mc_div = (reg >> MISC_CLKDIV_RATIO_2_OFFS) & MISC_CLKDIV_RATIO_2_MASK;

	init_ddr_freq_val = freq_val[init_ddr_freq];
	target_ddr_freq_val = freq_val[target_ddr_freq];

	/* calc mc & ddr target clk divider value */
	mc_target_div = mv_ddr_target_div_calc(mc_div, init_ddr_freq_val, target_ddr_freq_val);
	ddr_target_div = mv_ddr_target_div_calc(ddr_div, init_ddr_freq_val, target_ddr_freq_val);

	/* FIXME: temporary solution bypassing previously calculated values */
	if (target_ddr_freq == DDR_FREQ_LOW_FREQ) { /* 100MHz; clk src / 16 for mc, / 8 for ddr */
		mc_target_div = 16;
		ddr_target_div = 8;
	} else { /* 800MHz; clk src / 2 for mc, / 1 for ddr */
		mc_target_div = 2;
		ddr_target_div = 1;
	}

	reg = reg_read(DEV_GEN_CTRL1_REG_ADDR);
	reg &= ~(MISC_CLKDIV_RATIO_2_MASK << MISC_CLKDIV_RATIO_2_OFFS | MISC_CLKDIV_RATIO_1_MASK << MISC_CLKDIV_RATIO_1_OFFS);
	reg |= mc_target_div << MISC_CLKDIV_RATIO_2_OFFS | ddr_target_div << MISC_CLKDIV_RATIO_1_OFFS;
	reg_write(DEV_GEN_CTRL1_REG_ADDR, reg);

	/* Reload force, relax enable, align enable set */
	reg = reg_read(DEV_GEN_CTRL3_REG_ADDR);
	reg &= ~(MISC_CLKDIV_RELOAD_FORCE_MASK << MISC_CLKDIV_RELOAD_FORCE_OFFS |
		 MISC_CLKDIV_RELAX_EN_MASK << MISC_CLKDIV_RELAX_EN_OFFS |
		 MISC_CLKDIV_ALIGN_EN_MASK << MISC_CLKDIV_ALIGN_EN_OFFS);
	reg |= RELOAD_FORCE_VAL << MISC_CLKDIV_RELOAD_FORCE_OFFS |
	       RELAX_EN_VAL << MISC_CLKDIV_RELAX_EN_OFFS |
	       ALIGN_EN_VAL << MISC_CLKDIV_ALIGN_EN_OFFS;
	reg_write(DEV_GEN_CTRL3_REG_ADDR, reg);

	/* Reload smooth */
	reg = reg_read(DEV_GEN_CTRL4_REG_ADDR);
	reg &= ~(MISC_CLKDIV_RELOAD_SMOOTH_MASK << MISC_CLKDIV_RELOAD_SMOOTH_OFFS);
	reg |= RELOAD_SMOOTH_VAL << MISC_CLKDIV_RELOAD_SMOOTH_OFFS;
	reg_write(DEV_GEN_CTRL4_REG_ADDR, reg);

	/* Toggle reload ratio first 0x1 then 0x0*/
	reg = reg_read(DEV_GEN_CTRL4_REG_ADDR);
	reg &= ~(MISC_CLKDIV_RELOAD_RATIO_MASK << MISC_CLKDIV_RELOAD_RATIO_OFFS);
	reg |= RELOAD_RATIO_VAL << MISC_CLKDIV_RELOAD_RATIO_OFFS;
	reg_write(DEV_GEN_CTRL4_REG_ADDR, reg);

	/* delay between toggles */
	mdelay(10); /* TODO: check the delay value */
	reg = reg_read(DEV_GEN_CTRL4_REG_ADDR);
	reg &= ~(MISC_CLKDIV_RELOAD_RATIO_MASK << MISC_CLKDIV_RELOAD_RATIO_OFFS);
	reg_write(DEV_GEN_CTRL4_REG_ADDR, reg);

	/* Unblock phase_sync_mc_clk in RFU */
	reg = reg_read(CLKS_CTRL_REG_ADDR);
	reg &= ~(BLOCK_PHI_RST_TO_RING_TO_MC_CLK_MASK << BLOCK_PHI_RST_TO_RING_TO_MC_CLK_OFFS);
	reg_write(CLKS_CTRL_REG_ADDR, reg);

	/* delay between toggles */
	mdelay(10); /* TODO: check the delay value */

	/* Ring-MC clock f2s reset toggle */
	reg = reg_read(CLKS_CTRL_REG_ADDR);
	reg &= ~(RING_CLK_TO_ALL_CLK_PHI_RST_MASK << RING_CLK_TO_ALL_CLK_PHI_RST_OFFS);
	reg_write(CLKS_CTRL_REG_ADDR, reg);

	/* delay between toggles */
	mdelay(100); /* TODO: check the delay value */

	reg = reg_read(CLKS_CTRL_REG_ADDR);
	reg |= (RING_CLK_TO_ALL_CLK_PHI_RST_MASK << RING_CLK_TO_ALL_CLK_PHI_RST_OFFS);
	reg_write(CLKS_CTRL_REG_ADDR, reg);

	mdelay(50); /* TODO: check the delay value */

	reg = reg_read(CLKS_CTRL_REG_ADDR);
	reg |= (BLOCK_PHI_RST_TO_RING_TO_MC_CLK_MASK << BLOCK_PHI_RST_TO_RING_TO_MC_CLK_OFFS);
	reg_write(CLKS_CTRL_REG_ADDR, reg);

	return MV_OK;
}

/*
 * Name:     ddr3_tip_apn806_select_ddr_controller.
 * Desc:     Enable/Disable access to Marvell's server.
 * Args:     dev_num     - device number
 *           enable        - whether to enable or disable the server
 * Notes:
 * Returns:  MV_OK if success, other error code if fail.
 */
static int ddr3_tip_apn806_select_ddr_controller(u8 dev_num, int enable)
{
	u32 reg;

	reg = reg_read(CS_ENABLE_REG);

	if (enable)
		reg |= (1 << TUNING_ACTIVE_SEL_OFFS);
	else
		reg &= ~(1 << TUNING_ACTIVE_SEL_OFFS);

	reg_write(CS_ENABLE_REG, reg);

	return MV_OK;
}

/*
 * external read from memory
 */
int ddr3_tip_ext_read(u32 dev_num, u32 if_id, u32 reg_addr,
		      u32 num_of_bursts, u32 *data)
{
	u32 burst_num;

	for (burst_num = 0; burst_num < num_of_bursts * 8; burst_num++)
		data[burst_num] = readl(reg_addr + 4 * burst_num);

	return MV_OK;
}

/*
 * external write to memory
 */
int ddr3_tip_ext_write(u32 dev_num, u32 if_id, u32 reg_addr,
		       u32 num_of_bursts, u32 *data) {
	u32 burst_num;

	for (burst_num = 0; burst_num < num_of_bursts * 8; burst_num++)
		writel(data[burst_num], reg_addr + 4 * burst_num);

	return MV_OK;
}

/*
 * Name:     ddr3_tip_init_apn806_silicon.
 * Desc:     init Training SW DB.
 * Args:
 * Notes:
 * Returns:  MV_OK if success, other error code if fail.
 */
static int ddr3_tip_init_apn806_silicon(u32 dev_num, u32 board_id)
{
	struct hws_tip_config_func_db config_func;
#if !defined(CONFIG_DDR4)
	enum hws_ddr_freq ddr_freq = DDR_FREQ_LOW_FREQ;
#endif
	struct hws_topology_map *tm = ddr3_get_topology_map();

	/* new read leveling version */
	config_func.tip_dunit_read_func = ddr3_tip_apn806_if_read;
	config_func.tip_dunit_write_func = ddr3_tip_apn806_if_write;
	config_func.tip_dunit_mux_select_func =
		ddr3_tip_apn806_select_ddr_controller;
	config_func.tip_get_freq_config_info_func = mv_ddr_tip_freq_config_get;
	config_func.tip_set_freq_divider_func = mv_ddr_clk_dividers_set;
	config_func.tip_get_device_info_func = NULL;
	config_func.tip_get_temperature = NULL;
	config_func.tip_get_clock_ratio = mv_ddr_tip_clk_ratio_get;
	config_func.tip_external_read = ddr3_tip_ext_read;
	config_func.tip_external_write = ddr3_tip_ext_write;

	ddr3_tip_init_config_func(dev_num, &config_func);

#if defined(a80x0) || defined(a80x0_cust)
	ddr3_tip_register_dq_table(dev_num, dq_bit_map_2_phy_pin[DDR4_DIMM]);
#else
	ddr3_tip_register_dq_table(dev_num, dq_bit_map_2_phy_pin[DDR4_BRD]);
#endif
	/* set device attributes*/
	ddr3_tip_dev_attr_init(dev_num);
	ddr3_tip_dev_attr_set(dev_num, MV_ATTR_TIP_REV, MV_TIP_REV_4);
	ddr3_tip_dev_attr_set(dev_num, MV_ATTR_PHY_EDGE, MV_DDR_PHY_EDGE_NEGATIVE);
	ddr3_tip_dev_attr_set(dev_num, MV_ATTR_OCTET_PER_INTERFACE, DDR_INTERFACE_OCTETS_NUM);
	ddr3_tip_dev_attr_set(dev_num, MV_ATTR_INTERLEAVE_WA, 0);

#if defined(CONFIG_DDR4)
	mask_tune_func = (SET_LOW_FREQ_MASK_BIT |
			  LOAD_PATTERN_MASK_BIT |
			  SET_TARGET_FREQ_MASK_BIT |
			  WRITE_LEVELING_TF_MASK_BIT |
			  READ_LEVELING_TF_MASK_BIT |
			  RECEIVER_CALIBRATION_MASK_BIT |
			  DQ_VREF_CALIBRATION_MASK_BIT);
	rl_mid_freq_wa = 0;
#else /* CONFIG_DDR4 */
	mask_tune_func = (SET_LOW_FREQ_MASK_BIT |
			  LOAD_PATTERN_MASK_BIT |
			  SET_MEDIUM_FREQ_MASK_BIT | WRITE_LEVELING_MASK_BIT |
			  WRITE_LEVELING_SUPP_MASK_BIT |
			  READ_LEVELING_MASK_BIT |
			  PBS_RX_MASK_BIT |
			  PBS_TX_MASK_BIT |
			  SET_TARGET_FREQ_MASK_BIT |
			  WRITE_LEVELING_TF_MASK_BIT |
			  WRITE_LEVELING_SUPP_TF_MASK_BIT |
			  READ_LEVELING_TF_MASK_BIT |
			  CENTRALIZATION_RX_MASK_BIT |
			  CENTRALIZATION_TX_MASK_BIT);
	rl_mid_freq_wa = 1;

	if ((ddr_freq == DDR_FREQ_333) || (ddr_freq == DDR_FREQ_400)) {
		mask_tune_func = (WRITE_LEVELING_MASK_BIT |
				  LOAD_PATTERN_2_MASK_BIT |
				  WRITE_LEVELING_SUPP_MASK_BIT |
				  READ_LEVELING_MASK_BIT |
				  PBS_RX_MASK_BIT |
				  PBS_TX_MASK_BIT |
				  CENTRALIZATION_RX_MASK_BIT |
				  CENTRALIZATION_TX_MASK_BIT);
		rl_mid_freq_wa = 0; /* WA not needed if 333/400 is TF */
	}

	/* Supplementary not supported for ECC modes */
	if (1 == ddr3_if_ecc_enabled()) {
		mask_tune_func &= ~WRITE_LEVELING_SUPP_TF_MASK_BIT;
		mask_tune_func &= ~WRITE_LEVELING_SUPP_MASK_BIT;
		mask_tune_func &= ~PBS_TX_MASK_BIT;
		mask_tune_func &= ~PBS_RX_MASK_BIT;
	}
#endif /* CONFIG_DDR4 */

	ca_delay = 0;
	delay_enable = 1;
	dfs_low_freq = DFS_LOW_FREQ_VALUE;
	calibration_update_control = 1;

	init_freq = tm->interface_params[first_active_if].memory_freq;

	return MV_OK;
}

int ddr3_silicon_pre_init(void)
{
	int dev_num = 0;
	int board_id = 0;
	static int init_done;
	struct hws_topology_map *tm = ddr3_get_topology_map();

	if (init_done == 1)
		return MV_OK;

	if (tm == NULL)
		return MV_FAIL;

	hws_ddr3_tip_load_topology_map(dev_num, tm);
	ddr3_tip_init_apn806_silicon(dev_num, board_id);

	init_done = 1;
	return MV_OK;
}

int ddr3_post_run_alg(void)
{
	return MV_OK;
}

int ddr3_silicon_post_init(void)
{
	return MV_OK;
}

int mv_ddr_pre_training_soc_config(const char *ddr_type)
{
	return MV_OK;
}

int mv_ddr_post_training_soc_config(const char *ddr_type)
{
	/* set mux to MC6 */
	mmio_write_32(0xf00116d8, 0x38c);

	return MV_OK;
}

/* FIXME: this is a stub function required by DDR4 code */
u32 ddr3_tip_get_init_freq(void)
{
	return 0;
}

void mv_ddr_mc_config(void)
{
	/* Memory controller initializations */
	struct init_cntr_param init_param;
	int status;

	init_param.do_mrs_phy = 1;
	init_param.is_ctrl64_bit = 0;
	init_param.init_phy = 1;
	init_param.msys_init = 1;

	status = hws_ddr3_tip_init_controller(0, &init_param);
	if (MV_OK != status)
		printf("mv_ddr: mc init failed: err code 0x%x\n", status);

	/* TODO: Add MC MUX setting */

	/* TODO: Add McKinley6 init function */
}

#ifdef CONFIG_MC_STATIC
#if defined(a70x0) || defined(a70x0_cust) || defined(a80x0) || defined(a80x0_cust)
static void ddr_static_config(void)
{
	struct mk6_reg_data *reg_data = ddr_static_setup;

	for (; reg_data->offset != -1; reg_data++)
		mmio_write_32(reg_data->offset, reg_data->value);
}
#else
static void mk6_mac_init(void)
{
	struct mk6_reg_data *mac_regs = mk6_mac_setup;

	/* RFU crap setups */
	reg_write(0x6f0098, 0x0040004e);
	reg_write(0x6F0108, 0xFFFF0001);
	reg_write(0x841100, 0x80000000);

	for (; mac_regs->offset != -1 ; mac_regs++) {
		reg_write(MC6_BASE_ADDR + mac_regs->offset, mac_regs->value);
	}
}
#endif

int mv_ddr_mc_static_config(void)
{
#if defined(a70x0) || defined(a70x0_cust) || defined(a80x0) || defined(a80x0_cust)
	ddr_static_config();
#else
	mk6_mac_init();
#endif

	mdelay(10);

	return MV_OK;
}

#endif /* CONFIG_MV_DDR_STATIC_MC */

#ifdef CONFIG_PHY_STATIC
#if defined(a70x0) || defined(a70x0_cust) || defined(a80x0) || defined(a80x0_cust)
static int mv_ddr_apn806_phy_static_config(u32 if_id, u32 subphys_num, enum hws_ddr_phy subphy_type)
{
#if 0
	u32 i, mode, subphy_id, dev_num = 0;

	mode = ddr3_get_static_ddr_mode();
	if (subphy_type == DDR_PHY_DATA) {
		for (subphy_id = 0; subphy_id < subphys_num; subphy_id++) {
			i = 0;
			while (ddr_modes[mode].data_phy_regs[i].reg_addr != 0xffffffff) {
				ddr3_tip_bus_write(dev_num, ACCESS_TYPE_UNICAST, if_id, ACCESS_TYPE_UNICAST,
							subphy_id, subphy_type, ddr_modes[mode].data_phy_regs[i].reg_addr,
							ddr_modes[mode].data_phy_regs[i].reg_data[subphy_id]);
				i++;
			}
		}
	} else {
		for (subphy_id = 0; subphy_id < subphys_num; subphy_id++) {
			i = 0;
			while (ddr_modes[mode].ctrl_phy_regs[i].reg_addr != 0xffffffff) {
				ddr3_tip_bus_write(dev_num, ACCESS_TYPE_UNICAST, if_id, ACCESS_TYPE_UNICAST,
							subphy_id, subphy_type, ddr_modes[mode].ctrl_phy_regs[i].reg_addr,
							ddr_modes[mode].ctrl_phy_regs[i].reg_data[subphy_id]);
				i++;
			}
		}
	}
#endif
	return MV_OK;
}
#else
static void mk6_phy_init(void)
{
	struct mk6_reg_data *phy_regs = mk6_phy_setup;
	uint32_t reg, cs_mask;

	for (; phy_regs->offset != -1 ; phy_regs++) {
		reg_write(MC6_BASE_ADDR + phy_regs->offset, phy_regs->value);
	}

	/* Trigger DDR init for Channel 0, all Chip-Selects */
	reg = SDRAM_INIT_REQ_MASK;
	reg |= CMD_CH_ENABLE(0);
	cs_mask = 0x1;

	reg |= CMD_CS_MASK(cs_mask);
	reg_write(MC6_BASE_ADDR + MCK6_USER_COMMAND_0_REG, reg);
}
#endif

void mv_ddr_phy_static_config(void)
{
#if defined(a70x0) || defined(a70x0_cust) || defined(a80x0) || defined(a80x0_cust)
	/* TODO: Need to use variable for subphys number */
	mv_ddr_apn806_phy_static_config(0, 4, DDR_PHY_DATA);
	mv_ddr_apn806_phy_static_config(0, 3, DDR_PHY_CONTROL);
#else
	mk6_phy_init();
#endif
}
#endif /* CONFIG_PHY_STATIC */
