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
#if defined(CONFIG_64BIT)
	mask_tune_func = (SET_LOW_FREQ_MASK_BIT |
			  LOAD_PATTERN_MASK_BIT |
			  SET_TARGET_FREQ_MASK_BIT |
			  WRITE_LEVELING_TF_MASK_BIT |
			  READ_LEVELING_TF_MASK_BIT |
			  RECEIVER_CALIBRATION_MASK_BIT |
			  DQ_VREF_CALIBRATION_MASK_BIT);
#else
	mask_tune_func = (SET_LOW_FREQ_MASK_BIT |
			  LOAD_PATTERN_MASK_BIT |
			  SET_TARGET_FREQ_MASK_BIT |
			  WRITE_LEVELING_TF_MASK_BIT |
			  WRITE_LEVELING_SUPP_TF_MASK_BIT |
			  READ_LEVELING_TF_MASK_BIT |
			  RECEIVER_CALIBRATION_MASK_BIT |
			  DQ_VREF_CALIBRATION_MASK_BIT);
#endif
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

#if defined(a70x0) || defined(a70x0_cust) || defined(a80x0) || defined(a80x0_cust)
/*
 * Name:     mv_ddr_convert_read_params_from_tip2mc6.
 * Desc:     convert the read ready and the read sample from tip to mc6.
 * Args:
 * Notes:
 * Returns:
 */
static void mv_ddr_convert_read_params_from_tip2mc6(void)
{
	u32	if_id, cs, cl_val, cwl_val, phy_rl_cycle_dly_mc6, rd_smp_dly_tip, phy_rfifo_rptr_dly_val;
	u32	mb_read_data_latency;
	struct hws_topology_map *tm = ddr3_get_topology_map();
	u32 max_cs = ddr3_tip_max_cs_get(DEV_NUM_0);
	enum hws_speed_bin speed_bin_index;
	enum hws_ddr_freq freq;

	for (if_id = 0; if_id < MAX_INTERFACE_NUM; if_id++) {
		VALIDATE_IF_ACTIVE(tm->if_act_mask, if_id);
		speed_bin_index = tm->interface_params[if_id].speed_bin_index;
		freq = tm->interface_params[first_active_if].memory_freq;
		cl_val = cas_latency_table[speed_bin_index].cl_val[freq];
		cwl_val = cas_write_latency_table[speed_bin_index].cl_val[freq];

		reg_bit_clrset(MC6_REG_DRAM_CFG1,
			cwl_val << MC6_CWL_OFFS | cl_val << MC6_CL_OFFS,
			MC6_CWL_MASK << MC6_CWL_OFFS | MC6_CL_MASK << MC6_CL_OFFS);

		for (cs = 0; cs < max_cs; cs++) {
			ddr3_tip_apn806_if_read(DEV_NUM_0, PARAM_NOT_CARE, if_id, REG_READ_DATA_SAMPLE_DELAYS_ADDR,
						&rd_smp_dly_tip, MASK_ALL_BITS);

			rd_smp_dly_tip &= (REG_READ_DATA_SAMPLE_DELAYS_MASK <<
				(REG_READ_DATA_SAMPLE_DELAYS_OFFS * cs));

			phy_rl_cycle_dly_mc6 = 2 * (rd_smp_dly_tip - cl_val) + 1;

			/* if cl is odd value add 2 else decrease 2 */
			if (cl_val & 0x1)
				phy_rl_cycle_dly_mc6 += 2;
			else
				phy_rl_cycle_dly_mc6 -= 2;

			/* if cwl is odd add 2 */
			if (cwl_val & 0x1)
				phy_rl_cycle_dly_mc6 += 2;

			/* TODO: how to write to mc6 per interface */
			reg_bit_clrset(REG_CH0_PHY_RL_CTRL_ADDR(cs),
					  phy_rl_cycle_dly_mc6 << PHY_RL_CYCLE_DLY_MC6_OFFS,
					  PHY_RL_CYCLE_DLY_MC6_MASK << PHY_RL_CYCLE_DLY_MC6_OFFS);
		}
	}

	/* TODO: change these constant initialization below to functions */
	phy_rfifo_rptr_dly_val = 9;	/*FIXME: this parameter should be between 6 to 12 */
	reg_bit_clrset(REG_PHY_CONTROL_1_ADDR, phy_rfifo_rptr_dly_val << PHY_RFIFO_RPTR_DLY_VAL_OFFS,
			  PHY_RFIFO_RPTR_DLY_VAL_MASK << PHY_RFIFO_RPTR_DLY_VAL_OFFS);

	mb_read_data_latency = 8;	/*FIXME: this parameter should be between 4 to 12 */
	reg_bit_clrset(REG_RDP_CONTROL_ADDR, mb_read_data_latency << MB_READ_DATA_LATENCY_CH0_OFFS,
			  MB_READ_DATA_LATENCY_CH0_MASK << MB_READ_DATA_LATENCY_CH0_OFFS);
}
#endif

#if defined(a70x0) || defined(a70x0_cust) || defined(a80x0) || defined(a80x0_cust)
/*
 * Name:     mv_ddr3_tip_pre_charge.
 * Desc:     precharges the ddr banks before moving to mc6 controller
 * Args:
 * Notes:
 * Returns:
 */
static void mv_ddr3_tip_pre_charge(void)
{
	u32 if_id;

	struct hws_topology_map *tm = ddr3_get_topology_map();
	for (if_id = 0; if_id < MAX_INTERFACE_NUM; if_id++) {
		VALIDATE_IF_ACTIVE(tm->if_act_mask, if_id);

		for (if_id = 0; if_id < MAX_INTERFACE_NUM; if_id++) {
			VALIDATE_IF_ACTIVE(tm->if_act_mask, if_id)
			ddr3_tip_apn806_if_write(DEV_NUM_0, ACCESS_TYPE_MULTICAST, if_id, REG_SDRAM_OPERATION_ADDR,
						 (~tm->interface_params[if_id].as_bus_params[0].cs_bitmask) <<
						  REG_SDRAM_OPERATION_CS_OFFS |
						  CMD_PRECHARGE << REG_SDRAM_CMD_OFFS,
						  REG_SDRAM_CMD_MASK << REG_SDRAM_CMD_OFFS |
						  REG_SDRAM_OPERATION_CMD_MASK << REG_SDRAM_OPERATION_CS_OFFS);
		}

		for (if_id = 0; if_id < MAX_INTERFACE_NUM; if_id++) {
			VALIDATE_IF_ACTIVE(tm->if_act_mask, if_id)
			if (ddr3_tip_if_polling(DEV_NUM_0, ACCESS_TYPE_UNICAST, if_id, 0, REG_SDRAM_CMD_MASK,
						REG_SDRAM_OPERATION_ADDR, MAX_POLLING_ITERATIONS) != MV_OK)
				printf("Pre-charge: Poll fail");
		}
	}
}
#endif

int ddr3_post_run_alg(void)
{
#if defined(a70x0) || defined(a70x0_cust) || defined(a80x0) || defined(a80x0_cust)
	mv_ddr_convert_read_params_from_tip2mc6();
	mv_ddr3_tip_pre_charge();
#endif

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
/* FIXME: remove this configuration which is needed because
 * running over the static parameters when calling the timing function
 * during the DFS algorithm
 * add the two registers to the static configuration
 * these registers initialize the dunit and the mc6
 */
	reg_write(0x11480, 0x1);
	reg_write(0x20020, 0x13000001);
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

/*
 * TODO: dq to pad mapping detection code to be relocated
 * to the generic part of mv_ddr code.
 */
#if defined(MV_DDR_DQ_MAPPING_DETECT)
static u32 mv_ddr_pad_to_dq_detect(u32 dev_num, u32 iface, u32 subphy, u32 pad)
{
	enum hws_training_ip_stat train_result[MAX_INTERFACE_NUM];
	struct hws_topology_map *tm = ddr3_get_topology_map();
	u32 i, a, b, diff, max_diff, max_diff_cnt, dq;

	/*
	 * Note: HWS_LOW2HIGH direction didn't work because of asymmetry
	 * between tx windows (revealed by tap tuning function)
	 */
	enum hws_search_dir search_dir = HWS_HIGH2LOW;
	u8 prior_result[BUS_WIDTH_IN_BITS], post_result[BUS_WIDTH_IN_BITS];
	u32 *result[HWS_SEARCH_DIR_LIMIT];

	/* run training prior to any delay insertion */
	ddr3_tip_ip_training_wrapper(dev_num, ACCESS_TYPE_MULTICAST,
				     PARAM_NOT_CARE, ACCESS_TYPE_MULTICAST,
				     PARAM_NOT_CARE, RESULT_PER_BIT,
				     HWS_CONTROL_ELEMENT_ADLL,
				     PARAM_NOT_CARE, OPER_WRITE,
				     tm->if_act_mask, 0x0,
				     MAX_WINDOW_SIZE_TX - 1,
				     MAX_WINDOW_SIZE_TX - 1,
				     PATTERN_VREF, EDGE_FPF, CS_SINGLE,
				     PARAM_NOT_CARE, train_result);

	/* read training results */
	if (ddr3_tip_read_training_result(dev_num, iface,
					  ACCESS_TYPE_UNICAST, subphy,
					  ALL_BITS_PER_PUP, search_dir,
					  OPER_WRITE, RESULT_PER_BIT,
					  TRAINING_LOAD_OPERATION_UNLOAD,
					  CS_SINGLE, &result[search_dir],
					  1, 0, 0) != MV_OK)
		return MV_FAIL;

	/* save prior to delay insertion results */
	for (i = 0; i < BUS_WIDTH_IN_BITS; i++)
		prior_result[i] = result[search_dir][i] & 0xff;

	ddr3_hws_set_log_level(DEBUG_BLOCK_CENTRALIZATION, DEBUG_LEVEL_INFO);

#if MV_DDR_DQ_MAPPING_DETECT_VERBOSE == 1
	printf("MV_DDR: %s: Prior to DQ shift: if %d, subphy %d, pad %d,\n"
	       "\tregs: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
	       __func__, iface, subphy, pad,
	       prior_result[0], prior_result[1], prior_result[2], prior_result[3],
	       prior_result[4], prior_result[5], prior_result[6], prior_result[7]);
#endif

	/* insert delay to pad under test (max val is 0x1f) */
	ddr3_tip_bus_write(dev_num, ACCESS_TYPE_UNICAST,
			     iface, ACCESS_TYPE_UNICAST,
			     subphy, DDR_PHY_DATA,
			     PBS_TX_PHY_REG + pad, 0x1f);

	/* run training after delay insertion */
	ddr3_tip_ip_training_wrapper(dev_num, ACCESS_TYPE_MULTICAST,
				     PARAM_NOT_CARE, ACCESS_TYPE_MULTICAST,
				     PARAM_NOT_CARE, RESULT_PER_BIT,
				     HWS_CONTROL_ELEMENT_ADLL,
				     PARAM_NOT_CARE, OPER_WRITE,
				     tm->if_act_mask, 0x0,
				     MAX_WINDOW_SIZE_TX - 1,
				     MAX_WINDOW_SIZE_TX - 1,
				     PATTERN_VREF, EDGE_FPF, CS_SINGLE,
				     PARAM_NOT_CARE, train_result);

	/* read training results */
	if (ddr3_tip_read_training_result(dev_num, iface,
					  ACCESS_TYPE_UNICAST, subphy,
					  ALL_BITS_PER_PUP, search_dir,
					  OPER_WRITE, RESULT_PER_BIT,
					  TRAINING_LOAD_OPERATION_UNLOAD,
					  CS_SINGLE, &result[search_dir],
					  1, 0, 0) != MV_OK)
		return MV_FAIL;

	/* save post delay insertion results */
	for (i = 0; i < BUS_WIDTH_IN_BITS; i++)
		post_result[i] = result[search_dir][i] & 0xff;

#if MV_DDR_DQ_MAPPING_DETECT_VERBOSE == 1
	printf("MV_DDR: %s: After DQ shift: if %d, subphy %d, pad %d,\n"
	       "\tregs: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
	       __func__, iface, subphy, pad,
	       post_result[0], post_result[1], post_result[2], post_result[3],
	       post_result[4], post_result[5], post_result[6], post_result[7]);
#endif

	/* remove inserted to pad delay */
	ddr3_tip_bus_write(dev_num, ACCESS_TYPE_UNICAST,
			   iface, ACCESS_TYPE_UNICAST,
			   subphy, DDR_PHY_DATA,
			   PBS_TX_PHY_REG + pad, 0x0);

	/* find max diff and its occurrence num */
	max_diff = 0, max_diff_cnt = 0, dq = 0;
	for (i = 0; i < BUS_WIDTH_IN_BITS; i++) {
		a = prior_result[i];
		b = post_result[i];
		if (a > b)
			diff = a - b;
		else
			diff = 0; /* tx version */

		if (diff > max_diff) {
			max_diff = diff;
			dq = i;
			max_diff_cnt = 0;
		} else if (diff == max_diff) {
			max_diff_cnt++;
		}
	}
#if MV_DDR_DQ_MAPPING_DETECT_VERBOSE == 1
	printf("MV_DDR: %s: if %d, subphy %d, pad %d, max diff = %d, max diff count = %d, dq = %d\n",
	       __func__, iface, subphy, pad, max_diff, max_diff_cnt, dq);
#endif

	/* check for pad to dq pairing criteria */
	if (max_diff > 2 && max_diff_cnt == 0)
#if MV_DDR_DQ_MAPPING_DETECT_VERBOSE == 1
		printf("MV_DDR: %s: if %d, subphy %d, DQ[%d] = PAD[%d]\n",
		       __func__, iface, subphy, dq, pad);
#else
		;
#endif
	else
		dq = 0xff;

	return dq;
}

#define MV_DDR_DQ_MAPPING_DETECT_NTRIES 5

int mv_ddr_dq_mapping_detect(u32 dev_num)
{
	u32 iface, subphy, pad, dq_detected;
	int ntries;
	u32 cs_enable_reg_val[MAX_INTERFACE_NUM];
	u32 octets_per_if_num = ddr3_tip_dev_attr_get(dev_num, MV_ATTR_OCTET_PER_INTERFACE);
	u32 mv_ddr_dq_mapping_detected[MAX_INTERFACE_NUM][MAX_BUS_NUM][BUS_WIDTH_IN_BITS] = {0};
	struct hws_topology_map *tm = ddr3_get_topology_map();

	for (iface = 0; iface < MAX_INTERFACE_NUM; iface++) {
		VALIDATE_IF_ACTIVE(tm->if_act_mask, iface);
		/* save current cs enable reg val */
		ddr3_tip_if_read(dev_num, ACCESS_TYPE_UNICAST,
				 iface, CS_ENABLE_REG,
				 cs_enable_reg_val,
				 MASK_ALL_BITS);
		/* enable single cs */
		ddr3_tip_if_write(dev_num, ACCESS_TYPE_UNICAST,
				  iface, CS_ENABLE_REG,
				  (1 << 3), (1 << 3));

	}

	for (iface = 0; iface < MAX_INTERFACE_NUM; iface++) {
		VALIDATE_IF_ACTIVE(tm->if_act_mask, iface);
		for (subphy = 0; subphy < octets_per_if_num; subphy++) {
			VALIDATE_BUS_ACTIVE(tm->bus_act_mask, subphy);
			for (pad = 0; pad < 11; pad++) {
				ntries = MV_DDR_DQ_MAPPING_DETECT_NTRIES;
				/*
				 * TODO: This part is platform-dependent.
				 * For APN806 platform: pad 3 is DM, pad 4 & 5 are DQS ones.
				 */
				if (pad == 3 || pad == 4 || pad == 5)
					continue;
				do {
					dq_detected = mv_ddr_pad_to_dq_detect(dev_num, iface, subphy, pad);
					ntries--;
				} while (dq_detected == 0xff && ntries > 0);

				if (dq_detected == 0xff)
					printf("MV_DDR: %s: Error: if %d, subphy %d, DQ for PAD[%d] not found after %d tries!\n",
					       __func__, iface, subphy, pad, MV_DDR_DQ_MAPPING_DETECT_NTRIES - ntries);
				else
					mv_ddr_dq_mapping_detected[iface][subphy][dq_detected] = pad;
			}
		}
	}

	/* restore cs enable value */
	for (iface = 0; iface < MAX_INTERFACE_NUM; iface++) {
		VALIDATE_IF_ACTIVE(tm->if_act_mask, iface);
		ddr3_tip_if_write(dev_num, ACCESS_TYPE_UNICAST,
				  iface, CS_ENABLE_REG,
				  cs_enable_reg_val[iface],
				  MASK_ALL_BITS);
	}

	printf("MV_DDR: %s: dq to pad mapping detection results:\n", __func__);
	for (iface = 0; iface < MAX_INTERFACE_NUM; iface++) {
		VALIDATE_IF_ACTIVE(tm->if_act_mask, iface);
		printf("if/subphy:\tdq0\tdq1\tdq2\tdq3\tdq4\tdq5\tdq6\tdq7\n");
		for (subphy = 0; subphy < octets_per_if_num; subphy++) {
			VALIDATE_BUS_ACTIVE(tm->bus_act_mask, subphy);
			printf("%d/%d:\t\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", iface, subphy,
			       mv_ddr_dq_mapping_detected[iface][subphy][0],
			       mv_ddr_dq_mapping_detected[iface][subphy][1],
			       mv_ddr_dq_mapping_detected[iface][subphy][2],
			       mv_ddr_dq_mapping_detected[iface][subphy][3],
			       mv_ddr_dq_mapping_detected[iface][subphy][4],
			       mv_ddr_dq_mapping_detected[iface][subphy][5],
			       mv_ddr_dq_mapping_detected[iface][subphy][6],
			       mv_ddr_dq_mapping_detected[iface][subphy][7]);
		}
	}

	return MV_OK;
}
#endif
