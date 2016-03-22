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

#define DDR_INTERFACES_NUM		1
#define DDR_INTERFACE_OCTETS_NUM	5

#if 0 /* TODO */
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
#endif


static u32 dq_bit_map_2_phy_pin[] = {
	1, 0, 2, 6, 9, 8, 3, 7,	/* 0 */
	8, 9, 1, 7, 2, 6, 3, 0,	/* 1 */
	3, 9, 7, 8, 1, 0, 2, 6,	/* 2 */
	1, 0, 6, 2, 8, 3, 7, 9,	/* 3 */
	0, 1, 2, 9, 7, 8, 3, 6,	/* 4 */
};

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
	reg_addr += TIP_BASE_ADDRESS;
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

	reg_addr += TIP_BASE_ADDRESS;

	if (mask != MASK_ALL_BITS) {
		CHECK_STATUS(ddr3_tip_apn806_if_read
			     (dev_num, ACCESS_TYPE_UNICAST, if_id, reg_addr,
			      &ui_data_read, MASK_ALL_BITS));
		data_value = (ui_data_read & (~mask)) | (data_value & mask);
	}

	reg_write(reg_addr, data_value);

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
		reg |= (1 << 6);
	else
		reg &= ~(1 << 6);

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
	enum hws_ddr_freq ddr_freq = DDR_FREQ_LOW_FREQ; /* FIXME */
	struct hws_topology_map *tm = ddr3_get_topology_map();

	/* new read leveling version */
	config_func.tip_dunit_read_func = ddr3_tip_apn806_if_read;
	config_func.tip_dunit_write_func = ddr3_tip_apn806_if_write;
	config_func.tip_dunit_mux_select_func =
		ddr3_tip_apn806_select_ddr_controller;
	config_func.tip_get_freq_config_info_func = NULL;
	config_func.tip_set_freq_divider_func = NULL;
	config_func.tip_get_device_info_func = NULL;
	config_func.tip_get_temperature = NULL;
	config_func.tip_get_clock_ratio = NULL;
	config_func.tip_external_read = ddr3_tip_ext_read;
	config_func.tip_external_write = ddr3_tip_ext_write;

	ddr3_tip_init_config_func(dev_num, &config_func);

	ddr3_tip_register_dq_table(dev_num, dq_bit_map_2_phy_pin);

	/* set device attributes*/
	ddr3_tip_dev_attr_init(dev_num);
	ddr3_tip_dev_attr_set(dev_num, MV_ATTR_TIP_REV, MV_TIP_REV_4);
	ddr3_tip_dev_attr_set(dev_num, MV_ATTR_PHY_EDGE, MV_DDR_PHY_EDGE_POSITIVE);
	ddr3_tip_dev_attr_set(dev_num, MV_ATTR_OCTET_PER_INTERFACE, DDR_INTERFACE_OCTETS_NUM);
#ifdef CONFIG_ARMADA_39X
	ddr3_tip_dev_attr_set(dev_num, MV_ATTR_INTERLEAVE_WA, 1);
#else
	ddr3_tip_dev_attr_set(dev_num, MV_ATTR_INTERLEAVE_WA, 0);
#endif

#if defined(CONFIG_DDR4)
	mask_tune_func = (SET_LOW_FREQ_MASK_BIT |
			  LOAD_PATTERN_MASK_BIT |
			  SET_TARGET_FREQ_MASK_BIT |
			  WRITE_LEVELING_TF_MASK_BIT |
			  READ_LEVELING_TF_MASK_BIT |
			  RECEIVER_CALIBRATION_MASK_BIT |
			  WL_PHASE_CORRECTION_MASK_BIT |
			  DQ_VREF_CALIBRATION_MASK_BIT |
			  DQ_MAPPING_MASK_BIT);
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

	printf("MV_DDR::Hello from %s\n", __func__);

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
	printf("MV_DDR::Hello from %s\n", __func__);

	return MV_OK;
}

int mv_ddr_pre_training_soc_config(const char *ddr_type)
{
	printf("MV_DDR::Hello from %s\n", __func__);

	return MV_OK;
}

int mv_ddr_post_training_soc_config(const char *ddr_type)
{
	printf("MV_DDR::Hello from %s\n", __func__);

	return MV_OK;
}
