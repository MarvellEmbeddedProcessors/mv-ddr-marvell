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

static u32 bist_offset = 32;
enum hws_pattern sweep_pattern = PATTERN_KILLER_DQ0;

static int ddr3_tip_bist_operation(u32 dev_num,
				   enum hws_access_type access_type,
				   u32 if_id,
				   enum hws_bist_operation oper_type);

/*
 * BIST activate
 */
int ddr3_tip_bist_activate(u32 dev_num, enum hws_pattern pattern,
			   enum hws_access_type access_type, u32 if_num,
			   enum hws_dir dir,
			   enum hws_stress_jump addr_stress_jump,
			   enum hws_pattern_duration duration,
			   enum hws_bist_operation oper_type,
			   u32 offset, u32 cs_num, u32 pattern_addr_length)
{
	u32 tx_burst_size;
	u32 delay_between_burst;
	u32 rd_mode;
	struct pattern_info *pattern_table = ddr3_tip_get_pattern_table();

	/* odpg bist write enable */
	ddr3_tip_if_write(0, access_type, 0, ODPG_DATA_CTRL_REG,
			  (ODPG_WRBUF_WR_CTRL_ENA << ODPG_WRBUF_WR_CTRL_OFFS),
			  (ODPG_WRBUF_WR_CTRL_MASK << ODPG_WRBUF_WR_CTRL_OFFS));

	/* odpg bist read enable/disable */
	ddr3_tip_if_write(0, access_type, 0, ODPG_DATA_CTRL_REG,
			  (dir == OPER_READ) ? (ODPG_WRBUF_RD_CTRL_ENA << ODPG_WRBUF_RD_CTRL_OFFS) :
					       (ODPG_WRBUF_RD_CTRL_DIS << ODPG_WRBUF_RD_CTRL_OFFS),
			  (ODPG_WRBUF_RD_CTRL_MASK << ODPG_WRBUF_RD_CTRL_OFFS));

	ddr3_tip_load_pattern_to_odpg(0, access_type, 0, pattern, offset);

	ddr3_tip_if_write(0, access_type, 0, ODPG_DATA_BUFFER_SIZE_REG, pattern_addr_length, MASK_ALL_BITS);
	tx_burst_size = (dir == OPER_WRITE) ?
		pattern_table[pattern].tx_burst_size : 0;
	delay_between_burst = (dir == OPER_WRITE) ? 2 : 0;
	rd_mode = (dir == OPER_WRITE) ? 1 : 0;
	ddr3_tip_configure_odpg(0, access_type, 0, dir,
		      pattern_table[pattern].num_of_phases_tx, tx_burst_size,
		      pattern_table[pattern].num_of_phases_rx,
		      delay_between_burst,
		      rd_mode, cs_num, addr_stress_jump, duration);
	ddr3_tip_if_write(0, access_type, 0, ODPG_DATA_BUFFER_OFFS_REG, offset, MASK_ALL_BITS);

	if (oper_type == BIST_STOP) {
		ddr3_tip_bist_operation(0, access_type, 0, BIST_STOP);
	} else {
		ddr3_tip_bist_operation(0, access_type, 0, BIST_START);
		if (mv_ddr_is_odpg_done(MAX_POLLING_ITERATIONS) != MV_OK)
			return MV_FAIL;
		ddr3_tip_bist_operation(0, access_type, 0, BIST_STOP);
	}
	ddr3_tip_if_write(0, access_type, 0, ODPG_DATA_CTRL_REG, 0, MASK_ALL_BITS);

	return MV_OK;
}

/*
 * BIST read result
 */
int ddr3_tip_bist_read_result(u32 dev_num, u32 if_id,
			      struct bist_result *pst_bist_result)
{
	int ret;
	u32 read_data[MAX_INTERFACE_NUM];
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();

	if (IS_IF_ACTIVE(tm->if_act_mask, if_id) == 0)
		return MV_NOT_SUPPORTED;
	DEBUG_TRAINING_BIST_ENGINE(DEBUG_LEVEL_TRACE,
				   ("ddr3_tip_bist_read_result if_id %d\n",
				    if_id));
	ret = ddr3_tip_if_read(dev_num, ACCESS_TYPE_UNICAST, if_id,
			       ODPG_DATA_RX_WORD_ERR_DATA_HIGH_REG, read_data,
			       MASK_ALL_BITS);
	if (ret != MV_OK)
		return ret;
	pst_bist_result->bist_fail_high = read_data[if_id];
	ret = ddr3_tip_if_read(dev_num, ACCESS_TYPE_UNICAST, if_id,
			       ODPG_DATA_RX_WORD_ERR_DATA_LOW_REG, read_data,
			       MASK_ALL_BITS);
	if (ret != MV_OK)
		return ret;
	pst_bist_result->bist_fail_low = read_data[if_id];

	ret = ddr3_tip_if_read(dev_num, ACCESS_TYPE_UNICAST, if_id,
			       ODPG_DATA_RX_WORD_ERR_ADDR_REG, read_data,
			       MASK_ALL_BITS);
	if (ret != MV_OK)
		return ret;
	pst_bist_result->bist_last_fail_addr = read_data[if_id];
	ret = ddr3_tip_if_read(dev_num, ACCESS_TYPE_UNICAST, if_id,
			       ODPG_DATA_RX_WORD_ERR_CNTR_REG, read_data,
			       MASK_ALL_BITS);
	if (ret != MV_OK)
		return ret;
	pst_bist_result->bist_error_cnt = read_data[if_id];

	return MV_OK;
}

/*
 * BIST flow - Activate & read result
 */
int hws_ddr3_run_bist(u32 dev_num, enum hws_pattern pattern, u32 *result,
		      u32 cs_num)
{
	int ret;
	u32 i = 0;
	u32 win_base;
	struct bist_result st_bist_result;
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();

	for (i = 0; i < MAX_INTERFACE_NUM; i++) {
		VALIDATE_IF_ACTIVE(tm->if_act_mask, i);
		hws_ddr3_cs_base_adr_calc(i, cs_num, &win_base);
		ret = ddr3_tip_bist_activate(dev_num, pattern,
					     ACCESS_TYPE_UNICAST,
					     i, OPER_WRITE, STRESS_NONE,
					     DURATION_SINGLE, BIST_START,
					     bist_offset + win_base,
					     cs_num, 15);
		if (ret != MV_OK) {
			printf("ddr3_tip_bist_activate failed (0x%x)\n", ret);
			return ret;
		}

		ret = ddr3_tip_bist_activate(dev_num, pattern,
					     ACCESS_TYPE_UNICAST,
					     i, OPER_READ, STRESS_NONE,
					     DURATION_SINGLE, BIST_START,
					     bist_offset + win_base,
					     cs_num, 15);
		if (ret != MV_OK) {
			printf("ddr3_tip_bist_activate failed (0x%x)\n", ret);
			return ret;
		}

		ret = ddr3_tip_bist_read_result(dev_num, i, &st_bist_result);
		if (ret != MV_OK) {
			printf("ddr3_tip_bist_read_result failed\n");
			return ret;
		}
		result[i] = st_bist_result.bist_error_cnt;
	}

	return MV_OK;
}

/*
 * Set BIST Operation
 */

static int ddr3_tip_bist_operation(u32 dev_num,
				   enum hws_access_type access_type,
				   u32 if_id, enum hws_bist_operation oper_type)
{
	if (oper_type == BIST_STOP)
		mv_ddr_odpg_disable();
	else
		mv_ddr_odpg_enable();

	return MV_OK;
}

/*
 * Print BIST result
 */
void ddr3_tip_print_bist_res(void)
{
	u32 dev_num = 0;
	u32 i;
	struct bist_result st_bist_result[MAX_INTERFACE_NUM];
	int res;
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();

	for (i = 0; i < MAX_INTERFACE_NUM; i++) {
		VALIDATE_IF_ACTIVE(tm->if_act_mask, i);

		res = ddr3_tip_bist_read_result(dev_num, i, &st_bist_result[i]);
		if (res != MV_OK) {
			DEBUG_TRAINING_BIST_ENGINE(
				DEBUG_LEVEL_ERROR,
				("ddr3_tip_bist_read_result failed\n"));
			return;
		}
	}

	DEBUG_TRAINING_BIST_ENGINE(
		DEBUG_LEVEL_INFO,
		("interface | error_cnt | fail_low | fail_high | fail_addr\n"));

	for (i = 0; i < MAX_INTERFACE_NUM; i++) {
		VALIDATE_IF_ACTIVE(tm->if_act_mask, i);

		DEBUG_TRAINING_BIST_ENGINE(
			DEBUG_LEVEL_INFO,
			("%d |  0x%08x  |  0x%08x  |  0x%08x  | 0x%08x\n",
			 i, st_bist_result[i].bist_error_cnt,
			 st_bist_result[i].bist_fail_low,
			 st_bist_result[i].bist_fail_high,
			 st_bist_result[i].bist_last_fail_addr));
	}
}

enum {
	PASS,
	FAIL
};
/* TODO: to be defined as static when in use by another function */
#define TIP_ITERATION_NUM	31
int mv_ddr_tip_bist(enum hws_dir dir, u32 val, enum hws_pattern pattern, u32 cs, u32 *result)
{
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	enum hws_training_ip_stat training_result;
	u16 *reg_map = ddr3_tip_get_mask_results_pup_reg_map();
	u32 max_subphy = ddr3_tip_dev_attr_get(0, MV_ATTR_OCTET_PER_INTERFACE);
	u32 subphy, read_data;

	ddr3_tip_ip_training(0, ACCESS_TYPE_MULTICAST, 0, ACCESS_TYPE_MULTICAST, PARAM_NOT_CARE,
			     RESULT_PER_BYTE, HWS_CONTROL_ELEMENT_ADLL, HWS_LOW2HIGH, dir, tm->if_act_mask, val,
			     TIP_ITERATION_NUM, pattern, EDGE_FP, CS_SINGLE, cs, &training_result);

	for (subphy = 0; subphy < max_subphy; subphy++) {
		ddr3_tip_if_read(0, ACCESS_TYPE_UNICAST, 0, reg_map[subphy], &read_data, MASK_ALL_BITS);
		if (((read_data >> BLOCK_STATUS_OFFS) & BLOCK_STATUS_MASK) == BLOCK_STATUS_NOT_LOCKED)
			*result |= (FAIL << subphy);
	}

	return MV_OK;
}
