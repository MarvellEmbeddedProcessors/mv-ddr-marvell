/*
* ***************************************************************************
* Copyright (C) 2017 Marvell International Ltd.
* ***************************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* Neither the name of Marvell nor the names of its contributors may be used
* to endorse or promote products derived from this software without specific
* prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
***************************************************************************
*/

#include <stdio.h>
#include "ddr3_init.h"

#define DDR_INTERFACE_OCTETS_NUM	9

/*
 * Accessor functions for the registers
 */
void reg_write(u32 addr, u32 val)
{
}

u32 reg_read(u32 addr)
{
	return 0;
}

void reg_bit_set(u32 addr, u32 mask)
{
}

void reg_bit_clr(u32 addr, u32 mask)
{
}

void reg_bit_clrset(u32 addr, u32 val, u32 mask)
{
	ddr_reg_write(INTER_REGS_BASE + addr, val);
}

void mmio_write2_32(u32 val, u32 addr)
{
}

/*
 * Calculate number of CS
 */
int calc_cs_num(u32 dev_num, u32 if_id, u32 *cs_num)
{
	u32 cs;
	u32 bus_cnt;
	u32 cs_count;
	u32 cs_bitmask;
	u32 curr_cs_num = 0;
	u32 octets_per_if_num = ddr3_tip_dev_attr_get(dev_num, MV_ATTR_OCTET_PER_INTERFACE);
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();

	for (bus_cnt = 0; bus_cnt < octets_per_if_num; bus_cnt++) {
		VALIDATE_BUS_ACTIVE(tm->bus_act_mask, bus_cnt);
		cs_count = 0;
		cs_bitmask = tm->interface_params[if_id].
			as_bus_params[bus_cnt].cs_bitmask;
		for (cs = 0; cs < MAX_CS_NUM; cs++) {
			if ((cs_bitmask >> cs) & 1)
				cs_count++;
		}

		if (curr_cs_num == 0) {
			curr_cs_num = cs_count;
		} else if (cs_count != curr_cs_num) {
			DEBUG_TRAINING_IP(DEBUG_LEVEL_ERROR,
					  ("CS number is different per bus (IF %d BUS %d cs_num %d curr_cs_num %d)\n",
					   if_id, bus_cnt, cs_count,
					   curr_cs_num));
			return MV_NOT_SUPPORTED;
		}
	}
	*cs_num = curr_cs_num;

	return MV_OK;
}

/*
 * Translates topology map definitions to real memory size in bits
  * (per values in ddr3_training_ip_def.h)
 */
u32 mem_size[] = {
	ADDR_SIZE_512MB,
	ADDR_SIZE_1GB,
	ADDR_SIZE_2GB,
	ADDR_SIZE_4GB,
	ADDR_SIZE_8GB
};

uint64_t mv_ddr_mem_sz_per_cs_in_bits_get(void)
{
	uint64_t memory_size_per_cs;

	u32 bus_cnt, num_of_active_bus = 0;
	u32 num_of_sub_phys_per_ddr_unit = 0;

	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();

	u32 octets_per_if_num = ddr3_tip_dev_attr_get(0, MV_ATTR_OCTET_PER_INTERFACE);

#if 0
	/* count the number of active bus */
	for (bus_cnt = 0; bus_cnt < octets_per_if_num - 1/* ignore ecc octet */; bus_cnt++) {
		VALIDATE_BUS_ACTIVE(tm->bus_act_mask, bus_cnt);
		num_of_active_bus++;
	}
#endif
	for (bus_cnt = 0; bus_cnt < octets_per_if_num - 1/* ignore ecc octet */; bus_cnt++) {
		if ((((tm->bus_act_mask) >> (bus_cnt)) & 1) == 0)
			continue;
		num_of_active_bus++;
	}

	/* calculate number of sub-phys per ddr unit */
	if (tm->interface_params[0].bus_width/* supports only single interface */ == MV_DDR_DEV_WIDTH_16BIT)
		num_of_sub_phys_per_ddr_unit = MV_DDR_TWO_SPHY_PER_DUNIT;
	if (tm->interface_params[0].bus_width/* supports only single interface */ == MV_DDR_DEV_WIDTH_8BIT)
		num_of_sub_phys_per_ddr_unit = MV_DDR_ONE_SPHY_PER_DUNIT;

	/* calculate dram size per cs */
	memory_size_per_cs = (uint64_t)mem_size[tm->interface_params[0].memory_size] * (uint64_t)num_of_active_bus
		/ (uint64_t)num_of_sub_phys_per_ddr_unit * (uint64_t)MV_DDR_NUM_BITS_IN_BYTE;

	return memory_size_per_cs;
}


int mv_ddr_sw_db_init(u32 dev_num, u32 board_id)
{
	/* set device attributes*/
	ddr3_tip_dev_attr_init(dev_num);
	ddr3_tip_dev_attr_set(dev_num, MV_ATTR_TIP_REV, MV_TIP_REV_4);
	ddr3_tip_dev_attr_set(dev_num, MV_ATTR_PHY_EDGE, MV_DDR_PHY_EDGE_NEGATIVE);
	ddr3_tip_dev_attr_set(dev_num, MV_ATTR_OCTET_PER_INTERFACE, DDR_INTERFACE_OCTETS_NUM);
	ddr3_tip_dev_attr_set(dev_num, MV_ATTR_INTERLEAVE_WA, 0);

	return MV_OK;
}


static void mv_ddr_topology_map_set(struct mv_ddr_topology_map *map)
{
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	/* Return the board topology as defined in the board code */
	memcpy(tm, map, sizeof(struct mv_ddr_topology_map));
}

void ddr_controller_init(struct mv_ddr_topology_map *map)
{
	mv_ddr_topology_map_set(map);

	mv_ddr_sw_db_init(0, 0);

	mv_ddr_mc6_and_dram_timing_set();

	mv_ddr_mc6_sizes_cfg();
}
