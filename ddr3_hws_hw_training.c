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

#define REG_READ_DATA_SAMPLE_DELAYS_ADDR	0x1538
#define REG_READ_DATA_SAMPLE_DELAYS_MASK	0x1f
#define REG_READ_DATA_SAMPLE_DELAYS_OFFS	8

#define REG_READ_DATA_READY_DELAYS_ADDR		0x153c
#define REG_READ_DATA_READY_DELAYS_MASK		0x1f
#define REG_READ_DATA_READY_DELAYS_OFFS		8

int ddr3_if_ecc_enabled(void)
{
	struct hws_topology_map *tm = ddr3_get_topology_map();

	if (DDR3_IS_ECC_PUP4_MODE(tm->bus_act_mask) ||
	    DDR3_IS_ECC_PUP3_MODE(tm->bus_act_mask))
		return 1;
	else
		return 0;
}

int ddr3_pre_algo_config(void)
{
	struct hws_topology_map *tm = ddr3_get_topology_map();

	/* Set Bus3 ECC training mode */
	if (DDR3_IS_ECC_PUP3_MODE(tm->bus_act_mask)) {
		/* Set Bus3 ECC MUX */
		CHECK_STATUS(ddr3_tip_if_write
			     (0, ACCESS_TYPE_UNICAST, PARAM_NOT_CARE,
			      REG_SDRAM_PINS_MUX, 0x100, 0x100));
	}

	/* Set regular ECC training mode (bus4 and bus 3) */
	if ((DDR3_IS_ECC_PUP4_MODE(tm->bus_act_mask)) ||
	    (DDR3_IS_ECC_PUP3_MODE(tm->bus_act_mask))) {
		/* Enable ECC Write MUX */
		CHECK_STATUS(ddr3_tip_if_write
			     (0, ACCESS_TYPE_UNICAST, PARAM_NOT_CARE,
			      TRAINING_SW_2_REG, 0x100, 0x100));
		/* General ECC enable */
		CHECK_STATUS(ddr3_tip_if_write
			     (0, ACCESS_TYPE_UNICAST, PARAM_NOT_CARE,
			      REG_SDRAM_CONFIG_ADDR, 0x40000, 0x40000));
		/* Disable Read Data ECC MUX */
		CHECK_STATUS(ddr3_tip_if_write
			     (0, ACCESS_TYPE_UNICAST, PARAM_NOT_CARE,
			      TRAINING_SW_2_REG, 0x0, 0x2));
	}

	return MV_OK;
}

int ddr3_post_algo_config(void)
{
	struct hws_topology_map *tm = ddr3_get_topology_map();
	int status;

	status = ddr3_post_run_alg();
	if (MV_OK != status) {
		printf("DDR3 Post Run Alg - FAILED 0x%x\n", status);
		return status;
	}

	/* Un_set ECC training mode */
	if ((DDR3_IS_ECC_PUP4_MODE(tm->bus_act_mask)) ||
	    (DDR3_IS_ECC_PUP3_MODE(tm->bus_act_mask))) {
		/* Disable ECC Write MUX */
		CHECK_STATUS(ddr3_tip_if_write
			     (0, ACCESS_TYPE_UNICAST, PARAM_NOT_CARE,
			      TRAINING_SW_2_REG, 0x0, 0x100));
		/* General ECC and Bus3 ECC MUX remains enabled */
	}

	return MV_OK;
}

int ddr3_hws_hw_training(enum hws_algo_type algo_mode)
{
	int status;

	status = ddr3_pre_algo_config();
	if (MV_OK != status) {
		printf("DDR3 Pre Algo Config - FAILED 0x%x\n", status);
		return status;
	}

	/* run algorithm in order to configure the PHY */
	status = hws_ddr3_tip_run_alg(0, algo_mode);
	if (MV_OK != status) {
		printf("DDR3 run algorithm - FAILED 0x%x\n", status);
		return status;
	}

	status = ddr3_post_algo_config();
	if (MV_OK != status) {
		printf("DDR3 Post Algo Config - FAILED 0x%x\n", status);
		return status;
	}

	return MV_OK;
}
