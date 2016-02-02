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

#ifndef _DDR_TOPOLOGY_DEF_H
#define _DDR_TOPOLOGY_DEF_H

#include "ddr3_training_ip_def.h"
#include "ddr3_topology_def.h"

#if defined(CONFIG_ARMADA_38X)
#include "ddr3_a38x.h"
#endif

/* bus width in bits */
enum hws_bus_width {
	BUS_WIDTH_4,
	BUS_WIDTH_8,
	BUS_WIDTH_16,
	BUS_WIDTH_32
};

enum hws_temperature {
	HWS_TEMP_LOW,
	HWS_TEMP_NORMAL,
	HWS_TEMP_HIGH
};

enum hws_mem_size {
	MEM_512M,
	MEM_1G,
	MEM_2G,
	MEM_4G,
	MEM_8G,
	MEM_SIZE_LAST
};

struct bus_params {
	/* Chip Select (CS) bitmask (bits 0-CS0, bit 1- CS1 ...) */
	u8 cs_bitmask;

	/*
	 * mirror enable/disable
	 * (bits 0-CS0 mirroring, bit 1- CS1 mirroring ...)
	 */
	int mirror_enable_bitmask;

	/* DQS Swap (polarity) - true if enable */
	int is_dqs_swap;

	/* CK swap (polarity) - true if enable */
	int is_ck_swap;
};

struct if_params {
	/* bus configuration */
	struct bus_params as_bus_params[MAX_BUS_NUM];

	/* Speed Bin Table */
	enum hws_speed_bin speed_bin_index;

	/* bus width of memory */
	enum hws_bus_width bus_width;

	/* Bus memory size (MBit) */
	enum hws_mem_size memory_size;

	/* The DDR frequency for each interfaces */
	enum hws_ddr_freq memory_freq;

	/*
	 * delay CAS Write Latency
	 * - 0 for using default value (jedec suggested)
	 */
	u8 cas_wl;

	/*
	 * delay CAS Latency
	 * - 0 for using default value (jedec suggested)
	 */
	u8 cas_l;

	/* operation temperature */
	enum hws_temperature interface_temp;
};

struct hws_topology_map {
	/* Number of interfaces (default is 12) */
	u8 if_act_mask;

	/* Controller configuration per interface */
	struct if_params interface_params[MAX_INTERFACE_NUM];

	/* BUS per interface (default is 4) */
	u8 num_of_bus_per_interface;

	/* Bit mask for active buses */
	u8 bus_act_mask;
};

/* DDR3 training global configuration parameters */
struct tune_train_params {
	u32 ck_delay;
	u32 ck_delay_16;
	u32 p_finger;
	u32 n_finger;
	u32 phy_reg3_val;
};

#endif /* _DDR_TOPOLOGY_DEF_H */
