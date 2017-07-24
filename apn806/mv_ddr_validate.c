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

#include "ddr3_init.h"
#include "mv_ddr4_mpr_pda_if.h"
#include "mv_ddr_validate.h"

#define MAX_XOR_SRAM_DESC_SIZE	128
#define MAX_XOR_SRAM_DESC_NUM	1024
#define OSTD_RD_NUM		8

int hclk_cntr_lo = 0;
int hclk_cntr_hi = 0;
int print_log = 1;
int wr_xor_pat = 1;	/* 1: write */
int xor_src_to_dst = 1;	/* 1: do */
int xor_src_sram = 0;
int bank_map_conv[] = {	/* in KB */
	1,
	2,
	4,
	8,
	16,
	32,
	64,
	128,
	256,
	512,
	1024,
	2048,
	4096,
	8192,
	16384,
	32768
};
int area_len_conv[] = {	/* in MB */
	8,
	16,
	32,
	64,
	128,
	256,
	512,
	1024,
	2048,
	4096,
	8192,
	16384,
	32768
};
int bank_map = _1K;
unsigned long long area_len = 0;
int data_width = 4;	/* '4' is for 64-bit */
int page_size = _1K;
unsigned long long xor_byte_mask = 0x0;
enum mask_type mask = OR_MASK;
extern u8 dq_vref_vec[MAX_BUS_NUM];	/* stability support */
extern u8 rx_eye_hi_lvl[MAX_BUS_NUM];	/* vertical adjustment support */
extern u8 rx_eye_lo_lvl[MAX_BUS_NUM];	/* vertical adjustment support */

/* cast 32-bit to 64-bit int with padding remaining bits with zeros */
static unsigned long long lo_cast_32to64(int i)
{
	unsigned long long low;

	low = (unsigned long long)i;
	low &= 0x00000000ffffffff;

	return low;
}

static unsigned long long hi_cast_32to64(int i)
{
	unsigned long long high;

	high = (unsigned long long)i;
	high <<= 32;
	high &= 0xffffffff00000000;

	return high;
}

static void cpu_write(uintptr_t addr, uint64_t data)
{
	if (data_width == 4)
		mmio_write_64(addr, data);
	else
		mmio_write_32(addr, (u32)data);
}

static uint64_t cpu_read(uintptr_t addr)
{
	if (data_width == 4)
		return mmio_read_64(addr);
	else
		return (uint64_t)mmio_read_32(addr);
}


#define XOR_DESC_LINE_SIZE			0x4	/* in bytes */
#define XOR_DESC_MAX_BYTE_COUNT			0xffffff00
#define XOR_DESC_LINE_OFFSET(line_num)		((line_num) * XOR_DESC_LINE_SIZE)

#define XOR_ADDRESS_MAX_SIZE			0xffffffffffff
#define XOR_ADDRESS_MASK			XOR_ADDRESS_MAX_SIZE

#define XOR_DESC_STATUS_LINE			0
#define XOR_DESC_ID_OFFSET			0
#define XOR_DESC_ID_MASK			(0xffff << XOR_DESC_ID_OFFSET)
#define XOR_DESC_BCS_OFFSET			25
#define XOR_DESC_BCS_MASK			(0x1 << XOR_DESC_BCS_OFFSET)
#define XOR_DESC_FAILURE_CODE_OFFSET		26
#define XOR_DESC_FAILURE_CODE_MASK		(0x1f << XOR_DESC_FAILURE_CODE_OFFSET)
#define XOR_DESC_PRST_OFFSET			31
#define XOR_DESC_PRST_MASK			(0x1 << XOR_DESC_PRST_OFFSET)

#define XOR_DESC_CRC32_RESULT			1

#define XOR_DESC_COMMAND_LINE			2
#define XOR_DESC_OPERATION_MODE_OFFSET		28
#define XOR_DESC_OPERATION_MODE_MASK		(0xf << XOR_DESC_OPERATION_MODE_OFFSET)
#define XOR_DESC_AxATTR_SOURCE_OFFSET		21
#define XOR_DESC_AxATTR_SOURCE_MASK		(0x1 << XOR_DESC_AxATTR_SOURCE_OFFSET)
#define XOR_DESC_CRC_CMP_LAST_OFFSET		20
#define XOR_DESC_CRC_CMP_LAST_MASK		(0x1 << XOR_DESC_CRC_CMP_LAST_OFFSET)
#define XOR_DESC_CRC_CMP_FIRST_OFFSET		19
#define XOR_DESC_CRC_CMP_FIRST_MASKi		(0x1 << XOR_DESC_CRC_CMP_FIRST_OFFSET)

#define XOR_DESC_BYTE_COUNTER_LINE		3

#define XOR_DESC_SRC_BUF_ADDR_LOW_LINE		4

#define XOR_DESC_SRC_BUF_ADDR_HIGH_LINE		5
#define XOR_DESC_SRC_BUF_ADDR_HIGH_OFFSET	0
#define XOR_DESC_SRC_BUF_ADDR_HIGH_MASK		(0xffff << XOR_DESC_SRC_BUF_ADDR_HIGH_OFFSET)

#define XOR_DESC_DEST_BUFF_ADDR_LOW_LINE	6
#define XOR_DESC_SECOND_BUFF_ADDR_LOW_LINE	XOR_DESC_DEST_BUFF_ADDR_LOW_LINE

#define XOR_DESC_DEST_BUFF_ADDR_HIGH_LINE	7
#define XOR_DESC_SECOND_BUFF_ADDR_HIGH_LINE	XOR_DESC_DEST_BUFF_ADDR_HIGH_LINE
#define XOR_DESC_DEST_BUF_ADDR_HIGH_OFFSET	0
#define XOR_DESC_DEST_BUF_ADDR_HIGH_MASK	(0xffff << XOR_DESC_DEST_BUF_ADDR_HIGH_OFFSET)
#define XOR_DESC_SECOND_BUF_ADDR_HIGH_OFFSET	0
#define XOR_DESC_SECOND_BUF_ADDR_HIGH_MASK	(0xffff << XOR_DESC_DEST_BUF_ADDR_HIGH_OFFSET)

enum xor_op_mode {
	IDLE,
	PURE_DMA,
	MEM_FILL,
	MEM_INIT,
	MEM_CMP,
	CRC32,
	RAID5,
	RAID6,
	RAID6_RECOVERY
};

#define XOR_DESC_AXI_DOMAIN_OFFSET		0
#define XOR_DESC_AXI_DOMAIN_MASK		(0x3 << XOR_DESC_AXI_DOMAIN_OFFSET)
#define XOR_DESC_AXI_CACHE_OFFSET		2
#define XOR_DESC_AXI_CACHE_MASK			(0xf << XOR_DESC_AXI_CACHE_OFFSET)
#define CV_XOR_DESC_CACHE_BUFFERABLE_BIT	BIT0
#define CV_XOR_DESC_CACHE_MODIFIABLE_BIT	BIT1
#define CV_XOR_DESC_CACHE_OTHER_ALLOCATE_BIT	BIT2
#define CV_XOR_DESC_CACHE_ALLOCATE_BIT		BIT3

#define XOR_DESC_AXI_QOS_OFFSET			6
#define XOR_DESC_AXI_QOS_MASK			(0x3 << XOR_DESC_AXI_QOS_OFFSET)
#define XOR_DESC_AXI_PRIVILEGED_OFFSET		8
#define XOR_DESC_AXI_PRIVILEGED_MASK		(0x1<<XOR_DESC_AXI_PRIVILEGED_OFFSET)
#define XOR_DESC_AXI_PRIVILEGED_TYPE_OFFSET	9
#define XOR_DESC_AXI_PRIVILEGED_TYPE_MASK	(0x1 << XOR_DESC_AXI_PRIVILEGED_TYPE_OFFSET)
#define XOR_DESC_AXI_TRAFFIC_OFFSET		10
#define XOR_DESC_AXI_TRAFFIC_MASK		(0xf << XOR_DESC_AXI_TRAFFIC_OFFSET)

/* XOR_DESQ_BALR */
#define CV_XOR_REG_BASE(base)			((base) + 0x400000) /* ap */

#define CV_XOR_DESQ_BASE_ADDR_LOW_REG(base)	CV_XOR_REG_BASE(base)
#define CV_XOR_DESQ_BASE_ADDR_LOW_OFFSET	8
#define CV_XOR_DESQ_BASE_ADDR_LOW_MASK		(~0xff << CV_XOR_DESQ_BASE_ADDR_LOW_OFFSET)

/* XOR_DESQ_BAHR */
#define CV_XOR_DESQ_BASE_ADDR_HIGH_REG(base)	(CV_XOR_REG_BASE(base) + 0x4)
#define CV_XOR_DESQ_BASE_ADDR_HIGH_OFFSET	0
#define CV_XOR_DESQ_BASE_ADDR_HIGH_MASK		(0xffff << CV_XOR_DESQ_BASE_ADDR_HIGH_OFFSET)

/* XOR_DESQ_SIZE */
#define CV_XOR_DESQ_SIZE_REG(base)		(CV_XOR_REG_BASE(base) + 0x8)
#define CV_XOR_DESQ_SIZE_OFFSET			0
#define CV_XOR_DESQ_SIZE_MASK			(0x7fff << CV_XOR_DESQ_SIZE_OFFSET)

/* XOR_DESQ_DONE */
#define CV_XOR_DESQ_DONE_REG(base)		(CV_XOR_REG_BASE(base) + 0xc)
#define CV_XOR_DESQ_DONE_OFFSET			0
#define CV_XOR_DESQ_DONE_MASK			(0x7fff << CV_XOR_DESQ_DONE_OFFSET)
#define CV_XOR_DESQ_SW_READ_PTR_OFFSET		16
#define CV_XOR_DESQ_SW_READ_PTR_MASK		(0x7fff << CV_XOR_DESQ_SW_READ_PTR_OFFSET)

/* XOR_DESQ_ARATTR */
#define CV_XOR_DESQ_AR_ATTRIBUTES_REG(base)	(CV_XOR_REG_BASE(base) + 0x10)

/* XOR_DESQ_AWATTR */
#define CV_XOR_DESQ_AW_ATTRIBUTES_REG(base)	(CV_XOR_REG_BASE(base) + 0x1c)
#define CV_XOR_DESQ_DATA_SHARED_OFFSET		0
#define CV_XOR_DESQ_DATA_SHARED_MASK		(0x3 << CV_XOR_DESQ_DATA_SHARED_OFFSET)
#define CV_XOR_DESQ_DATA_CACHE_OFFSET		2
#define CV_XOR_DESQ_DATA_CACHE_MASK		(0xf << CV_XOR_DESQ_DATA_CACHE_OFFSET)
#define CV_XOR_DESQ_CACHE_BUFFERABLE_BIT	BIT0
#define CV_XOR_DESQ_CACHE_MODIFIABLE_BIT	BIT1
#define CV_XOR_DESQ_CACHE_ALLOCATE_BIT		BIT2
#define CV_XOR_DESQ_CACHE_OTHER_ALLOCATE_BIT	BIT3
#define CV_XOR_DESQ_DATA_QOS_ATTR_OFFSET	6
#define CV_XOR_DESQ_DATA_QOS_ATTR_MASK		(3 << CV_XOR_DESQ_DATA_QOS_ATTR_MASK)
#define CV_XOR_DESQ_DESC_SHARED_OFFSET		8
#define CV_XOR_DESQ_DESC_SHARED_MASK		(0x3 << CV_XOR_DESQ_DESC_SHARED_OFFSET)
#define CV_XOR_DESQ_DESC_CACHE_OFFSET		10
#define CV_XOR_DESQ_DESC_CACHE_MASK		(0xf << CV_XOR_DESQ_DESC_CACHE_OFFSET)
#define CV_XOR_DESQ_DESC_QOS_ATTR_OFFSET	14
#define CV_XOR_DESQ_DESC_QOS_ATTR_MASK		(3 << CV_XOR_DESQ_DESC_QOS_ATTR_OFFSET)
#define CV_XOR_DESQ_PRIVILEGED_BIT		BIT16
#define CV_XOR_DESQ_PRIVILEGED_TYPE_OFFSET	17
#define CV_XOR_DESQ_PRIVILEGED_TYPE_MASK	(0x1 << CV_XOR_DESQ_PRIVILEGED_TYPE_OFFSET)
#define CV_XOR_DESQ_DESC_TRAFIC_OFFSET		18
#define CV_XOR_DESQ_DESC_TRAFIC_MASK		(0xf << CV_XOR_DESQ_DESC_TRAFIC_OFFSET)
#define CV_XOR_DESQ_DATA_TRAFIC_OFFSET		22
#define CV_XOR_DESQ_DATA_TRAFIC_MASK		(0xf << CV_XOR_DESQ_DATA_TRAFIC_OFFSET)

/* XOR_IMSG_CDAT */
#define CV_XOR_IMSG_REG(base)			(CV_XOR_REG_BASE(base) + 0x14)
#define CV_XOR_IMG_VALUE_OFFSET			0
#define CV_XOR_IMG_VALUE_MASK			(0xffff << CV_XOR_IMG_VALUE_OFFSET)
#define CV_XOR_IMG_MASKED_DONE_BIT		BIT16
#define CV_XOR_IMG_MASKED_IOD_BIT		BIT17
#define CV_XOR_IMG_MASKED_TIMER_BIT		BIT18

/* XOR_IMSG_THRD */
#define CV_XOR_IMSG_THRESHOLD_REG(base)		(CV_XOR_REG_BASE(base) + 0x18)
#define CV_XOR_DONE_IMSG_THR_OFFSET		0
#define CV_XOR_DONE_IMSG_THR_MASK		(0x7f << CV_XOR_DONE_IMSG_THR_OFFSET)

/* XOR_DESQ_ALLOC */
#define CV_XOR_DESQ_ALLOCATED_REG(base)		(CV_XOR_REG_BASE(base) + 0x4c)
#define CV_XOR_DESQ_ALLOCATED_OFFSET		0
#define CV_XOR_DESQ_ALLOCATED_MASK		(0x7fff << CV_XOR_DESQ_ALLOCATED_OFFSET)
#define CV_XOR_DESQ_SW_WRITE_PTR_OFFSET		16
#define CV_XOR_DESQ_SW_WRITE_PTR_MASK		(0x7fff << CV_XOR_DESQ_SW_WRITE_PTR_OFFSET)

/* XOR_DESQ_CTRL */
#define CV_XOR_DESQ_CTRL_REG(base)		(CV_XOR_REG_BASE(base) + 0x100)
#define CV_XOR_ACT_DESC_SIZE_OFFSET		0
#define CV_XOR_ACT_DESC_SIZE_MASK		(0x7 << CV_XOR_ACT_DESC_SIZE_OFFSET)

/* XOR_DESQ_STOP */
#define CV_XOR_DESQ_STOP_REG(base)		(CV_XOR_REG_BASE(base) + 0x800)
#define CV_XOD_DESQ_DISABLE_BIT			BIT0
#define CV_XOD_DESQ_RESET_BIT			BIT1

/* XOR_DESQ_DEALLOC */
#define CV_XOR_DESQ_DEALLOC_REG(base)		(CV_XOR_REG_BASE(base) + 0x804)

/* XOR_DESQ_ADD */
#define CV_XOR_DESQ_ADD_REG(base)		(CV_XOR_REG_BASE(base) + 0x808)

/* XOR_HPTR_STTS */
#define CV_XOR_DESQ_HW_POINTERS_REG(base)	(CV_XOR_REG_BASE(base) + 0x848)
#define CV_XOR_DESQ_HW_READ_PTR_OFFSET		0
#define CV_XOR_DESQ_HW_READ_PTR_MASK		(0x7fff << CV_XOR_DESQ_SW_WRITE_READ_OFFSET)
#define CV_XOR_DESQ_HW_WRITE_PTR_OFFSET		16
#define CV_XOR_DESQ_HW_WRITE_PTR_MASK		(0x7fff << CV_XOR_DESQ_SW_WRITE_PTR_OFFSET)

/* XORG_BW_CTRL */
#define CV_XOR_BW_CTRL_REG(base)		(CV_XOR_REG_BASE(base) + 0x10004)
#define CV_XOR_NUM_OUTSTAND_READ_OFFSET		0
#define CV_XOR_NUM_OUTSTAND_READ_MASK		(0x7f << CV_XOR_NUM_OUTSTAND_READ_OFFSET)
#define CV_XOR_NUM_OUTSTAND_WRITE_OFFSET	8
#define CV_XOR_NUM_OUTSTAND_WRITE_MASK		(0xf << CV_XOR_NUM_OUTSTAND_WRITE_OFFSET)
#define CV_XOR_MUX_READ_BURST_OFFSET		12
#define CV_XOR_MUX_READ_BURST_MASK		(0x7 << CV_XOR_MUX_READ_BURST_OFFSET)
#define CV_XOR_MUX_WRITE_BURST_OFFSET		16
#define CV_XOR_MUX_WRITE_BURST_MASK		(0x7 << CV_XOR_MUX_WRITE_BURST_OFFSET)

/* XORG_SECURE */
#define CV_XOR_SECURE_REG(base)			(CV_XOR_REG_BASE(base) + 0x10300)
#define CV_XOR_SECURE_OFFSET			0
#define CV_XOR_SECURE_MASK			(0x1 << CV_XOR_SECURE_OFFSET)

/*
 * xor_memcpy function
 *
 * inputs:
 * desc_num:	number of descriptors
 * byte_cnt:	transfer size in bytes (per descriptor)
 * src_addr_lo:	low 32 bits of 64-bit source address
 * src_addr_hi:	high 16 bits of 64-bit source address
 * dst_addr_lo:	low 32 bits of 64-bit destination address
 * dst_addr_hi:	high 16 bits of 64-bit destination address
 * dst_gap:	gap between source and destnation; must be larger than byte_cnt
 * wr_burst_len: 0..7 (e.g., 4 is for 256B)
 * rd_burst_len: 0..7 (e.g., 4 is for 256B)
 *
 * flow:
 * - validate inputs
 * - write descriptors to memory
 * - configure the desc queue
 * - start the memory copy
 */
static void xor_memcpy(u32 desc_num, int byte_cnt,
		       u32 src_addr_lo, u32 src_addr_hi,
		       u32 dst_addr_lo, u32 dst_addr_hi, int dst_gap,
		       int wr_burst_len, int rd_burst_len,
		       u32 desq_addr_lo, u32 desq_addr_hi,
		       int ostd_rd_num)
{
	int transfer_desc[MAX_XOR_SRAM_DESC_NUM];
	int idx = 0, desc_idx = 0;
	int align_idx = 0;
	int desq_done_idx = 0;
	int i, paddr;
	u32 rd_reg = 0;
	uint64_t glob_reg_base = 0xf0000000;
	uint64_t value;
	uintptr_t addr_lo;
	unsigned long long src_addr = hi_cast_32to64(src_addr_hi) | lo_cast_32to64(src_addr_lo);
	unsigned long long desq_addr = hi_cast_32to64(desq_addr_hi) | lo_cast_32to64(desq_addr_lo);
	unsigned long long dst = hi_cast_32to64(dst_addr_hi) | lo_cast_32to64(dst_addr_lo);
	unsigned long long desc_num_th = (desc_num * 0x40 + desq_addr);
	unsigned long long desc_num_incr;

	for (align_idx = 0; align_idx < 64; align_idx++) {
		paddr = ((uintptr_t)(&transfer_desc[align_idx]) & (uintptr_t)0xff);
#ifdef DBG_PRINT
		printf("align idx = %d -> &transfer_desc[align_idx] = %p\n", align_idx, &transfer_desc[align_idx]);
#endif
		if (paddr == 0x0)
			break;
	}

#ifdef DBG_PRINT
	printf("%s: found align_idx = %d -> &transfer_desc[align_idx] = %p",
	       __func__, align_idx, &transfer_desc[align_idx]);
	printf("desc_num x byte_cnt = 0x%x\n", desc_num * byte_cnt);
#endif

	if (byte_cnt > 0x1000) {
		desc_num = (desc_num * byte_cnt) / (byte_cnt | 0x1000);
		byte_cnt = byte_cnt | 0x1000; /* a known issue; must enable bit 12 */
	}

	wr_burst_len &= 0x7;
	rd_burst_len &= 0x7;
	if ((desc_num_th > src_addr) && (desc_num > MAX_XOR_SRAM_DESC_SIZE)) { /* TODO: extend to >32-bit address */
#ifdef DBG_PRINT
		printf("%s: out of memory in desq for desc_num %d\n ", __func__, desc_num);
		printf("src_addr 0x%16llx\n", src_addr);
		printf("src_addr_lo 0x%08x\n", src_addr_lo);
		printf("src_addr_hi 0x%08x\n", src_addr_hi);
		printf("desc_num_th  = 0x%16llx\n", desc_num_th);
#endif
		desc_num = (u32)((src_addr - desq_addr) / 0x40) - 1;
#ifdef DBG_PRINT
		printf("desc_num changed to %d\n", desc_num);
#endif
	}

	if (desc_num > 0x7fff) {
#ifdef DBG_PRINT
		printf("%s: too many descriptors %d; changed to 0x7fff\n", __func__, desc_num);
#endif
		desc_num = 0x7fff;
	}

#ifdef DBG_PRINT
	printf("desc_num x byte_cnt = 0x%x\n", desc_num * byte_cnt);
	printf("%d * %d\n", desc_num, byte_cnt);
	printf("%s: 0x%08x -> (0x%08x + desc_num x 0x%x), byte_cnt 0x%x, desc_num %d\n",
	       __func__, src_addr_lo, dst_addr_lo, dst_gap, byte_cnt, desc_num);
#endif

	/* write descriptors to memory */
	for (idx = 0; idx < desc_num; idx++) {
		if (desc_num > MAX_XOR_SRAM_DESC_SIZE)
			desc_idx = 0;
		else
			desc_idx = 8 * idx + align_idx;

		/* init descriptors array */
		for (i = 0; i < 8; i++)
			transfer_desc[i + desc_idx] = 0;

		/* configure descriptors array */
		transfer_desc[XOR_DESC_COMMAND_LINE + desc_idx] = PURE_DMA << 28;
		transfer_desc[XOR_DESC_BYTE_COUNTER_LINE + desc_idx] = byte_cnt;
		transfer_desc[XOR_DESC_SRC_BUF_ADDR_LOW_LINE + desc_idx] = src_addr_lo;
		transfer_desc[XOR_DESC_SRC_BUF_ADDR_HIGH_LINE + desc_idx] = src_addr_hi & 0x00000ffff;
		dst += (unsigned long long)byte_cnt;
		dst_addr_lo = (u32)(dst & 0x00000000ffffffff);
		dst_addr_hi =  (u32)(dst >> 32);
		transfer_desc[XOR_DESC_DEST_BUFF_ADDR_LOW_LINE + desc_idx] = dst_addr_lo;
		transfer_desc[XOR_DESC_DEST_BUFF_ADDR_HIGH_LINE + desc_idx] = dst_addr_hi & 0xffff;

		/* write desrciptors to memory */
		desc_num_incr = 0x20 * idx;
		addr_lo = hi_cast_32to64(desq_addr_hi) + lo_cast_32to64(desq_addr_lo) + desc_num_incr;

#ifdef DBG_PRINT
		printf("%s: new desq_addr\n", __func__);
		printf("desq_addr_hi 0x%08x\n", desq_addr_hi);
		printf("desq_addr_lo %08x\n", desq_addr_lo);
		printf("src_addr_hi 0x%08x\n", src_addr_hi);
		printf("src_addr_lo %08x\n", src_addr_lo);
		printf("dst_addr_hi 0x%08x\n", dst_addr_hi);
		printf("dst_addr_lo %08x\n", dst_addr_lo);
		printf("byte_cnt 0x%08x\n", byte_cnt);
#endif

		if (desc_num > MAX_XOR_SRAM_DESC_SIZE) {
			for (i = 0; i < 4; i++) {
				value = hi_cast_32to64(transfer_desc[2 * i + 1 + desc_idx]) |
					lo_cast_32to64(transfer_desc[2 * i + desc_idx]);
				mmio_write_64(addr_lo, value);
				addr_lo += 8;
			}
		}
	}

	/* desq config */
	if (desc_num > MAX_XOR_SRAM_DESC_SIZE) {
		mmio_write_32(CV_XOR_DESQ_BASE_ADDR_LOW_REG(glob_reg_base), desq_addr);
		mmio_write_32(CV_XOR_DESQ_BASE_ADDR_HIGH_REG(glob_reg_base), desq_addr_hi);
	} else {
		/* enable secure mode in xor engine 0 xorg secure reg */
		mmio_write_32(CV_XOR_SECURE_REG(glob_reg_base), 0x0 << CV_XOR_SECURE_OFFSET);
		paddr = ((uintptr_t)&transfer_desc[align_idx]) & (uintptr_t)(0xffffffff);
		mmio_write_32(CV_XOR_DESQ_BASE_ADDR_LOW_REG(glob_reg_base), paddr);
		paddr = ((uintptr_t)(&transfer_desc[align_idx]) & ((uintptr_t)0xffffffff00000000)) >> 32;
		mmio_write_32(CV_XOR_DESQ_BASE_ADDR_HIGH_REG(glob_reg_base), paddr);
	}
	mmio_write_32(CV_XOR_DESQ_SIZE_REG(glob_reg_base), desc_num);
	mmio_write_32(CV_XOR_DESQ_AR_ATTRIBUTES_REG(glob_reg_base), 0x3e3e);
	mmio_write_32(CV_XOR_DESQ_AW_ATTRIBUTES_REG(glob_reg_base), 0x3e3e);
	mmio_write_32(CV_XOR_IMSG_THRESHOLD_REG(glob_reg_base), 0);
	mmio_write_32(CV_XOR_DESQ_CTRL_REG(glob_reg_base), 0);
	mmio_write_32(CV_XOR_DESQ_STOP_REG(glob_reg_base), 0);
	mmio_write_32(CV_XOR_BW_CTRL_REG(glob_reg_base),
		      ostd_rd_num |
		      (rd_burst_len << CV_XOR_MUX_READ_BURST_OFFSET) |
		      (wr_burst_len<<CV_XOR_MUX_WRITE_BURST_OFFSET));

	/* add desc to xor; start */
	mmio_write_32(CV_XOR_DESQ_ADD_REG(glob_reg_base), desc_num);

	while (desq_done_idx < desc_num) {
		rd_reg = mmio_read_32(CV_XOR_DESQ_DONE_REG(glob_reg_base));
		desq_done_idx = (CV_XOR_DESQ_DONE_MASK & rd_reg) >> CV_XOR_DESQ_DONE_OFFSET;
	}

	/* remove desc from xor */
	mmio_write_32(CV_XOR_DESQ_DEALLOC_REG(glob_reg_base), desc_num);

	/* disable secure mode in xor engine 0 xorg secure reg */
	if (desc_num <= MAX_XOR_SRAM_DESC_SIZE)
		mmio_write_32(CV_XOR_SECURE_REG(glob_reg_base), 0x1 << CV_XOR_SECURE_OFFSET);
}

/*
 * xor_memcmp function
 *
 * inputs:
 * desc_num:	number of descriptors
 * byte_cnt:	transfer size in bytes (per descriptor)
 * src_addr_lo:	low 32 bits of 64-bit source address
 * src_addr_hi:	high 16 bits of 64-bit source address
 * dst_addr_lo:	low 32 bits of 64-bit destination address
 * dst_addr_hi:	high 16 bits of 64-bit destination address
 * dst_gap:	gap between source and destnation; must be larger than byte_cnt
 * rd_burst_len: 0..7 (e.g., 4 is for 256B)
 *
 * flow:
 * - validate inputs
 * - write descriptors to memory
 * - configure the desc queue
 * - start the memory compare
 * - poll desc status and count failures
 */
static int xor_memcmp(u32 desc_num, int byte_cnt,
		      u32 src_addr_lo, u32 src_addr_hi,
		      u32 dst_addr_lo, u32 dst_addr_hi,
		      int dst_gap, int rd_burst_len,
		      u32 desq_addr_lo, u32 desq_addr_hi,
		      int ostd_rd_num)
{
	int transfer_desc[MAX_XOR_SRAM_DESC_NUM];
	int rd_reg = 0;
	int idx = 0, align_idx = 0, desc_idx = 0;
	int bcs_idx = 0;
	int i, paddr;
	int data = 0x0;
	int desq_done_idx = 0;
	int result = 0;
	uint64_t glob_reg_base = 0xf0000000;
	uint64_t value;
	uint64_t bitmask32;
	unsigned long long src_addr = hi_cast_32to64(src_addr_hi) | lo_cast_32to64(src_addr_lo);
	unsigned long long desq_addr = hi_cast_32to64(desq_addr_hi) | lo_cast_32to64(desq_addr_lo);
	unsigned long long dst = hi_cast_32to64(dst_addr_hi) | lo_cast_32to64(dst_addr_lo);
	unsigned long long desc_num_th = (desc_num * 0x40 + desq_addr);
	unsigned long long desc_num_incr;
	uintptr_t addr_lo;

	for (align_idx = 0; align_idx < 64; align_idx++) {
		paddr = ((uintptr_t)(&transfer_desc[align_idx]) & (uintptr_t)0xff);
#ifdef DBG_PRINT
		printf("align idx = %d -> &transfer_desc[align_idx] = %p\n", align_idx, &transfer_desc[align_idx]);
#endif
		if (paddr == 0x0)
			break;
	}

#ifdef DBG_PRINT
	printf("%s: found align_idx = %d -> &transfer_desc[align_idx] = %p",
	       __func__, align_idx, &transfer_desc[align_idx]);
#endif

	if (byte_cnt > 0x1000) {
		desc_num = (desc_num * byte_cnt) / (byte_cnt | 0x1000);
		byte_cnt = byte_cnt | 0x1000; /* a known issue; must enable bit 12 */
	}

	rd_burst_len &= 0x7;
	if ((desc_num_th > src_addr) && (desc_num > MAX_XOR_SRAM_DESC_SIZE)) { /* TODO: extend to >32-bit address */
		desc_num = (u32)((src_addr - desq_addr) / 0x40) - 1;
	}

	if (desc_num > 0x7fff)
		desc_num = 0x7fff;

#ifdef DBG_PRINT
	printf("%s: 0x%08x -> (0x%08x + desc_num x 0x%x), byte_cnt 0x%x, desc_num %d\n",
	       __func__, src_addr_lo, dst_addr_lo, dst_gap, byte_cnt, desc_num);
#endif
	/* write descriptors to memory */
	for (idx = 0; idx < desc_num; idx++) {
		if (desc_num > MAX_XOR_SRAM_DESC_SIZE)
			desc_idx = 0;
		else
			desc_idx = 8 * idx + align_idx;

		/* init descriptors array */
		for (i = 0; i < 8; i++)
			transfer_desc[i + desc_idx] = 0;

		/* configure descriptors array */
		transfer_desc[XOR_DESC_COMMAND_LINE + desc_idx] =
			((MEM_CMP << XOR_DESC_OPERATION_MODE_OFFSET) | (0x3 << 19));
		transfer_desc[XOR_DESC_BYTE_COUNTER_LINE + desc_idx] = byte_cnt;
		transfer_desc[XOR_DESC_SRC_BUF_ADDR_LOW_LINE + desc_idx] = src_addr_lo;
		transfer_desc[XOR_DESC_SRC_BUF_ADDR_HIGH_LINE + desc_idx] = src_addr_hi & 0x00000ffff;
		dst += (unsigned long long)byte_cnt;
		dst_addr_lo = (u32)(dst & 0x00000000ffffffff);
		dst_addr_hi =  (u32)(dst >> 32);
		transfer_desc[XOR_DESC_DEST_BUFF_ADDR_LOW_LINE + desc_idx] = dst_addr_lo;
		transfer_desc[XOR_DESC_DEST_BUFF_ADDR_HIGH_LINE + desc_idx] = dst_addr_hi & 0xffff;

		/* write descriptors to memory */
		desc_num_incr = 0x20 * idx;
		addr_lo = hi_cast_32to64(desq_addr_hi) + lo_cast_32to64(desq_addr_lo) + desc_num_incr;

		if (desc_num > MAX_XOR_SRAM_DESC_SIZE) {
			for (i = 0; i < 4; i++) {
				value = hi_cast_32to64(transfer_desc[2 * i + 1 + desc_idx]) |
					lo_cast_32to64(transfer_desc[2 * i + desc_idx]);
				mmio_write_64(addr_lo, value);
				addr_lo += 8;
			}
		}
	}

	/* desq config */
	if (desc_num > MAX_XOR_SRAM_DESC_SIZE) {
		mmio_write_32(CV_XOR_DESQ_BASE_ADDR_LOW_REG(glob_reg_base), desq_addr);
		mmio_write_32(CV_XOR_DESQ_BASE_ADDR_HIGH_REG(glob_reg_base), desq_addr_hi);
	} else {
		/* enable secure mode in xor engine 0 xorg secure reg */
		mmio_write_32(CV_XOR_SECURE_REG(glob_reg_base), 0x0 << CV_XOR_SECURE_OFFSET);
		paddr = ((uintptr_t)&transfer_desc[align_idx]) & (uintptr_t)(0xffffffff);
		mmio_write_32(CV_XOR_DESQ_BASE_ADDR_LOW_REG(glob_reg_base), paddr);
		paddr = ((uintptr_t)(&transfer_desc[align_idx])&((uintptr_t)0xffffffff00000000)) >> 32;
		mmio_write_32(CV_XOR_DESQ_BASE_ADDR_HIGH_REG(glob_reg_base), paddr);
	}
	mmio_write_32(CV_XOR_DESQ_SIZE_REG(glob_reg_base), desc_num);
	mmio_write_32(CV_XOR_DESQ_AR_ATTRIBUTES_REG(glob_reg_base), 0x3e3e);
	mmio_write_32(CV_XOR_DESQ_AW_ATTRIBUTES_REG(glob_reg_base), 0x3e3e);
	mmio_write_32(CV_XOR_IMSG_THRESHOLD_REG(glob_reg_base), 0);
	mmio_write_32(CV_XOR_DESQ_CTRL_REG(glob_reg_base), 0);
	mmio_write_32(CV_XOR_DESQ_STOP_REG(glob_reg_base), 0);
	mmio_write_32(CV_XOR_BW_CTRL_REG(glob_reg_base),
		      ostd_rd_num |
		      rd_burst_len << CV_XOR_MUX_READ_BURST_OFFSET);

	/* add desc to xor; start */
	mmio_write_32(CV_XOR_DESQ_ADD_REG(glob_reg_base), desc_num);

	idx = 0;
	desq_done_idx = 0;
	result = 0;
	bitmask32 = 0xffffffff;
	/* wait for desc are finished */
	while (desq_done_idx < desc_num) {
		rd_reg = mmio_read_32(CV_XOR_DESQ_DONE_REG(glob_reg_base));
		desq_done_idx = (CV_XOR_DESQ_DONE_MASK & rd_reg) >> CV_XOR_DESQ_DONE_OFFSET;
	}

	/* get status */
	while (idx < desc_num) {
		desc_idx =  8 * idx + align_idx;
		desc_num_incr = 0x20 * idx + XOR_DESC_STATUS_LINE;
		addr_lo = hi_cast_32to64(desq_addr_hi) + lo_cast_32to64(desq_addr) + desc_num_incr;
		if (desc_num > MAX_XOR_SRAM_DESC_SIZE)
			data = (int)(mmio_read_64(addr_lo) & bitmask32);
		else
			data = transfer_desc[XOR_DESC_STATUS_LINE + desc_idx];
		data = (data >> XOR_DESC_BCS_OFFSET) & 0x1;
		result |= data;

		if ((data == 0) && (idx > 0)) { /* fail */
			if (data == 0) {
				bcs_idx++;
#ifdef DBG_PRINT
				printf("%s: desc %d, done %d, bcs %s\n",
				       __func__, idx, desq_done_idx,
				       (data == 1) ? "pass" : "fail");
#endif
			}
		}
		/* remove desc from xor */
		mmio_write_32(CV_XOR_DESQ_DEALLOC_REG(glob_reg_base), 1);
		idx++;
	}

	/* disable secure mode in xor engine 0 xorg secure reg */
	if (desc_num <= MAX_XOR_SRAM_DESC_SIZE)
		mmio_write_32(CV_XOR_SECURE_REG(glob_reg_base), 0x1 << CV_XOR_SECURE_OFFSET);

	return bcs_idx;
}

static int xor_test(u32 tot_test_data,
		    u32 src_addr_lo, u32 src_addr_hi,
		    u32 dst_addr_lo, u32 dst_addr_hi,
		    int byte_cnt, int pattern_type,
		    u32 desq_addr, u32 desq_addr_hi,
		    int ostd_rd_num)
{
	int dst_gap = 0;
	int wr_burst_len = 4;
	int rd_burst_len = 4;
	int desc_num, pattern_length_cnt, data32 = 0;
	int res = 0;
	uintptr_t src_data_sram[128];
	int x;
	if (xor_src_sram == 1) {
		for (x = 0; x < 64; x++) {
			src_data_sram[2 * x] = 0x0;
			src_data_sram[2 * x + 1] = 0xffffffffffffffff;
		}
	}
	uint64_t data = 0x0, data2 = 0x0;
	uintptr_t addr = 0x0;
	desc_num = tot_test_data / byte_cnt;
	struct pattern_info *pattern_table = ddr3_tip_get_pattern_table();
	if (xor_src_sram == 1) {
		src_addr_lo = (u32)(((uintptr_t)&src_data_sram[0]) & 0x00000000ffffffff);
		src_addr_hi =  (u32)(((uintptr_t)&src_data_sram[0]) >> 32);
	}
	addr = (uintptr_t)src_addr_lo + ((uintptr_t)src_addr_hi << 32);
	int loop_num = byte_cnt / (256 * 2 * 8);
	int loop_idx;
	int pattern;
	int idx, data32_2;
	int writethruodpg = (pattern_type > 2) ? 0 : 1;
	int dev_byte_num;
	int shift;
	int byte_idx;
	int repeat;
	if (wr_xor_pat == 1) { /* 1: write */
		if (writethruodpg) {
			loop_num = (loop_num == 0) ? 1 : loop_num;
#ifdef DBG_PRINT
			printf("tot_test_data 0x%x & byte_cnt %d -> desc_num %d -> loop_num %d\n",
			       tot_test_data, byte_cnt, desc_num, loop_num);
#endif
			for (loop_idx = 0; loop_idx < loop_num; loop_idx++) {
				if (pattern_type == 0) { /* 0: killer pattern */
					for (pattern = PATTERN_KILLER_DQ0;
					     pattern <= PATTERN_KILLER_DQ7; pattern++) {
						for (pattern_length_cnt = 0;
						     pattern_length_cnt <
						     pattern_table[pattern].pattern_len;
						     pattern_length_cnt++) {
							data32 = pattern_table_get_word(0, pattern,
											(u8)pattern_length_cnt);
							if (data_width == 0x3) {
								cpu_write(addr, data32);
								addr += 4;
							} else {
								data = (uint64_t)data32 + ((uint64_t)data32 << 32);
								data |= 0xffffffffffffff00;
								if (mask == OR_MASK)
									cpu_write(addr, data | xor_byte_mask);
								else
									cpu_write(addr, data & xor_byte_mask);
								addr += 8;
							}
						}
					}
				}

				if (pattern_type == 1) { /* 1: killer inv pattern */
					for (pattern = PATTERN_KILLER_DQ0_INV;
					     pattern <= PATTERN_KILLER_DQ7_INV; pattern++) {
						for (pattern_length_cnt = 0;
						     pattern_length_cnt <
						     pattern_table[pattern].pattern_len;
						     pattern_length_cnt++) {
							data32 = pattern_table_get_word(0, pattern,
											(u8)pattern_length_cnt);
							if (data_width == 0x3) {
								cpu_write(addr, data32);
								addr += 4;
							} else {
								data = (uint64_t)data32 + ((uint64_t)data32 << 32);
								if (mask == OR_MASK)
									cpu_write(addr, data | xor_byte_mask);
								else
									cpu_write(addr, data & xor_byte_mask);
								addr += 8;
							}
						}
					}
				}

				if (pattern_type == 2) { /* 2: resonance pattern */
					loop_num = byte_cnt / (256 * 8 * 8); /* opdg is 256 */
					for (pattern = PATTERN_RESONANCE_2T;
					     pattern <= PATTERN_RESONANCE_9T; pattern++) {
						for (idx = 0; idx < 8; idx++) {
							for (pattern_length_cnt = 0;
							     pattern_length_cnt <
							     pattern_table[pattern].pattern_len;
							     pattern_length_cnt++) {
								data32 =
									pattern_table_get_word(0, pattern,
											       (u8)pattern_length_cnt);
								if (data_width == 0x3) {
									cpu_write(addr, data32);
									addr += 4;
								} else {
									data = (uint64_t)data32 +
									       ((uint64_t)data32 << 32);
									if (mask == OR_MASK)
										cpu_write(addr, data | xor_byte_mask);
									else
										cpu_write(addr, data & xor_byte_mask);
									addr += 8;
								}
							}
						}
					}
				}

				if (pattern_type == 3) { /* 32-bit only */
					for (idx = 0; idx < 32; idx++) {
						data32_2 = ~(0x0 | 1 << idx);
						for (pattern_length_cnt = 0;
						     pattern_length_cnt < 128; pattern_length_cnt++) {
							data32 = (pattern_length_cnt % 2) ? 0xffffffff : 0x00000000;
							data32 = data32_2 & data32;
							addr = addr & 0x00000000ffffffff;
							mmio_write_32(addr, data32);
							addr += 4;
						}
					}
				}
			}
		} else { /* write with cpu regardsless of the odpg pattern */
			dev_byte_num = (data_width == 0x3) ? 4 : 8;
			loop_num = (data_width == 0x3) ? (byte_cnt / 4) : (byte_cnt / 8);
			for (loop_idx = 0; loop_idx < loop_num; loop_idx++) {
				if (pattern_type == MOVE_ONE_ZERO) {
					if (loop_idx == 0)
						data = 127;
					else
						data = data2 & 0xff;
					if ((loop_idx % 3 == 0) || (loop_idx % 3 == 1))
						data = 255 - data; /* inverse */
					else
						data = (data == 0xfe) ? 127 : (128 + data / 2);
				}
				if (pattern_type == MOVE_ZERO_ONE) {
					if (loop_idx == 0)
						data = 128;
					else
						data = data2 & 0xff;
					if ((loop_idx % 3 == 0) || (loop_idx % 3 == 1))
						data = 255 - data;
					else
						data = (data == 0x1) ? 128 : data / 2;
				}
				if (pattern_type == 5) {
					data = pattern_table_get_word(0, PATTERN_KILLER_DQ0,
								      (u8)(loop_idx %
								      pattern_table[PATTERN_KILLER_DQ0].pattern_len));
				}
				if (pattern_type == 6) {
					data = pattern_table_get_word(0, PATTERN_VREF,
								      (u8)(loop_idx %
								      pattern_table[PATTERN_KILLER_DQ0].pattern_len));
				}
				if (pattern_type == 7) {
					data = pattern_table_get_word(0, PATTERN_RESONANCE_9T,
								      (u8)(loop_idx %
								      pattern_table[PATTERN_KILLER_DQ0].pattern_len));
				}
				if (pattern_type == 8) {
					if (loop_idx % 2 == 0)
						data = 0x0;
					else
						data = 0xffffffffffffffff;
				}
				data2 = 0x0;
				shift = 0;
				for (byte_idx = 0; byte_idx < dev_byte_num; byte_idx++) {
					shift = shift + 8 * byte_idx;
					data2 |= ((data % 256) << shift);
				}

				if (mask == OR_MASK)
					cpu_write(addr, data2 | xor_byte_mask);
				else
					cpu_write(addr, data2 & xor_byte_mask);
				addr += dev_byte_num;
			}
		}
	}

	for (repeat = 0; repeat < 1; repeat++) {
		if (xor_src_to_dst == 1)  { /* 1: do */
			xor_memcpy(desc_num, byte_cnt,
				   src_addr_lo, src_addr_hi,
				   dst_addr_lo, dst_addr_hi, dst_gap,
				   wr_burst_len, rd_burst_len,
				   desq_addr, desq_addr_hi,
				   ostd_rd_num);
		}
		res += xor_memcmp(desc_num, byte_cnt,
				  src_addr_lo, src_addr_hi,
				  dst_addr_lo, dst_addr_hi,
				  dst_gap, rd_burst_len,
				  desq_addr, desq_addr_hi,
				  ostd_rd_num);
		repeat = (res > 0) ? 20 : repeat;
	}

	return res;
}

static void xor_memcpy_v(int desc_num, int byte_cnt,
			 int src_addr_lo, int src_addr_hi,
			 int dst_addr_lo, int dst_addr_hi,
			 int dst_gap, int wr_burst_len, int rd_burst_len)
{
	u32 transfer_desc[8];
	u32 rd_reg;
	int i, desc_idx = 0;
	int desq_done_idx;
	uint64_t glob_reg_base = 0xf0000000;
	uint64_t glob_reg_base_v = 0xf0000000;
	int xor_num = 4, xor_idx = 0;
	uint64_t data = 0x0;
	uintptr_t addr = 0x0;

	if (byte_cnt > 0x1000)
		byte_cnt |= 0x1000; /* wa: must enable bit 12 */

	/* src and dst addr 512M aligned */
	src_addr_hi = 0x0;
	dst_addr_hi = 0x0;
	src_addr_lo %= _512M;
	dst_addr_lo %= _512M;

	wr_burst_len &= 0x7;
	rd_burst_len &= 0x7;

	if ((desc_num * 0x40) > src_addr_lo)
		desc_num = (int)(src_addr_lo / 0x40) - 1;

	if (desc_num > 0x7fff)
		desc_num = 0x7fff;

#ifdef DBG_PRINT
	printf("%s: src addr_lo 0x%x, dst_addr_lo 0x%x, dst_gap 0x%x, byte_cnt 0x%x, desc_num %d\n",
	       __func__, src_addr_lo, dst_addr_lo, dst_gap, byte_cnt, desc_num);
#endif

	/* write descriptors to memory */
	for (xor_idx = 0; xor_idx < xor_num; xor_idx++) {
		for (desc_idx = 0; desc_idx < desc_num; desc_idx++) {
			/* init desc array */
			for (i = 0; i < 8; i++)
				transfer_desc[i] = 0;
			/* config desc array */
			transfer_desc[XOR_DESC_COMMAND_LINE] = PURE_DMA << 28;
			transfer_desc[XOR_DESC_BYTE_COUNTER_LINE] = byte_cnt;
			transfer_desc[XOR_DESC_SRC_BUF_ADDR_LOW_LINE] = src_addr_lo + _512M * xor_idx; /* src */
			transfer_desc[XOR_DESC_SRC_BUF_ADDR_HIGH_LINE] = src_addr_hi & 0x00000ffff;
			transfer_desc[XOR_DESC_DEST_BUFF_ADDR_LOW_LINE] =
				dst_addr_lo + _512M * xor_idx + desc_idx * dst_gap; /* dst */
			transfer_desc[XOR_DESC_DEST_BUFF_ADDR_HIGH_LINE] = dst_addr_hi & 0xffff;

			/* write desc to memory */
			addr = _512M * xor_idx + (uintptr_t)(0x20 * desc_idx);
			for (i = 0; i < 4; i++) {
				data = (uint64_t)(transfer_desc[2 * i]) |
				       ((uint64_t)(transfer_desc[2 * i + 1]) << 32);
				mmio_write_64(addr, data);
				addr += 8;
			}
		}

		glob_reg_base_v = glob_reg_base + 0x20000 * xor_idx;
		/* config desq */
		mmio_write_32(CV_XOR_DESQ_BASE_ADDR_LOW_REG(glob_reg_base_v), _512M * xor_idx);
		mmio_write_32(CV_XOR_DESQ_BASE_ADDR_HIGH_REG(glob_reg_base_v), 0);
		mmio_write_32(CV_XOR_DESQ_SIZE_REG(glob_reg_base_v), desc_num);
		mmio_write_32(CV_XOR_DESQ_AR_ATTRIBUTES_REG(glob_reg_base_v), 0x3e3e);
		mmio_write_32(CV_XOR_DESQ_AW_ATTRIBUTES_REG(glob_reg_base_v), 0x3e3e);
		mmio_write_32(CV_XOR_IMSG_THRESHOLD_REG(glob_reg_base_v), 0);
		mmio_write_32(CV_XOR_DESQ_CTRL_REG(glob_reg_base_v), 0); /* 32B desc size */
		mmio_write_32(CV_XOR_DESQ_STOP_REG(glob_reg_base_v), 0);
		mmio_write_32(CV_XOR_BW_CTRL_REG(glob_reg_base_v),
			      (rd_burst_len << CV_XOR_MUX_READ_BURST_OFFSET |
			       wr_burst_len << CV_XOR_MUX_WRITE_BURST_OFFSET));
	}

	/* enqueue desc */
	for (xor_idx = 0; xor_idx < xor_num; xor_idx++) {
		glob_reg_base_v = glob_reg_base + 0x20000 * xor_idx;
#ifdef DBG_PRINT
		printf("%s: start xor %d, addr 0x%jx\n", __func__, xor_idx, glob_reg_base_v);
#endif
		mmio_write_32(CV_XOR_DESQ_ADD_REG(glob_reg_base_v), desc_num);
	}

	for (xor_idx = 0; xor_idx < xor_num; xor_idx++) {
		glob_reg_base_v = glob_reg_base + 0x20000 * xor_idx;
		desq_done_idx = 0;
		rd_reg = 0;
		while (desq_done_idx < desc_num) {
			rd_reg = mmio_read_32(CV_XOR_DESQ_DONE_REG(glob_reg_base_v));
			desq_done_idx = (CV_XOR_DESQ_DONE_MASK & rd_reg) >> CV_XOR_DESQ_DONE_OFFSET;
#ifdef DBG_PRINT
			printf("%s: xor %d: desq_done_idx %d, desc_num %d\n",
			       __func__, xor_idx, desq_done_idx, desc_num);
#endif
		}
		/* dequeue desc */
		mmio_write_32(CV_XOR_DESQ_DEALLOC_REG(glob_reg_base_v), desc_num);
	}
}

static int xor_memcmp_v(int desc_num, int byte_cnt,
			int src_addr_lo, int src_addr_hi,
			int dst_addr_lo, int dst_addr_hi,
			int dst_gap, int rd_burst_len)
{
	/* desq per xor = 0x0 + xor_idx * 512M */
	u32 transfer_desc[8];
	u32 rd_reg = 0;
	int i, desc_idx = 0;
	int desq_done_idx;
	int bcs_idx = 0;
	int print_ena = 0;
	int xor_num = 4, xor_idx = 0;
	uint64_t glob_reg_base = 0xf0000000;
	uint64_t glob_reg_base_v = 0xf0000000;
	uint64_t data = 0x0;
	uintptr_t addr = 0x0;

	if (byte_cnt > 0x1000)
		byte_cnt |= 0x1000; /* wa: must enable bit 12 */

	/* src and dst addr 512M aligned */
	src_addr_hi = 0x0;
	dst_addr_hi = 0x0;
	src_addr_lo %= _512M;
	dst_addr_lo %= _512M;

	rd_burst_len &= 0x7;

	if ((desc_num * 0x40) > src_addr_lo)
		desc_num = (int)(src_addr_lo / 0x40) - 1;

	if (desc_num > 0x7fff)
		desc_num = 0x7fff;

#ifdef DBG_PRINT
	printf("%s: src addr_lo 0x%x, dst_addr_lo 0x%x, dst_gap 0x%x, byte_cnt 0x%x, desc_num %d\n",
	       __func__, src_addr_lo, dst_addr_lo, dst_gap, byte_cnt, desc_num);
#endif

	/* write descriptors to memory */
	for (xor_idx = 0; xor_idx < xor_num; xor_idx++) {
		for (desc_idx = 0; desc_idx < desc_num; desc_idx++) {
			/* init desc array */
			for (i = 0; i < 8; i++)
				transfer_desc[i] = 0;
			/* config desc array */
			transfer_desc[XOR_DESC_COMMAND_LINE] =
				((MEM_CMP<<XOR_DESC_OPERATION_MODE_OFFSET) | (0x3 << 19));
			transfer_desc[XOR_DESC_BYTE_COUNTER_LINE] = byte_cnt;
			transfer_desc[XOR_DESC_SRC_BUF_ADDR_LOW_LINE] = src_addr_lo + _512M * xor_idx; /* src */
			transfer_desc[XOR_DESC_SRC_BUF_ADDR_HIGH_LINE] = src_addr_hi & 0x00000ffff;
			transfer_desc[XOR_DESC_DEST_BUFF_ADDR_LOW_LINE] =
				dst_addr_lo + _512M * xor_idx + desc_idx * dst_gap; /* dst */
			transfer_desc[XOR_DESC_DEST_BUFF_ADDR_HIGH_LINE] = dst_addr_hi & 0xffff;

			/* write desc to memory */
			addr = _512M * xor_idx + (uintptr_t)(0x20 * desc_idx);
			for (i = 0; i < 4; i++) {
				data = (uint64_t)(transfer_desc[2 * i]) |
				       ((uint64_t)(transfer_desc[2 * i + 1]) << 32);
				mmio_write_64(addr, data);
				addr += 8;
			}
		}

		glob_reg_base_v = glob_reg_base + 0x20000 * xor_idx;
		/* config desq */
		mmio_write_32(CV_XOR_DESQ_BASE_ADDR_LOW_REG(glob_reg_base_v), _512M * xor_idx);
		mmio_write_32(CV_XOR_DESQ_BASE_ADDR_HIGH_REG(glob_reg_base_v), 0);
		mmio_write_32(CV_XOR_DESQ_SIZE_REG(glob_reg_base_v), desc_num);
		mmio_write_32(CV_XOR_DESQ_AR_ATTRIBUTES_REG(glob_reg_base_v), 0x3e3e);
		mmio_write_32(CV_XOR_DESQ_AW_ATTRIBUTES_REG(glob_reg_base_v), 0x3e3e);
		mmio_write_32(CV_XOR_IMSG_THRESHOLD_REG(glob_reg_base_v), 0);
		mmio_write_32(CV_XOR_DESQ_CTRL_REG(glob_reg_base_v), 0); /* 32B desc size */
		mmio_write_32(CV_XOR_DESQ_STOP_REG(glob_reg_base_v), 0);
		mmio_write_32(CV_XOR_BW_CTRL_REG(glob_reg_base_v),
			      rd_burst_len << CV_XOR_MUX_READ_BURST_OFFSET);
	}

	/* enqueue desc */
	for (xor_idx = 0; xor_idx < xor_num; xor_idx++) {
		glob_reg_base_v = glob_reg_base + 0x20000 * xor_idx;
#ifdef DBG_PRINT
		printf("%s: start xor %d, addr 0x%jx\n", __func__, xor_idx, glob_reg_base_v);
#endif
		mmio_write_32(CV_XOR_DESQ_ADD_REG(glob_reg_base_v), desc_num);
	}

	/* wait for desc are finished */
	for (xor_idx = 0; xor_idx < xor_num; xor_idx++) {
		glob_reg_base_v = glob_reg_base + 0x20000 * xor_idx;
		desc_idx = 0;
		desq_done_idx = 0;
		while (desc_idx < desc_num) {
			rd_reg = mmio_read_32(CV_XOR_DESQ_DONE_REG(glob_reg_base_v));
			desq_done_idx = (CV_XOR_DESQ_DONE_MASK & rd_reg) >> CV_XOR_DESQ_DONE_OFFSET;
			if (desq_done_idx > 0) {
				/* dequeue desc */
				mmio_write_32(CV_XOR_DESQ_DEALLOC_REG(glob_reg_base_v), 1);
				addr =  (uintptr_t)(0x20 * desc_idx + (uintptr_t)(_512M * xor_idx)) +
					(uintptr_t)XOR_DESC_STATUS_LINE;
				data = mmio_read_64(addr);
#ifdef DBG_PRINT
				printf("%s: desc data 0x%jx, CV_XOR_DESQ_DONE_REG 0x%x\n",
				       __func__, data, rd_reg);
#endif
				data = (data >> XOR_DESC_BCS_OFFSET) & 0x1;
#ifdef DBG_PRINT
				printf("%s: xor %d, addr 0x%jx, desc %d, desq_done_idx %d, bcs %s\n",
				       __func__, xor_idx, addr, desc_idx, desq_done_idx,
				       (data == 1) ? "pass" : "fail");
#endif
				if ((data == 0) & (desc_idx > 0)) { /* fail */
					bcs_idx++;
					if (print_ena == 1) {
#ifdef DBG_PRINT
						printf("%s: desc %d, desq_done_idx %d, bcs %s\n",
						       __func__, desc_idx, desq_done_idx,
						       (data == 1) ? "pass" : "fail");
#endif
					}
				}
				desc_idx++;
			}
		}
	}

	return bcs_idx;
}

static int xor_test_v(int tot_test_data,
		      int src_addr_lo, int src_addr_hi,
		      int dst_addr_lo, int dst_addr_hi,
		      int byte_cnt, int pattern_type,
		      int ostd_rd_num)
{
	int dst_gap = byte_cnt * 2 + 8;
	int wr_burst_len = 7;
	int rd_burst_len = 7;
	int desc_num;
	int xor_num = 4, xor_idx = 0;
	int res = 0;
	int loop_idx, loop_num;
	int dev_byte_num;
	int shift;
	int byte_idx;
	uint64_t data = 0x0, data2 = 0x0;
	uintptr_t addr = 0x0;
	struct pattern_info *pattern_table = ddr3_tip_get_pattern_table();

	desc_num = tot_test_data / byte_cnt;
#ifdef DBG_PRINT
	printf("%s: src addr_lo 0x%x, dst_addr_lo 0x%x, dst_gap 0x%x, byte_cnt 0x%x, desc_num %d\n",
	       __func__, src_addr_lo, dst_addr_lo, dst_gap, byte_cnt, desc_num);
#endif
	/* src and dst addr 512M aligned */
	src_addr_hi = 0x0;
	dst_addr_hi = 0x0;
	src_addr_lo %= _512M;
	dst_addr_lo %= _512M;

	for (xor_idx = 0; xor_idx < xor_num; xor_idx++) {
		addr = src_addr_lo + _512M * xor_idx;
		loop_num = byte_cnt / (256 * 2 * 8); /* odpg is 256 */
		loop_num = (loop_num == 0) ? 1 : loop_num;
		dev_byte_num = (data_width == 0x3) ? 4 : 8;
		loop_num = (data_width == 0x3) ? (byte_cnt / 4) : (byte_cnt / 8);
		for (loop_idx = 0; loop_idx < loop_num; loop_idx++) {
			if (pattern_type == MOVE_ONE_ZERO) {
				if (loop_idx == 0)
					data = 127;
				else
					data = data2 & 0xff;
				if ((loop_idx % 3 == 0) || (loop_idx % 3 == 1))
					data = 255 - data; /* inv */
				else
					data = (data == 0xfe) ? 127 : (128 + data / 2);
			}
			if (pattern_type == MOVE_ZERO_ONE) {
				if (loop_idx == 0)
					data = 128;
				else
					data = data2 & 0xff;
				if ((loop_idx % 3 == 0) || (loop_idx % 3 == 1))
					data = 255 - data;
				else
					data = (data == 0x1) ? 128 : data / 2;
			}
			if (pattern_type == 5) {
				data = pattern_table_get_word(0, PATTERN_KILLER_DQ0,
							      (u8)(loop_idx %
								   pattern_table[PATTERN_KILLER_DQ0].pattern_len));
			}
			if (pattern_type == 6) {
				data = pattern_table_get_word(0, PATTERN_VREF,
							      (u8)(loop_idx %
								   pattern_table[PATTERN_KILLER_DQ0].pattern_len));
			}
			if (pattern_type == 7) {
				data = pattern_table_get_word(0, PATTERN_RESONANCE_9T,
							      (u8)(loop_idx %
								   pattern_table[PATTERN_KILLER_DQ0].pattern_len));
			}
			if (pattern_type == 8) {
				if (loop_idx % 2 == 0)
					data = 0x0;
				else
					data = 0xffffffffffffffff;
			}

			data2 = 0x0;
			shift = 0;
			for (byte_idx = 0; byte_idx < dev_byte_num; byte_idx++) {
				shift += (8 * byte_idx);
				data2 |= ((data % 256) << shift);
			}

			if (mask == OR_MASK)
				cpu_write(addr, data2 | xor_byte_mask);
			else
				cpu_write(addr, data2 & xor_byte_mask);

			addr += dev_byte_num;
		}
	}

	xor_memcpy_v(desc_num, byte_cnt,
		     src_addr_lo, src_addr_hi,
		     dst_addr_lo, dst_addr_hi, dst_gap,
		     wr_burst_len, rd_burst_len);
	res = xor_memcmp_v(desc_num, byte_cnt,
			   src_addr_lo, src_addr_hi,
			   dst_addr_lo, dst_addr_hi,
			   dst_gap, rd_burst_len);

	return res;
}

/* do not access not dram region (3.75..4G); do not exceed 128 xor descriptors */
static int xor_gradual_test(u8 depth_stage)
{
	u32 tot_test_data;
	enum validation_pattern pattern = 0;
	int byte_cnt;
	int ret = 0;
	int idx;
	u32 dst_addr_lo;
	u32 dst_addr_hi;
	u32 src_addr_lo = (reg_read(MC6_CH0_MMAP_LOW_REG(0)) & (0x1ff << 23));
	u32 src_addr_hi = reg_read(MC6_CH0_MMAP_HIGH_REG(0));
	u32 src_offs;
	u32 dst_offs;
	unsigned long long tmp_area_len = 0;
	unsigned long long src_addr = (((unsigned long long)src_addr_hi << 32) |
				       (unsigned long long)src_addr_lo) +
				       (unsigned long long)(effective_cs * area_len);
	unsigned long long end_addr_bytes = (((unsigned long long)src_addr_hi << 32) |
					     (unsigned long long)src_addr_lo) +
					     (unsigned long long)((effective_cs + 1) * area_len);

	src_addr_lo = (u32)(src_addr & 0x00000000ffffffff);
	src_addr_hi = (u32)(src_addr >> 32);
	/* prevent access to non dram region (3.75..4G) */
	tmp_area_len = area_len;
	if ((area_len == _4G) && (effective_cs == 0))
		tmp_area_len = _4G - _1G;

	if ((area_len == _2G) && (effective_cs == 1))
		tmp_area_len = _4G - _1G;

	/* depth 0 */
	if (depth_stage == 0) {
		byte_cnt = 1024;
		tot_test_data = byte_cnt * 120;
		pattern = MOVE_ONE_ZERO;
		end_addr_bytes = (unsigned long long)((effective_cs + 1) * tmp_area_len) -
				 (unsigned long long)(3 * tot_test_data);
		dst_addr_lo = (u32)(end_addr_bytes & 0x00000000ffffffff);
		dst_addr_hi = (u32)(end_addr_bytes >> 32);
		xor_src_sram = 1;
		ret += xor_test(tot_test_data, src_addr_lo, src_addr_hi,
				dst_addr_lo, dst_addr_hi, byte_cnt, pattern, 0x0, 0x0, OSTD_RD_NUM);
		xor_src_sram = 0;
		if (ret != 0)
			return MV_FAIL;
		if (depth_stage == 0)
			return MV_OK;
	}

	/* depth 1 */
	tot_test_data = _8K;
	byte_cnt = tot_test_data / 8;
	pattern = 8; /* sso */
	xor_src_sram = 0;
	end_addr_bytes = (unsigned long long)((effective_cs + 1) * tmp_area_len) -
			 (unsigned long long)(3 * tot_test_data);
	dst_addr_lo = (u32)(end_addr_bytes & 0x00000000ffffffff);
	dst_addr_hi = (u32)(end_addr_bytes >> 32);
	ret += xor_test(tot_test_data, src_addr_lo, src_addr_hi,
			dst_addr_lo, dst_addr_hi, byte_cnt, pattern, 0x0, 0x0, OSTD_RD_NUM);
	if (ret != 0)
		return ret;
	if (depth_stage == 1)
		return MV_OK;

	/* depth 2 */
	tot_test_data = _8K;
	byte_cnt = tot_test_data / 8;
	pattern = MOVE_ONE_ZERO;
	xor_src_sram = 0;
	end_addr_bytes = (unsigned long long)((effective_cs + 1) * tmp_area_len) -
			 (unsigned long long)(3 * tot_test_data);
	dst_addr_lo =  (u32)(end_addr_bytes & 0x00000000ffffffff);
	dst_addr_hi =  (u32)(end_addr_bytes >> 32);
	ret += xor_test(tot_test_data, src_addr_lo, src_addr_hi,
			dst_addr_lo, dst_addr_hi, byte_cnt, pattern, 0x0, 0x0, OSTD_RD_NUM);
	if (ret != 0)
		return ret;
	if (depth_stage == 2)
		return MV_OK;

	/* depth 3 */
	tot_test_data = _8M;
	tot_test_data = _1M;
	byte_cnt = tot_test_data / 64;
	pattern = ALL_AGRESSIVE_INV;
	end_addr_bytes = (unsigned long long)((effective_cs + 1) * tmp_area_len) -
			 (unsigned long long)(3 * tot_test_data);
	dst_addr_lo =  (u32)(end_addr_bytes & 0x00000000ffffffff);
	dst_addr_hi =  (u32)(end_addr_bytes >> 32);
	ret += xor_test(tot_test_data, src_addr_lo, src_addr_hi,
			dst_addr_lo, dst_addr_hi, byte_cnt, pattern, 0x0, 0x0, OSTD_RD_NUM);
	if (ret != 0)
		return ret;
	if (depth_stage == 3)
		return MV_OK;

	/* depth 4 */
	src_offs = _1M;
	dst_offs = _256M; /* from source of cs */
	for (idx  = 3; idx < 6; idx++) {
		idx = (idx == 2) ? 3 : idx; /* skip pattern 2 */
		tot_test_data = _1M;
		byte_cnt = tot_test_data / 2048;
		pattern = idx;
		xor_src_sram = 0; /* data in dram */
		src_addr = (unsigned long long)(effective_cs * tmp_area_len) + (unsigned long long)src_offs;
		src_addr_lo = (u32)(src_addr & 0x00000000ffffffff);
		src_addr_hi =  (u32)(src_addr >> 32);
		end_addr_bytes =  (unsigned long long)(effective_cs * tmp_area_len) + (unsigned long long)dst_offs;
		dst_addr_lo = (u32)(end_addr_bytes & 0x00000000ffffffff);
		dst_addr_hi =  (u32)(end_addr_bytes >> 32);
		ret += xor_test_v(tot_test_data, src_addr_lo, src_addr_hi,
				 dst_addr_lo, dst_addr_hi, byte_cnt, pattern, OSTD_RD_NUM);
		if (ret != 0)
			return ret;
	}
	if (depth_stage >= 4)
		return MV_OK;

	/* depth 4 */
	dst_addr_lo = src_addr_lo;
	dst_addr_hi = src_addr_hi;
	for (idx = 4; idx < (tmp_area_len / _64K - 1); idx *= 2) {
		tot_test_data = _64K;
		byte_cnt = tot_test_data / 2;
		pattern = (pattern == MOVE_ZERO_ONE) ? MOVE_ONE_ZERO : MOVE_ZERO_ONE;
		xor_src_sram = 0;
		src_addr = (unsigned long long)(effective_cs * tmp_area_len) +
			   (unsigned long long)(idx * tot_test_data);
		src_addr_lo =  (u32)(src_addr & 0x00000000ffffffff);
		src_addr_hi =  (u32)(src_addr >> 32);
		ret += xor_test(tot_test_data, src_addr_lo, src_addr_hi,
				dst_addr_lo, dst_addr_hi, byte_cnt, pattern, 0x0, 0x0, OSTD_RD_NUM);
		if (ret != 0)
			return ret;
	}
	if (depth_stage >= 4)
		return MV_OK;

	return MV_OK;
}

static int xor_search_1d_1e(enum hws_edge_compare edge, enum hws_search_dir search_dir,
			    u32 step, u32 init_val, u32 end_val, u8 depth_stage,
			    u16 byte_num, enum search_element element)
{
	int result;
	int bs_left = (int)init_val;
	int bs_middle;
	int bs_right = (int)end_val;
	int bs_found = -1;
	u32 reg_addr = (element == CRX) ? CRX_PHY_BASE : CTX_PHY_BASE;

	reg_addr += (4 * effective_cs);

	if (search_dir == HWS_LOW2HIGH) {
		while (bs_left <= bs_right) {
			bs_middle = (bs_left + bs_right) / 2;

			if (element == REC_CAL) {
				ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num,
						   DDR_PHY_DATA, VREF_BCAST_PHY_REG(effective_cs), bs_middle);
				ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num,
						   DDR_PHY_DATA, VREF_PHY_REG(effective_cs, 4), bs_middle);
				ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num,
						   DDR_PHY_DATA, VREF_PHY_REG(effective_cs, 5), bs_middle);
			} else {
				ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num,
						   DDR_PHY_DATA, reg_addr, bs_middle);
			}
			result = xor_gradual_test(depth_stage);

			/*
			 * continue to search for the leftmost fail if pass found in pass-to-fail case or
			 * for the leftmost pass if fail found in fail-to-pass case
			 */
			if ((result == 0 && edge == EDGE_PF) |
			    (result != 0 && edge == EDGE_FP))
				bs_left = bs_middle + 1;

			/*
			 * save recently found fail in pass-to-fail case or
			 * recently found pass in fail-to-pass case
			 */
			if ((result != 0 && edge == EDGE_PF) |
			    (result == 0 && edge == EDGE_FP)) {
				bs_found = bs_middle;
				bs_right = bs_middle - 1;
			}
		}
	} else { /* search_dir == HWS_HIGH2LOW */
		while (bs_left >= bs_right) {
			bs_middle = (bs_left + bs_right) / 2;

			if (element == REC_CAL) {
				ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num,
						   DDR_PHY_DATA, VREF_BCAST_PHY_REG(effective_cs), bs_middle);
				ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num,
						   DDR_PHY_DATA, VREF_PHY_REG(effective_cs, 4), bs_middle);
				ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num,
						   DDR_PHY_DATA, VREF_PHY_REG(effective_cs, 5), bs_middle);
			} else {
				ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num,
						   DDR_PHY_DATA, reg_addr, bs_middle);
			}
			result = xor_gradual_test(depth_stage);

			/*
			 * continue to search for the leftmost fail if pass found in pass-to-fail case or
			 * for the leftmost pass if fail found in fail-to-pass case
			 */
			if ((result == 0 && edge == EDGE_PF) |
			    (result != 0 && edge == EDGE_FP))
				bs_left = bs_middle - 1;

			/*
			 * save recently found fail in pass-to-fail case or
			 * recently found pass in fail-to-pass case
			 */
			if ((result != 0 && edge == EDGE_PF) |
			    (result == 0 && edge == EDGE_FP)) {
				bs_found = bs_middle;
				bs_right = bs_middle + 1;
			}
		}
	}

	if (bs_found >= 0)
		result = bs_found;
	else if (edge == EDGE_PF)
		result = bs_right;
	else
		result = 255;

#ifdef DBG_PRINT
	printf("%d, %d, %d\n", init_val, end_val, result);
#endif
	return result;
}

static int xor_search_1d_2e(enum hws_edge_compare search_concept, u32 step, u32 init_val, u32 end_val,
			    u8 depth_stage, u16 byte_num, enum search_element element, u8 *vw_vector)
{
	u32 reg_addr = (element == CRX) ? CRX_PHY_BASE : CTX_PHY_BASE;
	u32 reg_data;
	reg_addr += (4 * effective_cs);
	vw_vector[0] = end_val;
	vw_vector[1] = init_val;

	if (element == REC_CAL) {
		ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
				  VREF_BCAST_PHY_REG(effective_cs), &reg_data);
	} else if (element == CRX) {
		ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
				  CRX_PHY_REG(effective_cs), &reg_data);
	} else if (element == CTX) {
		ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
				  CTX_PHY_REG(effective_cs), &reg_data);
	}

	/* xor_search_1d_1e returns the edge transition value of the elements */
	if (search_concept == EDGE_FP) {
		vw_vector[0] = xor_search_1d_1e(EDGE_FP, HWS_LOW2HIGH, step, init_val, end_val,
						depth_stage, byte_num, element);
	vw_vector[0] = (vw_vector[0] == reg_data) ? 255 : vw_vector[0];
	vw_vector[1] = xor_search_1d_1e(EDGE_FP, HWS_HIGH2LOW, step, end_val, init_val,
					depth_stage, byte_num, element);
	vw_vector[1] = (vw_vector[1] == reg_data) ? 255 : vw_vector[1];
	} else {
		vw_vector[1] = xor_search_1d_1e(EDGE_PF, HWS_LOW2HIGH, step, reg_data, end_val,
						depth_stage, byte_num, element);
#ifdef DBG_PRINT
		printf("%s: vw_vector[1] = %d\n", __func__, vw_vector[1]);
#endif
		vw_vector[0] = xor_search_1d_1e(EDGE_PF, HWS_HIGH2LOW, step, reg_data, init_val,
						depth_stage, byte_num, element);
#ifdef DBG_PRINT
		printf("%s: vw_vector[0] = %d\n", __func__, vw_vector[0]);
#endif
		if ((vw_vector[1] == reg_data) && (vw_vector[0] == reg_data)) {
			vw_vector[1] = 255;
			vw_vector[0] = 255;
		} else {
			vw_vector[1] = vw_vector[1]; /* -step */
			vw_vector[0] = vw_vector[0]; /* +step */
		}
#ifdef DBG_PRINT
		printf("%s: vw_vector[0] = %d\n", __func__, vw_vector[0]);
		printf("%s: vw_vector[1] = %d\n", __func__, vw_vector[1]);
#endif
	}

	if (element == REC_CAL) {
		ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
				   VREF_BCAST_PHY_REG(effective_cs), reg_data);
		ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
				   VREF_PHY_REG(effective_cs, 4), reg_data);
		ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
				   VREF_PHY_REG(effective_cs, 5), reg_data);
	} else if (element == CRX) {
		ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
				   CRX_PHY_REG(effective_cs), reg_data);
	} else if (element == CTX) {
		ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
				   CTX_PHY_REG(effective_cs), reg_data);
	}

	if ((vw_vector[1] == 255) || (vw_vector[0] == 255)) {
		return 255;
	} else {
		if (vw_vector[1] < vw_vector[0]) {
			if (search_concept == EDGE_PF) {
#ifdef DBG_PRINT
				printf("%s: end_val %d, init_val %d, step %d, nominal %d\n",
				       __func__, end_val, init_val, step, reg_data);
#endif
			}
			return 0;
		} else {
			return vw_vector[1] - vw_vector[0];
		}
	}

	return 0;
}

static int xor_search_2d_1e(enum hws_edge_compare edge,
			    enum search_element element1, enum hws_search_dir search_dir1,
			    u32 step1, u32 init_val1, u32 end_val1,
			    enum search_element element2, enum hws_search_dir search_dir2,
			    u32 step2, u32 init_val2, u32 end_val2,
			    u8 depth_stage, u16 byte_num)
{
	int result = (edge == EDGE_PF) ? 0 : 1; /* start from 0 (pass), if pass */
	int sign_step1 = (search_dir1 == HWS_LOW2HIGH) ? step1 : -step1;
	int param1 = (int)init_val1;
	u32 steps_num1 = (search_dir1 == HWS_LOW2HIGH) ? (end_val1 - init_val1) / step1 :
							 (init_val1 - end_val1) / step1;
	int sign_step2 = (search_dir2 == HWS_LOW2HIGH) ? step2 : -step2;
	int param2 = (int)init_val2;
	u32 steps_num2 = (search_dir2 == HWS_LOW2HIGH) ? (end_val2 - init_val2) / step2 :
							 (init_val2 - end_val2) / step2;
	u32 steps_num = (steps_num1 < steps_num2) ? steps_num1 : steps_num2;
	u32 step_idx = 0;
	u32 reg_addr1 = CRX_PHY_BASE;
	reg_addr1 += (4 * effective_cs);

#ifdef DBG_PRINT
	printf("%s: byte_num %d, param1 %d, param2 %d\n", __func__, byte_num, param1, param2);
#endif

	for (step_idx = 0; step_idx <= steps_num; step_idx++) {
		ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
				   VREF_BCAST_PHY_REG(effective_cs), param2);
		ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
				   VREF_PHY_REG(effective_cs, 4), param2);
		ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
				   VREF_PHY_REG(effective_cs, 5), param2);
		ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
				   reg_addr1, param1);
		result = xor_gradual_test(depth_stage);
#ifdef DBG_PRINT
		printf("%s: param1 %d, param2 %d -> %d\n", __func__, param1, param2, result);
#endif
		if (((result != 0) && (edge == EDGE_PF)) || ((result == 0) && (edge == EDGE_FP))) {
#ifdef DBG_PRINT
			printf("%s: param1 %d, param2 %d\n", __func__, param1, param2);
#endif
			return step_idx;
		}

		param1 += sign_step1;
		param2 += sign_step2;
	}

	return 255; /* search on positive numbers; -1 represents fail */
}

static int xor_search_2d_2e(enum hws_edge_compare edge,
			    enum search_element element1, enum hws_search_dir search_dir1,
			    u32 step1, u32 init_val1, u32 end_val1,
			    enum  search_element element2, enum hws_search_dir search_dir2,
			    u32 step2, u32 init_val2, u32 end_val2,
			    u8 depth_stage, u16 byte_num, u8 *vw_vector)
{
	u32 reg_addr = CRX_PHY_BASE;
	u32 reg_data1, reg_data2;
	reg_addr += (4 * effective_cs);
	vw_vector[0] = end_val1;
	vw_vector[1] = init_val1;
	/* read nominal search element */
	ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
			  VREF_BCAST_PHY_REG(effective_cs), &reg_data2);
	ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
			  CRX_PHY_REG(effective_cs), &reg_data1);

#ifdef DBG_PRINT
	printf("%s: byte_num %d, nominal %d, %d\n", __func__, byte_num, reg_data1, reg_data2);
#endif
	vw_vector[0] = xor_search_2d_1e(EDGE_FP, element1, search_dir1, step1, init_val1, end_val1,
					element2, search_dir2, step2, init_val2, end_val2,
					depth_stage, byte_num);
	search_dir2 = (search_dir2 == HWS_LOW2HIGH) ? HWS_HIGH2LOW : HWS_LOW2HIGH;
	search_dir1 = (search_dir1 == HWS_LOW2HIGH) ? HWS_HIGH2LOW : HWS_LOW2HIGH;
	ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
			  VREF_BCAST_PHY_REG(effective_cs), &reg_data2);
	ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
			  CRX_PHY_REG(effective_cs), &reg_data1);
#ifdef DBG_PRINT
	printf("%s: byte_num %d, nominal %d, %d\n", __func__, byte_num, reg_data1, reg_data2);
#endif
	vw_vector[1] = xor_search_2d_1e(EDGE_FP, element1, search_dir1, step1, end_val1, init_val1,
					element2, search_dir2, step2, end_val2, init_val2,
					depth_stage, byte_num);
	ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
			   VREF_BCAST_PHY_REG(effective_cs), reg_data2);
	ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
			   VREF_PHY_REG(effective_cs, 4), reg_data2);
	ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
			   VREF_PHY_REG(effective_cs, 5), reg_data2);
	ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, byte_num, DDR_PHY_DATA,
			   CRX_PHY_REG(effective_cs), reg_data1);

	if ((vw_vector[1] == 255) || (vw_vector[0] == 255))
		return 255;
	else
		return vw_vector[1] - vw_vector[0];

	return 0;
}

/*
 * print stability log info
 * printout matches the excel tool
 */
int stability_info_print(void)
{
	u8 if_id = 0, csindex = 0, bus_id = 0, idx = 0;
	u32 reg_data;
#if defined(CONFIG_DDR4)
	u32 reg_data1;
#endif /* CONFIG_DDR4 */
	u32 read_data[MAX_INTERFACE_NUM];
	u32 max_cs = ddr3_tip_max_cs_get(0);
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();

	/* title print */
	for (if_id = 0; if_id < MAX_INTERFACE_NUM; if_id++) {
		VALIDATE_IF_ACTIVE(tm->if_act_mask, if_id);
		printf("title: i/f, tj, cal_n0, cal_p0, cal_n1, cal_p1, "
		       "cal_n2, cal_p2, cal_n3, cal_p3, ");
		for (csindex = 0; csindex < max_cs; csindex++) {
			printf("cs%d, ", csindex);
			for (bus_id = 0; bus_id < MAX_BUS_NUM; bus_id++) {
				VALIDATE_BUS_ACTIVE(tm->bus_act_mask, bus_id);
#if defined(CONFIG_DDR4)
				printf("dmin_tx, area_tx, dmin_rx, area_rx, wl_tot, wl_adll, wl_ph, "
				       "rl_tot, rl_adll, rl_ph, rl_smp, ctx, crx, vref, dq_vref, ");
				for (idx = 0; idx < 11; idx++)
					printf("dc-pad%d, ", idx);
#else /* CONFIG_DDR4 */
				printf("vw_tx, vw_rx, wl_tot, wl_adll, wl_ph, rl_tot, rl_adll, rl_ph, "
				       "rl_smp, ctx, crx, vref, dq_vref, ");
#endif /* CONFIG_DDR4 */
				for (idx = 0; idx < 11; idx++)
					printf("pbs_tx-pad%d, ", idx);

				for (idx = 0; idx < 11; idx++)
					printf("pbs_rx-pad%d, ", idx);
			}
		}
	}
	printf("\n");

	/* data print */
	for (if_id = 0; if_id < MAX_INTERFACE_NUM; if_id++) {
		VALIDATE_IF_ACTIVE(tm->if_act_mask, if_id);

		printf("data: %d,%d,", if_id,
		       (config_func_info[0].tip_get_temperature != NULL) ?
		       (config_func_info[0].tip_get_temperature(0)) : (0));

		ddr3_tip_if_read(0, ACCESS_TYPE_UNICAST, if_id, 0x14c8,
				 read_data, MASK_ALL_BITS);
		printf("%d,%d,", ((read_data[if_id] & 0x3f0) >> 4),
		       ((read_data[if_id] & 0xfc00) >> 10));
		ddr3_tip_if_read(0, ACCESS_TYPE_UNICAST, if_id, 0x17c8,
				 read_data, MASK_ALL_BITS);
		printf("%d,%d,", ((read_data[if_id] & 0x3f0) >> 4),
		       ((read_data[if_id] & 0xfc00) >> 10));
		ddr3_tip_if_read(0, ACCESS_TYPE_UNICAST, if_id, 0x1dc8,
				 read_data, MASK_ALL_BITS);
		printf("%d,%d,", ((read_data[if_id] & 0x3f0000) >> 16),
		       ((read_data[if_id] & 0xfc00000) >> 22));
		ddr3_tip_if_read(0, ACCESS_TYPE_UNICAST, if_id, 0x1ec8,
				 read_data, MASK_ALL_BITS);
		printf("%d,%d,", ((read_data[if_id] & 0x3f0000) >> 16),
		       ((read_data[if_id] & 0xfc00000) >> 22));

		for (csindex = 0; csindex < max_cs; csindex++) {
			printf("cs%d,", csindex);
			for (bus_id = 0; bus_id < MAX_BUS_NUM; bus_id++) {
				VALIDATE_BUS_ACTIVE(tm->bus_act_mask, bus_id);
#if defined(CONFIG_DDR4)
				/* dmin_tx, area_tx */
				ddr3_tip_bus_read(0, if_id,
						  ACCESS_TYPE_UNICAST,
						  bus_id, DDR_PHY_DATA,
						  RESULT_PHY_REG +
						  csindex, &reg_data);
				ddr3_tip_bus_read(0, if_id,
						  ACCESS_TYPE_UNICAST,
						  dmin_phy_reg_table
						  [csindex * 5 + bus_id][0],
						  DDR_PHY_CONTROL,
						  dmin_phy_reg_table
						  [csindex * 5 + bus_id][1],
						  &reg_data1);
				printf("%d,%d,", 2 * (reg_data1 & 0xff),
				       reg_data);
				/* dmin_rx, area_rx */
				ddr3_tip_bus_read(0, if_id,
						  ACCESS_TYPE_UNICAST,
						  bus_id, DDR_PHY_DATA,
						  RESULT_PHY_REG +
						  csindex + 4, &reg_data);
				ddr3_tip_bus_read(0, if_id,
						  ACCESS_TYPE_UNICAST,
						  dmin_phy_reg_table
						  [csindex * 5 + bus_id][0],
						  DDR_PHY_CONTROL,
						  dmin_phy_reg_table
						  [csindex * 5 + bus_id][1],
						  &reg_data1);
				printf("%d,%d,", 2 * (reg_data1 >> 8),
				       reg_data);
#else /* CONFIG_DDR4 */
				ddr3_tip_bus_read(0, if_id,
						  ACCESS_TYPE_UNICAST,
						  bus_id, DDR_PHY_DATA,
						  RESULT_PHY_REG +
						  csindex, &reg_data);
				printf("%d,%d,", (reg_data & 0x1f),
				       ((reg_data & 0x3e0) >> 5));
#endif /* CONFIG_DDR4 */
				/* wl */
				ddr3_tip_bus_read(0, if_id,
						  ACCESS_TYPE_UNICAST,
						  bus_id, DDR_PHY_DATA,
						  WL_PHY_REG(csindex),
						  &reg_data);
				printf("%d,%d,%d,",
				       (reg_data & 0x1f) +
				       ((reg_data & 0x1c0) >> 6) * 32,
				       (reg_data & 0x1f),
				       (reg_data & 0x1c0) >> 6);
				/* rl */
				CHECK_STATUS(ddr3_tip_if_read
					     (0, ACCESS_TYPE_UNICAST,
					      if_id,
					      RD_DATA_SMPL_DLYS_REG,
					      read_data, MASK_ALL_BITS));
				read_data[if_id] =
					(read_data[if_id] &
					 (0x1f << (8 * csindex))) >>
					(8 * csindex);
				ddr3_tip_bus_read(0, if_id,
						  ACCESS_TYPE_UNICAST, bus_id,
						  DDR_PHY_DATA,
						  RL_PHY_REG(csindex),
						  &reg_data);
				printf("%d,%d,%d,%d,",
				       (reg_data & 0x1f) +
				       ((reg_data & 0x1c0) >> 6) * 32 +
				       read_data[if_id] * 64,
				       (reg_data & 0x1f),
				       ((reg_data & 0x1c0) >> 6),
				       read_data[if_id]);
				/* centralization */
				ddr3_tip_bus_read(0, if_id,
						  ACCESS_TYPE_UNICAST, bus_id,
						  DDR_PHY_DATA,
						  CTX_PHY_REG(csindex),
						  &reg_data);
				printf("%d,", (reg_data & 0x3f));
				ddr3_tip_bus_read(0, if_id,
						  ACCESS_TYPE_UNICAST, bus_id,
						  DDR_PHY_DATA,
						  CRX_PHY_REG(csindex),
						  &reg_data);
				printf("%d,", (reg_data & 0x1f));
				/* vref */
				ddr3_tip_bus_read(0, if_id,
						  ACCESS_TYPE_UNICAST, bus_id,
						  DDR_PHY_DATA,
						  PAD_CFG_PHY_REG,
						  &reg_data);
				printf("%d,", (reg_data & 0xF));
				/* dq_vref */
				/* need to add the read function from device */
				printf("%d,", dq_vref_vec[bus_id]);
#if defined(CONFIG_DDR4)
				for (idx = 0; idx < 12; idx++) {
					ddr3_tip_bus_read(0, if_id,
							  ACCESS_TYPE_UNICAST,
							  bus_id, DDR_PHY_DATA,
							  0xd0 + 12 * csindex +
							  idx, &reg_data);
					printf("%d,", (reg_data & 0x3f));
				}
#endif /* CONFIG_DDR4 */
				for (idx = 0; idx < 12; idx++) {
					ddr3_tip_bus_read(0, if_id,
							  ACCESS_TYPE_UNICAST,
							  bus_id, DDR_PHY_DATA,
							  0xd0 +
							  12 * csindex +
							  idx, &reg_data);
					printf("%d,", (reg_data & 0x3f));
				}
				for (idx = 0; idx < 12; idx++) {
					ddr3_tip_bus_read(0, if_id,
							  ACCESS_TYPE_UNICAST,
							  bus_id, DDR_PHY_DATA,
							  0x10 +
							  16 * csindex +
							  idx, &reg_data);
					printf("%d,", (reg_data & 0x3f));
				}
				for (idx = 0; idx < 12; idx++) {
					ddr3_tip_bus_read(0, if_id,
							  ACCESS_TYPE_UNICAST,
							  bus_id, DDR_PHY_DATA,
							  0x50 +
							  16 * csindex +
							  idx, &reg_data);
					printf("%d,", (reg_data & 0x3f));
				}
			} /* end byte loop */
		} /* end cs loop */
	} /* end i/f loop */
	printf("\n");

	return MV_OK;
}

/*
 * support 32-/64-bit controllers
 * can run on the entire width or per byte of the controller
 * inputs:
 * start_addr_lo:	low 32-bit of the start address
 * start_addr_hi:	high 16-bit of the start address
 * end_addr_lo:		low 32-bit of the end address
 * end_addr_hi:		high 16-bit of the end address
 * access_num:		number of accesses to dram on each test
 * byte_num:		byte to access:
 *	options are: 0, 1, 2, 3, 4, 5, 6, 7, 8(ecc), 32, 64, 72
 * print_ena:		print debug notes
 * test_map:		each bit represent a test
 *	bit#: test name
 *	0: wr_to_rd: incr data, incr addr, access data width (32/64) each time and wr incremented number 0..255
 *	1: ww2rr: incr data (2nd data is inv), incr addr, access data width (32/64) each time and wr incremented
 *		number 0..255
 * return number of failed acceses
 */
static int memtest_per_byte(u32 start_addr_lo, u32 start_addr_hi, u32 end_addr_lo, u32 end_addr_hi,
			    u32 access_num, u32 byte_num, u32 print_ena, u32 test_map)
{
	uint64_t data = 0x0, data2 = 0x0, data_read = 0x0, data_read2 = 0x0, byte_mask = 0;
	uintptr_t addr = 0x0, addr2 = 0, start_addr = 0, end_addr;
	unsigned long long loc_area_len = area_len;
	uint64_t pownum;
	uint64_t idx;
	int cnt;
	int wr_to_rd = 0, rd_to_wr_to_rd = 0, walk_bit = 0, off_pat = 0;
	int shift_to_byte = byte_num * 8;
	int addr_sync = (data_width == 0x3) ? 4 : 8;
	int dev_byte_num;
	int loop_num, loop_idx;
	int iter, byte_idx, shift;
	int mux_reg = mmio_read_32(0xf00116d8);

	if (mux_reg == 0x3cc)
		loc_area_len = 0;

	start_addr = (uintptr_t)start_addr_lo | ((uintptr_t)start_addr_hi << 32);
	end_addr = (uintptr_t)end_addr_lo | ((uintptr_t)end_addr_hi << 32);
	byte_mask =  (uint64_t)((uint64_t)0xff << shift_to_byte);

	if (byte_num == 32 || byte_num == 64 || byte_num == 72) {
		shift_to_byte = 0;
		byte_mask = (byte_num == 32) ? 0xffffffff : 0xffffffffffffffff;
	}

#ifdef DBG_PRINT
	printf("%s: byte_mask 0x%16jx\n", __func__, byte_mask);
#endif

	if ((test_map & 0x1) == 0x1) {
		if (print_ena) {
#ifdef DBG_PRINT
			printf("\n");
#endif
		}
		for (idx = 0; idx < access_num; idx += 1) {
			addr = idx * addr_sync + start_addr;
			if (byte_num == 64 || byte_num == 72) {
				data = ((idx % 256) << 56) |
				       ((idx % 256) << 48) |
				       ((idx % 256) << 40) |
				       ((idx % 256) << 32) |
				       ((idx % 256) << 24) |
				       ((idx % 256) << 16) |
				       ((idx % 256) << 8) |
				       (idx % 256);
			} else if (byte_num == 32) {
				data = ((idx % 256) << 24) |
				       ((idx % 256) << 16) |
				       ((idx % 256) << 8) |
				       (idx % 256);
			} else {
				data = (idx % 256) << shift_to_byte;
			}
#ifdef DBG_PRINT
			printf("addr 0x%lx, data: 0x%lx\n", addr, data);
#endif
			cpu_write(loc_area_len * effective_cs + addr, data);
			data_read = cpu_read(loc_area_len * effective_cs + addr);

			if ((data_read & byte_mask) != (data & byte_mask)) {
				if (print_ena) {
#ifdef DBG_PRINT
					printf("wr_to_rd: incr addr 0x%16jx, rd 0x%16jx, expected 0x%16jx\n",
					       addr, data_read, data);
#endif
				}
				wr_to_rd++;
			}
		}
	}

	if ((test_map & 0x2) == 0x2) {
		if (print_ena) {
#ifdef DBG_PRINT
			printf("\n");
#endif
		}

		for (idx = 0; idx < access_num; idx += 1) {
			addr = idx * addr_sync + start_addr;
			if (byte_num == 64 || byte_num == 72) {
				data = ((idx % 256) << 56) |
				       ((idx % 256) << 48) |
				       ((idx % 256) << 40) |
				       ((idx % 256) << 32) |
				       ((idx % 256) << 24) |
				       ((idx % 256) << 16) |
				       ((idx % 256) << 8) |
				       (idx % 256);
			} else if (byte_num == 32) {
				data = ((idx % 256) << 24) |
				       ((idx % 256) << 16) |
				       ((idx % 256) << 8) |
				       (idx % 256);
			} else {
				data = (idx % 256) << shift_to_byte;
			}

			cpu_write(loc_area_len * effective_cs + addr, data);
			cpu_write(loc_area_len * effective_cs + addr + addr_sync, ~data);
			data_read = cpu_read(loc_area_len * effective_cs + addr);
			data_read2 = cpu_read(loc_area_len * effective_cs + addr + addr_sync);

			if (((data_read & byte_mask) != (data & byte_mask)) ||
			    ((data_read2 & byte_mask) != (~data & byte_mask))) {
				if (print_ena) {
#ifdef DBG_PRINT
					printf("ww2rr: addr 0x%16jx, rd 0x%16jx, expected 0x%16jx, "
					       "rd 0x%16jx, expected 0x%16jx\n",
					       addr, data_read, data, data_read2, ~data);
#endif
				}
				wr_to_rd++;
			}
		}
	}

	if ((test_map & 0x4) == 0x4) {
		if (print_ena) {
#ifdef DBG_PRINT
			printf("\n");
#endif
		}

		for (idx = 0; idx < access_num; idx += 1) {
			addr = idx * addr_sync * 72;
			addr2 = (addr * addr) % (end_addr - start_addr);
			addr += start_addr;
			addr2 += start_addr;
			data = cpu_read(loc_area_len * effective_cs + addr);
			cpu_write(loc_area_len * effective_cs + addr2, data);
			data_read = cpu_read(loc_area_len * effective_cs + addr2);

			if ((data_read & byte_mask) != (data & byte_mask)) {
				if (print_ena) {
#ifdef DBG_PRINT
				printf("rd_to_wr_to_rd: rand addr: 0x%16jx, rd 0x%16jx, expected 0x%16jx\n",
				       addr2, data_read, data);
#endif
				}
				rd_to_wr_to_rd++;
			}
		}
	}

	/* mix between walking one and walking zero */
	if ((test_map & 0x8) == 0x8) {
		if (print_ena) {
#ifdef DBG_PRINT
			printf("\n");
#endif
		}

		addr = end_addr; /* start addr */
		data = 0xffffffffffffffff; /* init data */
		data2 = 0x0;
		pownum = 1;
		cnt = 0;
		for (idx = 0; idx < access_num; idx += 1) {
			addr2 = addr - idx * addr_sync;
			if (idx % 2 == 0) {
				data2 = ~data2;
			} else if (idx % 2 == 1) {
				data2 = data - pownum;
				pownum *= 2;
				cnt++;
				if (cnt == 64)
					pownum = 1;
			} else {
				data2 = ~data2;
			}
			cpu_write(loc_area_len * effective_cs + addr2, data2);
			data_read = cpu_read(loc_area_len * effective_cs + addr2);

			if ((data_read & byte_mask) != (data2 & byte_mask)) {
				if (print_ena) {
#ifdef DBG_PRINT
					printf("walking 1: addr 0x%16jx, rd 0x%16jx, expected 0x%16jx\n",
					       addr2, data_read, data2);
#endif
				}
				walk_bit++;
			}
		}
	}

	if ((test_map & 0x10) == 0x10) {
		dev_byte_num = (data_width == 0x3) ? 4 : 8;
		loop_num = access_num;
		for (loop_idx = 0; loop_idx < loop_num; loop_idx++) {
			if (loop_idx == 0)
				data = 127;
			else
				data = data2 & 0xff;
			if ((loop_idx % 3 == 0) || (loop_idx % 3 == 1))
				data = 255 - data; /* inv */
			else
				data = (data == 0xfe) ? 127 : (128 + data / 2);

			data2 = 0x0;
			shift = 0;
			for (byte_idx = 0; byte_idx < dev_byte_num; byte_idx++) {
				shift += 8 * byte_idx;
				data2 |= (data % 256) << shift;
			}
			cpu_write(addr, data2);
			data_read = 0;
			for (iter = 0; iter < loop_num; iter++)
				data_read |= cpu_read(addr);
			if ((data_read & byte_mask) != (data2 & byte_mask)) {
				if (print_ena) {
#ifdef DBG_PRINT
					printf("off pattern: addr 0x%16jx, rd 0x%16jx, expected 0x%16jx\n",
					       addr2, data_read, data2);
#endif
				}
				off_pat++;
			}
			addr += dev_byte_num;
		}
	}

	if (byte_num == 8)
		return  mmio_read_32(0xf0020364); /* return ecc error */
	else
		return wr_to_rd + rd_to_wr_to_rd + walk_bit + off_pat;
}

/*
 * repeat:	number of test repetitions
 * test_len:	length of the memory test; min is 0x10
 * dir:		0 - tx, 1 - rx
 * mode:	opt; subphy num or 0xff for all subphys
 */
int cpu_sweep_test(u32 repeat, u32 test_len, u32 dir, u32 mode)
{
	/*
	 * test_len:	0 - CPU only,
	 *		1 - xor 128M at 128K,
	 *		2 - partial write 1M at 1K + 1/2/3,
	 *		3 - xor 1G at 16M
	 */

	u32 sphy = 0, start_sphy = 0, end_sphy = 0, duty_cycle = 0;
	u32 adll = 0;
	int print_ena = 0;
	u32 res = 0;
	u32 vector_vtap[MAX_BUS_NUM] = {0};
	int duty_cycle_idx;
	u32 adll_value = 0;
	int reg = (dir == 0) ? CTX_PHY_BASE : CRX_PHY_BASE;
	int step = 3;
	int max_v = (dir == 0) ? 73 : 63;
	enum hws_access_type sphy_access;
	u32 octets_per_if_num = ddr3_tip_dev_attr_get(0, MV_ATTR_OCTET_PER_INTERFACE);
	u32 adll_reg_val, vtap_reg_val;
	int mux_reg;

	if (mode == 0xff) {
		/* per sphy */
		start_sphy = 0;
		end_sphy = octets_per_if_num - 1;
		sphy_access = ACCESS_TYPE_UNICAST;
	} else if (mode == 0x1ff) {
		/* per sphy */
		start_sphy = 0;
		end_sphy = octets_per_if_num;
		sphy_access = ACCESS_TYPE_UNICAST;
	} else { /* TODO: need to check mode for valid sphy number */
		start_sphy = mode;
		end_sphy = mode + 1;
		sphy_access = ACCESS_TYPE_UNICAST;
	}

	if ((ddr3_if_ecc_enabled() == 1) && (start_sphy != 8)) {
		reg_bit_clrset(MC6_RAS_CTRL_REG, 0x0 << ECC_EN_OFFS, ECC_EN_MASK << ECC_EN_OFFS);
		reg_write(MC6_CH0_ECC_1BIT_ERR_COUNTER_REG, 0x0);
	}

	ddr3_tip_if_write(0, ACCESS_TYPE_MULTICAST, PARAM_NOT_CARE, ODPG_DATA_CTRL_REG,
			  effective_cs << ODPG_DATA_CS_OFFS,
			  ODPG_DATA_CS_MASK << ODPG_DATA_CS_OFFS);

	mux_reg = mmio_read_32(0xf00116d8);
	for (sphy = start_sphy; sphy < end_sphy; sphy++) {
		if (dir == 1) { /* dir 0 saved in dq_vref_vec array */
			ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST, sphy, DDR_PHY_DATA,
					  VREF_BCAST_PHY_REG(effective_cs), &vtap_reg_val);
			vector_vtap[sphy] = vtap_reg_val;
		}

		/* save adll position and voltage */
		ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST, start_sphy, DDR_PHY_DATA,
				  reg + effective_cs * 0x4, &adll_reg_val);
		printf("byte %d, nominal adll %d, rc %d\n",
		       sphy, adll_reg_val, vtap_reg_val);
		for (duty_cycle = 0; duty_cycle < max_v; duty_cycle += step) {
			wr_xor_pat = 1; /* 1 - write, 0 - skip */
			xor_src_to_dst = 1; /* 1 - do, 0 - skip */
			if (dir == 0) {
				mmio_write_32(0xf00116d8, 0x3cc);
				duty_cycle_idx = duty_cycle;
				/* insert dram to vref training mode */
				mv_ddr4_vref_training_mode_ctrl(0, 0, ACCESS_TYPE_MULTICAST, 1);
				/* set new vref training value in dram */
				mv_ddr4_vref_tap_set(0, 0, ACCESS_TYPE_MULTICAST, duty_cycle_idx,
						     MV_DDR4_VREF_TAP_START);
				/* close vref range */
				mv_ddr4_vref_tap_set(0, 0, ACCESS_TYPE_MULTICAST, duty_cycle_idx,
						     MV_DDR4_VREF_TAP_END);
				/* set mux to mc6 */
				mmio_write_32(0xf00116d8, mux_reg);
			} else {
				/* set new receiver dc training value in dram */
				ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
						    VREF_BCAST_PHY_REG(effective_cs), duty_cycle);
				ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
						   VREF_PHY_REG(effective_cs, 4), duty_cycle);
				ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
						   VREF_PHY_REG(effective_cs, 5), duty_cycle);
			}

			for (adll = 0; adll < ADLL_LENGTH; adll++) {
				res = 0;
				adll_value = (dir == 0) ? (adll * 2) : adll;
				ddr3_tip_bus_write(0, ACCESS_TYPE_MULTICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
						   reg + effective_cs * 0x4, adll_value);
				res += memtest_per_byte(0x0, 0x0, _2M, 0, 0x10, sphy, print_ena, 0xf);
				if ((res == 0) && (test_len >= 0)) {
					res += memtest_per_byte(0x0, 0x0, _2M, 0, 0x100, sphy, print_ena, 0x10);
					if ((res == 0) && (test_len >= 1)) {
						/* in case cpu test passes, run xor test */
						res += xor_gradual_test(test_len);
					}
				}

				if (sphy == 8) {
					printf("########## subphy: %d #############\n", sphy);
					/* need to recover ecc value */
					if (dir == 1) {
						ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy,
								   DDR_PHY_DATA,
								   VREF_BCAST_PHY_REG(effective_cs),
								   vector_vtap[sphy]);
						ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy,
								   DDR_PHY_DATA,
								   VREF_PHY_REG(effective_cs, 4),
								   vector_vtap[sphy]);
						ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy,
								   DDR_PHY_DATA,
								   VREF_PHY_REG(effective_cs, 5),
								   vector_vtap[sphy]);
					} else {
						mmio_write_32(0xf00116d8, 0x3cc);
						/* insert dram to vref training mode */
						mv_ddr4_vref_training_mode_ctrl(0, 0, ACCESS_TYPE_MULTICAST, 1);
						/* set new vref training value in dram */
						mv_ddr4_vref_tap_set(0, 0, ACCESS_TYPE_MULTICAST, dq_vref_vec[sphy],
								     MV_DDR4_VREF_TAP_START);
						/* close vref range */
						mv_ddr4_vref_tap_set(0, 0, ACCESS_TYPE_MULTICAST, dq_vref_vec[sphy],
								     MV_DDR4_VREF_TAP_END);
						/* set mux to mc6 */
						mmio_write_32(0xf00116d8, 0x38c);
					}
					ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
							   reg + effective_cs * 0x4, adll_reg_val);
					/* re-run tests */
					reg_bit_clrset(MC6_RAS_CTRL_REG, 0x8, 0x8); /* ignore ecc */
					memtest_per_byte(0x0, 0x0, _1G, 0, 0x10, sphy, print_ena, 0xf);
					if (test_len >= 0) {
						memtest_per_byte(0x0, 0x0, _1G, 0, 0x4000, sphy, print_ena, 0xf);
						if (test_len >= 1) {
							/* in case cpu test passes, run xor test */
							xor_test(_1K, _128M, 0x0, _1G, 0x0, _128K,
								 0x0, 0x0, 0x0, OSTD_RD_NUM);
							if (test_len >= 2) {
								xor_test(_1M, _1M, 0x0, _1G, 0x0, _1K + 1,
									 0x1, 0x0, 0x0, OSTD_RD_NUM);
								xor_test(_1M, _1M, 0x0, _1G, 0x0, _1K + 2,
									 0x2, 0x0, 0x0, OSTD_RD_NUM);
								xor_test(_1M, _1M, 0x0, _1G, 0x0, _1K + 3,
									 0x4, 0x0, 0x0, OSTD_RD_NUM);
								if (test_len >= 3) {
									xor_test(_256M, _1G, 0x0, _2G, 0x0, _16M,
										 0x1, 0x0, 0x0, OSTD_RD_NUM);
								}
							}
						}
					}
					reg_bit_clrset(MC6_RAS_CTRL_REG, 0x0, 0x8); /* ignore ecc */
					/* reset vref value */
					if (dir == 1) {
						ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy,
								   DDR_PHY_DATA,
								   VREF_BCAST_PHY_REG(effective_cs),
								   duty_cycle);
						ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy,
								   DDR_PHY_DATA,
								   VREF_PHY_REG(effective_cs, 4),
								   duty_cycle);
						ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy,
								   DDR_PHY_DATA,
								   VREF_PHY_REG(effective_cs, 5),
								   duty_cycle);
					} else {
						mmio_write_32(0xf00116d8, 0x3cc);
						duty_cycle_idx = duty_cycle;
						/* insert dram to vref training mode */
						mv_ddr4_vref_training_mode_ctrl(0, 0, ACCESS_TYPE_MULTICAST, 1);
						/* set new vref training value in dram */
						mv_ddr4_vref_tap_set(0, 0, ACCESS_TYPE_MULTICAST, duty_cycle_idx,
								     MV_DDR4_VREF_TAP_START);
						/* close vref range */
						mv_ddr4_vref_tap_set(0, 0, ACCESS_TYPE_MULTICAST, duty_cycle_idx,
								     MV_DDR4_VREF_TAP_END);
						/* set mux to mc6 */
						mmio_write_32(0xf00116d8, mux_reg);
					}
					reg_write(MC6_CH0_ECC_1BIT_ERR_COUNTER_REG, 0x0);
				} /* ecc re-fix */

				if (dir == 0) {
					if (((dq_vref_vec[sphy] <= duty_cycle) &&
					     (dq_vref_vec[sphy] >= (duty_cycle - step))) &&
					    (adll_reg_val >= adll_value) &&
					    (adll_reg_val <= (adll_value + 2))) {
						res = -1;
					}
				} else {
					if (((vtap_reg_val <= duty_cycle) &&
					     (vtap_reg_val >= (duty_cycle - step))) &&
					    (adll_reg_val == adll)) {
						res = -1;
					}
				}
				printf("%d, ", res);
			}
			printf("\n");
		} /* end of dutye cycle */
		if (dir == 1) {
			ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
					   VREF_BCAST_PHY_REG(effective_cs), vector_vtap[sphy]);
			ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
					   VREF_PHY_REG(effective_cs, 4), vector_vtap[sphy]);
			ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
					   VREF_PHY_REG(effective_cs, 5), vector_vtap[sphy]);
		} else {
			duty_cycle_idx = duty_cycle;
			/* insert dram to vref training mode */
			mv_ddr4_vref_training_mode_ctrl(0, 0, ACCESS_TYPE_MULTICAST, 1);
			/* set new vref training value in dram */
			mv_ddr4_vref_tap_set(0, 0, ACCESS_TYPE_MULTICAST, dq_vref_vec[sphy], MV_DDR4_VREF_TAP_START);
			/* close vref range*/
			mv_ddr4_vref_tap_set(0, 0, ACCESS_TYPE_MULTICAST, dq_vref_vec[sphy],  MV_DDR4_VREF_TAP_END);
		}
		ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
				   reg + effective_cs * 0x4, adll_reg_val);
	}

	/* rewrite adll and voltage nominal values */
	ddr3_tip_bus_write(0, ACCESS_TYPE_MULTICAST, 0, sphy_access, end_sphy - 1, DDR_PHY_DATA,
			   reg + effective_cs * 0x4, adll_reg_val);
	if ((ddr3_if_ecc_enabled() == 1) && (start_sphy != 8)) {
		reg_write(MC6_CH0_ECC_1BIT_ERR_COUNTER_REG, 0x0);
		reg_bit_clrset(MC6_RAS_CTRL_REG, 0x1 << ECC_EN_OFFS, ECC_EN_MASK << ECC_EN_OFFS);
	}
	ddr3_tip_reset_fifo_ptr(0);

	return 0;
}

static int vertical_adjust(u8 depth_stage, int ena_mask, u8 byte_num)
{
	u8 sphy = 0, start_sphy = 0, end_sphy = 0;
	u8 vw[2];
	int print_ena = 0;
	int step = (ena_mask == 0) ? 2 : 4;
	int opt_rc = 0;
	int fpf;
	int result;
	enum hws_access_type sphy_access;
	u32 reg_data = 0;
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	u32 octets_per_if_num = ddr3_tip_dev_attr_get(0, MV_ATTR_OCTET_PER_INTERFACE);

	printf(".");

	if (byte_num == 0xf) { /* 0xf - all bytes */
		start_sphy = 0;
		end_sphy = octets_per_if_num - 1;
	} else {
		start_sphy = byte_num;
		end_sphy = byte_num + 1;
	}
	sphy_access = ACCESS_TYPE_UNICAST;
	if ((ddr3_if_ecc_enabled() == 1) && (start_sphy != 8)) {
		reg_bit_clrset(MC6_RAS_CTRL_REG, 0x0 << ECC_EN_OFFS, ECC_EN_MASK << ECC_EN_OFFS);
		reg_write(MC6_CH0_ECC_1BIT_ERR_COUNTER_REG, 0x0);
	}

	ddr3_tip_if_write(0, ACCESS_TYPE_MULTICAST, PARAM_NOT_CARE, ODPG_DATA_CTRL_REG,
			  effective_cs << ODPG_DATA_CS_OFFS, ODPG_DATA_CS_MASK << ODPG_DATA_CS_OFFS);

	for (sphy = start_sphy; sphy < end_sphy; sphy++) {
		VALIDATE_BUS_ACTIVE(tm->bus_act_mask, sphy);
		if (print_ena == 1) {
			ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST, sphy, DDR_PHY_DATA,
					  VREF_BCAST_PHY_REG(effective_cs), &reg_data);
#ifdef DBG_PRINT
			printf("cs[%d]: byte %d rc %d [%d,%d] ->",
			       effective_cs, sphy, reg_data, rx_eye_lo_lvl[sphy], rx_eye_hi_lvl[sphy]);
#endif
		}
		if (ena_mask == PER_IF) {
			xor_byte_mask = 0;
			mask = OR_MASK;
		} else if (ena_mask == PER_BYTE_RES1) {
			xor_byte_mask = (unsigned long long)0x0 | ((unsigned long long)0xff << (sphy * 8));
			xor_byte_mask = ~xor_byte_mask;
			mask = OR_MASK;
		} else if (ena_mask == PER_BYTE_RES0) {
			xor_byte_mask = (unsigned long long)0x0 | ((unsigned long long)0xff << (sphy * 8));
			mask = AND_MASK;
		} else if (ena_mask == PER_DQ0) { /* only dq0 toggles */
			xor_byte_mask = (unsigned long long)0x0 | ((unsigned long long)0x1 << (sphy * 8));
			xor_byte_mask = ~xor_byte_mask;
			mask = OR_MASK;
		}

		result = xor_gradual_test(depth_stage);
		if (result == 0) {
			fpf = xor_search_1d_2e(EDGE_PF, step, rx_eye_lo_lvl[sphy], rx_eye_hi_lvl[sphy],
					       depth_stage, sphy, REC_CAL, vw);
		} else {
			printf("%s: failed on first try (fail to pass)\n", __func__);
			fpf = xor_search_1d_2e(EDGE_FP, step, rx_eye_lo_lvl[sphy], rx_eye_hi_lvl[sphy],
					       depth_stage, sphy, REC_CAL, vw);
		}

		if (fpf != 255) {
			opt_rc = 0.5 * ((int)vw[1] + (int)vw[0]);
		} else {
			if (print_ena == 1) {
#ifdef DBG_PRINT
				printf("%s: byte %d - no lock\n", __func__, sphy);
#endif
			}
			ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
					   VREF_BCAST_PHY_REG(effective_cs), reg_data);
			ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
					   VREF_PHY_REG(effective_cs, 4), reg_data);
			ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
					   VREF_PHY_REG(effective_cs, 5), reg_data);

			return 1;
		}

#ifdef DBG_PRINT
		printf("%s: opt rc %d, vw0 %d, vw1 %d\n", __func__, opt_rc, vw[0], vw[1]);
#endif
		ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
				   VREF_BCAST_PHY_REG(effective_cs), opt_rc);
		ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
				   VREF_PHY_REG(effective_cs, 4), opt_rc);
		ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
				   VREF_PHY_REG(effective_cs, 5), opt_rc);
	} /* end of sphy */

	ddr3_tip_reset_fifo_ptr(0);

	return 0;
}

static int horizontal_adjust(u8 depth_stage, int ena_mask, u8 byte_num)
{
	u32 sphy = 0, start_sphy = 0, end_sphy = 0;
	u8 vw[2];
	int print_ena = 0;
	int step = (ena_mask == 0) ? 1 : 2;
	int opt_crx = 0;
	int fpf;
	int result;
	enum hws_access_type sphy_access;
	u32 reg_data;
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	u32 octets_per_if_num = ddr3_tip_dev_attr_get(0, MV_ATTR_OCTET_PER_INTERFACE);

	printf(".");

	if (byte_num == 0xf) { /* 0xf - all bytes */
		start_sphy = 0;
		end_sphy = octets_per_if_num - 1;
	} else {
		start_sphy = byte_num;
		end_sphy = byte_num + 1;
	}
	sphy_access = ACCESS_TYPE_UNICAST;
	if ((ddr3_if_ecc_enabled() == 1) && (start_sphy != 8)) {
		reg_bit_clrset(MC6_RAS_CTRL_REG, 0x0 << ECC_EN_OFFS, ECC_EN_MASK << ECC_EN_OFFS);
		reg_write(MC6_CH0_ECC_1BIT_ERR_COUNTER_REG, 0x0);
	}

	ddr3_tip_if_write(0, ACCESS_TYPE_MULTICAST, PARAM_NOT_CARE, ODPG_DATA_CTRL_REG,
			  effective_cs << ODPG_DATA_CS_OFFS, ODPG_DATA_CS_MASK << ODPG_DATA_CS_OFFS);

	for (sphy = start_sphy; sphy < end_sphy; sphy++) {
		VALIDATE_BUS_ACTIVE(tm->bus_act_mask, sphy);
		if (print_ena == 1) {
			ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST, sphy, DDR_PHY_DATA,
					  CRX_PHY_REG(effective_cs), &reg_data);
#ifdef DBG_PRINT
			printf("cs[%d]: byte %d crx %d ->", effective_cs, sphy, reg_data);
#endif
		}

		if (ena_mask == PER_IF) {
			xor_byte_mask = 0;
			mask = OR_MASK;
		} else if (ena_mask == PER_BYTE_RES1) {
			xor_byte_mask = (unsigned long long)0x0 | ((unsigned long long)0xff << (sphy * 8));
			xor_byte_mask = ~xor_byte_mask;
			mask = OR_MASK;
		} else if (ena_mask == PER_BYTE_RES0) {
			xor_byte_mask = (unsigned long long)0x0 | ((unsigned long long)0xff << (sphy * 8));
			mask = AND_MASK;
		} else if (ena_mask == PER_DQ0) { /* only dq0 toggles */
			xor_byte_mask = (unsigned long long)0x0 | ((unsigned long long)0x01 << (sphy * 8));
			xor_byte_mask = ~xor_byte_mask;
			mask = OR_MASK;
		}

		result = xor_gradual_test(depth_stage);
		if (result == 0) {
			fpf = xor_search_1d_2e(EDGE_PF, step, 0, 31, depth_stage, sphy, CRX, vw);
		} else {
			printf("%s: failed on first try (fail to pass)\n", __func__);
			fpf = xor_search_1d_2e(EDGE_FP, step, 0, 31, depth_stage, sphy, CRX, vw);
		}

		if (fpf != 255) {
			opt_crx = 0.5 * ((int)vw[1] + (int)vw[0]);
		} else {
			if (print_ena == 1) {
#ifdef DBG_PRINT
				printf("%s: byte %d - no lock\n", __func__, sphy);
#endif
			}

			return 1;
		}

#ifdef DBG_PRINT
		printf("%s: opt crx %d, vw0 %d, vw1 %d\n", opt_crx, vw[0], vw[1]);
#endif
		ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
				   CRX_PHY_REG(effective_cs), opt_crx);
	} /* end of sphy */

	ddr3_tip_reset_fifo_ptr(0);

	return 0;
}

static int diagonal_adjust(u8 depth_stage, int ena_mask, u8 byte_num)
{
	u32 sphy = 0, start_sphy = 0, end_sphy = 0;
	u32 nom_adll = 0, nom_rc = 0, tmp_nom_adll = 0, tmp_nom_rc = 0;
	u32 step1, init_val1, end_val1, step2, init_val2, end_val2;
	u8 vw[2];
	enum hws_access_type sphy_access;
	u32 octets_per_if_num = ddr3_tip_dev_attr_get(0, MV_ATTR_OCTET_PER_INTERFACE);
	int step;
	int final;

	printf(".");

	if (byte_num == 0xf) { /* 0xf - all bytes */
		start_sphy = 0;
		end_sphy = octets_per_if_num - 1;
	} else {
		start_sphy = byte_num;
		end_sphy = byte_num + 1;
	}
	sphy_access = ACCESS_TYPE_UNICAST;
	if ((ddr3_if_ecc_enabled() == 1) && (start_sphy != 8)) {
		reg_bit_clrset(MC6_RAS_CTRL_REG, 0x0 << ECC_EN_OFFS, ECC_EN_MASK << ECC_EN_OFFS);
		reg_write(MC6_CH0_ECC_1BIT_ERR_COUNTER_REG, 0x0);
	}
	ddr3_tip_if_write(0, ACCESS_TYPE_MULTICAST, PARAM_NOT_CARE, ODPG_DATA_CTRL_REG,
			  effective_cs << ODPG_DATA_CS_OFFS, ODPG_DATA_CS_MASK << ODPG_DATA_CS_OFFS);

	for (sphy = start_sphy; sphy < end_sphy; sphy++) {
		if (ena_mask == PER_IF) {
			xor_byte_mask = 0;
			mask = OR_MASK;
		} else if (ena_mask == PER_BYTE_RES1) {
			xor_byte_mask = (unsigned long long)0x0 | ((unsigned long long)0xff << (sphy * 8));
			xor_byte_mask = ~xor_byte_mask;
			mask = OR_MASK;
		} else if (ena_mask == PER_BYTE_RES0) {
			xor_byte_mask = (unsigned long long)0x0 | ((unsigned long long)0xff << (sphy * 8));
			mask = AND_MASK;
		} else if (ena_mask == PER_DQ0) { /* only dq0 toggles */
			xor_byte_mask = (unsigned long long)0x0 | ((unsigned long long)0x01 << (sphy * 8));
			xor_byte_mask = ~xor_byte_mask;
			mask = OR_MASK;
		}

		if (sphy == 8) {
#ifdef DBG_PRINT
			printf("%s: ecc byte: xor byte mask 0x%llx\n", __func__, xor_byte_mask);
#endif
		}

		/* read nominal start point */
		ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST, sphy, DDR_PHY_DATA,
				  CRX_PHY_REG(effective_cs), &nom_adll);
		ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST, sphy, DDR_PHY_DATA,
				  VREF_BCAST_PHY_REG(effective_cs), &nom_rc);
		/* define slope of the search */
		step1 = 1; /* adll step */
		step2 = 3; /* rc step */
		/*
		 * diag search limits are unknown
		 * the limits will be takne per adll-rc plane limits
		 * the step is the minimum between adll and rc steps
		 */
		step = ((nom_adll / step1) > (nom_rc / step2)) ? (nom_rc / step2) : (nom_adll / step1);
		init_val1 = nom_adll - step * step1;
		init_val2 = nom_rc - step * step2;
		step = (((31 - nom_adll) / step1) > ((63 - nom_rc) / step2)) ?
		       ((63 - nom_rc) / step2) : ((31 - nom_adll) / step1);
		end_val1 = nom_adll + step * step1;
		end_val2 = nom_rc + step * step2;
		if (ena_mask == 0) {
			xor_byte_mask = 0;
		} else {
			xor_byte_mask = (unsigned long long)0x0 | ((unsigned long long)0xff << (sphy * 8));
#ifdef DBG_PRINT
			printf("%s: xor byte mask = 0x%llx\n", __func__, xor_byte_mask);
#endif
			xor_byte_mask = ~xor_byte_mask;
#ifdef DBG_PRINT
			printf("%s: xor byte mask = 0x%llx\n", __func__, xor_byte_mask);
#endif
		}
		final = xor_search_2d_2e(EDGE_FPF, CRX,
					 HWS_LOW2HIGH, step1, init_val1, end_val1, REC_CAL,
					 HWS_LOW2HIGH, step2, init_val2, end_val2, depth_stage, sphy, vw);

		if (final != 255) {
			/* vw contains a number of steps done per search */
			tmp_nom_adll = 0.5 * ((vw[0] * step1 + init_val1) + (end_val1 - vw[1] * step1));
			ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
					   CRX_PHY_REG(effective_cs), tmp_nom_adll);
			tmp_nom_rc = 0.5 * ((vw[0] * step2 + init_val2) + (end_val2 - vw[1] * step2));
			ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
					   VREF_BCAST_PHY_REG(effective_cs), tmp_nom_rc);
			ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
					   VREF_PHY_REG(effective_cs, 4), tmp_nom_rc);
			ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
					   VREF_PHY_REG(effective_cs, 5), tmp_nom_rc);
#ifdef DBG_PRINT
			printf("cs[%d]: byte %d: first xor_search_2d_2e [%d, %d]->[%d,%d]\n",
			       effective_cs, sphy, nom_adll, nom_rc, tmp_nom_adll, tmp_nom_rc);
#endif
		} else {
			printf("%s: cs[%d]: byte %d: first xor_search_2d_2e failed\n",
			       __func__, effective_cs, sphy);
		}
	}

	for (sphy = start_sphy; sphy < end_sphy; sphy++) {
		ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST, sphy, DDR_PHY_DATA,
				  CRX_PHY_REG(effective_cs), &nom_adll);
		ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST, sphy, DDR_PHY_DATA,
				  VREF_BCAST_PHY_REG(effective_cs), &nom_rc);
		step1 = 1;
		step2 = 3;
		step = ((nom_adll / step1) > ((63 - nom_rc) / step2)) ? ((63 - nom_rc) / step2) : (nom_adll / step1);
		init_val1 = nom_adll - step * step1;
		init_val2 = nom_rc + step * step2;
		step = (((31 - nom_adll) / step1) > (nom_rc / step2)) ? (nom_rc / step2) : ((31 - nom_adll) / step1);
		end_val1 = nom_adll + step * step1;
		end_val2 = nom_rc - step * step2;

		if (ena_mask == 0) {
			xor_byte_mask = 0;
		} else {
			xor_byte_mask = (unsigned long long)0x0 | ((unsigned long long)0xff << (sphy * 8));
#ifdef DBG_PRINT
			printf("%s: xor byte mask = 0x%llx\n", __func__, xor_byte_mask);
#endif
			xor_byte_mask = ~xor_byte_mask;
#ifdef DBG_PRINT
			printf("%s: xor byte mask = 0x%llx\n", __func__, xor_byte_mask);
#endif
		}
		final = xor_search_2d_2e(EDGE_FPF, CRX,
					 HWS_LOW2HIGH, step1, init_val1, end_val1, REC_CAL,
					 HWS_HIGH2LOW, step2, init_val2, end_val2, depth_stage, sphy, vw);
#ifdef DBG_PRINT
		printf("%s: xor_search_2d_2e solution: %d [%d, %d]\n",
		       __func__, final, vw[0], vw[1]);
#endif
		if (final != 255) {
			tmp_nom_adll = 0.5 * ((vw[0] * step1 + init_val1) + (end_val1 - vw[1] * step1));
			ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
					   CRX_PHY_REG(effective_cs), tmp_nom_adll);
			tmp_nom_rc = 0.5 * ((init_val2 - vw[0] * step2) + (end_val2 + vw[1] * step2));
			ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
					   VREF_BCAST_PHY_REG(effective_cs), tmp_nom_rc);
			ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
					   VREF_PHY_REG(effective_cs, 4), tmp_nom_rc);
			ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, sphy_access, sphy, DDR_PHY_DATA,
					   VREF_PHY_REG(effective_cs, 5), tmp_nom_rc);
#ifdef DBG_PRINT
			printf("cs[%d]: byte %d: second xor_search_2d_2e [%d, %d]->[%d, %d]\n",
			       effective_cs, sphy, nom_adll, nom_rc, tmp_nom_adll, tmp_nom_rc);
#endif
		} else {
			printf("%s: cs[%d]: byte %d: second xor_search_2d_2e failed\n",
			       __func__, effective_cs, sphy);
		}
	}

	ddr3_tip_reset_fifo_ptr(0);

	return 0;
}

static int rx_adjust(void)
{
	int dbg_flag = 1;
	enum mask_def ena_mask = PER_BYTE_RES1;
	enum mask_def start_mask = PER_IF;
	enum search_type search = VERTICAL;
	int result = 1;
	int alg_loop = 0;
	u8 depth_stage = 2;
	u32 octets_per_if_num = ddr3_tip_dev_attr_get(0, MV_ATTR_OCTET_PER_INTERFACE);
	int start_byte = 0;
	int end_byte = octets_per_if_num;
	int byte;

	/* check i/f severity */
	data_width = 4;
	xor_byte_mask = 0;
	mask = OR_MASK;
	result = xor_gradual_test(depth_stage);
	if (result == 0) {
		start_mask = PER_IF;
		/* check worst case severity */
		depth_stage = 3;
		result = xor_gradual_test(depth_stage);
	} else {
		start_mask = PER_BYTE_RES1;
	}

	if (result != 0) { /* depth_stage = 3 failed */
#ifdef DBG_PRINT
		printf("%s: going to v-h first level search\n", __func__);
#endif
		for (byte = start_byte; byte < end_byte; byte++) {
			if ((ddr3_if_ecc_enabled() == 1) && (byte != 8)) {
				reg_bit_clrset(MC6_RAS_CTRL_REG, 0x0 << ECC_EN_OFFS, ECC_EN_MASK << ECC_EN_OFFS);
				reg_write(MC6_CH0_ECC_1BIT_ERR_COUNTER_REG, 0x0);
			}
			depth_stage = 2;
			search = VERTICAL;
			ena_mask = start_mask;

			for (alg_loop = 0; alg_loop < 4; alg_loop++) {
				if (search == VERTICAL)
					result = vertical_adjust(depth_stage, ena_mask, byte);
				else
					result = horizontal_adjust(depth_stage, ena_mask, byte);
				if (byte == 8) {
					reg_bit_clrset(MC6_RAS_CTRL_REG, 0x8, 0x8); /* ignore ecc */
					reg_bit_clrset(MC6_RAS_CTRL_REG, 0x0, 0x8); /* ignore ecc */
					reg_write(MC6_CH0_ECC_1BIT_ERR_COUNTER_REG, 0x0);
				}
				switch (alg_loop) {
				case 0:
					if (result == 0) { /* pass; go to horizontal search */
						search = HORIZONTAL;
						if (dbg_flag == 2) {
#ifdef DBG_PRINT
							printf("%s: byte %d, alg loop %d: pass\n",
							       __func__, byte, alg_loop);
#endif
						}
						alg_loop = 2;
						continue;
					} else { /* failed; try another mask */
						ena_mask = PER_BYTE_RES0;
						if (dbg_flag == 2) {
#ifdef DBG_PRINT
							printf("%s: byte %d, alg loop %d: failed\n",
							       __func__, byte, alg_loop);
#endif
						}
					}
					break;
				case 1:
					if (result == 0) { /* pass; go to horizontal search */
						search = HORIZONTAL;
						if (dbg_flag == 2) {
#ifdef DBG_PRINT
							printf("%s: byte %d, alg loop %d: pass\n",
							       __func__, byte, alg_loop);
#endif
						}
						alg_loop = 2;
						continue;
					} else { /* still failed; try horizontal search first */
						search = HORIZONTAL;
						ena_mask = PER_BYTE_RES1;
						if (dbg_flag == 2) {
#ifdef DBG_PRINT
							printf("%s: byte %d, alg loop %d: failed\n",
							       __func__, byte, alg_loop);
#endif
						}
					}
					break;
				case 2:
					if (result == 0) { /* pass; go to vertical search */
						search = VERTICAL;
						if (dbg_flag == 2) {
#ifdef DBG_PRINT
							printf("%s: byte %d, alg loop %d: pass\n",
							       __func__, byte, alg_loop);
#endif
						}
						alg_loop = 2;
						continue;
					} else { /* fail */
						printf("%s: byte %d, alg loop %d: failed; no coverage\n",
						       __func__, byte, alg_loop);
						return MV_FAIL;
					}
					break;
				case 3:
					if (result == 0) { /* passed vertical and horizontal searches */
						/* pass; got to next byte */
						if (dbg_flag == 2) {
#ifdef DBG_PRINT
							printf("%s: byte %d, alg loop %d: pass\n",
							       __func__, byte, alg_loop);
#endif
						}
					} else { /* fail */
						printf("%s: byte %d, alg loop %d: failed; no coverage\n",
						       __func__, byte, alg_loop);
						return MV_FAIL;
					}
					break;
				} /* end of switch */
			} /* end of alg loop */
			if ((ddr3_if_ecc_enabled() == 1) && (byte != 8)) {
				reg_write(MC6_CH0_ECC_1BIT_ERR_COUNTER_REG, 0x0);
				reg_bit_clrset(MC6_RAS_CTRL_REG, 0x1 << ECC_EN_OFFS, ECC_EN_MASK << ECC_EN_OFFS);
			}
		} /* end of byte_num loop */
	}
#ifdef DBG_PRINT
	printf("%s: going to v-h second level search\n", __func__);
#endif
	for (byte = start_byte; byte < end_byte; byte++) {
		if ((ddr3_if_ecc_enabled() == 1) && (byte != 8)) {
			reg_bit_clrset(MC6_RAS_CTRL_REG, 0x0 << ECC_EN_OFFS, ECC_EN_MASK << ECC_EN_OFFS);
			reg_write(MC6_CH0_ECC_1BIT_ERR_COUNTER_REG, 0x0);
		}

		if ((ddr3_if_ecc_enabled() == 1) && (byte == 8))
			depth_stage = 3;
		else
			depth_stage = 4;

		ena_mask = PER_IF;
		result = horizontal_adjust(depth_stage, ena_mask, byte);
		if (result != 0) {
#ifdef DBG_PRINT
			printf("%s: cs[%d]: byte %d, second level of horizontal adjust failed\n",
			       __func__, effective_cs, byte);
#endif
		}
		result = vertical_adjust(depth_stage, ena_mask, byte);
		if (result != 0) {
#ifdef DBG_PRINT
			printf("%s: cs[%d]: byte %d, second level of vertical adjust failed\n",
			       __func__, effective_cs, byte);
#endif
		}
		if (byte == 8) {
			reg_bit_clrset(MC6_RAS_CTRL_REG, 0x8, 0x8); /* ignore ecc */
			reg_bit_clrset(MC6_RAS_CTRL_REG, 0x0, 0x8); /* ignore ecc */
			reg_write(MC6_CH0_ECC_1BIT_ERR_COUNTER_REG, 0x0);
		}
		if ((ddr3_if_ecc_enabled() == 1) && (byte != 8)) {
			reg_write(MC6_CH0_ECC_1BIT_ERR_COUNTER_REG, 0x0);
			reg_bit_clrset(MC6_RAS_CTRL_REG, 0x1 << ECC_EN_OFFS, ECC_EN_MASK << ECC_EN_OFFS);
		}
	}

	ena_mask = PER_IF;
#ifdef DBG_PRINT
	printf("%s: going to diagonal search\n", __func__);
#endif
	for (byte = start_byte; byte < end_byte; byte++) {
		if ((ddr3_if_ecc_enabled() == 1) && (byte != 8)) {
			reg_bit_clrset(MC6_RAS_CTRL_REG, 0x0 << ECC_EN_OFFS, ECC_EN_MASK << ECC_EN_OFFS);
			reg_write(MC6_CH0_ECC_1BIT_ERR_COUNTER_REG, 0x0);
		}

		if ((ddr3_if_ecc_enabled() == 1) && (byte == 8))
			depth_stage = 3;
		else
			depth_stage = 4;

		diagonal_adjust(depth_stage, ena_mask, byte);
		if (byte == 8) {
			reg_bit_clrset(MC6_RAS_CTRL_REG, 0x8, 0x8); /* ignore ecc */
			reg_bit_clrset(MC6_RAS_CTRL_REG, 0x0, 0x8); /* ignore ecc */
			reg_write(MC6_CH0_ECC_1BIT_ERR_COUNTER_REG, 0x0);
		}
		if ((ddr3_if_ecc_enabled() == 1) && (byte != 8)) {
			reg_write(MC6_CH0_ECC_1BIT_ERR_COUNTER_REG, 0x0);
			reg_bit_clrset(MC6_RAS_CTRL_REG, 0x1 << ECC_EN_OFFS, ECC_EN_MASK << ECC_EN_OFFS);
		}
	}

	return 0;
}

static int glob_validate_set(void)
{
	int area_len_dec = (reg_read(MC6_CH0_MMAP_LOW_REG(0)) & (0x1f << 16)) >> 16;
	bank_map = (reg_read(MC6_CH0_MC_CFG_REG(0)) & (0x1f << 24)) >> 24;
	area_len = 0;
	data_width = (reg_read(MC6_MC_CTRL0_REG) & (0x7 << 8)) >> 8;

	switch ((reg_read(MC6_CH0_MC_CFG_REG(0)) & (0x3 << 16)) >> 16) {
	case 0:
		page_size = 0x200;
		break;
	case 1:
		page_size = _1K;
		break;
	case 2:
		page_size = _1K * 2;
		break;
	}

	area_len = area_len_conv[area_len_dec - 7];
	bank_map = bank_map_conv[bank_map - 3];

	if (print_log == 1) {
#ifdef DBG_PRINT
		printf("bus width %d\n", (data_width == 0x3) ? 32 : 64);
		printf("area_len %lldMB\n", area_len);
		printf("bank map %dKB\n", bank_map);
#endif
	}

	area_len = area_len * (unsigned long long)_1M;

	return 0;
}

int mv_ddr_validate(void)
{
	u32 max_cs = ddr3_tip_max_cs_get(0);

#ifdef DBG_PRINT
	stability_info_print();
#endif
	glob_validate_set();

	effective_cs = 0;
	data_width = 4;
	xor_byte_mask = 0x0;

	for (effective_cs = 0; effective_cs < max_cs; effective_cs++)
		rx_adjust();

	printf("\n");

#if 0 /* enable to get stability and sweep printout */
	int repeat = 1;
	int depth = 4;
	int sphy = 0;

	stability_info_print();

	for (effective_cs = 0; effective_cs < max_cs; effective_cs++) {
		/* print out rx stats */
		for (sphy = 0; sphy <= 7; sphy++)
			cpu_sweep_test(repeat, depth, 0x1, sphy);
#if 0
		/* print out tx stats */
		for (sphy = 0; sphy <= 7; sphy++)
			cpu_sweep_test(repeat, depth, 0x0, sphy);
#endif
	}
#endif

	return 0;
}
