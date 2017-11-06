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

#include "mv_ddr_mc6.h"
#include "ddr3_init.h"

void mv_ddr_mc6_timing_regs_cfg(unsigned int freq_mhz)
{
	struct mv_ddr_mc6_timing mc6_timing;
	unsigned int page_size;
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	struct page_element *page_param = mv_ddr_page_tbl_get();

	/* get the spped bin index */
	enum hws_speed_bin speed_bin_index = tm->interface_params[IF_ID_0].speed_bin_index;

	/* calculate memory size */
	enum mv_ddr_die_capacity memory_size = tm->interface_params[IF_ID_0].memory_size;

	/* calculate page size */
	page_size = tm->interface_params[IF_ID_0].bus_width == MV_DDR_DEV_WIDTH_8BIT ?
		page_param[memory_size].page_size_8bit : page_param[memory_size].page_size_16bit;
	/* printf("page_size = %d\n", page_size); */

	/* calculate t_clck */
	mc6_timing.t_ckclk = MEGA / freq_mhz;
	/* printf("t_ckclk = %d\n", mc6_timing.t_ckclk); */

	/* calculate t_refi  */
	mc6_timing.t_refi = (tm->interface_params[IF_ID_0].interface_temp == MV_DDR_TEMP_HIGH) ? TREFI_HIGH : TREFI_LOW;

	/* the t_refi is in nsec */
	mc6_timing.t_refi = mc6_timing.t_refi / (MEGA / FCLK_KHZ);
	/* printf("t_refi = %d\n", mc6_timing.t_refi); */
	mc6_timing.t_wr = speed_bin_table(speed_bin_index, SPEED_BIN_TWR);
	/* printf("t_wr = %d\n", mc6_timing.t_wr); */
	mc6_timing.t_wr = time_to_nclk(mc6_timing.t_wr, mc6_timing.t_ckclk);
	/* printf("t_wr = %d\n", mc6_timing.t_wr); */

	/* calculate t_rrd */
	mc6_timing.t_rrd = (page_size == 1) ? speed_bin_table(speed_bin_index, SPEED_BIN_TRRD1K) :
		speed_bin_table(speed_bin_index, SPEED_BIN_TRRD2K);
	/* printf("t_rrd = %d\n", mc6_timing.t_rrd); */
	mc6_timing.t_rrd = GET_MAX_VALUE(mc6_timing.t_ckclk * 4, mc6_timing.t_rrd);
	/* printf("t_rrd = %d\n", mc6_timing.t_rrd); */
	mc6_timing.t_rrd = time_to_nclk(mc6_timing.t_rrd, mc6_timing.t_ckclk);
	/* printf("t_rrd = %d\n", mc6_timing.t_rrd); */

	/* calculate t_faw */
	if (page_size == 1) {
		mc6_timing.t_faw = speed_bin_table(speed_bin_index, SPEED_BIN_TFAW1K);
		/* printf("t_faw = %d\n", mc6_timing.t_faw); */
		mc6_timing.t_faw = GET_MAX_VALUE(mc6_timing.t_ckclk * 20, mc6_timing.t_faw);
		mc6_timing.t_faw = time_to_nclk(mc6_timing.t_faw, mc6_timing.t_ckclk);
		/* printf("t_faw = %d\n", mc6_timing.t_faw); */
	} else {	/* page size =2, we do not support page size 0.5k */
		mc6_timing.t_faw = speed_bin_table(speed_bin_index, SPEED_BIN_TFAW2K);
		/* printf("t_faw = %d\n", mc6_timing.t_faw); */
		mc6_timing.t_faw = GET_MAX_VALUE(mc6_timing.t_ckclk * 28, mc6_timing.t_faw);
		mc6_timing.t_faw = time_to_nclk(mc6_timing.t_faw, mc6_timing.t_ckclk);
		/* printf("t_faw = %d\n", mc6_timing. t_faw); */
	}

	/* calculate t_rtp */
	mc6_timing.t_rtp = speed_bin_table(speed_bin_index, SPEED_BIN_TRTP);
	/* printf("t_rtp = %d\n", mc6_timing.t_rtp); */
	mc6_timing.t_rtp = GET_MAX_VALUE(mc6_timing.t_ckclk * 4, mc6_timing.t_rtp);
	/* printf("t_rtp = %d\n", mc6_timing.t_rtp); */
	mc6_timing.t_rtp = time_to_nclk(mc6_timing.t_rtp, mc6_timing.t_ckclk);
	/* printf("t_rtp = %d\n", mc6_timing.t_rtp); */

	/* calculate t_mode */
	mc6_timing.t_mod = speed_bin_table(speed_bin_index, SPEED_BIN_TMOD);
	/* printf("t_mod = %d\n", mc6_timing.t_mod); */
#ifdef CONFIG_DDR4
	mc6_timing.t_mod = GET_MAX_VALUE(mc6_timing.t_ckclk * 24, mc6_timing.t_mod);
#else /* CONFIG_DDR3 */
	mc6_timing.t_mod = GET_MAX_VALUE(mc6_timing.t_ckclk * 12, mc6_timing.t_mod);
#endif
	/* printf("t_mod = %d\n", mc6_timing.t_mod); */
	mc6_timing.t_mod = time_to_nclk(mc6_timing.t_mod, mc6_timing.t_ckclk);
	/* printf("t_mod = %d\n",mc6_timing. t_mod); */

	/* calculate t_wtr */
	mc6_timing.t_wtr = speed_bin_table(speed_bin_index, SPEED_BIN_TWTR);
	/* printf("t_wtr = %d\n", mc6_timing.t_wtr); */
	mc6_timing.t_wtr = GET_MAX_VALUE(mc6_timing.t_ckclk * 2, mc6_timing.t_wtr);
	/* printf("t_wtr = %d\n", mc6_timing.t_wtr); */
	mc6_timing.t_wtr = time_to_nclk(mc6_timing.t_wtr, mc6_timing.t_ckclk);
	/* printf("t_wtr = %d\n", mc6_timing.t_wtr); */

#ifdef CONFIG_DDR4
	/* calculate t_wtr_l */
	mc6_timing.t_wtr_l = speed_bin_table(speed_bin_index, SPEED_BIN_TWTRL);
	/* printf("t_wtr_l = %d\n", mc6_timing.t_wtr_l); */
	mc6_timing.t_wtr_l = GET_MAX_VALUE(mc6_timing.t_ckclk * 4, mc6_timing.t_wtr_l);
	/* printf("t_wtr_l = %d\n", mc6_timing.t_wtr_l); */
	mc6_timing.t_wtr_l = time_to_nclk(mc6_timing.t_wtr_l, mc6_timing.t_ckclk);
	/* printf("t_wtr_l = %d\n", mc6_timing.t_wtr_l); */
#endif

	/* calculate t_xp */
	mc6_timing.t_xp = TIMING_T_XP;
	/* printf("t_xp = %d\n", mc6_timing.t_xp); */
	mc6_timing.t_xp = GET_MAX_VALUE(mc6_timing.t_ckclk * 4, mc6_timing.t_xp);
	/* printf("t_xp = %d\n", mc6_timing.t_xp); */
	mc6_timing.t_xp = time_to_nclk(mc6_timing.t_xp, mc6_timing.t_ckclk);
	/* printf("t_xp = %d\n", mc6_timing.t_xp); */

#ifdef CONFIG_DDR3
	/* calculate t_xpdll */
	mc6_timing.t_xpdll = speed_bin_table(speed_bin_index, SPEED_BIN_TXPDLL);
	mc6_timing.t_xpdll = GET_MAX_VALUE(mc6_timing.t_ckclk * 10, mc6_timing.t_xpdll);
	mc6_timing.t_xpdll = time_to_nclk(mc6_timing.t_xpdll, mc6_timing.t_ckclk);
#endif

	/* calculate t_cke */
	mc6_timing.t_cke = TIMING_T_CKE;
	/* printf("t_cke = %d\n", mc6_timing.t_cke); */
	mc6_timing.t_cke = GET_MAX_VALUE(mc6_timing.t_ckclk * 3, mc6_timing.t_cke);
	/* printf("t_cke = %d\n", mc6_timing.t_cke); */
	mc6_timing.t_cke = time_to_nclk(mc6_timing.t_cke, mc6_timing.t_ckclk);
	/* printf("t_cke = %d\n", mc6_timing.t_cke); */

	/* calculate t_ckesr */
	mc6_timing.t_ckesr = mc6_timing.t_cke + 1;
	/* printf("t_ckesr = %d\n", mc6_timing.t_ckesr); */

	/* calculate t_cpded */
#ifdef CONFIG_DDR4
	mc6_timing.t_cpded = 4;
#else /* CONFIG_DDR3 */
	mc6_timing.t_cpded = 1;
#endif
	/* printf("t_cpded = %d\n", mc6_timing.t_cpded); */

	/* calculate t_cksrx */
	mc6_timing.t_cksrx = TIMING_T_CKSRX;
	/* printf("t_cksrx = %d\n", mc6_timing.t_cksrx); */
	mc6_timing.t_cksrx = GET_MAX_VALUE(mc6_timing.t_ckclk * 5, mc6_timing.t_cksrx);
	/* printf("t_cksrx = %d\n", mc6_timing.t_cksrx); */
	mc6_timing.t_cksrx = time_to_nclk(mc6_timing.t_cksrx, mc6_timing.t_ckclk);
	/* printf("t_cksrx = %d\n", mc6_timing.t_cksrx); */

	/* calculate t_cksre */
	mc6_timing.t_cksre = mc6_timing.t_cksrx;
	/* printf("t_cksre = %d\n", mc6_timing.t_cksre); */

	/* calculate t_ras */
	mc6_timing.t_ras = speed_bin_table(speed_bin_index, SPEED_BIN_TRAS);
	/* printf("t_ras = %d\n", mc6_timing.t_ras); */
	mc6_timing.t_ras = time_to_nclk(mc6_timing.t_ras, mc6_timing.t_ckclk);
	/* printf("t_ras = %d\n", mc6_timing.t_ras); */

	/* calculate t_rcd */
	mc6_timing.t_rcd = speed_bin_table(speed_bin_index, SPEED_BIN_TRCD);
	/* printf("t_rcd = %d\n", mc6_timing.t_rcd); */
	mc6_timing.t_rcd = time_to_nclk(mc6_timing.t_rcd, mc6_timing.t_ckclk);
	/* printf("t_rcd = %d\n", mc6_timing.t_rcd); */

	/* calculate t_rp */
	mc6_timing.t_rp = speed_bin_table(speed_bin_index, SPEED_BIN_TRP);
	/* printf("t_rp = %d\n", mc6_timing.t_rp); */
	mc6_timing.t_rp = time_to_nclk(mc6_timing.t_rp, mc6_timing.t_ckclk);
	/* printf("t_rp = %d\n", mc6_timing.t_rp); */

	/*calculate t_rfc */
	mc6_timing.t_rfc = time_to_nclk(rfc_table[memory_size] * 1000, mc6_timing.t_ckclk);
	/* printf("t_rfc = %d\n", mc6_timing.t_rfc); */

	/* calculate t_xs */
	mc6_timing.t_xs = mc6_timing.t_rfc + time_to_nclk(TIMING_T_XS_OVER_TRFC, mc6_timing.t_ckclk);
	/* printf("t_xs = %d\n", mc6_timing.t_xs); */

#ifdef CONFIG_DDR4
	/* calculate t_rrd_l */
	mc6_timing.t_rrd_l = (page_size == 1) ? speed_bin_table(speed_bin_index, SPEED_BIN_TRRDL1K) :
						speed_bin_table(speed_bin_index, SPEED_BIN_TRRDL2K);
	/* printf("t_rrd_l = %d\n", mc6_timing.t_rrd_l); */
	mc6_timing.t_rrd_l = GET_MAX_VALUE(mc6_timing.t_ckclk * 4, mc6_timing.t_rrd_l);
	/* printf("t_rrd_l = %d\n", mc6_timing. t_rrd_l); */
	mc6_timing.t_rrd_l = time_to_nclk(mc6_timing.t_rrd_l, mc6_timing.t_ckclk);
	/* printf("t_rrd_l = %d\n", mc6_timing.t_rrd_l); */

	/* calculate t_ccd_l */
	mc6_timing.t_ccd_l = speed_bin_table(speed_bin_index, SPEED_BIN_TCCDL);
	mc6_timing.t_ccd_l = GET_MAX_VALUE(mc6_timing.t_ckclk * 5, mc6_timing.t_ccd_l);
	mc6_timing.t_ccd_l = time_to_nclk(mc6_timing.t_ccd_l, mc6_timing.t_ckclk);
	/* printf("t_ccd_l = %d\n", mc6_timing.t_ccd_l); */
#endif

	/* calculate t_rc */
	mc6_timing.t_rc = speed_bin_table(speed_bin_index, SPEED_BIN_TRC);
	/* printf("t_rc = %d\n", mc6_timing.t_rc); */
	mc6_timing.t_rc = time_to_nclk(mc6_timing.t_rc, mc6_timing.t_ckclk);
	/* printf("t_rc = %d\n", mc6_timing.t_rc); */

	/* constant timing parameters */
	mc6_timing.read_gap_extend = TIMING_READ_GAP_EXTEND;
	/* printf("read_gap_extend = %d\n", mc6_timing.read_gap_extend); */

	mc6_timing.t_res = TIMING_T_RES;
	/* printf("t_res = %d\n", mc6_timing.t_res); */
	mc6_timing.t_res = time_to_nclk(mc6_timing.t_res, mc6_timing.t_ckclk);
	/* printf("t_res = %d\n", mc6_timing.t_res); */

	mc6_timing.t_resinit = TIMING_T_RESINIT;
	/* printf("t_resinit = %d\n", mc6_timing.t_resinit); */
	mc6_timing.t_resinit = time_to_nclk(mc6_timing.t_resinit, mc6_timing.t_ckclk);
	/* printf("t_resinit = %d\n", mc6_timing.t_resinit); */

	mc6_timing.t_restcke = TIMING_T_RESTCKE;
	/* printf("t_restcke = %d\n", mc6_timing.t_restcke); */
	mc6_timing.t_restcke = time_to_nclk(mc6_timing.t_restcke, mc6_timing.t_ckclk);
	/* printf("t_restcke = %d\n", mc6_timing.t_restcke); */

	mc6_timing.t_actpden = TIMING_T_ACTPDEN;
	/* printf("t_actpden = %d\n", mc6_timing.t_actpden); */

#ifdef CONFIG_DDR4
	mc6_timing.t_zqinit = TIMING_T_ZQINIT;
	mc6_timing.t_zqoper = TIMING_T_ZQOPER;
	mc6_timing.t_zqcs = TIMING_T_ZQCS;
#else /* CONFIG_DDR3 */
	mc6_timing.t_zqinit = TIMING_T_ZQINIT;
	mc6_timing.t_zqinit = GET_MAX_VALUE(mc6_timing.t_ckclk * 512, mc6_timing.t_zqinit);
	mc6_timing.t_zqinit = time_to_nclk(mc6_timing.t_zqinit, mc6_timing.t_ckclk);

	/* tzqoper is 1/2 of tzqinit per jedec spec */
	mc6_timing.t_zqoper = mc6_timing.t_zqinit / 2;
	/* tzqcs is 1/8 of tzqinit per jedec spec */
	mc6_timing.t_zqcs = mc6_timing.t_zqinit / 8;
#endif
	/* printf("t_zqinit = %d\n", mc6_timing.t_zqinit); */
	/* printf("t_zqoper = %d\n", mc6_timing.t_zqoper); */
	/* printf("t_zqcs = %d\n", mc6_timing.t_zqcs); */

	mc6_timing.t_ccd = TIMING_T_CCD;
	/* printf("t_ccd = %d\n", mc6_timing.t_ccd); */

	mc6_timing.t_mrd = TIMING_T_MRD;
	/* printf("t_mrd = %d\n", mc6_timing.t_mrd); */

	mc6_timing.t_mpx_lh = TIMING_T_MPX_LH;
	/* printf("t_mpx_lh = %d\n", mc6_timing.t_mpx_lh); */

	mc6_timing.t_mpx_s = TIMING_T_MPX_S;
	/* printf("t_mpx_s = %d\n", mc6_timing.t_mpx_s); */

	mc6_timing.t_xmp = mc6_timing.t_rfc + time_to_nclk(TIMING_T_XMP_OVER_TRFC, mc6_timing.t_ckclk);
	/* printf("t_xmp = %d\n", mc6_timing.t_xmp); */

	mc6_timing.t_mrd_pda = TIMING_T_MRD_PDA;
	/* printf("t_mrd_pda = %d\n", mc6_timing.t_mrd_pda); */
	mc6_timing.t_mrd_pda = GET_MAX_VALUE(mc6_timing.t_ckclk * 16, mc6_timing.t_mrd_pda);
	/* printf("t_mrd_pda = %d\n", mc6_timing. t_mrd_pda); */
	mc6_timing.t_mrd_pda = time_to_nclk(mc6_timing.t_mrd_pda, mc6_timing.t_ckclk);
	/* printf("t_mrd_pda = %d\n", mc6_timing. t_mrd_pda); */

	mc6_timing.t_xsdll = speed_bin_table(speed_bin_index, SPEED_BIN_TXSDLL);
	/* printf("t_xsdll = %d\n", mc6_timing.t_xsdll); */

	mc6_timing.t_rwd_ext_dly = TIMING_T_RWD_EXT_DLY;
	/* printf("t_rwd_ext_dly = %d\n", mc6_timing.t_rwd_ext_dly); */

	mc6_timing.t_wl_early = TIMING_T_WL_EARLY;
	/* printf("t_wl_early = %d\n", mc6_timing.t_wl_early); */

	mc6_timing.t_ccd_ccs_wr_ext_dly = TIMING_T_CCD_CCS_WR_EXT_DLY;
	/* printf("t_ccd_ccs_wr_ext_dly = %d\n", mc6_timing.t_ccd_ccs_wr_ext_dly); */

	mc6_timing.t_ccd_ccs_ext_dly = TIMING_T_CCD_CCS_EXT_DLY;
	/* printf("t_ccd_ccs_ext_dly = %d\n", mc6_timing.t_ccd_ccs_ext_dly); */

	mc6_timing.cl = tm->interface_params[IF_ID_0].cas_l;
	mc6_timing.cwl = tm->interface_params[IF_ID_0].cas_wl;

#ifndef CONFIG_A3700
	/* configure the timing registers */
		reg_bit_clrset(MC6_CH0_DRAM_CFG1_REG,
			       mc6_timing.cwl << CWL_OFFS | mc6_timing.cl << CL_OFFS,
			       CWL_MASK << CWL_OFFS | CL_MASK << CL_OFFS);
	/* printf("MC6_CH0_DRAM_CFG1_REG addr 0x%x, data 0x%x\n", MC6_CH0_DRAM_CFG1_REG,
	       reg_read(MC6_CH0_DRAM_CFG1_REG)); */
#endif

	reg_bit_clrset(MC6_CH0_DDR_INIT_TIMING_CTRL0_REG,
		       mc6_timing.t_restcke << INIT_COUNT_NOP_OFFS,
		       INIT_COUNT_NOP_MASK << INIT_COUNT_NOP_OFFS);
	/* printf("MC6_CH0_DDR_INIT_TIMING_CTRL0_REG addr 0x%x, data 0x%x\n", MC6_CH0_DDR_INIT_TIMING_CTRL0_REG,
	       reg_read(MC6_CH0_DDR_INIT_TIMING_CTRL0_REG)); */

	reg_bit_clrset(MC6_CH0_DDR_INIT_TIMING_CTRL1_REG,
		       mc6_timing.t_resinit << INIT_COUNT_OFFS,
		       INIT_COUNT_MASK << INIT_COUNT_OFFS);
	/* printf("MC6_CH0_DDR_INIT_TIMING_CTRL1_REG addr 0x%x, data 0x%x\n", MC6_CH0_DDR_INIT_TIMING_CTRL1_REG,
	       reg_read(MC6_CH0_DDR_INIT_TIMING_CTRL1_REG)); */

	reg_bit_clrset(MC6_CH0_DDR_INIT_TIMING_CTRL2_REG,
		       mc6_timing.t_res << RESET_COUNT_OFFS,
		       RESET_COUNT_MASK << RESET_COUNT_OFFS);
	/* printf("MC6_CH0_DDR_INIT_TIMING_CTRL2_REG addr 0x%x, data 0x%x\n", MC6_CH0_DDR_INIT_TIMING_CTRL2_REG,
	       reg_read(MC6_CH0_DDR_INIT_TIMING_CTRL2_REG)); */

	reg_bit_clrset(MC6_CH0_ZQC_TIMING0_REG,
		       mc6_timing.t_zqinit << TZQINIT_OFFS,
		       TZQINIT_MASK << TZQINIT_OFFS);
	/* printf("MC6_CH0_ZQC_TIMING0_REG addr 0x%x, data 0x%x\n", MC6_CH0_ZQC_TIMING0_REG,
	       reg_read(MC6_CH0_ZQC_TIMING0_REG)); */

	reg_bit_clrset(MC6_CH0_ZQC_TIMING1_REG,
		       mc6_timing.t_zqoper << TZQCL_OFFS |
		       mc6_timing.t_zqcs << TZQCS_OFFS,
		       TZQCL_MASK << TZQCL_OFFS |
		       TZQCS_MASK << TZQCS_OFFS);
	/* printf("MC6_CH0_ZQC_TIMING1_REG addr 0x%x, data 0x%x\n", MC6_CH0_ZQC_TIMING1_REG,
	       reg_read(MC6_CH0_ZQC_TIMING1_REG)); */

	reg_bit_clrset(MC6_CH0_REFRESH_TIMING_REG,
		       mc6_timing.t_refi << TREFI_OFFS |
		       mc6_timing.t_rfc << TRFC_OFFS,
		       TREFI_MASK << TREFI_OFFS |
		       TRFC_MASK << TRFC_OFFS);
	/* printf("MC6_CH0_REFRESH_TIMING_REG addr 0x%x, data 0x%x\n", MC6_CH0_REFRESH_TIMING_REG,
	       reg_read(MC6_CH0_REFRESH_TIMING_REG)); */

	reg_bit_clrset(MC6_CH0_SELFREFRESH_TIMING0_REG,
		       mc6_timing.t_xsdll << TXSRD_OFFS |
		       mc6_timing.t_xs << TXSNR_OFFS,
		       TXSRD_MASK << TXSRD_OFFS |
		       TXSNR_MASK << TXSNR_OFFS);
	/* printf("MC6_CH0_SELFREFRESH_TIMING0_REG addr 0x%x, data 0x%x\n", MC6_CH0_SELFREFRESH_TIMING0_REG,
	       reg_read(MC6_CH0_SELFREFRESH_TIMING0_REG)); */

	reg_bit_clrset(MC6_CH0_SELFREFRESH_TIMING1_REG,
		       mc6_timing.t_cksrx << TCKSRX_OFFS |
		       mc6_timing.t_cksre << TCKSRE_OFFS,
		       TCKSRX_MASK << TCKSRX_OFFS |
		       TCKSRE_MASK << TCKSRE_OFFS);
	/* printf("MC6_CH0_SELFREFRESH_TIMING1_REG addr 0x%x, data 0x%x\n", MC6_CH0_SELFREFRESH_TIMING1_REG,
	       reg_read(MC6_CH0_SELFREFRESH_TIMING1_REG)); */

	reg_bit_clrset(MC6_CH0_PWRDOWN_TIMING0_REG,
#ifdef CONFIG_DDR4
		       TXARDS_VAL << TXARDS_OFFS |
#else /* CONFIG_DDR3 */
		       mc6_timing.t_xpdll << TXARDS_OFFS |
#endif
		       mc6_timing.t_xp << TXP_OFFS |
		       mc6_timing.t_ckesr << TCKESR_OFFS |
		       mc6_timing.t_cpded << TCPDED_OFFS,
		       TXARDS_MASK << TXARDS_OFFS |
		       TXP_MASK << TXP_OFFS |
		       TCKESR_MASK << TCKESR_OFFS |
		       TCPDED_MASK << TCPDED_OFFS);
	/* printf("MC6_CH0_PWRDOWN_TIMING0_REG addr 0x%x, data 0x%x\n", MC6_CH0_PWRDOWN_TIMING0_REG,
	       reg_read(MC6_CH0_PWRDOWN_TIMING0_REG)); */

	reg_bit_clrset(MC6_CH0_PWRDOWN_TIMING1_REG,
		       mc6_timing.t_actpden << TPDEN_OFFS,
		       TPDEN_MASK << TPDEN_OFFS);
	/* printf("MC6_CH0_PWRDOWN_TIMING1_REG addr 0x%x, data 0x%x\n", MC6_CH0_PWRDOWN_TIMING1_REG,
	       reg_read(MC6_CH0_PWRDOWN_TIMING1_REG)); */

	reg_bit_clrset(MC6_CH0_MRS_TIMING_REG,
		       mc6_timing.t_mrd << TMRD_OFFS |
		       mc6_timing.t_mod << TMOD_OFFS,
		       TMRD_MASK << TMRD_OFFS |
		       TMOD_MASK << TMOD_OFFS);
	/* printf("MC6_CH0_MRS_TIMING_REG addr 0x%x, data 0x%x\n", MC6_CH0_MRS_TIMING_REG,
	       reg_read(MC6_CH0_MRS_TIMING_REG)); */

	reg_bit_clrset(MC6_CH0_ACT_TIMING_REG,
		       mc6_timing.t_ras << TRAS_OFFS |
		       mc6_timing.t_rcd << TRCD_OFFS |
		       mc6_timing.t_rc << TRC_OFFS |
		       mc6_timing.t_faw << TFAW_OFFS,
		       TRAS_MASK << TRAS_OFFS |
		       TRCD_MASK << TRCD_OFFS |
		       TRC_MASK << TRC_OFFS |
		       TFAW_MASK << TFAW_OFFS);
	/* printf("MC6_CH0_ACT_TIMING_REG addr 0x%x, data 0x%x\n", MC6_CH0_ACT_TIMING_REG,
	       reg_read(MC6_CH0_ACT_TIMING_REG)); */

	reg_bit_clrset(MC6_CH0_PRECHARGE_TIMING_REG,
		       mc6_timing.t_rp << TRP_OFFS |
		       mc6_timing.t_rtp << TRTP_OFFS |
		       mc6_timing.t_wr << TWR_OFFS |
		       mc6_timing.t_rp << TRPA_OFFS,
		       TRP_MASK << TRP_OFFS |
		       TRTP_MASK << TRTP_OFFS |
		       TWR_MASK << TWR_OFFS |
		       TRPA_MASK << TRPA_OFFS);
	/* printf("MC6_CH0_PRECHARGE_TIMING_REG addr 0x%x, data 0x%x\n", MC6_CH0_PRECHARGE_TIMING_REG,
	       reg_read(MC6_CH0_PRECHARGE_TIMING_REG)); */

#ifdef CONFIG_DDR4
	reg_bit_clrset(MC6_CH0_CAS_RAS_TIMING0_REG,
		       mc6_timing.t_wtr << TWTR_S_OFFS |
		       mc6_timing.t_wtr_l << TWTR_OFFS |
		       mc6_timing.t_ccd << TCCD_S_OFFS |
		       mc6_timing.t_ccd_l << TCCD_OFFS,
		       TWTR_S_MASK << TWTR_S_OFFS |
		       TWTR_MASK << TWTR_OFFS |
		       TCCD_S_MASK << TCCD_S_OFFS |
		       TCCD_MASK << TCCD_OFFS);
#else /* CONFIG_DDR3 */
	reg_bit_clrset(MC6_CH0_CAS_RAS_TIMING0_REG,
		       mc6_timing.t_wtr << TWTR_OFFS |
		       mc6_timing.t_ccd << TCCD_OFFS,
		       TWTR_MASK << TWTR_OFFS |
		       TCCD_MASK << TCCD_OFFS);
#endif
	/* printf("MC6_CH0_CAS_RAS_TIMING0_REG addr 0x%x, data 0x%x\n", MC6_CH0_CAS_RAS_TIMING0_REG,
	       reg_read(MC6_CH0_CAS_RAS_TIMING0_REG)); */

#ifdef CONFIG_DDR4
	/* TODO: check why change default of 17:16 tDQS2DQ from '1' to '0' */
	reg_bit_clrset(MC6_CH0_CAS_RAS_TIMING1_REG,
		       mc6_timing.t_rrd << TRRD_S_OFFS |
		       mc6_timing.t_rrd_l << TRRD_OFFS |
		       TDQS2DQ_VAL << TDQS2DQ_OFFS,
		       TRRD_S_MASK << TRRD_S_OFFS |
		       TRRD_MASK << TRRD_OFFS |
		       TDQS2DQ_MASK << TDQS2DQ_OFFS);
#else /* CONFIG_DDR3 */
	reg_bit_clrset(MC6_CH0_CAS_RAS_TIMING1_REG,
		       mc6_timing.t_rrd << TRRD_OFFS |
		       TDQS2DQ_VAL << TDQS2DQ_OFFS,
		       TRRD_MASK << TRRD_OFFS |
		       TDQS2DQ_MASK << TDQS2DQ_OFFS);
#endif
	/* printf("MC6_CH0_CAS_RAS_TIMING1_REG addr 0x%x, data 0x%x\n", MC6_CH0_CAS_RAS_TIMING1_REG,
	       reg_read(MC6_CH0_CAS_RAS_TIMING1_REG)); */

#ifndef CONFIG_A3700
	reg_bit_clrset(MC6_CH0_OFF_SPEC_TIMING0_REG,
		       mc6_timing.t_ccd_ccs_ext_dly << TCCD_CCS_EXT_DLY_OFFS |
		       mc6_timing.t_ccd_ccs_wr_ext_dly << TCCD_CCS_WR_EXT_DLY_OFFS |
		       mc6_timing.t_rwd_ext_dly << TRWD_EXT_DLY_OFFS |
		       mc6_timing.t_wl_early << TWL_EARLY_OFFS,
		       TCCD_CCS_EXT_DLY_MASK << TCCD_CCS_EXT_DLY_OFFS |
		       TCCD_CCS_WR_EXT_DLY_MASK << TCCD_CCS_WR_EXT_DLY_OFFS |
		       TRWD_EXT_DLY_MASK << TRWD_EXT_DLY_OFFS |
		       TWL_EARLY_MASK << TWL_EARLY_OFFS);
	/* printf("MC6_CH0_OFF_SPEC_TIMING0_REG addr 0x%x, data 0x%x\n", MC6_CH0_OFF_SPEC_TIMING0_REG,
	       reg_read(MC6_CH0_OFF_SPEC_TIMING0_REG)); */

	reg_bit_clrset(MC6_CH0_OFF_SPEC_TIMING1_REG,
		       mc6_timing.read_gap_extend << READ_GAP_EXTEND_OFFS |
		       mc6_timing.t_ccd_ccs_ext_dly << TCCD_CCS_EXT_DLY_MIN_OFFS |
		       mc6_timing.t_ccd_ccs_wr_ext_dly << TCCD_CCS_WR_EXT_DLY_MIN_OFFS,
		       READ_GAP_EXTEND_MASK << READ_GAP_EXTEND_OFFS |
		       TCCD_CCS_EXT_DLY_MIN_MASK << TCCD_CCS_EXT_DLY_MIN_OFFS |
		       TCCD_CCS_WR_EXT_DLY_MIN_MASK << TCCD_CCS_WR_EXT_DLY_MIN_OFFS);
	/* printf("MC6_CH0_OFF_SPEC_TIMING1_REG addr 0x%x, data 0x%x\n", MC6_CH0_OFF_SPEC_TIMING1_REG,
	       reg_read(MC6_CH0_OFF_SPEC_TIMING1_REG)); */

	/* TODO: check why change default of 3:0 tDQSCK from '3' to '0' */
	reg_bit_clrset(MC6_CH0_DRAM_READ_TIMING_REG,
		       TDQSCK_VAL << TDQSCK_OFFS,
		       TDQSCK_MASK << TDQSCK_OFFS);
	/* printf("MC6_CH0_DRAM_READ_TIMING_REG addr 0x%x, data 0x%x\n", MC6_CH0_DRAM_READ_TIMING_REG,
	       reg_read(MC6_CH0_DRAM_READ_TIMING_REG)); */

	reg_bit_clrset(MC6_CH0_DRAM_MPD_TIMING_REG,
		       mc6_timing.t_xmp << TXMP_OFFS |
		       mc6_timing.t_mpx_s << TMPX_S_OFFS |
		       mc6_timing.t_mpx_lh << TMPX_LH_OFFS,
		       TXMP_MASK << TXMP_OFFS |
		       TMPX_S_MASK << TMPX_S_OFFS |
		       TMPX_LH_MASK << TMPX_LH_OFFS);
	/* printf("MC6_CH0_DRAM_MPD_TIMING_REG addr 0x%x, data 0x%x\n", MC6_CH0_DRAM_MPD_TIMING_REG,
	       reg_read(MC6_CH0_DRAM_MPD_TIMING_REG)); */

	reg_bit_clrset(MC6_CH0_DRAM_PDA_TIMING_REG,
		       mc6_timing.t_mrd_pda << TMRD_PDA_OFFS,
		       TMRD_PDA_MASK << TMRD_PDA_OFFS);
	/* printf("MC6_CH0_DRAM_PDA_TIMING_REG addr 0x%x, data 0x%x\n", MC6_CH0_DRAM_PDA_TIMING_REG,
	       reg_read(MC6_CH0_DRAM_PDA_TIMING_REG)); */
#endif
}

void mv_ddr_mc6_and_dram_timing_set(void)
{
	/* get the frequency */
	u32 freq_mhz;
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();

	/* get the frequency form the topology */
	freq_mhz = freq_val[tm->interface_params[IF_ID_0].memory_freq];

	mv_ddr_mc6_timing_regs_cfg(freq_mhz);
}


struct mv_ddr_addr_table {
	unsigned int num_of_bank_groups;
	unsigned int num_of_bank_addr_in_bank_group;
	unsigned int row_addr;
	unsigned int column_addr;
	unsigned int page_size_k_byte;
};

#define MV_DDR_DIE_CAP_MIN_IDX	MV_DDR_DIE_CAP_2GBIT
#define MV_DDR_DIE_CAP_MAX_IDX	MV_DDR_DIE_CAP_16GBIT
#define MV_DDR_DIE_CAP_SZ	(MV_DDR_DIE_CAP_MAX_IDX - MV_DDR_DIE_CAP_MIN_IDX + 1)
#define MV_DDR_DEV_WID_MIN_IDX	MV_DDR_DEV_WIDTH_8BIT
#define MV_DDR_DEV_WID_MAX_IDX	MV_DDR_DEV_WIDTH_16BIT
#define MV_DDR_DEV_WID_SZ	(MV_DDR_DEV_WID_MAX_IDX - MV_DDR_DEV_WID_MIN_IDX + 1)

static struct mv_ddr_addr_table addr_table_db[MV_DDR_DIE_CAP_SZ][MV_DDR_DEV_WID_SZ] = {
	{ /* MV_DDR_DIE_CAP_2GBIT */
		{4, 4, 14, 10, 1}, /* MV_DDR_DEV_WIDTH_8BIT*/
		{2, 4, 14, 10, 2}, /* MV_DDR_DEV_WIDTH_16BIT*/
	},
	{ /* MV_DDR_DIE_CAP_4GBIT */
		{4, 4, 15, 10, 1},
		{2, 4, 15, 10, 2}
	},
	{ /* MV_DDR_DIE_CAP_8GBIT */
		{4, 4, 16, 10, 1},
		{2, 4, 16, 10, 2},
	},
	{ /* MV_DDR_DIE_CAP_16GBIT */
		{4, 4, 17, 10, 1},
		{2, 4, 17, 10, 2},
	},
};

static int mv_ddr_addr_table_set(struct mv_ddr_addr_table *addr_table,
				 enum mv_ddr_die_capacity mem_size,
				 enum mv_ddr_dev_width bus_width)
{
	if (mem_size < MV_DDR_DIE_CAP_MIN_IDX ||
	    mem_size > MV_DDR_DIE_CAP_MAX_IDX) {
		printf("%s: unsupported memory size found\n", __func__);
		return -1;
	}

	if (bus_width < MV_DDR_DEV_WID_MIN_IDX ||
	    bus_width > MV_DDR_DEV_WID_MAX_IDX) {
		printf("%s: unsupported bus width found\n", __func__);
		return -1;
	}

	memcpy((void *)addr_table,
	       (void *)&addr_table_db[mem_size - MV_DDR_DIE_CAP_MIN_IDX]
				   [bus_width - MV_DDR_DEV_WID_MIN_IDX],
	       sizeof(struct mv_ddr_addr_table));

	return 0;
}

unsigned int mv_ddr_area_length_convert(unsigned int area_length)
{
	unsigned int area_length_map = 0xffffffff;

	switch (area_length) {
	case 384:
		area_length_map = 0x0;
		break;
	case 768:
		area_length_map = 0x1;
		break;
	case 1536:
		area_length_map = 0x2;
		break;
	case 3072:
		area_length_map = 0x3;
		break;
	case 8:
		area_length_map = 0x7;
		break;
	case 16:
		area_length_map = 0x8;
		break;
	case 32:
		area_length_map = 0x9;
		break;
	case 64:
		area_length_map = 0xA;
		break;
	case 128:
		area_length_map = 0xB;
		break;
	case 256:
		area_length_map = 0xC;
		break;
	case 512:
		area_length_map = 0xD;
		break;
	case 1024:
		area_length_map = 0xE;
		break;
	case 2048:
		area_length_map = 0xF;
		break;
	case 4096:
		area_length_map = 0x10;
		break;
	case 8192:
		area_length_map = 0x11;
		break;
	case 16384:
		area_length_map = 0x12;
		break;
	case 32768:
		area_length_map = 0x13;
		break;
	default:
		/* over than 32GB is not supported */
		printf("%s: unsupported area length %d\n", __func__, area_length);
	}

	return area_length_map;
}

unsigned int mv_ddr_bank_addr_convert(unsigned int num_of_bank_addr_in_bank_group)
{
	unsigned int num_of_bank_addr_in_bank_group_map = 0xff;

	switch (num_of_bank_addr_in_bank_group) {
	case 2:
		num_of_bank_addr_in_bank_group_map = 0x0;
		break;
	case 4:
		num_of_bank_addr_in_bank_group_map = 0x1;
		break;
	case 8:
		num_of_bank_addr_in_bank_group_map = 0x2;
		break;
	default:
		printf("%s: number of bank address in bank group %d is not supported\n", __func__,
		       num_of_bank_addr_in_bank_group);
	}

	return num_of_bank_addr_in_bank_group_map;
}

unsigned int mv_ddr_bank_groups_convert(unsigned int num_of_bank_groups)
{
	unsigned int num_of_bank_groups_map = 0xff;

	switch (num_of_bank_groups) {
	case 1:
		num_of_bank_groups_map = 0x0;
		break;
	case 2:
		num_of_bank_groups_map = 0x1;
		break;
	case 4:
		num_of_bank_groups_map = 0x2;
		break;
	default:
		printf("%s: number of bank group %d is not supported\n", __func__,
		       num_of_bank_groups);
	}

	return num_of_bank_groups_map;
}

unsigned int mv_ddr_column_num_convert(unsigned int column_addr)
{
	unsigned int column_addr_map = 0xff;

	switch (column_addr) {
	case 8:
		column_addr_map = 0x1;
		break;
	case 9:
		column_addr_map = 0x2;
		break;
	case 10:
		column_addr_map = 0x3;
		break;
	case 11:
		column_addr_map = 0x4;
		break;
	case 12:
		column_addr_map = 0x5;
		break;
	default:
		printf("%s: number of columns %d is not supported\n", __func__,
		       column_addr);
	}

	return column_addr_map;
}

unsigned int mv_ddr_row_num_convert(unsigned int row_addr)
{
	unsigned int row_addr_map = 0xff;

	switch (row_addr) {
	case 11:
		row_addr_map = 0x1;
		break;
	case 12:
		row_addr_map = 0x2;
		break;
	case 13:
		row_addr_map = 0x3;
		break;
	case 14:
		row_addr_map = 0x4;
		break;
	case 15:
		row_addr_map = 0x5;
		break;
	case 16:
		row_addr_map = 0x6;
		break;
	default:
		printf("%s: number of rows %d is not supported\n", __func__,
		       row_addr);
	}

	return row_addr_map;
}

unsigned int mv_ddr_stack_addr_num_convert(unsigned int stack_addr)
{
	unsigned int stack_addr_map = 0xff;

	switch (stack_addr) {
	case 1:
		stack_addr_map = 0x0;
		break;
	case 2:
		stack_addr_map = 0x1;
		break;
	case 4:
		stack_addr_map = 0x2;
		break;
	case 8:
		stack_addr_map = 0x3;
		break;
	default:
		printf("%s: number of stacks %d is not supported\n", __func__,
		       stack_addr);
	}

	return stack_addr_map;
}

unsigned int mv_ddr_device_type_convert(enum mv_ddr_dev_width bus_width)
{
	unsigned int device_type_map = 0xff;

	switch (bus_width) {
	case MV_DDR_DEV_WIDTH_8BIT:
		device_type_map = 0x1;
		break;
	case MV_DDR_DEV_WIDTH_16BIT:
		device_type_map = 0x2;
		break;
	default:
		printf("%s: device type is not supported\n", __func__);
	}

	return device_type_map;
}

unsigned int mv_ddr_bank_map_convert(enum mv_ddr_mc6_bank_boundary mc6_bank_boundary)
{
	unsigned int mv_ddr_mc6_bank_boundary_map = 0xff;

	switch (mc6_bank_boundary) {
	case BANK_MAP_512B:
		mv_ddr_mc6_bank_boundary_map = 0x2;
		break;
	case BANK_MAP_1KB:
		mv_ddr_mc6_bank_boundary_map = 0x3;
		break;
	case BANK_MAP_2KB:
		mv_ddr_mc6_bank_boundary_map = 0x4;
		break;
	case BANK_MAP_4KB:
		mv_ddr_mc6_bank_boundary_map = 0x5;
		break;
	case BANK_MAP_8KB:
		mv_ddr_mc6_bank_boundary_map = 0x6;
		break;
	case BANK_MAP_16KB:
		mv_ddr_mc6_bank_boundary_map = 0x7;
		break;
	case BANK_MAP_32KB:
		mv_ddr_mc6_bank_boundary_map = 0x8;
		break;
	case BANK_MAP_64KB:
		mv_ddr_mc6_bank_boundary_map = 0x9;
		break;
	case BANK_MAP_128KB:
		mv_ddr_mc6_bank_boundary_map = 0xa;
		break;
	case BANK_MAP_256KB:
		mv_ddr_mc6_bank_boundary_map = 0xb;
		break;
	case BANK_MAP_512KB:
		mv_ddr_mc6_bank_boundary_map = 0xc;
		break;
	case BANK_MAP_1MB:
		mv_ddr_mc6_bank_boundary_map = 0xd;
		break;
	case BANK_MAP_2MB:
		mv_ddr_mc6_bank_boundary_map = 0xe;
		break;
	case BANK_MAP_4MB:
		mv_ddr_mc6_bank_boundary_map = 0xf;
		break;
	case BANK_MAP_8MB:
		mv_ddr_mc6_bank_boundary_map = 0x10;
		break;
	case BANK_MAP_16MB:
		mv_ddr_mc6_bank_boundary_map = 0x11;
		break;
	case BANK_MAP_32MB:
		mv_ddr_mc6_bank_boundary_map = 0x12;
		break;
	case BANK_MAP_64MB:
		mv_ddr_mc6_bank_boundary_map = 0x13;
		break;
	case BANK_MAP_128MB:
		mv_ddr_mc6_bank_boundary_map = 0x14;
		break;
	case BANK_MAP_256MB:
		mv_ddr_mc6_bank_boundary_map = 0x15;
		break;
	case BANK_MAP_512MB:
		mv_ddr_mc6_bank_boundary_map = 0x16;
		break;
	case BANK_MAP_1GB:
		mv_ddr_mc6_bank_boundary_map = 0x17;
		break;
	case BANK_MAP_2GB:
		mv_ddr_mc6_bank_boundary_map = 0x18;
		break;
	case BANK_MAP_4GB:
		mv_ddr_mc6_bank_boundary_map = 0x19;
		break;
	case BANK_MAP_8GB:
		mv_ddr_mc6_bank_boundary_map = 0x1a;
		break;
	case BANK_MAP_16GB:
		mv_ddr_mc6_bank_boundary_map = 0x1b;
		break;
	case BANK_MAP_32GB:
		mv_ddr_mc6_bank_boundary_map = 0x1c;
		break;
	case BANK_MAP_64GB:
		mv_ddr_mc6_bank_boundary_map = 0x1d;
		break;
	default:
		printf("%s: bank_boundary is not supported\n", __func__);
	}

	return mv_ddr_mc6_bank_boundary_map;
}

void mv_ddr_mc6_sizes_cfg(void)
{
	unsigned int cs_idx;
	unsigned int cs_num;
#ifndef CONFIG_A3700
	unsigned int reserved_mem_idx;
#endif
	unsigned long long area_length_bits;
	unsigned int are_length_mega_bytes;
	unsigned long long start_addr_bytes;
	unsigned int start_addr_low, start_addr_high;

	struct mv_ddr_addr_table addr_tbl = {0};
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	calc_cs_num(DEV_NUM_0, IF_ID_0, &cs_num);

	area_length_bits = mv_ddr_get_memory_size_per_cs_in_bits();
	are_length_mega_bytes = area_length_bits / (MV_DDR_MEGA_BITS * MV_DDR_NUM_BITS_IN_BYTE);

	mv_ddr_addr_table_set(&addr_tbl,
			      tm->interface_params[0].memory_size,
			      tm->interface_params[0].bus_width);
	/* configure all length per cs here and activate the cs */
	for (cs_idx = 0; cs_idx < cs_num; cs_idx++) {
		start_addr_bytes = area_length_bits / MV_DDR_NUM_BITS_IN_BYTE * cs_idx;
		start_addr_low = start_addr_bytes & MV_DDR_32_BITS_MASK;
		start_addr_high = (start_addr_bytes >> START_ADDR_HTOL_OFFS) & MV_DDR_32_BITS_MASK;

		reg_bit_clrset(MC6_CH0_MMAP_LOW_REG(cs_idx),
			       CS_VALID_ENA << CS_VALID_OFFS |
			       INTERLEAVE_DIS << INTERLEAVE_OFFS |
			       mv_ddr_area_length_convert(are_length_mega_bytes) << AREA_LENGTH_OFFS |
			       start_addr_low,
			       CS_VALID_MASK << CS_VALID_OFFS |
			       INTERLEAVE_MASK << INTERLEAVE_OFFS |
			       AREA_LENGTH_MASK << AREA_LENGTH_OFFS |
			       START_ADDRESS_L_MASK << START_ADDRESS_L_OFFS);
		/* printf("MC6_CH0_MMAP_LOW_REG(cs_idx) addr 0x%x, data 0x%x\n", MC6_CH0_MMAP_LOW_REG(cs_idx),
			  reg_read(MC6_CH0_MMAP_LOW_REG(cs_idx))); */

		reg_bit_clrset(MC6_CH0_MMAP_HIGH_REG(cs_idx),
			       start_addr_high << START_ADDRESS_H_OFFS,
			       START_ADDRESS_H_MASK << START_ADDRESS_H_OFFS);
		/* printf("MC6_CH0_MMAP_HIGH_REG(cs_idx) addr 0x%x, data 0x%x\n", MC6_CH0_MMAP_HIGH_REG(cs_idx),
			  reg_read(MC6_CH0_MMAP_HIGH_REG(cs_idx))); */

		reg_bit_clrset(MC6_CH0_MC_CFG_REG(cs_idx),
			       mv_ddr_bank_addr_convert(addr_tbl.num_of_bank_addr_in_bank_group) <<
			       BA_NUM_OFFS |
			       mv_ddr_bank_groups_convert(addr_tbl.num_of_bank_groups) <<
			       BG_NUM_OFFS |
			       mv_ddr_column_num_convert(addr_tbl.column_addr) <<
			       CA_NUM_OFFS |
			       mv_ddr_row_num_convert(addr_tbl.row_addr) <<
			       RA_NUM_OFFS |
			       mv_ddr_stack_addr_num_convert(SINGLE_STACK) <<
			       SA_NUM_OFFS |
			       mv_ddr_device_type_convert(tm->interface_params[IF_ID_0].bus_width) <<
			       DEVICE_TYPE_OFFS |
			       mv_ddr_bank_map_convert(BANK_MAP_4KB) <<
			       BANK_MAP_OFFS,
			       BA_NUM_MASK << BA_NUM_OFFS |
			       BG_NUM_MASK << BG_NUM_OFFS |
			       CA_NUM_MASK << CA_NUM_OFFS |
			       RA_NUM_MASK << RA_NUM_OFFS |
			       SA_NUM_MASK << SA_NUM_OFFS |
			       DEVICE_TYPE_MASK << DEVICE_TYPE_OFFS |
			       BANK_MAP_MASK << BANK_MAP_OFFS);
	}

#ifndef CONFIG_A3700
	/* configure here the channel 1 reg_map_low and reg_map_high to unused memory area due to mc6 bug */
	for (cs_idx = 0, reserved_mem_idx = cs_num; cs_idx < cs_num; cs_idx++, reserved_mem_idx++) {
		start_addr_bytes = area_length_bits / MV_DDR_NUM_BITS_IN_BYTE * reserved_mem_idx;
		start_addr_low = start_addr_bytes & MV_DDR_32_BITS_MASK;
		start_addr_high = (start_addr_bytes >> START_ADDR_HTOL_OFFS) & MV_DDR_32_BITS_MASK;

		reg_bit_clrset(MC6_CH1_MMAP_LOW_REG(cs_idx),
			       CS_VALID_ENA << CS_VALID_OFFS |
			       INTERLEAVE_DIS << INTERLEAVE_OFFS |
			       mv_ddr_area_length_convert(are_length_mega_bytes) << AREA_LENGTH_OFFS |
			       start_addr_low,
			       CS_VALID_MASK << CS_VALID_OFFS |
			       INTERLEAVE_MASK << INTERLEAVE_OFFS |
			       AREA_LENGTH_MASK << AREA_LENGTH_OFFS |
			       START_ADDRESS_L_MASK << START_ADDRESS_L_OFFS);
		/* printf("MC6_CH1_MMAP_LOW_REG(cs_idx) addr 0x%x, data 0x%x\n", MC6_CH1_MMAP_LOW_REG(cs_idx),
			  reg_read(MC6_CH1_MMAP_LOW_REG(cs_idx))); */

		reg_bit_clrset(MC6_CH1_MMAP_HIGH_REG(cs_idx),
			       start_addr_high << START_ADDRESS_H_OFFS,
			       START_ADDRESS_H_MASK << START_ADDRESS_H_OFFS);
		/* printf("MC6_CH1_MMAP_HIGH_REG(cs_idx) addr 0x%x, data 0x%x\n", MC6_CH1_MMAP_HIGH_REG(cs_idx),
			  reg_read(MC6_CH1_MMAP_HIGH_REG(cs_idx))); */
	}
#endif
}

void  mv_ddr_mc6_ecc_enable(void)
{
	reg_bit_clrset(MC6_RAS_CTRL_REG,
		       ECC_EN_ENA << ECC_EN_OFFS,
		       ECC_EN_MASK << ECC_EN_OFFS);
}

