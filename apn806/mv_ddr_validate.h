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

#ifndef CVXORDEBUG_hH
#define CVXORDEBUG_hH

enum  SearchElement{
	CRx,
    CTx,
    RecCal
};
enum MaskType{
	OR_Mask,
    AND_Mask
};
enum Maskdef{
    PerIF,
    PerByte_res1,
    PerByte_res0,
    PerDQ0
};
enum  SearchType{
	Vertic,
    Horizon,
    diagon
};

enum VP_tests{
    XOR_CS0_general_1M,
    XOR_CS0_general_multiaccessSize,
    XOR_CS0_FullAddressAccess,
    CPU_CS0_general_1M,
    TIP2MC_equation,
    Eye_sweep_AllCS,
    Eye_sweepTx_CS0,
    Eye_sweepRx_CS0,
	Eye_sweepRx_CS0_perbit,
    Eye_sweepRx_CS0_woPBS,
    XOR_CS0_PartialWrite,
    XOR_CS1_FullAddressAccess,
    XOR_CrossCs_FullAddressAccess,
    XOR_CS0_general_1M_AVS_Sweep,
    CPU_RESET,
    XOR_CS1_PartialWrite,
    XOR_In_PageTest,
    XOR_Page2PageTest,
    XOR_In_BankTest,
    Funcional_WL,
    VP_full
};
enum VP_pattern{
    All_Aggresive,
    All_AggresiveInv,
    Resonance,
    Moving_one_croosand_zero,
    Moving_zero_croosand_one,
    VP_pattern_Last
};
enum CStest{
    CS0ToCS0,
    CS0ToCS1,
    CS0ToCS2,
    CS0ToCS3,
    CS1ToCS0,
    CS1ToCS1,
    CS1ToCS2,
    CS1ToCS3,
    CS2ToCS0,
    CS2ToCS1,
    CS2ToCS2,
    CS2ToCS3,
    CS3ToCS0,
    CS3ToCS1,
    CS3ToCS2,
    CS3ToCS3
  
};
unsigned long long LowCast32to64(int var);

unsigned long long HighCast32to64(int var);

int XOT_Test_ConfigurationCheck(unsigned long long Source_Address, u32 TotalData2Test, unsigned long long End_addr_bytes);

void  XorPrint(u32 TotalData2Test,int ByteCount,u32 DescBaseAddrL,u32 DescBaseAddrH,u32 Source_AddressL,u32 Source_AddressH,
              u32 Destanation_AddressL,u32 Destanation_AddressH);
void EccDisableEnable();

void Counter(int ClockOp);

int SetValidationGlobals();

void XOR_S2D_func(u32 NOD, int ByteCount, u32 Source_AddressL, u32 Source_AddressH, u32 Destanation_AddressL, 
                  u32 Destanation_AddressH, int Destnation_Jap, int WrBurstLen, int RdBurstLen,u32 DESQAddr,u32 DESQAddrH,int nNumOstdRd);

int XOR_ScmpD_func(u32 NOD, int ByteCount, u32 Source_AddressL, u32 Source_AddressH, u32 Destanation_AddressL, 
                  u32 Destanation_AddressH, int Destnation_Jap, int RdBurstLen,u32 DESQAddr,u32 DESQAddrH,int nNumOstdRd);



int XOR_Test(u32 TotalData2Test,u32 Source_AddressL, u32 Source_AddressH, u32 Destanation_AddressL, 
                  u32 Destanation_AddressH, int ByteCount,int PatternType,u32 DESQAddr,u32 DESQAddrH,int nNumOstdRd);

int XOR_Gradual_Test(u8 DepthStage);
int XORsearch_1D_1E(enum hws_edge_compare edge, enum hws_search_dir search_dir , u32 Step, u32 init_val,u32 end_val,
                    u8 DepthStage,u16 ByteNum, enum  SearchElement element);

int XORsearch_1D_2E(enum hws_edge_compare SearchConcept, u32 Step, u32 init_val,u32 end_val,u8 DepthStage,u16 ByteNum, enum  SearchElement element,u8 (*VW_vector));

int XORsearch_2D_1E(enum hws_edge_compare edge, enum  SearchElement element1, enum hws_search_dir search_dir1 , 
                    u32 Step1, u32 init_val1,u32 end_val1,enum  SearchElement element2, enum hws_search_dir search_dir2 , 
                    u32 Step2, u32 init_val2,u32 end_val2,u8 DepthStage,u16 ByteNum);

int XORsearch_2D_2E(enum hws_edge_compare edge, enum  SearchElement element1, enum hws_search_dir search_dir1 , 
                    u32 Step1, u32 init_val1,u32 end_val1,enum  SearchElement element2, enum hws_search_dir search_dir2 , 
                    u32 Step2, u32 init_val2,u32 end_val2,u8 DepthStage,u16 ByteNum,u8 (*VW_vector));
int XORsearch_RL(enum hws_edge_compare edge, enum hws_search_dir search_dir , u32 Step, u32 init_val,u32 end_val,u8 DepthStage,u16 ByteNum);

int XOT_Test_Wrapper(u32 TotalData2Test,enum CStest CSinvovle,enum VP_pattern Pattern,int ByteCount,int index);

int InnerPagetransactions(void);

int Page2Pagetransactions(void);

int InnerBanktransactions(void);

int print_stability_log(u32 dev_num);

int MemoryTestPerByte(u32 start_addres_low, u32 start_addres_high, u32 End_addres_low, u32 End_addres_high,
                      u32 Access_Numbers, u32 ByteNum,u32 printEn,u32 TestMap);

int MC_CL_CWL_impact(void);

int TipIpBIST(int dev_num, int if_id, enum hws_dir direction,int init_value_used, int pattern, int cs_num);

int ddr4_cpu_sweep_test(int dev_num, u32 repeat_num, u32 testLength ,u32 direction,u32 mode);

int ddr4_VerticalAdjusment(int dev_num, u8 DepthStage,int EnMask, u8 byteNumber);

int ddr4_HorezintalAdjusment(int dev_num, u8 DepthStage,int EnMask, u8 byteNumber);

int RunValidation(enum VP_tests testNum);

int RxAdjust();

int bit_flip();



void XOR_S2D_funcV(int NOD, int ByteCount, int Source_AddressL, int Source_AddressH, int Destanation_AddressL, 
                  int Destanation_AddressH, int Destnation_Jap, int WrBurstLen, int RdBurstLen);
int XOR_ScmpD_funcV(int NOD, int ByteCount, int Source_AddressL, int Source_AddressH, int Destanation_AddressL, 
                  int Destanation_AddressH, int Destnation_Jap, int RdBurstLen);
int XOR_TestV(int TotalData2Test,int Source_AddressL, int Source_AddressH, int Destanation_AddressL, 
                  int Destanation_AddressH, int ByteCount,int PatternType, int nNumOstdRd);
int mv_ddr_validate(void);
#endif //CVXORDEBUG_hH
