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
////////////////////////////////////////////////
///
#define _4G	0x100000000
#define READ_CENTRALIZATION_PHY_REG	0x3
#define WRITE_CENTRALIZATION_PHY_REG	0x1
#define CSN_IOB_VREF_REG(cs)		(0xdb + (cs * 12))
#define CSN_IO_BASE_VREF_REG(cs)	(0xd0 + (cs * 12))
#define RESULT_DB_PHY_REG_ADDR		0xc0
#define READ_DATA_SAMPLE_DELAY		0x1538
#define PAD_CONFIG_PHY_REG		0xa8
#define ODPG_DATA_CONTROL_REG		0x1630
#define CS_BYTE_GAP(cs_num)		((cs_num) * 0x4)
#define MC6_BASE_ADDR			0x20000
#define MC6_REG_MC_CONFIG(_cs_)		(MC6_BASE_ADDR + 0x220 + ((_cs_) * 4))
#define MC6_REG_MMAP_LOW_CH0(_cs_)	(MC6_BASE_ADDR + 0x200 + ((_cs_) * 8))
#define MC6_REG_MMAP_HIGH_CH0(_cs_)	(MC6_BASE_ADDR + 0x204 + ((_cs_) * 8))
#define MC6_REG_RAS_CTRL	(MC6_BASE_ADDR + 0x4c)
#define MC6_ECC_ENABLE_OFFS	1
#define MC6_ECC_ENABLE_MASK	0x1
#define MC6_REG_ECC_1bit_err_counter	(MC6_BASE_ADDR + 0x364)
#define ECC_ENABLE		1

int Hclk_CounterLow = 0;
int Hclk_CounterHigh = 0;
int printlog = 1;
int WriteXORPat = 1; // 1 - write, 0 - dont
int XORS2D = 1; // 1 - Do, 0 - skip
int XORSourceSRAM = 0;
int BankMap_conv[] = {1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8182,16384,32768}; //in kbyte
int BankMap =_1K;
unsigned long long Area_length = 0;
int DataWidth = 4; //4 is 64 bit
int PageSize = _1K;
unsigned long long XorByteMask = 0x0;
enum MaskType TypeOfMask = OR_Mask;
int stamstam = 64;
int  Max_Sram_Xor_Desc  = 128;
int Max_Sram_Xor_DescArray = 1024;
int NumOstdRd = 8;

///////////////////////////////////////////////////////////////////////////
///Extern
extern u8 dq_vref_vec[MAX_BUS_NUM];	/* stability support */
extern u8 rx_eye_hi_lvl[MAX_BUS_NUM];	/* vertical adjustment support */
extern u8 rx_eye_lo_lvl[MAX_BUS_NUM];	/* vertical adjustment support */
extern u32 nominal_avs;

////////////////////////////////////////////////
// ClockOp = 0 - Stop , Reset and start the counter include prints
// ClockOp = 1 - Reset and start the counter
// ClockOp = 2 - Stop the counter include prints
// ClockOp = 3 - Stop , Reset and start the counter not include prints
// ClockOp = 4 - Stop the counter not include prints
void Counter(int ClockOp)
{
    if(ClockOp == 1) {//reset & Start
        Hclk_CounterHigh = 0;
        Hclk_CounterLow = 0;
        mmio_write_32(0xf0011590, 0x10000);
        mmio_write_32(0xf0011590, 0x20000);
    }
    if(ClockOp == 0) {//Stop & reset & Start
        mmio_write_32(0xf0011590, 0x0000);
        u32 CounterLow[MAX_INTERFACE_NUM],CounterHigh[MAX_INTERFACE_NUM];
        //Read HCLKC Counter
        ddr3_tip_if_read(0, ACCESS_TYPE_UNICAST, 0,
                                      0x15A4, CounterLow,MASK_ALL_BITS);
        ddr3_tip_if_read(0, ACCESS_TYPE_UNICAST, 0,
                                      0x15A8, CounterHigh,MASK_ALL_BITS);
        printf(" HCLK Counter Stop after 0x%08x %08x Cycles\n",CounterHigh[0],CounterLow[0]);

    }
    if(ClockOp == 3) {//Stop & reset & Start
        mmio_write_32(0xf0011590, 0x0000);
        u32 CounterLow[MAX_INTERFACE_NUM],CounterHigh[MAX_INTERFACE_NUM];
        //Read HCLKC Counter
        ddr3_tip_if_read(0, ACCESS_TYPE_UNICAST, 0,
                                      0x15A4, CounterLow,MASK_ALL_BITS);
        ddr3_tip_if_read(0, ACCESS_TYPE_UNICAST, 0,
                                      0x15A8, CounterHigh,MASK_ALL_BITS);
        Hclk_CounterHigh = CounterHigh[0];
        Hclk_CounterLow = CounterLow[0];

    }
    if(ClockOp == 2) {//Stop 
        mmio_write_32(0xf0011590, 0x0000);
        u32 CounterLow[MAX_INTERFACE_NUM],CounterHigh[MAX_INTERFACE_NUM];
        //Read HCLKC Counter
        ddr3_tip_if_read(0, ACCESS_TYPE_UNICAST, 0,
                                      0x15A4, CounterLow,MASK_ALL_BITS);
        ddr3_tip_if_read(0, ACCESS_TYPE_UNICAST, 0,
                                      0x15A8, CounterHigh,MASK_ALL_BITS);
        printf(" HCLK Counter Stop after 0x%08x %08x Cycles\n",CounterHigh[0],CounterLow[0]);
        mmio_write_32(0xf0011590, 0x10000);
        mmio_write_32(0xf0011590, 0x20000);
    }
    if(ClockOp == 4) {//Stop without print
        mmio_write_32(0xf0011590, 0x0000);
        u32 CounterLow[MAX_INTERFACE_NUM],CounterHigh[MAX_INTERFACE_NUM];
        //Read HCLKC Counter
        ddr3_tip_if_read(0, ACCESS_TYPE_UNICAST, 0,
                                      0x15A4, CounterLow,MASK_ALL_BITS);
        ddr3_tip_if_read(0, ACCESS_TYPE_UNICAST, 0,
                                      0x15A8, CounterHigh,MASK_ALL_BITS);
        Hclk_CounterHigh = CounterHigh[0];
        Hclk_CounterLow = CounterLow[0];
        mmio_write_32(0xf0011590, 0x10000);
        mmio_write_32(0xf0011590, 0x20000);
    }


}
/////////////////////////////////////////////////////////
/// casting the 32bit int to 64 bit int with padding zero the remaining bits
unsigned long long LowCast32to64(int var){
    unsigned long long LowVar;
    LowVar = (unsigned long long)var;
    LowVar = LowVar & 0x00000000FFFFFFFF;
    return LowVar;
}

unsigned long long HighCast32to64(int var){
    unsigned long long HignVar;
    HignVar = (unsigned long long)var;
    HignVar = HignVar<<32;
    HignVar = HignVar & 0xFFFFFFFF00000000;
    return HignVar;
}
////////////////// Validation Platrform XOR print routing
void XorPrint(u32 TotalData2Test,int ByteCount,u32 DescBaseAddrL,u32 DescBaseAddrH,u32 Source_AddressL,u32 Source_AddressH,
              u32 Destanation_AddressL,u32 Destanation_AddressH){
    if(printlog == 1) {
        printf("\nVP: XOR test ");
        printf("\tD[H,L] = [0x%04x,0x%08x]",DescBaseAddrH,DescBaseAddrL);
    }
    if(printlog <3) {
        printf("\tS[H,L]->D[H,L]=[0x%04x,0x%08x]->", Source_AddressH,Source_AddressL);
        printf("[0x%04x,0x%08x]", Destanation_AddressH,Destanation_AddressL);
    }   
    if(printlog <4) {
        printf("\n\tTransfer 0x%08xb in chunks of %8db",TotalData2Test,ByteCount);
    }
}

////////////////// Support different DRAm access

void cpu_write(uintptr_t addr, uint64_t value)
{
    if(DataWidth == 4 ) {
        mmio_write_64(addr, value);
    }else
    {
        u32 data = value;

        mmio_write_32(addr, data);
    }

}

uint64_t cpu_read(uintptr_t addr)
{
	
    if(DataWidth == 4 ) {
        return mmio_read_64(addr);
    }else
    {
        return (uint64_t) mmio_read_32(addr);
    }
}


//////////////////Descriptors 
#define XOR_DESC_LINE_SIZE                           0x4 //in Bytes
#define XOR_DESC_MAX_BYTE_COUNT                      0xffffff00
#define XOR_DESC_LINE_OFFSET(LineNum)               (LineNum*XOR_DESC_LINE_SIZE)

#define XOR_ADDRESS_MAX_SIZE                         0xffffffffffff
#define XOR_ADDRESS_MASK                             XOR_ADDRESS_MAX_SIZE                       


#define XOR_DESC_STATUS_LINE                         0
#define XOR_DESC_ID_OFFSET                           0 
#define XOR_DESC_ID_MASK                            (0xffff<<XOR_DESC_ID_OFFSET) 
#define XOR_DESC_BCS_OFFSET                          25
#define XOR_DESC_BCS_MASK                           (0x1<<XOR_DESC_BCS_OFFSET)
#define XOR_DESC_FAILURE_CODE_OFFSET                 26
#define XOR_DESC_FAILURE_CODE_MASK                  (0x1f<<XOR_DESC_FAILURE_CODE_OFFSET)
#define XOR_DESC_PRST_OFFSET                         31
#define XOR_DESC_PRST_MASK                          (0x1<<XOR_DESC_PRST_OFFSET)


#define XOR_DESC_CRC32_RESULT                        1

#define XOR_DESC_COMMAND_LINE                        2
#define XOR_DESC_OPERATION_MODE_OFFSET               28
#define XOR_DESC_OPERATION_MODE_MASK                (0xf<<XOR_DESC_OPERATION_MODE_OFFSET)
#define XOR_DESC_AxATTR_SOURCE_OFFSET                21
#define XOR_DESC_AxATTR_SOURCE_MASK                 (0x1<<XOR_DESC_AxATTR_SOURCE_OFFSET)
#define XOR_DESC_CRC_CMP_LAST_OFFSET                 20
#define XOR_DESC_CRC_CMP_LAST_MASK                  (0x1<<XOR_DESC_CRC_CMP_LAST_OFFSET)
#define XOR_DESC_CRC_CMP_FIRST_OFFSET                19                              
#define XOR_DESC_CRC_CMP_FIRST_MASK                 (0x1<<XOR_DESC_CRC_CMP_FIRST_OFFSET)

#define XOR_DESC_BYTE_COUNTER_LINE                   3

#define XOR_DESC_SRC_BUF_ADDR_LOW_LINE               4

#define XOR_DESC_SRC_BUF_ADDR_HIGH_LINE              5
#define XOR_DESC_SRC_BUF_ADDR_HIGH_OFFSET            0
#define XOR_DESC_SRC_BUF_ADDR_HIGH_MASK             (0xffff<<XOR_DESC_SRC_BUF_ADDR_HIGH_OFFSET) 

#define XOR_DESC_DEST_BUFF_ADDR_LOW_LINE             6
#define XOR_DESC_SECOND_BUFF_ADDR_LOW_LINE           XOR_DESC_DEST_BUFF_ADDR_LOW_LINE

#define XOR_DESC_DEST_BUFF_ADDR_HIGH_LINE            7
#define XOR_DESC_SECOND_BUFF_ADDR_HIGH_LINE          XOR_DESC_DEST_BUFF_ADDR_HIGH_LINE
#define XOR_DESC_DEST_BUF_ADDR_HIGH_OFFSET           0
#define XOR_DESC_DEST_BUF_ADDR_HIGH_MASK            (0xffff<<XOR_DESC_DEST_BUF_ADDR_HIGH_OFFSET)
#define XOR_DESC_SECOND_BUF_ADDR_HIGH_OFFSET         0
#define XOR_DESC_SECOND_BUF_ADDR_HIGH_MASK          (0xffff<<XOR_DESC_DEST_BUF_ADDR_HIGH_OFFSET)

enum Operation_Mode{
    Idle,// operation (do nothing). No action on reception of such IBD 
    Pure_DMA,//
    Memory_Fill,// (memory-set) operation
    Memory_Init ,//(ECC clean) operation
    Byte_Compare,// operation
    CRC32_calculation,//
    RAID5,// (XOR) operation
    RAID6,// P&Q-generation
    RAID6_Recovery// (for one or two faulty buffers of ten (D0 to D7, P and Q).
};

//AXI_ATTR
#define XOR_DESC_AXI_DOMAIN_OFFSET                   0
#define XOR_DESC_AXI_DOMAIN_MASK                    (0x3 << XOR_DESC_AXI_DOMAIN_OFFSET)
#define XOR_DESC_AXI_CACHE_OFFSET                    2
#define XOR_DESC_AXI_CACHE_MASK                     (0xf << XOR_DESC_AXI_CACHE_OFFSET)
#define CV_XOR_DESC_CACHE_BUFFERABLE_BIT             BIT0 
#define CV_XOR_DESC_CACHE_MODIFIABLE_BIT             BIT1 
#define CV_XOR_DESC_CACHE_OTHER_ALLOCATE_BIT         BIT2 
#define CV_XOR_DESC_CACHE_ALLOCATE_BIT               BIT3 

#define XOR_DESC_AXI_QOS_OFFSET                      6
#define XOR_DESC_AXI_QOS_MASK                       (0x3 << XOR_DESC_AXI_QOS_OFFSET)
#define XOR_DESC_AXI_PRIVILEGED_OFFSET               8
#define XOR_DESC_AXI_PRIVILEGED_MASK                (0x1<<XOR_DESC_AXI_PRIVILEGED_OFFSET)
#define XOR_DESC_AXI_PRIVILEGED_TYPE_OFFSET          9
#define XOR_DESC_AXI_PRIVILEGED_TYPE_MASK           (0x1<<XOR_DESC_AXI_PRIVILEGED_TYPE_OFFSET)
#define XOR_DESC_AXI_TRAFFIC_OFFSET                  10
#define XOR_DESC_AXI_TRAFFIC_MASK                   (0xf<<XOR_DESC_AXI_TRAFFIC_OFFSET)






//////////////////////Registers
/*XOR_DESQ_BALR*/
#define CV_XOR_REG_BASE(nRegBase)                      nRegBase + 0x400000//AP

#define CV_XOR_DESQ_BASE_ADDR_LOW_REG(nRegBase)         CV_XOR_REG_BASE(nRegBase)
#define CV_XOR_DESQ_BASE_ADDR_LOW_OFFSET               8
#define CV_XOR_DESQ_BASE_ADDR_LOW_MASK                 (~0xff)<<CV_XOR_DESQ_BASE_ADDR_LOW_OFFSET

/*XOR_DESQ_BAHR*/
#define CV_XOR_DESQ_BASE_ADDR_HIGH_REG(nRegBase)       (CV_XOR_REG_BASE(nRegBase) + 0x4)
#define CV_XOR_DESQ_BASE_ADDR_HIGH_OFFSET              0
#define CV_XOR_DESQ_BASE_ADDR_HIGH_MASK               (0xffff)<<CV_XOR_DESQ_BASE_ADDR_HIGH_OFFSET

/*XOR_DESQ_SIZE*/
#define CV_XOR_DESQ_SIZE_REG(nRegBase)                 (CV_XOR_REG_BASE(nRegBase) + 0x8)
#define CV_XOR_DESQ_SIZE_OFFSET                        0 
#define CV_XOR_DESQ_SIZE_MASK                         (0x7fff)<<CV_XOR_DESQ_SIZE_OFFSET

/*XOR_DESQ_DONE*/                                                     
#define CV_XOR_DESQ_DONE_REG(nRegBase)                 (CV_XOR_REG_BASE(nRegBase) + 0xC)
#define CV_XOR_DESQ_DONE_OFFSET                        0
#define CV_XOR_DESQ_DONE_MASK                          (0x7FFF<<CV_XOR_DESQ_DONE_OFFSET)
#define CV_XOR_DESQ_SW_READ_PTR_OFFSET                 16
#define CV_XOR_DESQ_SW_READ_PTR_MASK                  (0x7FFF << CV_XOR_DESQ_SW_READ_PTR_OFFSET)

/*XOR_DESQ_ARATTR*/                                                     
#define CV_XOR_DESQ_AR_ATTRIBUTES_REG(nRegBase)        (CV_XOR_REG_BASE(nRegBase) + 0x10)


/*XOR_DESQ_AWATTR*/
#define CV_XOR_DESQ_AW_ATTRIBUTES_REG(nRegBase)        (CV_XOR_REG_BASE(nRegBase) + 0x1C)
#define CV_XOR_DESQ_DATA_SHARED_OFFSET               0 
#define CV_XOR_DESQ_DATA_SHARED_MASK                 (0x3 << CV_XOR_DESQ_DATA_SHARED_OFFSET)
#define CV_XOR_DESQ_DATA_CACHE_OFFSET                2 
#define CV_XOR_DESQ_DATA_CACHE_MASK                  (0xF << CV_XOR_DESQ_DATA_CACHE_OFFSET)
#define CV_XOR_DESQ_CACHE_BUFFERABLE_BIT             BIT0
#define CV_XOR_DESQ_CACHE_MODIFIABLE_BIT             BIT1
#define CV_XOR_DESQ_CACHE_ALLOCATE_BIT               BIT2
#define CV_XOR_DESQ_CACHE_OTHER_ALLOCATE_BIT         BIT3
#define CV_XOR_DESQ_DATA_QOS_ATTR_OFFSET             6 
#define CV_XOR_DESQ_DATA_QOS_ATTR_MASK               (3 << CV_XOR_DESQ_DATA_QOS_ATTR_MASK)
#define CV_XOR_DESQ_DESC_SHARED_OFFSET               8 
#define CV_XOR_DESQ_DESC_SHARED_MASK                 (0x3 << CV_XOR_DESQ_DESC_SHARED_OFFSET)
#define CV_XOR_DESQ_DESC_CACHE_OFFSET                10
#define CV_XOR_DESQ_DESC_CACHE_MASK                  (0xF << CV_XOR_DESQ_DESC_CACHE_OFFSET)
#define CV_XOR_DESQ_DESC_QOS_ATTR_OFFSET             14
#define CV_XOR_DESQ_DESC_QOS_ATTR_MASK               (3 << CV_XOR_DESQ_DESC_QOS_ATTR_OFFSET)
#define CV_XOR_DESQ_PRIVILEGED_BIT                   BIT16
#define CV_XOR_DESQ_PRIVILEGED_TYPE_OFFSET           17
#define CV_XOR_DESQ_PRIVILEGED_TYPE_MASK             (0x1<<CV_XOR_DESQ_PRIVILEGED_TYPE_OFFSET)
#define CV_XOR_DESQ_DESC_TRAFIC_OFFSET               18
#define CV_XOR_DESQ_DESC_TRAFIC_MASK                 (0xF << CV_XOR_DESQ_DESC_TRAFIC_OFFSET)
#define CV_XOR_DESQ_DATA_TRAFIC_OFFSET               22
#define CV_XOR_DESQ_DATA_TRAFIC_MASK                 (0xF << CV_XOR_DESQ_DATA_TRAFIC_OFFSET)      

/*XOR_IMSG_CDAT*/                                                     
#define CV_XOR_IMSG_REG(nRegBase)                      (CV_XOR_REG_BASE(nRegBase) + 0x14)
#define CV_XOR_IMG_VALUE_OFFSET                        0  
#define CV_XOR_IMG_VALUE_MASK                          (0xFFFF<<CV_XOR_IMG_VALUE_OFFSET)
#define CV_XOR_IMG_MASKED_DONE_BIT                     BIT16
#define CV_XOR_IMG_MASKED_IOD_BIT                      BIT17
#define CV_XOR_IMG_MASKED_TIMER_BIT                    BIT18
 
/*XOR_IMSG_THRD*/                                                    
#define CV_XOR_IMSG_THRESHOLD_REG(nRegBase)            (CV_XOR_REG_BASE(nRegBase) + 0x18)
#define CV_XOR_DONE_IMSG_THR_OFFSET                   0
#define CV_XOR_DONE_IMSG_THR_MASK                     (0x7f<<CV_XOR_DONE_IMSG_THR_OFFSET)

/*XOR_DESQ_ALLOC*/                                      
#define CV_XOR_DESQ_ALLOCATED_REG(nRegBase)            (CV_XOR_REG_BASE(nRegBase) + 0x4C)
#define CV_XOR_DESQ_ALLOCATED_OFFSET                   0
#define CV_XOR_DESQ_ALLOCATED_MASK                     (0x7FFF<<CV_XOR_DESQ_ALLOCATED_OFFSET)
#define CV_XOR_DESQ_SW_WRITE_PTR_OFFSET                16
#define CV_XOR_DESQ_SW_WRITE_PTR_MASK                 (0x7FFF << CV_XOR_DESQ_SW_WRITE_PTR_OFFSET)


/*XOR_DESQ_CTRL*/                                        
#define CV_XOR_DESQ_CTRL_REG(nRegBase)                 (CV_XOR_REG_BASE(nRegBase) + 0x100)
#define CV_XOR_ACT_DESC_SIZE_OFFSET                    0 
#define CV_XOR_ACT_DESC_SIZE_MASK                      (0x7<<CV_XOR_ACT_DESC_SIZE_OFFSET)

/*XOR_DESQ_STOP*/                                      
#define CV_XOR_DESQ_STOP_REG(nRegBase)                 (CV_XOR_REG_BASE(nRegBase) + 0x800)
#define CV_XOD_DESQ_DISABLE_BIT                        BIT0
#define CV_XOD_DESQ_RESET_BIT                          BIT1

/*XOR_DESQ_DEALLOC*/                                                       
#define CV_XOR_DESQ_DEALLOC_REG(nRegBase)                (CV_XOR_REG_BASE(nRegBase) + 0x804)

/*XOR_DESQ_ADD*/                                                       
#define CV_XOR_DESQ_ADD_REG(nRegBase)                  (CV_XOR_REG_BASE(nRegBase) + 0x808)

/*XOR_HPTR_STTS*/
#define CV_XOR_DESQ_HW_POINTERS_REG(nRegBase)         (CV_XOR_REG_BASE(nRegBase) + 0x848)
#define CV_XOR_DESQ_HW_READ_PTR_OFFSET                0
#define CV_XOR_DESQ_HW_READ_PTR_MASK                 (0x7FFF << CV_XOR_DESQ_SW_WRITE_READ_OFFSET)
#define CV_XOR_DESQ_HW_WRITE_PTR_OFFSET                16
#define CV_XOR_DESQ_HW_WRITE_PTR_MASK                 (0x7FFF << CV_XOR_DESQ_SW_WRITE_PTR_OFFSET)
         

/*----------------------XOR_GLOB_REGS-----------------------------*/

/*XORG_BW_CTRL*/
#define CV_XOR_BW_CTRL_REG(nRegBase)                   (CV_XOR_REG_BASE(nRegBase) + 0x10004)
#define CV_XOR_NUM_OUTSTAND_READ_OFFSET                  0
#define CV_XOR_NUM_OUTSTAND_READ_MASK                 (0x7F << CV_XOR_NUM_OUTSTAND_READ_OFFSET)
#define CV_XOR_NUM_OUTSTAND_WRITE_OFFSET               8
#define CV_XOR_NUM_OUTSTAND_WRITE_MASK                (0xF << CV_XOR_NUM_OUTSTAND_WRITE_OFFSET)
#define CV_XOR_MUX_READ_BURST_OFFSET                   12
#define CV_XOR_MUX_READ_BURST_MASK                    (0x7 << CV_XOR_MUX_READ_BURST_OFFSET)
#define CV_XOR_MUX_WRITE_BURST_OFFSET                  16
#define CV_XOR_MUX_WRITE_BURST_MASK                   (0x7 << CV_XOR_MUX_WRITE_BURST_OFFSET)

/*XORG_SECURE*/
#define CV_XOR_SECURE_REG(nRegBase)                   (CV_XOR_REG_BASE(nRegBase) + 0x10300)
#define CV_XOR_Secure_OFFSET                                    0
#define CV_XOR_Secure_MASK                              (0x1 << CV_XOR_Secure)
#


/*========================================================================*/
        
/*
 * XOR Transfer function 
 * -----------------
 * Descroption - this function only perfrom a source to destnation transfer. with full controll
 * on the user for the number of transfer block, the tranfer block size and its segmentation. 
 * All description are starting at address 0x0 and increment from thier. 
 * 
 * Input definition
 * ################
 * NOD - Number of descriptors 
 * ByteCount - Number of Byte to transfer [31:0] 
 * Source_AddressL - [31:0] 
 * Source_AddressH - [15:0] 
 * Destanation_AddressL - [31:0] 
 * Destanation AddressH - [15:0] 
 * Destnation_Jap - [31:0] - the Jap between destnation, need to be larger than the Bytecount  
 * WrBurstLen - 0-7 ; Write Burst length = WrBurstLen*32B 
 * RdBurstLen - 0-7 ; Read Burst length = RdBurstLen*32B 
 *  
 * Test Flow 
 * ---------- 
 * 1. Validate inputs. 
 * 2. write all descriptions thru the CPU to the Memory. 
 * 3. configure the Q. 
 * 4. start the Q. 
 * Exit 
 *  
 */
void XOR_S2D_func(u32 NOD, int ByteCount, u32 Source_AddressL, u32 Source_AddressH, u32 Destanation_AddressL, 
                  u32 Destanation_AddressH, int Destnation_Jap, int WrBurstLen, int RdBurstLen,u32 DESQAddrL,u32 DESQAddrH,int nNumOstdRd)
{
   //int TransferDesc[8];
   int TransferDesc[Max_Sram_Xor_DescArray];
   int indexNOD = 0,Align256Index = 0,DescIndex = 0;   //int BCDesc[8];
   uint64_t GlobalRegBase = 0xf0000000;
   unsigned long long Source_Address = HighCast32to64(Source_AddressH)| LowCast32to64(Source_AddressL) ;
   unsigned long long DESQAddr = HighCast32to64(DESQAddrH)| LowCast32to64(DESQAddrL) ;
   unsigned long long Dest = HighCast32to64(Destanation_AddressH)| LowCast32to64(Destanation_AddressL) ;
   unsigned long long NOD_TH = (NOD*0x40 + DESQAddr);

   for(Align256Index = 0; Align256Index < 64;Align256Index++ ) {
        int paddr = ((uintptr_t)(&TransferDesc[Align256Index]) & (uintptr_t)0xFF);
        //printf("\nAlign256Index = %d --> &TransferDesc[Align256Index] = %p\n",Align256Index,&TransferDesc[Align256Index]);
       if(paddr == 0x0) break;
   }
   //printf("\nS2D::Found Align256Index = %d --> &TransferDesc[Align256Index] = %p",Align256Index,&TransferDesc[Align256Index]);
  // 1.Validate inputs.
  // printf("NOD x BC = 0x%x\n",(NOD*ByteCount));
    if(ByteCount > 0x1000) {
        NOD = (NOD*ByteCount)/(ByteCount | 0x1000);
        ByteCount = ByteCount | 0x1000;// a know bug, need to enable bit 12.
    }

    WrBurstLen = WrBurstLen & 0x7;
    RdBurstLen = RdBurstLen & 0x7;
    if((NOD_TH> Source_Address)&& (NOD > Max_Sram_Xor_Desc) ) {//oferb - need to extend to address larger than 32bit
        printf("\n\nValidationPlatform: %s :: No sufficient place to %d NOD\n ",__func__,NOD);
        printf("Source_Address\t=\t0x%16llx\n",Source_Address);
        printf("Source_AddressL\t=\t0x%08x\n",Source_AddressL);
        printf("Source_AddressH\t=\t0x%08x\n",Source_AddressH);
        printf("NOD_TH\t=\t0x%16llx\n",NOD_TH);
        NOD = (u32)((Source_Address - DESQAddr)/0x40) - 1;
        printf("change to %d \n",NOD);
    }
    if(NOD> 0x7FFF) {
        printf("ValidationPlatform: %s :: Found to many NOD %d changed to 0x7FFF\n",__func__,NOD);
        NOD = 0x7FFF;
    }
   // printf("NOD x BC = 0x%x\n",(NOD*ByteCount));
   // printf("%d x %d \n",NOD,ByteCount);
    //------------------------------------------
    //printf("XOR_S2D_func:: 0x%08x --> (0x%08x + NOD x 0x%x ) .. bytecount 0x%x - NOD = %d\n"
           // ,Source_AddressL,Destanation_AddressL,Destnation_Jap,ByteCount,NOD);
    // write all descriptions thru the CPU to the Memory

   for(indexNOD = 0 ; indexNOD < NOD;indexNOD++ ) {
       if(NOD > Max_Sram_Xor_Desc) {
           DescIndex = 0;
       }else
       {
           DescIndex = 8*indexNOD + Align256Index;
       }
       //initilaize the descreptor array
       for (int i =0; i < 8; i++)
       {
          TransferDesc[i + DescIndex] = 0;
       }
       // configure the descreptor array
       TransferDesc[XOR_DESC_COMMAND_LINE + DescIndex] = Pure_DMA<<28;
       TransferDesc[XOR_DESC_BYTE_COUNTER_LINE + DescIndex] = ByteCount; //byte count
       TransferDesc[XOR_DESC_SRC_BUF_ADDR_LOW_LINE + DescIndex] = Source_AddressL; // source
       TransferDesc[XOR_DESC_SRC_BUF_ADDR_HIGH_LINE + DescIndex] = (Source_AddressH & 0x00000FFFF);
       Dest = Dest + (unsigned long long)(ByteCount )  ;
       Destanation_AddressL =  (u32)(Dest& 0x00000000FFFFFFFF);
       Destanation_AddressH =  (u32)(Dest>>32);
       TransferDesc[XOR_DESC_DEST_BUFF_ADDR_LOW_LINE + DescIndex] = (Destanation_AddressL ); // dest.
       TransferDesc[XOR_DESC_DEST_BUFF_ADDR_HIGH_LINE + DescIndex] = (Destanation_AddressH & 0xFFFF); // dest.


       // write desrciptor into the Memory
       unsigned long long NOD_incre = (0x20*indexNOD);
       uintptr_t addrL = HighCast32to64(DESQAddrH) + LowCast32to64(DESQAddrL) + NOD_incre;
      /* printf("\n\nValidationPlatform: %s :: New DESQAddr\n ",__func__);
       printf("\tDESQAddrH\t=\t0x%08x.",DESQAddrH);
       printf("%08x\n",DESQAddrL);
       printf("\tSource_AddressH\t=\t0x%08x.",Source_AddressH);
       printf("%08x\n",Source_AddressL);
       printf("\tDest_AddressH\t=\t0x%08x.",Destanation_AddressH);
       printf("%08x\n",Destanation_AddressL);
       printf("\tByteCount\t=\t0x%08x\n",ByteCount);*/
	  // addr = DESQAddr + (u32)(0x20*indexNOD);
       
       /*due to the fact that the mmio library is missing a fuction to write 32 nit data to address higher than 4G 
         a combine of the descriptor is needed*/
       for (int i =0; i < 4; i++)
       {     
           // printf("\t\taddrL\t=\t0x%16lx\n",addrL);
           uint64_t value = HighCast32to64(TransferDesc[2*i + 1 + DescIndex]) | LowCast32to64(TransferDesc[2*i + DescIndex]) ;
           //printf("addrL\t=\t0x%16lx \t Value\t=\t0x%16lx\n",addrL,value);
           mmio_write_64( addrL, value);
          addrL+= 8;
       }

       
   }
   //--------------------------------------/-------------------------
   //Q definition
   if(NOD>128) {
       mmio_write_32(CV_XOR_DESQ_BASE_ADDR_LOW_REG(GlobalRegBase), DESQAddr);
       mmio_write_32(CV_XOR_DESQ_BASE_ADDR_HIGH_REG(GlobalRegBase),DESQAddrH);
   }else{
       mmio_write_32(CV_XOR_SECURE_REG(GlobalRegBase),0x0<<CV_XOR_Secure_OFFSET);
       int paddr = ((uintptr_t)&TransferDesc[Align256Index]) & (uintptr_t)(0xFFFFFFFF);
      // printf("\tpaddr = 0x%08x - ",paddr);
       mmio_write_32(CV_XOR_DESQ_BASE_ADDR_LOW_REG(GlobalRegBase), paddr);
       paddr = ((uintptr_t)(&TransferDesc[Align256Index])&((uintptr_t)0xFFFFFFFF00000000))>>32;
      // printf("\tpaddr = 0x%08x - ",paddr);
       mmio_write_32(CV_XOR_DESQ_BASE_ADDR_HIGH_REG(GlobalRegBase),paddr);
   }
   mmio_write_32(CV_XOR_DESQ_SIZE_REG(GlobalRegBase), NOD);
   mmio_write_32(CV_XOR_DESQ_AR_ATTRIBUTES_REG(GlobalRegBase), 0x3e3e);
   mmio_write_32(CV_XOR_DESQ_AW_ATTRIBUTES_REG(GlobalRegBase), 0x3e3e);
   mmio_write_32(CV_XOR_IMSG_THRESHOLD_REG(GlobalRegBase), 0);
   mmio_write_32(CV_XOR_DESQ_CTRL_REG(GlobalRegBase), 0);//0 - 32Byte
   mmio_write_32(CV_XOR_DESQ_STOP_REG(GlobalRegBase), 0);
   mmio_write_32(CV_XOR_BW_CTRL_REG(GlobalRegBase), (/*0x300 |*/ nNumOstdRd | RdBurstLen<<CV_XOR_MUX_READ_BURST_OFFSET | WrBurstLen<<CV_XOR_MUX_WRITE_BURST_OFFSET));
  // printf("\n CV_XOR_BW_CTRL_REG(GlobalRegBase) 0x%08x RdBurstLen = %d \n",(u32)CV_XOR_BW_CTRL_REG(GlobalRegBase),RdBurstLen);

   // add descriptor to the XOR - "Start"
   mmio_write_32(CV_XOR_DESQ_ADD_REG(GlobalRegBase),NOD);

   int DESQ_DONE_index = 0;
   int ReadReg = 0;
   while(DESQ_DONE_index < NOD) {
       ReadReg = mmio_read_32(CV_XOR_DESQ_DONE_REG(GlobalRegBase));
       DESQ_DONE_index = (CV_XOR_DESQ_DONE_MASK & ReadReg)>>CV_XOR_DESQ_DONE_OFFSET;
   }
   // send to the DMA massage that we read des.
	mmio_write_32(CV_XOR_DESQ_DEALLOC_REG(GlobalRegBase), NOD); 
}
/*========================================================================*/
        
/*
 * XOR Compare function 
 * -----------------
 * Descroption - this function perfrom a source to destnation compare. with full controll
 * on the user for the number of compare block, the block size and its segmentation. 
 * All description are starting at address 0x0 and increment from thier. 
 * 
 * Input definition
 * ################
 * NOD - Number of descriptors 
 * ByteCount - Number of Byte to Compare [31:0] 
 * Source_AddressL - [31:0] 
 * Source_AddressH - [15:0] 
 * Destanation_AddressL - [31:0] 
 * Destanation AddressH - [15:0] 
 * Destnation_Jap - [31:0] - the Jap between destnation, need to be larger than the Bytecount 
 *                  if euale zero meaning that the function will reread the same destanation 
 * RdBurstLen - 0-7 ; Read Burst length = RdBurstLen*32B 
 *  
 * Test Flow 
 * ---------- 
 * 1. Validate inputs. 
 * 2. write all descriptions thru the CPU to the Memory. 
 * 3. configure the Q. 
 * 4. start the Q. 
 * 5. Poll on the descriptor status and count fails 
 * Exit 
 *  
 */
int XOR_ScmpD_func(u32 NOD, int ByteCount, u32 Source_AddressL, u32 Source_AddressH, u32 Destanation_AddressL, 
                  u32 Destanation_AddressH, int Destnation_Jap, int RdBurstLen,u32 DESQAddrL,u32 DESQAddrH,int nNumOstdRd)
{
    //int TransferDesc[8];
   int TransferDesc[Max_Sram_Xor_DescArray];
   
   int ReadReg = 0,DESQ_SW_RdPtr = 0xFF/*,DESQ_SW_RdPtr_Last = 0xFF*/;
   int indexNOD = 0,Align256Index = 0,DescIndex = 0;
   int BCSIndex = 0;
   int printEn = 0;
   uint64_t GlobalRegBase = 0xf0000000;
   unsigned long long Source_Address = HighCast32to64(Source_AddressH)| LowCast32to64(Source_AddressL) ;
   unsigned long long DESQAddr = HighCast32to64(DESQAddrH)| LowCast32to64(DESQAddrL) ;
   unsigned long long Dest = HighCast32to64(Destanation_AddressH)| LowCast32to64(Destanation_AddressL) ;
   unsigned long long NOD_TH = (NOD*0x40 + DESQAddr);
   uintptr_t addrL;
   int data = 0x0;

   for(Align256Index = 0; Align256Index < 64;Align256Index++ ) {
        int paddr = ((uintptr_t)(&TransferDesc[Align256Index]) & (uintptr_t)0xFF);
        //printf("\nAlign256Index = %d --> &TransferDesc[Align256Index] = %p\n",Align256Index,&TransferDesc[Align256Index]);
       if(paddr == 0x0) break;
   }
  // printf("\nScmpD::Found Align256Index = %d --> &TransferDesc[Align256Index] = %p",Align256Index,&TransferDesc[Align256Index]);

   // 1.Validate inputs.
    if(ByteCount > 0x1000) {
        NOD = (NOD*ByteCount)/(ByteCount | 0x1000);
        ByteCount = ByteCount | 0x1000;// a know bug, need to enable bit 12.
    }
    
    RdBurstLen = RdBurstLen & 0x7;
    if((NOD_TH> Source_Address)&& (NOD > Max_Sram_Xor_Desc) ) {
        NOD = (u32)((Source_Address - (unsigned long long)DESQAddr)/0x40) - 1;
    }
    if(NOD> 0x7FFF) {
        NOD = 0x7FFF;
    }

    //------------------------------------------
    //printf("XOR_ScmpD_func:: 0x%08x --> (0x%08x + NOD x 0x%x ) .. bytecount 0x%x - NOD = %d\n"
            //,Source_AddressL,Destanation_AddressL,Destnation_Jap,ByteCount,NOD);
    // write all descriptions thru the CPU to the Memory
   for(indexNOD = 0 ; indexNOD < NOD;indexNOD++ ) {
       if(NOD > Max_Sram_Xor_Desc) {
           DescIndex = 0;
       }else
       {
           DescIndex = 8*indexNOD + Align256Index;
       }
       //initilaize the descreptor array
       for (int i =0; i < 8; i++)
       {
          TransferDesc[i + DescIndex] = 0;
       }
       // configure the descreptor array
       TransferDesc[XOR_DESC_COMMAND_LINE + DescIndex] = ((Byte_Compare<<XOR_DESC_OPERATION_MODE_OFFSET) | (0x3 << 19));;
       TransferDesc[XOR_DESC_BYTE_COUNTER_LINE + DescIndex] = ByteCount; //byte count
       TransferDesc[XOR_DESC_SRC_BUF_ADDR_LOW_LINE + DescIndex] = Source_AddressL; // source
       TransferDesc[XOR_DESC_SRC_BUF_ADDR_HIGH_LINE + DescIndex] = (Source_AddressH & 0x00000FFFF);
       Dest = Dest + (unsigned long long)(ByteCount )  ;
       Destanation_AddressL =  (u32)(Dest& 0x00000000FFFFFFFF);
       Destanation_AddressH =  (u32)(Dest>>32);
       TransferDesc[XOR_DESC_DEST_BUFF_ADDR_LOW_LINE + DescIndex] = (Destanation_AddressL ); // dest.
       TransferDesc[XOR_DESC_DEST_BUFF_ADDR_HIGH_LINE + DescIndex] = (Destanation_AddressH & 0xFFFF); // dest.

       // write desrciptor into the Memory
        unsigned long long NOD_incre = (0x20*indexNOD);
       
        addrL = HighCast32to64(DESQAddrH) + LowCast32to64(DESQAddrL) + NOD_incre;
       
       /*due to teh fact that the mmio library is missing a fuction to write 32 nit data to address higher than 4G 
         a combine of the descriptor is needed*/
        if(NOD > Max_Sram_Xor_Desc) {
           for (int i =0; i < 4; i++)
           {     
               // printf("addrL\t=\t0x%16lx\n",addrL);
               uint64_t value = HighCast32to64(TransferDesc[2*i + 1 ]) | LowCast32to64(TransferDesc[2*i ]) ;
               // printf("addrL\t=\t0x%16lx \t Value\t=\t0x%16lx\n",addrL,value);
               mmio_write_64( addrL, value);
               addrL+= 8;
           }
        }

       
   }
   //---------------------------------------------------------------
   //printf("\n&TransferDesc[0] = %lx\t&TransferDesc[1] = %lx\t&TransferDesc[2] = %lx\n",&TransferDesc[0],&TransferDesc[1],&TransferDesc[2]);
   //Q definition
   if(NOD>Max_Sram_Xor_Desc ) {
       mmio_write_32(CV_XOR_DESQ_BASE_ADDR_LOW_REG(GlobalRegBase), DESQAddr);
       mmio_write_32(CV_XOR_DESQ_BASE_ADDR_HIGH_REG(GlobalRegBase),DESQAddrH);
   }else{
       mmio_write_32(CV_XOR_SECURE_REG(GlobalRegBase),0x0<<CV_XOR_Secure_OFFSET);
       int paddr = ((uintptr_t)&TransferDesc[Align256Index]) & (uintptr_t)(0xFFFFFFFF);
      // printf("\tpaddr = 0x%08x - ",paddr);
       mmio_write_32(CV_XOR_DESQ_BASE_ADDR_LOW_REG(GlobalRegBase), paddr);
       paddr = ((uintptr_t)(&TransferDesc[Align256Index])&((uintptr_t)0xFFFFFFFF00000000))>>32;
      // printf("\tpaddr = 0x%08x - ",paddr);
       mmio_write_32(CV_XOR_DESQ_BASE_ADDR_HIGH_REG(GlobalRegBase),paddr);
   }
   mmio_write_32(CV_XOR_DESQ_SIZE_REG(GlobalRegBase), NOD);
   mmio_write_32(CV_XOR_DESQ_AR_ATTRIBUTES_REG(GlobalRegBase), 0x3e3e);
   mmio_write_32(CV_XOR_DESQ_AW_ATTRIBUTES_REG(GlobalRegBase), 0x3e3e);
   mmio_write_32(CV_XOR_IMSG_THRESHOLD_REG(GlobalRegBase), 0);
   mmio_write_32(CV_XOR_DESQ_CTRL_REG(GlobalRegBase), 0);//0 - 32Byte
   mmio_write_32(CV_XOR_DESQ_STOP_REG(GlobalRegBase), 0);
   mmio_write_32(CV_XOR_BW_CTRL_REG(GlobalRegBase), (/*0x300 |*/ nNumOstdRd | RdBurstLen<<CV_XOR_MUX_READ_BURST_OFFSET ));
   //------------------------------------------------------------------
   // add descriptor to the XOR - "Start"
   mmio_write_32(CV_XOR_DESQ_ADD_REG(GlobalRegBase),NOD);

   // polling result from the descreptor when it is ready.
   //----------------------------------------------------
   //poll on SW pointer untill the descriptor is ready 
   indexNOD = 0;
   int DESQ_DONE_index=0;
	int result =0;
    uint64_t bitmask32 = 0xFFFFFFFF;
    //stage 1: Read that all the desq finished.
   while(DESQ_DONE_index < NOD) {
	
       ReadReg = mmio_read_32(CV_XOR_DESQ_DONE_REG(GlobalRegBase));
       DESQ_DONE_index = (CV_XOR_DESQ_DONE_MASK & ReadReg)>>CV_XOR_DESQ_DONE_OFFSET;
		//printf("First while %d (%d)\n",DESQ_DONE_index,NOD);
	}
       
   //Stage 2: read result of all desq.
       while(indexNOD<NOD) {
                DescIndex =  8*indexNOD +  + Align256Index;
                DESQ_SW_RdPtr = (ReadReg & CV_XOR_DESQ_SW_READ_PTR_MASK) >> CV_XOR_DESQ_SW_READ_PTR_OFFSET;
				   //DESQ_SW_RdPtr_Last = DESQ_SW_RdPtr;
				   //mdelay(1); //bug at 7040
                unsigned long long NOD_incre = (0x20*indexNOD) + XOR_DESC_STATUS_LINE;
                addrL = HighCast32to64(DESQAddrH) + LowCast32to64(DESQAddr) + NOD_incre;
                if(NOD>Max_Sram_Xor_Desc) {
                    //printf("XOR_ScmpD_func:: descriptor addrL 0x%16jx --- ",addrL );
                    data = (int)(mmio_read_64(addrL)&bitmask32);
                }else{
                   //printf("XOR_ScmpD_func:: descriptor addrL %p --- ",&TransferDesc[XOR_DESC_STATUS_LINE + 8*indexNOD]);
                    data = TransferDesc[XOR_DESC_STATUS_LINE + DescIndex];
                   }
				   //printf(" data 0x%08x\n",data );
				   data =( data >> XOR_DESC_BCS_OFFSET) & 0x1;
				   result |= data;
					
	
           if((data == 0) && (indexNOD>0) ) {//fail
				if(data == 0){
                    printEn = 0;
                   /* addrL = ((unsigned long long)DESQAddr + (unsigned long long)(0x20*DESQ_SW_RdPtr + XOR_DESC_STATUS_LINE +0x18) 
                           + (((unsigned long long)DESQAddrH)<<32) )  ;*/
					//addrL = (int)(mmio_read_64(addrL)&bitmask32);;
					/*for (int stamloop = 0 ; stamloop < 10 ; stamloop++) {
						for(int b = 0 ; b < (int)(ByteCount/4);b++) {
							//printf("%d)failed with bc = %d at offset\t0x%08x\tdata\t0x%08x\n",stamloop,ByteCount,(addr + b*4),mmio_read_32(addr + b*4));
						}
					}*/
					BCSIndex++;
				   if(printEn == 1) printf("XOR_ScmpD_func:: Descriptor %d(DONE - %d), offset 0x%08x , BCS %s\n",indexNOD,DESQ_DONE_index,DESQ_SW_RdPtr,((data == 1)?"OK":"FAIL"));
				}
           }
		   // send to the DMA massage that we read des.
           mmio_write_32(CV_XOR_DESQ_DEALLOC_REG(GlobalRegBase), 1); 
           indexNOD++;
       }
       
   return BCSIndex;
}

int XOR_Test(u32 TotalData2Test,u32 Source_AddressL, u32 Source_AddressH, u32 Destanation_AddressL, 
                  u32 Destanation_AddressH, int ByteCount,int PatternType,u32 DESQAddr,u32 DESQAddrH,int nNumOstdRd)
{
    int Destnation_Jap = 0;//ByteCount*2 + 8; // The 8 is not to be cas align, for 32bit DRAM nee to be 4.
    int WrBurstLen = 4;
    int RdBurstLen = 4;
    int NOD,pattern_length_cnt,data32 = 0;
    int res = 0;
    uintptr_t SourceDataSRAM[128];
    // This is a special case need to see that the ByteCount eqale to the SRAM memory allocated (512byte today)
    if(XORSourceSRAM == 1) {
        for(int x = 0 ; x < 64 ; x++) {
            SourceDataSRAM[2*x] = 0x0;
            SourceDataSRAM[2*x+1] = 0xFFFFFFFFFFFFFFFF;
        }
    }

    //u32 data = 0x0;
    uint64_t data = 0x0,data2 = 0x0;
    uintptr_t addr = 0x0;
    NOD = TotalData2Test/ByteCount;
    struct pattern_info *pattern_table = ddr3_tip_get_pattern_table();
    //NOD = NOD - 1;
   // printf("XOR_Test:: 0x%08x --> (0x%08x + NOD x 0x%x ) .. bytecount 0x%x - NOD = %d\n"
    //        ,Source_AddressL,Destanation_AddressL,Destnation_Jap,ByteCount,NOD);
    if(XORSourceSRAM == 1) {
        Source_AddressL = (u32)(((uintptr_t)&SourceDataSRAM[0])& 0x00000000FFFFFFFF);
        Source_AddressH =  (u32)(((uintptr_t)&SourceDataSRAM[0])>>32);
    }
    addr = ((uintptr_t)Source_AddressL) + (((uintptr_t)(Source_AddressH) )<< 32 ); 
    int NumOfLoop = ByteCount/(256*2*8);//The ODPG is 256
    int writethruodpg = (PatternType > 2)?MV_FALSE:MV_TRUE;
    if( (WriteXORPat == 1)){ // 1 - write, 0 - dont

        if(writethruodpg) {
            NumOfLoop = (NumOfLoop == 0)?1:NumOfLoop;
           // printf("TotalData2Test 0x%x & ByteCount %d --> NOD %d --> NumOfLoop %d\n",TotalData2Test, ByteCount,NOD,NumOfLoop);
            for(int NumOfLoopI = 0 ; NumOfLoopI<NumOfLoop; NumOfLoopI++) {

                if(PatternType == 0) {// 0 - is killer
                    for(int pattern = PATTERN_KILLER_DQ0; pattern <= PATTERN_KILLER_DQ7;pattern++) {
                        for (pattern_length_cnt = 0;pattern_length_cnt < pattern_table[pattern].pattern_len;
                              pattern_length_cnt++) {
                            data32 = pattern_table_get_word(0, pattern,(u8) (pattern_length_cnt));
                            
                            if(DataWidth == 0x3) {
                                cpu_write(addr,data32);
                                addr += 4;
                            }else{
                                data = ((uint64_t)data32) + ((uint64_t)((uint64_t)data32)<<32);
                                data |= 0xFFFFFFFFFFFFFF00; 
                                if(TypeOfMask == OR_Mask) {
                                    cpu_write(addr,(data|XorByteMask));
                                }else{
                                    cpu_write(addr,(data & XorByteMask));
                                }
                                
                                addr += 8;
                            }
                            
                        }//end pattern_length_cnt
                    }//end int pattern
                }

                if(PatternType == 1) {// 1 - is killer inv
                    for(int pattern = PATTERN_KILLER_DQ0_INV; pattern <= PATTERN_KILLER_DQ7_INV;pattern++) {
                        for (pattern_length_cnt = 0;pattern_length_cnt < pattern_table[pattern].pattern_len;
                              pattern_length_cnt++) {
                            data32 = pattern_table_get_word(0, pattern,(u8) (pattern_length_cnt));
                            if(DataWidth == 0x3) {
                                cpu_write(addr,data32);
                                addr += 4;
                            }else{
                                data = ((uint64_t)data32) + ((uint64_t)((uint64_t)data32)<<32);
                                if(TypeOfMask == OR_Mask) {
                                    cpu_write(addr,(data|XorByteMask));
                                }else{
                                    cpu_write(addr,(data & XorByteMask));
                                }
                                addr += 8;
                            }
                        }//end pattern_length_cnt
                    }
                }

                if(PatternType == 2) {// 2 - resonance
                    NumOfLoop = ByteCount/(256*8*8);//The ODPG is 256
                    for(int pattern = PATTERN_RESONANCE_2T; pattern <= PATTERN_RESONANCE_9T;pattern++) {
                        for(int index = 0 ; index < 8 ; index++) {
                            for (pattern_length_cnt = 0;pattern_length_cnt < pattern_table[pattern].pattern_len;
                                  pattern_length_cnt++) {
                                data32 = pattern_table_get_word(0, pattern,(u8) (pattern_length_cnt));
                                if(DataWidth == 0x3) {
                                cpu_write(addr,data32);
                                addr += 4;
                                }else{
                                    data = ((uint64_t)data32) + ((uint64_t)((uint64_t)data32)<<32);
                                    if(TypeOfMask == OR_Mask) {
                                    cpu_write(addr,(data|XorByteMask));
                                    }else{
                                        cpu_write(addr,(data & XorByteMask));
                                    }
                                    addr += 8;
                                }
                            }//end pattern_length_cnt
                        }
                    }
                }
                if(PatternType == 3) {//for 32bit only
                    for(int index = 0 ; index < 32 ; index++) {
                            int data = ~(0x0 | 1<<index);
                            for (pattern_length_cnt = 0;pattern_length_cnt < 128;pattern_length_cnt++) {
                                data32 = (pattern_length_cnt%2)? 0xFFFFFFFF:0x00000000;
                                //printf("data = 0x%08x ; data32 = 0x%08x --> ",data,data32);
                                data32 = data & data32;
                                //printf("data32 = 0x%08x\n ",data32);
                                addr = addr & 0x00000000FFFFFFFF;
                                mmio_write_32(addr,data32);
                                
                                addr += 4;
                            }//end pattern_length_cnt
                    }
                }
            }
        }else // write thru CPU,regardsless of the ODPG pattern
        {
            int NumOfByteInDevice = (DataWidth == 0x3)?(4):(8);
            NumOfLoop = (DataWidth == 0x3)?(ByteCount/4):(ByteCount/8);
            for(int NumOfLoopI = 0 ; NumOfLoopI<NumOfLoop; NumOfLoopI++) {
                if(PatternType == Moving_one_croosand_zero) {
                    if(NumOfLoopI == 0) {
                        data = 127;
                    }else{
                        data =data2&0xFF;
                        
                    }
                    if((NumOfLoopI%3 == 0)||(NumOfLoopI%3 == 1)) {
                        data = 255 - data;//inverse
                    }else{
                        data = (data==0xFE)?127:(128+ (data/2));
                    }
                         
                 }
                if(PatternType == Moving_zero_croosand_one) {
                    if(NumOfLoopI == 0) {
                        data = 128;
                    }else{
                        data =data2&0xFF;
                        
                    }
                    if((NumOfLoopI%3 == 0)||(NumOfLoopI%3 == 1)) {
                        data = 255 - data;
                    }else{
                        data = (data==0x1)?128:data/2;
                    }
                         
                 }
                if(PatternType == 5) {
                    data = pattern_table_get_word(0, PATTERN_KILLER_DQ0,
                                                  (u8) (NumOfLoopI%pattern_table[PATTERN_KILLER_DQ0].pattern_len));
                 }
                if(PatternType == 6) {
                    data = pattern_table_get_word(0, PATTERN_VREF,
                                                  (u8) (NumOfLoopI%pattern_table[PATTERN_KILLER_DQ0].pattern_len));
                 }
                if(PatternType == 7) {
                    data = pattern_table_get_word(0, PATTERN_RESONANCE_9T,
                                                  (u8) (NumOfLoopI%pattern_table[PATTERN_KILLER_DQ0].pattern_len));
                 }
                if(PatternType == 8) {
                    if(NumOfLoopI%2 == 0) {
                        data = 0x0;
                    }else{
                        data =0xFFFFFFFFFFFFFFFF;
                        
                    }
                                            
                 }
                data2 = 0x0;
                int shift = 0;
                for(int byteindex = 0; byteindex < NumOfByteInDevice; byteindex++){
                    shift = shift + 8*byteindex;
                    data2 |= ((data%256) << (shift));
                 }
                
                if(TypeOfMask == OR_Mask) {
                    cpu_write(addr,(data2|XorByteMask));
                }else{
                    cpu_write(addr,(data2 & XorByteMask));
                }
               // printf("cpu_write:\t\t0x%16lx\t0x%16lx\n",addr,data2);
                addr = addr + NumOfByteInDevice;
            }
        }
    } // 
    for(int Rp = 0 ; Rp < 1 ; Rp++) {
    if(XORS2D == 1)  {// 1 - Do, 0 - skip
        XOR_S2D_func(NOD, ByteCount, Source_AddressL, Source_AddressH, Destanation_AddressL, 
                      Destanation_AddressH, Destnation_Jap, WrBurstLen, RdBurstLen,DESQAddr,DESQAddrH, nNumOstdRd);
    }
	/* get rd and wr transaction counters of ddr rd and wr axi monitors */
	//int rd_cnt = axim_counter_get(&axim_ddr_rd);
	//printf("########### axim read before 0x%x\n", rd_cnt);
    //Counter(1);
    
        res += XOR_ScmpD_func(NOD, ByteCount, Source_AddressL, Source_AddressH, Destanation_AddressL, 
                      Destanation_AddressH, Destnation_Jap, RdBurstLen,DESQAddr,DESQAddrH, nNumOstdRd);
        Rp = (res > 0)? 20:Rp;

	//Counter(2);
	//rd_cnt = axim_counter_get(&axim_ddr_rd);
	//printf("########### axim dread after 0x%x\n", rd_cnt);
    }

    return res;
}


int XOT_Test_ConfigurationCheck(unsigned long long Source_Address, u32 TotalData2Test, unsigned long long End_addr_bytes)
{
    if(
       (    (   (   Source_Address + TotalData2Test) >=0xF0000000) && ((Source_Address + TotalData2Test) < 0x100000000)) || 
       (    (   (   Source_Address ) >=0xF0000000  ) && (  (Source_Address ) < 0x100000000 )    )      
       )
    {
        if(printlog < 4 ) {
            printf("No Valid test place - source position \n");
        }
        return 1;
    }
    if(((Source_Address + TotalData2Test) > End_addr_bytes) && 
                   ((Source_Address) < End_addr_bytes) ) {
        if(printlog < 4) {
            printf("No Valid test place - Source + data cross the destination \n");
        }
        return 1;
    }
    if(((End_addr_bytes + TotalData2Test) > Source_Address) &&
                    ((End_addr_bytes) < Source_Address) ) {
        if(printlog < 4) {
            printf("No Valid test place - destination + data cross the Source \n");
        }
        return 1;
    }
    if((((End_addr_bytes + TotalData2Test) >=0xF0000000) && ((End_addr_bytes + TotalData2Test) < 0x100000000)) || 
       (((End_addr_bytes ) >=0xF0000000) && ((End_addr_bytes ) < 0x100000000)) )
    {
        if(printlog < 4) {
            printf("No Valid test place - destination position \n");
        }
        return 1;
    }
    return 0;
}

int BA2BA_Test(int PatternType){
    
    int BankMap = (reg_read(MC6_REG_MC_CONFIG(0))&(0x1F<<24))>>24;
    BankMap = BankMap_conv[BankMap-3];
    int TotalData2Test = BankMap/4;
    
    for (int BAnum = 0;BAnum < (8-1);BAnum++) {
        u32 Source_AddressL = BankMap*BAnum + BankMap/4; 
        u32 Source_AddressH = 0x0;
        u32 Destanation_AddressL = Source_AddressL + 2 * BankMap/4;
        u32 Destanation_AddressH = 0x0;
        int ByteCount = TotalData2Test/64;
        u32 DESQAddr = BankMap*BAnum ;
        u32 DESQAddrH = 0x0;

        if(printlog == 1) {
            printf("ValidationPlatform: CS0 Inside the BA XOR test ");
            printf("\n\tDescBaseAddr[H,L] = [0x%04x\t,\t0x%08x] ",DESQAddrH,DESQAddr);
            printf("\n\tSource_Address[H,L] = [0x%04x\t,\t0x%08x] ", Source_AddressH,Source_AddressL);
            printf("\n\tDestanation_Address[H,L] = [0x%04x\t,\t0x%08x] ", Destanation_AddressH,Destanation_AddressL);
                
        }
        XOR_Test(TotalData2Test, Source_AddressL, Source_AddressH, Destanation_AddressL, 
                          Destanation_AddressH, ByteCount, PatternType, DESQAddr, DESQAddrH,NumOstdRd);
    }

    return 0;
}
/******************************************************************************************** 
InnerPagetransactions - The test will read and write data between a pages will cover all pages 
                        one bank
*********************************************************************************************/
int InnerPagetransactions(void){
    
    u32 TotalData2Test;
    int CSindex = 0;
    enum VP_pattern Pattern = VP_pattern_Last;
    int ByteCount;
   // int index=1;
   int NumOfPageInBank = BankMap*_1K/PageSize;
   int PageIndex2Test = 1000 + 8*NumOfPageInBank ;
    int PageIndex = 0;
    int NOD[5] = {10,6,4,3,2};
    int IndexLim = 5;
    int ret=0;
    int patternStart = (Pattern == VP_pattern_Last)? 0:Pattern;
    int patternEnd   = (Pattern == VP_pattern_Last)? Pattern:Pattern+1;
    

    u32 Source_AddressL = (reg_read(MC6_REG_MMAP_LOW_CH0(0))&(0x1FF<<23));
    u32 Source_AddressH = reg_read(MC6_REG_MMAP_HIGH_CH0(0)); 
 
    unsigned long long tmp_Source_Address = (((unsigned long long)Source_AddressH<<32)| ((unsigned long long)Source_AddressL) ) 
                                        + (unsigned long long)((CSindex)* Area_length) ;
                                        
    unsigned long long tmp_End_addr_bytes = (((unsigned long long)Source_AddressH<<32)| ((unsigned long long)Source_AddressL) ) 
                                        + (unsigned long long)((CSindex)* Area_length) ;
                                           

    if(PageSize == _1K){
        IndexLim = 4;
        for(int index = 0;index < IndexLim;index++ ) {
            NOD[index] = NOD[index] / 2 ;
        }
    }
    int IndexTH = PageIndex2Test/4;
    for(PageIndex = 0 ; PageIndex < PageIndex2Test ; PageIndex++) {
        if(PageIndex == NumOfPageInBank) {
            PageIndex = PageIndex*8;
        }
         if(PageIndex%IndexTH==0) {
                    printf("\n----------------------------------");
                    printf(" InnerPage transactions PageIndex = %d (%d)",PageIndex,PageIndex2Test);
                    printf("----------------------------------\n");
                    printlog = 1;

         }
        
        for(int index = 0 ; index < IndexLim ; index++) {
            int desc = NOD[index]*0x40;
            TotalData2Test = ((int)(((PageSize - desc)/2)/64))*64;
            ByteCount = TotalData2Test/NOD[index];

            unsigned long long End_addr_bytes = tmp_End_addr_bytes + (unsigned long long)PageIndex * PageSize + (unsigned long long)desc + (unsigned long long)TotalData2Test;
            u32 Destanation_AddressL =  (u32)((End_addr_bytes)& 0x00000000FFFFFFFF);
            u32 Destanation_AddressH =  (u32)((End_addr_bytes)>>32);
            
            unsigned long long Source_Address = tmp_Source_Address + (unsigned long long)PageIndex * PageSize + (unsigned long long)desc;
            Source_AddressL =  (u32)((Source_Address)& 0x00000000FFFFFFFF);
            Source_AddressH =  (u32)((Source_Address)>>32);
            ////////////////////////////////////////////////////////
            int CheckFlag = XOT_Test_ConfigurationCheck(Source_Address, TotalData2Test, End_addr_bytes);
            if(CheckFlag == 1) {
                continue;//go to next
            }
            ////////////////////////////////////////////////////////
            /////using the Source_Address parameter to calculate the descriptor position
            Source_Address = tmp_Source_Address + (unsigned long long)PageIndex * PageSize ;

            u32 DescBaseAddrL = (u32)((Source_Address)& 0x00000000FFFFFFFF);
            u32 DescBaseAddrH = (u32)((Source_Address)>>32);
            ///////////////////////////////////////////////////////
            XorPrint(TotalData2Test,ByteCount,DescBaseAddrL,DescBaseAddrH,Source_AddressL,Source_AddressH,
                     Destanation_AddressL,Destanation_AddressH);
            
            ///////////////////////////////////////////////////////
            for(int patI = patternStart ; patI < patternEnd ; patI++) {
                ret += XOR_Test(TotalData2Test,Source_AddressL, Source_AddressH, Destanation_AddressL,Destanation_AddressH, ByteCount,patI,DescBaseAddrL,DescBaseAddrH,NumOstdRd);
                if(printlog < 5) printf("\n\t\t- %d errors on pattern %d\t;\tRun Time[HCLK]: 0x%04x.%08x",ret,patI,Hclk_CounterHigh,Hclk_CounterLow);
                if(ret != 0) {
                    printf("\tSource[H,L] -> Destanation[H,L]= [0x%04x,0x%08x]-> ", Source_AddressH,Source_AddressL);
                    printf("[0x%04x,0x%08x] ", Destanation_AddressH,Destanation_AddressL);
                }
            }
            if((PageIndex==1) || (PageIndex%IndexTH==0)) {
                printlog = 5;
            }
        }
    }

    return ret;

}
/******************************************************************************************** 
Page2Pagetransactions - The test will read and write data between different pages on the same Bank
*********************************************************************************************/
int Page2Pagetransactions(void){
    
   int ret=0;
   u32 TotalData2Test;
    int CSindex = 0;
    enum VP_pattern Pattern = VP_pattern_Last;
    int ByteCount;
   // int index=1;
   int NumOfPageInBank = BankMap*_1K/PageSize;
   int PageIndex2Test = 1000 + 8*NumOfPageInBank ;
    int PageIndex = 0;
    int NOD[5] = {1,2,4,8,16};
    int IndexLim = 5;
    
    int patternStart = (Pattern == VP_pattern_Last)? 0:Pattern;
    int patternEnd   = (Pattern == VP_pattern_Last)? Pattern:Pattern+1;
    

    u32 Source_AddressL = (reg_read(MC6_REG_MMAP_LOW_CH0(0))&(0x1FF<<23));
    u32 Source_AddressH = reg_read(MC6_REG_MMAP_HIGH_CH0(0)); 
 
    unsigned long long tmp_Source_Address = (((unsigned long long)Source_AddressH<<32)| ((unsigned long long)Source_AddressL) ) 
                                        + (unsigned long long)((CSindex)* Area_length) ;
                                        
    unsigned long long tmp_End_addr_bytes = (((unsigned long long)Source_AddressH<<32)| ((unsigned long long)Source_AddressL) ) 
                                        + (unsigned long long)((CSindex)* Area_length) ;
                                           

    if(PageSize == _1K){
        IndexLim = 4;
    }
    int IndexTH = (PageIndex2Test-1)/4;
    for(PageIndex = 0 ; PageIndex < (PageIndex2Test-1) ; PageIndex++) {
        if(PageIndex == (NumOfPageInBank-1)) {
            PageIndex = NumOfPageInBank*8;
        }
        if(PageIndex%IndexTH==0) {
                    printf("\n----------------------------------");
                    printf(" Page 2 Page transactions PageIndex = %d (%d)",PageIndex,PageIndex2Test-1);
                    printf("----------------------------------\n");
                    printlog = 1;

         }
        
        for(int index = 0 ; index < IndexLim ; index++) {
            int desc = NOD[index]*0x40;
            TotalData2Test = ((int)(((PageSize - desc))/64))*64;
            ByteCount = TotalData2Test/NOD[index];

            unsigned long long End_addr_bytes = tmp_End_addr_bytes + (unsigned long long)(PageIndex+1) * PageSize  ;
            u32 Destanation_AddressL =  (u32)((End_addr_bytes)& 0x00000000FFFFFFFF);
            u32 Destanation_AddressH =  (u32)((End_addr_bytes)>>32);
            
            unsigned long long Source_Address = tmp_Source_Address + (unsigned long long)PageIndex * PageSize + (unsigned long long)desc;
            Source_AddressL =  (u32)((Source_Address)& 0x00000000FFFFFFFF);
            Source_AddressH =  (u32)((Source_Address)>>32);
            ////////////////////////////////////////////////////////
            int CheckFlag = XOT_Test_ConfigurationCheck(Source_Address, TotalData2Test, End_addr_bytes);
            if(CheckFlag == 1) {
                continue;//go to next
            }
            ////////////////////////////////////////////////////////
            /////using the Source_Address parameter to calculate the descriptor position
            Source_Address = tmp_Source_Address + (unsigned long long)PageIndex * PageSize ;

            u32 DescBaseAddrL = (u32)((Source_Address)& 0x00000000FFFFFFFF);
            u32 DescBaseAddrH = (u32)((Source_Address)>>32);
            ///////////////////////////////////////////////////////
            XorPrint(TotalData2Test,ByteCount,DescBaseAddrL,DescBaseAddrH,Source_AddressL,Source_AddressH,
                     Destanation_AddressL,Destanation_AddressH);
            
            ///////////////////////////////////////////////////////
            for(int patI = patternStart ; patI < patternEnd ; patI++) {
                ret += XOR_Test(TotalData2Test,Source_AddressL, Source_AddressH, Destanation_AddressL,Destanation_AddressH, ByteCount,patI,DescBaseAddrL,DescBaseAddrH,NumOstdRd);
                if(printlog < 5) printf("\n\t\t- %d errors on pattern %d\t;\tRun Time[HCLK]: 0x%04x.%08x",ret,patI,Hclk_CounterHigh,Hclk_CounterLow);
                if(ret != 0) {
                    printf("\tSource[H,L] -> Destanation[H,L]= [0x%04x,0x%08x]-> ", Source_AddressH,Source_AddressL);
                    printf("[0x%04x,0x%08x] ", Destanation_AddressH,Destanation_AddressL);
                }
            }
            if((PageIndex==1) || (PageIndex%IndexTH==0)) {
                printlog = 5;
            }
        }
    }

    return ret;

}
/******************************************************************************************** 
InnerBanktransactions - The test will read and write data on the same bank address will cover 
                        all banks in the bank Group
*********************************************************************************************/
int InnerBanktransactions(void){
    
    u32 TotalData2Test;
    int CSindex = 0;
    enum VP_pattern Pattern = VP_pattern_Last;
    int ByteCount;
   // int index=1;
    int NumOfBankInBG = (BankMap*_1K)*8;
    int BankIndex = 0;
    int BankIndex2Test = 1000;
    
    int ret=0;
    int patternStart = (Pattern == VP_pattern_Last)? 0:Pattern;
    int patternEnd   = (Pattern == VP_pattern_Last)? Pattern:Pattern+1;
    

    u32 Source_AddressL = (reg_read(MC6_REG_MMAP_LOW_CH0(0))&(0x1FF<<23));
    u32 Source_AddressH = reg_read(MC6_REG_MMAP_HIGH_CH0(0)); 
 
                                    
    unsigned long long tmp_Source_Address = (((unsigned long long)Source_AddressH<<32)| ((unsigned long long)Source_AddressL) ) 
                                        + (unsigned long long)((CSindex)* Area_length) ;
                                        
    unsigned long long tmp_End_addr_bytes = (((unsigned long long)Source_AddressH<<32)| ((unsigned long long)Source_AddressL) ) 
                                        + (unsigned long long)((CSindex)* Area_length) ;

    int IndexTH = BankIndex2Test/4;
    for(BankIndex = 1 ; BankIndex < BankIndex2Test ; BankIndex++) {
         if(BankIndex%IndexTH==0) {
                    printf("\n----------------------------------");
                    printf(" InnerBank transactions PageIndex = %d (%d)",BankIndex,BankIndex2Test);
                    printf("----------------------------------\n");
                    printlog = 1;

         }
        
        
       int nod = (BankIndex%10) + 1; 
       int desc = nod*0x40;
       TotalData2Test = ((int)((((BankMap*_1K) - desc)/2)/64))*64;
       ByteCount = TotalData2Test/nod;

       unsigned long long End_addr_bytes = tmp_End_addr_bytes + (unsigned long long)BankIndex * NumOfBankInBG + (unsigned long long)desc + (unsigned long long)TotalData2Test;
       u32 Destanation_AddressL =  (u32)((End_addr_bytes)& 0x00000000FFFFFFFF);
       u32 Destanation_AddressH =  (u32)((End_addr_bytes)>>32);
       
       unsigned long long Source_Address = tmp_Source_Address + (unsigned long long)BankIndex * NumOfBankInBG + (unsigned long long)desc;
       Source_AddressL =  (u32)((Source_Address)& 0x00000000FFFFFFFF);
       Source_AddressH =  (u32)((Source_Address)>>32);
       ////////////////////////////////////////////////////////
       int CheckFlag = XOT_Test_ConfigurationCheck(Source_Address, TotalData2Test, End_addr_bytes);
       if(CheckFlag == 1) {
           continue;//go to next
       }
       ////////////////////////////////////////////////////////
       /////using the Source_Address parameter to calculate the descriptor position
       Source_Address = tmp_Source_Address + (unsigned long long)BankIndex * NumOfBankInBG ;

       u32 DescBaseAddrL = (u32)((Source_Address)& 0x00000000FFFFFFFF);
       u32 DescBaseAddrH = (u32)((Source_Address)>>32);
       ///////////////////////////////////////////////////////
       XorPrint(TotalData2Test,ByteCount,DescBaseAddrL,DescBaseAddrH,Source_AddressL,Source_AddressH,
                Destanation_AddressL,Destanation_AddressH);
       
       ///////////////////////////////////////////////////////
       for(int patI = patternStart ; patI < patternEnd ; patI++) {
           ret += XOR_Test(TotalData2Test,Source_AddressL, Source_AddressH, Destanation_AddressL,Destanation_AddressH, ByteCount,patI,DescBaseAddrL,DescBaseAddrH,NumOstdRd);
           if(printlog < 5) printf("\n\t\t- %d errors on pattern %d\t;\tRun Time[HCLK]: 0x%04x.%08x",ret,patI,Hclk_CounterHigh,Hclk_CounterLow);
           if(ret != 0) {
               printf("\tSource[H,L] -> Destanation[H,L]= [0x%04x,0x%08x]-> ", Source_AddressH,Source_AddressL);
               printf("[0x%04x,0x%08x] ", Destanation_AddressH,Destanation_AddressL);
           }
       }
       if((BankIndex==1) || (BankIndex%IndexTH==0)) {
           printlog = 5;
       }
        
    }

    return ret;

}
/******************************************************************************************** 
  XOR Graduate test
  The Target of this test is to save time by finding a fast fail if exist. To give the
  ability to have a massive test for harsh systems. The XOR test needs to see that it
  does not access the non-permitted zone (3.75G-4G). In addition and not to accede
  128 descriptors - so they all be inside the SRAM memory space.
*********************************************************************************************/ 
int XOR_Gradual_Test(u8 DepthStage)
{
    u32 TotalData2Test;
    
    enum VP_pattern Pattern = 0; //All_AggresiveInv;//All_Aggresive;//Moving_one_croosand_zero
    int ByteCount;
    int ret=0;
    u32 Destanation_AddressL ;
    u32 Destanation_AddressH ;
    u32 Source_AddressL = (reg_read(MC6_REG_MMAP_LOW_CH0(0))&(0x1FF<<23));
    u32 Source_AddressH = reg_read(MC6_REG_MMAP_HIGH_CH0(0)); 
    unsigned long long tmpArea_length = 0;
 
                                    
    unsigned long long Source_Address = (((unsigned long long)Source_AddressH<<32)| ((unsigned long long)Source_AddressL) ) 
                                        + (unsigned long long)((effective_cs)* Area_length) ;
                                        
    unsigned long long End_addr_bytes = (((unsigned long long)Source_AddressH<<32)| ((unsigned long long)Source_AddressL) ) 
                                        + (unsigned long long)((effective_cs+1)* Area_length) ;

    Source_AddressL =  (u32)((Source_Address)& 0x00000000FFFFFFFF);
    Source_AddressH =  (u32)((Source_Address)>>32);
    // preventing from accessing to 3.75G - 4G
    tmpArea_length = Area_length;
    if((Area_length == _4G)&&(effective_cs == 0)) {
        tmpArea_length = _4G - _1G;
    }
    if((Area_length == _2G)&&(effective_cs == 1)) {
        tmpArea_length = _4G - _1G;
    }
    
    /******************************************************************/
    /* Depth 0                                                      ***/
    if(DepthStage == 0) {
        
        ByteCount = 1024;
        TotalData2Test = ByteCount*120;
        Pattern = Moving_one_croosand_zero;//All_Aggresive;
        End_addr_bytes =  (unsigned long long)((effective_cs+1)* tmpArea_length) - (unsigned long long)(3*TotalData2Test);
        Destanation_AddressL =  (u32)((End_addr_bytes)& 0x00000000FFFFFFFF);
        Destanation_AddressH =  (u32)((End_addr_bytes)>>32);
        XORSourceSRAM = 1;
        ret += XOR_Test(TotalData2Test,Source_AddressL, Source_AddressH, Destanation_AddressL,Destanation_AddressH, ByteCount,Pattern,0x0,0x0,NumOstdRd);
        XORSourceSRAM = 0;
        if(ret != 0)    return MV_FAIL;
        if(DepthStage == 0) return MV_OK;
    }
    /******************************************************************/
    /* Depth 1                                                      ***/
    TotalData2Test = _8K;
    ByteCount = TotalData2Test/8;
    Pattern = 8;//SSO
    XORSourceSRAM = 0;
    End_addr_bytes =   (unsigned long long)((effective_cs+1)* tmpArea_length) - (unsigned long long)(3*TotalData2Test);
    Destanation_AddressL =  (u32)((End_addr_bytes)& 0x00000000FFFFFFFF);
    Destanation_AddressH =  (u32)((End_addr_bytes)>>32);

    ret += XOR_Test(TotalData2Test,Source_AddressL, Source_AddressH, Destanation_AddressL,Destanation_AddressH, ByteCount,Pattern,0x0,0x0,NumOstdRd);
    if(ret != 0)    return ret;
    if(DepthStage == 1) return MV_OK;
    /******************************************************************/
    /* Depth 2                                                      ***/
    TotalData2Test = _8K;
    ByteCount = TotalData2Test/8;
    Pattern = Moving_one_croosand_zero;
    XORSourceSRAM = 0;
    End_addr_bytes =   (unsigned long long)((effective_cs+1)* tmpArea_length) - (unsigned long long)(3*TotalData2Test);
    Destanation_AddressL =  (u32)((End_addr_bytes)& 0x00000000FFFFFFFF);
    Destanation_AddressH =  (u32)((End_addr_bytes)>>32);

    ret += XOR_Test(TotalData2Test,Source_AddressL, Source_AddressH, Destanation_AddressL,Destanation_AddressH, ByteCount,Pattern,0x0,0x0,NumOstdRd);
    if(ret != 0)    return ret;
    if(DepthStage == 2) return MV_OK;
    /******************************************************************/
    /* Depth 3                                                      ***/
    TotalData2Test = _8M;
    ByteCount = TotalData2Test/64;
    Pattern = All_AggresiveInv;
    End_addr_bytes =   (unsigned long long)((effective_cs+1)* tmpArea_length) - (unsigned long long)(3*TotalData2Test);
    Destanation_AddressL =  (u32)((End_addr_bytes)& 0x00000000FFFFFFFF);
    Destanation_AddressH =  (u32)((End_addr_bytes)>>32);

    ret += XOR_Test(TotalData2Test,Source_AddressL, Source_AddressH, Destanation_AddressL,Destanation_AddressH, ByteCount,Pattern,0x0,0x0,NumOstdRd);
    if(ret != 0)    return ret;
    if(DepthStage == 3) return MV_OK;
    /******************************************************************/
    /* Depth 4                                                      ***/
    
    u32 sourcoffset = _1M;
    u32 destoffset = _256M;//from source of CS
    for(int index  = 3 ; index < 6 ;index++)
    {
        index = (index == 2)?3:index; // skip pattern 2 is CHARA
        TotalData2Test = _4M;
        ByteCount = TotalData2Test/2048;
        Pattern = index;//(Pattern == Moving_zero_croosand_one)?Moving_one_croosand_zero:Moving_zero_croosand_one ;
        XORSourceSRAM = 0;//data is inside the memory
        Source_Address =   (unsigned long long)((effective_cs)* tmpArea_length) + (unsigned long long)(sourcoffset);
        Source_AddressL =  (u32)((Source_Address)& 0x00000000FFFFFFFF);
        Source_AddressH =  (u32)((Source_Address)>>32);
        End_addr_bytes =  (unsigned long long)((effective_cs)* tmpArea_length) + (unsigned long long)(destoffset);
        Destanation_AddressL =  (u32)((End_addr_bytes)& 0x00000000FFFFFFFF);
        Destanation_AddressH =  (u32)((End_addr_bytes)>>32);
		ret += XOR_TestV(TotalData2Test,Source_AddressL,Source_AddressH,Destanation_AddressL,Destanation_AddressH,ByteCount,Pattern,NumOstdRd);//adam
        //ret += XOR_Test(TotalData2Test,Source_AddressL, Source_AddressH, Destanation_AddressL,Destanation_AddressH, ByteCount,Pattern,0x0,0x0,NumOstdRd);
        if(ret != 0)    return ret;       
    }
    if(DepthStage >= 4) return MV_OK;
    /******************************************************************/
    /* Depth 5                                                      ***/
    Destanation_AddressL = Source_AddressL;
    Destanation_AddressH = Source_AddressH;
    for(int index  = 4 ; index < ((tmpArea_length / _64K)-1) ;index=index*2)
    {
        TotalData2Test = _64K;
        ByteCount = TotalData2Test/2;
        Pattern = (Pattern == Moving_zero_croosand_one)?Moving_one_croosand_zero:Moving_zero_croosand_one ;
        XORSourceSRAM = 0;
        Source_Address =   (unsigned long long)((effective_cs)* tmpArea_length) + (unsigned long long)(index*TotalData2Test);
        Source_AddressL =  (u32)((Source_Address)& 0x00000000FFFFFFFF);
        Source_AddressH =  (u32)((Source_Address)>>32);

        ret += XOR_Test(TotalData2Test,Source_AddressL, Source_AddressH, Destanation_AddressL,Destanation_AddressH, ByteCount,Pattern,0x0,0x0,NumOstdRd);
        if(ret != 0)    return ret;
        

    }
    if(DepthStage >= 4) return MV_OK;
    return MV_OK;    
} 
/******************************************************************************************** 
  1D_XORsearch(PF, H2L,step, Rcindex-1,AlgorithmLow)
 
*********************************************************************************************/
int XORsearch_1D_1E(enum hws_edge_compare edge, enum hws_search_dir search_dir , u32 Step, u32 init_val,u32 end_val,u8 DepthStage,u16 ByteNum, enum  SearchElement element)
{
    int result = (edge == EDGE_PF)? 0 : 1;// If pass start from 0(pass)
    int SignStep = (search_dir == HWS_LOW2HIGH)? Step:(-Step);
    int Param = (int)init_val;
    u32 NumOfSteps = (search_dir == HWS_LOW2HIGH)?((end_val-init_val)/Step):((init_val - end_val)/Step);
    u32 StepIndex = 0;
    u32 Regadd = (element == CRx)?(READ_CENTRALIZATION_PHY_REG):(WRITE_CENTRALIZATION_PHY_REG);
    Regadd +=  + 4*effective_cs;
    if(edge == EDGE_PF) {
       // printf("\nXORsearch_1D: NumOfSteps %d end_val %d init_val %d Step %d\n",NumOfSteps,end_val,init_val,Step);
    }
    for(StepIndex = 0 ;StepIndex <= NumOfSteps ; StepIndex++) {
        if(element == RecCal){
            
            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, \
                                CSN_IOB_VREF_REG(effective_cs), Param);
            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, \
                                CSN_IO_BASE_VREF_REG(effective_cs)+4, Param);
            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, \
                            CSN_IO_BASE_VREF_REG(effective_cs)+5, Param);
        }else
        {
            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, \
                                Regadd, Param);
        }
        result = XOR_Gradual_Test(DepthStage);
       if(edge == EDGE_PF) {
          // printf("\t\t%2d) %d,",Param,result);
       }
        if(((result != 0) && (edge == EDGE_PF))|| ((result == 0) && (edge == EDGE_FP))) {
            return Param;
        }
        Param = Param + SignStep;
    }
   // printf("\n");
   if((edge == EDGE_PF)) {
        return (Param-SignStep); // All the search are on positive numbers so -1 is representinga fail.
   }else
       return 255;

}
/******************************************************************************************** 
  2D_XORsearch Support EDGE_FPF only
 
*********************************************************************************************/
int XORsearch_1D_2E(enum hws_edge_compare SearchConcept, u32 Step, u32 init_val,u32 end_val,u8 DepthStage,u16 ByteNum, enum  SearchElement element,u8 (*VW_vector))
{
    u32 Regadd = (element == CRx)?(READ_CENTRALIZATION_PHY_REG):(WRITE_CENTRALIZATION_PHY_REG);
    Regadd +=  + 4*effective_cs;
    VW_vector[0] = end_val;
    VW_vector[1] = init_val;
    u32 reg_data;
    if(element == RecCal) {
        ddr3_tip_bus_read(0, 0,ACCESS_TYPE_UNICAST, ByteNum,DDR_PHY_DATA,CSN_IOB_VREF_REG(effective_cs),&reg_data);
    }else if(element == CRx){
        ddr3_tip_bus_read(0, 0,ACCESS_TYPE_UNICAST, ByteNum,DDR_PHY_DATA,READ_CENTRALIZATION_PHY_REG + 4*effective_cs,&reg_data);
    }else if(element == CTx){
        ddr3_tip_bus_read(0, 0,ACCESS_TYPE_UNICAST, ByteNum,DDR_PHY_DATA,WRITE_CENTRALIZATION_PHY_REG + 4*effective_cs,&reg_data);
    }

    /*******************************/
    /* XORsearch_1D_1E return the edge transition value of the elements */
    if(SearchConcept == EDGE_FP) {
        VW_vector[0] = XORsearch_1D_1E(EDGE_FP,HWS_LOW2HIGH,Step,init_val,end_val,DepthStage,ByteNum, element);
        VW_vector[0] = (VW_vector[0] == reg_data)? 255:VW_vector[0];
        VW_vector[1] = XORsearch_1D_1E(EDGE_FP,HWS_HIGH2LOW,Step,end_val,init_val,DepthStage,ByteNum, element);
        VW_vector[1] = (VW_vector[1] == reg_data)? 255:VW_vector[1];
        
    }else
    {
        //init_val = reg_data;// we start from the nominal
        VW_vector[1] = XORsearch_1D_1E(EDGE_PF,HWS_LOW2HIGH,Step,reg_data,end_val,DepthStage,ByteNum, element);
        //printf("\tVW_vector[1] = %d \n",VW_vector[1]);
        
        
        VW_vector[0] = XORsearch_1D_1E(EDGE_PF,HWS_HIGH2LOW,Step,reg_data,init_val,DepthStage,ByteNum, element);
        //printf("\tVW_vector[0] = %d \n",VW_vector[0]);
        if((VW_vector[1] == reg_data)&&(VW_vector[0] == reg_data)) {
            VW_vector[1] = 255;
            VW_vector[0] = 255;
        }else{
            VW_vector[1] = VW_vector[1] /*- Step*/;
            VW_vector[0] = VW_vector[0] /*+ Step*/;
        }
        //printf("\tVW_vector[0] = %d \n",VW_vector[0]);
        //printf("\tVW_vector[1] = %d \n",VW_vector[1]);
    }
    /*******************************/

    if(element == RecCal) {
        ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, \
                                CSN_IOB_VREF_REG(effective_cs), reg_data);
        ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, \
                                CSN_IO_BASE_VREF_REG(effective_cs)+4, reg_data);
        ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, \
                            CSN_IO_BASE_VREF_REG(effective_cs)+5, reg_data);
    }else if(element == CRx){
        ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, \
                                READ_CENTRALIZATION_PHY_REG + 4*effective_cs, reg_data);
    }else if(element == CTx){
        ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, \
                                WRITE_CENTRALIZATION_PHY_REG + 4*effective_cs, reg_data);
    }
    /******************************/
    if ((VW_vector[1] == 255)||(VW_vector[0] == 255)) {
        return 255;
    }else
        if(VW_vector[1]<VW_vector[0]) {
            if(SearchConcept == EDGE_PF) {
                //printf("\nXORsearch_1D_2E: end_val %d init_val %d Step %d Nominal %d\n",end_val,init_val,Step,reg_data);
            }
    
            return 0;
    }
        else
            return (VW_vector[1]-VW_vector[0]);
    
    
    return 0;

}
/******************************************************************************************** 
  1D_XORsearch(PF, H2L,step, Rcindex-1,AlgorithmLow)
 
*********************************************************************************************/
int XORsearch_2D_1E(enum hws_edge_compare edge, enum  SearchElement element1, enum hws_search_dir search_dir1 , u32 Step1, u32 init_val1,u32 end_val1,\
                                                enum  SearchElement element2, enum hws_search_dir search_dir2 , u32 Step2, u32 init_val2,u32 end_val2,\
                    u8 DepthStage,u16 ByteNum)
{
    int result = (edge == EDGE_PF)? 0 : 1;// If pass start from 0(pass)
    int SignStep1 = (search_dir1 == HWS_LOW2HIGH)? Step1:(-Step1);
    int Param1 = (int)init_val1;
    u32 NumOfSteps1 = (search_dir1 == HWS_LOW2HIGH)?((end_val1-init_val1)/Step1):((init_val1 - end_val1)/Step1);
    int SignStep2 = (search_dir2== HWS_LOW2HIGH)? Step2:(-Step2);
    int Param2 = (int)init_val2;
    u32 NumOfSteps2 = (search_dir2 == HWS_LOW2HIGH)?((end_val2-init_val2)/Step2):((init_val2 - end_val2)/Step2);
    u32 NumOfSteps = (NumOfSteps1 < NumOfSteps2)?NumOfSteps1:NumOfSteps2;
    u32 StepIndex = 0;
    u32 Regadd1 = READ_CENTRALIZATION_PHY_REG;
    Regadd1 +=  + 4*effective_cs;
    //printf("\tXORsearch_2D_1E[%d]\t\t[%2d,%2d]-->",ByteNum,Param1,Param2);
    
    for(StepIndex = 0 ;StepIndex <= NumOfSteps ; StepIndex++) {
        ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, \
                                CSN_IOB_VREF_REG(effective_cs), Param2);
        ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, \
                                CSN_IO_BASE_VREF_REG(effective_cs)+4, Param2);
        ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, \
                            CSN_IO_BASE_VREF_REG(effective_cs)+5, Param2);

        ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, \
                                Regadd1, Param1);
        
        result = XOR_Gradual_Test(DepthStage);
       // printf("\t\t[%2d,%2d] --> %d\n",Param1,Param2,result);
        if(((result != 0) && (edge == EDGE_PF))|| ((result == 0) && (edge == EDGE_FP))) {
            //printf("[%2d,%2d]\t\t",Param1,Param2);
            return StepIndex;
        }
        Param1 = Param1 + SignStep1;
        Param2 = Param2 + SignStep2;
    }
    //printf("\n");
    return 255; // All the search are on positive numbers so -1 is representinga fail.

}
/******************************************************************************************** 
  2D_XORsearch Support EDGE_FPF only
 
*********************************************************************************************/
int XORsearch_2D_2E(enum hws_edge_compare edge, enum  SearchElement element1, enum hws_search_dir search_dir1 , u32 Step1, u32 init_val1,u32 end_val1,
                                                enum  SearchElement element2, enum hws_search_dir search_dir2 , u32 Step2, u32 init_val2,u32 end_val2,
                    u8 DepthStage,u16 ByteNum, u8 (*VW_vector))
{
    u32 Regadd = (READ_CENTRALIZATION_PHY_REG);
    Regadd +=  + 4*effective_cs;
    VW_vector[0] = end_val1;
    VW_vector[1] = init_val1;
    u32 reg_data1,reg_data2;
    // Read the nominal search element
    ddr3_tip_bus_read(0, 0,ACCESS_TYPE_UNICAST, ByteNum,DDR_PHY_DATA,CSN_IOB_VREF_REG(effective_cs),&reg_data2);
    ddr3_tip_bus_read(0, 0,ACCESS_TYPE_UNICAST, ByteNum,DDR_PHY_DATA,READ_CENTRALIZATION_PHY_REG + 4*effective_cs,&reg_data1);
    
   // printf("\nXORsearch_2D_2E[%d]\tNom:[%2d,%2d]----",ByteNum,reg_data1,reg_data2);
    /*******************************/
    VW_vector[0] = XORsearch_2D_1E(EDGE_FP, element1, search_dir1 , Step1, init_val1, end_val1, 
                                         element2, search_dir2 , Step2, init_val2, end_val2,    
                                         DepthStage,ByteNum);
    search_dir2 = (search_dir2 == HWS_LOW2HIGH)?HWS_HIGH2LOW:HWS_LOW2HIGH;
    search_dir1 = (search_dir1 == HWS_LOW2HIGH)?HWS_HIGH2LOW:HWS_LOW2HIGH;
    ddr3_tip_bus_read(0, 0,ACCESS_TYPE_UNICAST, ByteNum,DDR_PHY_DATA,CSN_IOB_VREF_REG(effective_cs),&reg_data2);
    ddr3_tip_bus_read(0, 0,ACCESS_TYPE_UNICAST, ByteNum,DDR_PHY_DATA,READ_CENTRALIZATION_PHY_REG + 4*effective_cs,&reg_data1);
    
   // printf("\nXORsearch_2D_2E[%d]\tNom:[%2d,%2d]----",ByteNum,reg_data1,reg_data2);
    VW_vector[1] = XORsearch_2D_1E(EDGE_FP, element1, search_dir1 , Step1, end_val1, init_val1, 
                                         element2, search_dir2 , Step2, end_val2,init_val2,     
                                         DepthStage,ByteNum);
    /*******************************/

    ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, 
                                CSN_IOB_VREF_REG(effective_cs), reg_data2);
    ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, 
                                CSN_IO_BASE_VREF_REG(effective_cs)+4, reg_data2);
    ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, 
                            CSN_IO_BASE_VREF_REG(effective_cs)+5, reg_data2);
    ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, 
                                READ_CENTRALIZATION_PHY_REG + 4*effective_cs, reg_data1);
    
    /******************************/
    if ((VW_vector[1] == 255)||(VW_vector[0] == 255)) {
        return 255;
    }else
        return (VW_vector[1]-VW_vector[0]);
    
    
    return 0;

}
/*******************************************************************************************/
int XORsearch_RL(enum hws_edge_compare edge, enum hws_search_dir search_dir , u32 Step, u32 init_val,u32 end_val,u8 DepthStage,u16 ByteNum)
{
    int result = (edge == EDGE_PF)? 0 : 1;// If pass start from 0(pass)
    int SignStep = (search_dir == HWS_LOW2HIGH)? Step:(-Step);
    int Param = (int)init_val;
    u32 NumOfSteps = (search_dir == HWS_LOW2HIGH)?((end_val-init_val)/Step):((init_val - end_val)/Step);
    u32 StepIndex = 0;
    u32 reg_data,i,rl_adll_val,rl_phase_val;
    ddr3_tip_bus_read(0, 0,ACCESS_TYPE_UNICAST, ByteNum,DDR_PHY_DATA,RL_PHY_REG(0),&reg_data);

    i = (reg_data&0x1F) + 32*((0x1C0&reg_data)>>6);//skip RdSmp
    printf("XORsearch_RL::RL reg = 0x%08x --> %d Taps , NumOfStep - %d\n",reg_data,i,NumOfSteps);
    for(StepIndex = 0 ;StepIndex < NumOfSteps ; StepIndex++) {
        if(i == 0) {
            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, \
                            RL_PHY_REG(0), reg_data);
            return 0;
        }
        i = i + SignStep;
        rl_adll_val = i % 32;
        rl_phase_val = (i - rl_adll_val) / 32;
        Param = (rl_adll_val )| ((rl_phase_val ) << 0x6);
        (ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, \
                            RL_PHY_REG(0), Param));
        
        result = XOR_Gradual_Test(DepthStage);
        if(result != 0) {
            printf("RL 0x%08x\tFail\n",Param);
            ddr3_tip_reset_fifo_ptr(0);
        }
        if(result == 0) {
            printf("RL 0x%08x\tPass\n",Param);
            
        }
        
    }
    ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, ACCESS_TYPE_UNICAST, ByteNum, DDR_PHY_DATA, \
                            RL_PHY_REG(0), reg_data);
    return -1; // All the search are on positive numbers so -1 is representinga fail.

}
/******************************************************************************************** 
print Stability - print all data we need for the statistic analyze of the algorithm. 
                    The print is match to the excle tool 
*********************************************************************************************/
/*
 * Print stability log info
 */
int print_stability_log(u32 dev_num)
{
	u8 if_id = 0, csindex = 0, bus_id = 0, idx = 0;
	u32 reg_data;
#if defined(CONFIG_DDR4)
	u32 reg_data1;
#endif /* CONFIG_DDR4 */
	u32 read_data[MAX_INTERFACE_NUM];
	u32 max_cs = ddr3_tip_max_cs_get(dev_num);
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();

	/* Title print */
	for (if_id = 0; if_id < MAX_INTERFACE_NUM; if_id++) {
		VALIDATE_IF_ACTIVE(tm->if_act_mask, if_id);
		printf("Title: I/F# , Tj, Calibration_n0, Calibration_p0, Calibration_n1, Calibration_p1, Calibration_n2, Calibration_p2,Calibration_n3, Calibration_p3,");
		for (csindex = 0; csindex < max_cs; csindex++) {
			printf("CS%d , ", csindex);
			
            for (bus_id = 0; bus_id < MAX_BUS_NUM; bus_id++) {
                VALIDATE_BUS_ACTIVE(tm->bus_act_mask, bus_id);
    #if defined(CONFIG_DDR4)
                printf("DminTx, AreaTx, DminRx, AreaRx, WL_tot, WL_ADLL, WL_PH, RL_Tot, RL_ADLL, RL_PH, RL_Smp, CenTx, CenRx, Vref, DQVref,");
                for (idx = 0; idx < 11; idx++)
                    printf("DC-Pad%d,", idx);
    #else /* CONFIG_DDR4 */
                printf("VWTx, VWRx, WL_tot, WL_ADLL, WL_PH, RL_Tot, RL_ADLL, RL_PH, RL_Smp, Cen_tx, Cen_rx, Vref, DQVref,");
    #endif /* CONFIG_DDR4 */
                for (idx = 0; idx < 11; idx++)
                    printf("PBSTx-Pad%d,", idx);
                
                for (idx = 0; idx < 11; idx++)
                    printf("PBSRx-Pad%d,", idx);
            }
		}
	}
	printf("\n");

	/* Data print */
	for (if_id = 0; if_id < MAX_INTERFACE_NUM; if_id++) {
		VALIDATE_IF_ACTIVE(tm->if_act_mask, if_id);

		printf("Data: %d,%d,", if_id,
		       (config_func_info[dev_num].tip_get_temperature != NULL)
		       ? (config_func_info[dev_num].
			  tip_get_temperature(dev_num)) : (0));

		CHECK_STATUS(ddr3_tip_if_read
			     (dev_num, ACCESS_TYPE_UNICAST, if_id, 0x14c8,
			      read_data, MASK_ALL_BITS));
		printf("%d,%d,", ((read_data[if_id] & 0x3f0) >> 4),
		       ((read_data[if_id] & 0xfc00) >> 10));
		CHECK_STATUS(ddr3_tip_if_read
			     (dev_num, ACCESS_TYPE_UNICAST, if_id, 0x17c8,
			      read_data, MASK_ALL_BITS));
		printf("%d,%d,", ((read_data[if_id] & 0x3f0) >> 4),
		       ((read_data[if_id] & 0xfc00) >> 10));
		CHECK_STATUS(ddr3_tip_if_read
			     (dev_num, ACCESS_TYPE_UNICAST, if_id, 0x1dc8,
			      read_data, MASK_ALL_BITS));
		printf("%d,%d,", ((read_data[if_id] & 0x3f0000) >> 16),
		       ((read_data[if_id] & 0xfc00000) >> 22));
        CHECK_STATUS(ddr3_tip_if_read
			     (dev_num, ACCESS_TYPE_UNICAST, if_id, 0x1ec8,
			      read_data, MASK_ALL_BITS));
		printf("%d,%d,", ((read_data[if_id] & 0x3f0000) >> 16),
		       ((read_data[if_id] & 0xfc00000) >> 22));

		for (csindex = 0; csindex < max_cs; csindex++) {
			printf("CS%d , ", csindex);
			for (bus_id = 0; bus_id < MAX_BUS_NUM; bus_id++) {
				printf("\t");
				VALIDATE_BUS_ACTIVE(tm->bus_act_mask, bus_id);
#if defined(CONFIG_DDR4)
				/* DminTx, areaTX */
				ddr3_tip_bus_read(dev_num, if_id,
						  ACCESS_TYPE_UNICAST,
						  bus_id, DDR_PHY_DATA,
						  RESULT_DB_PHY_REG_ADDR +
						  csindex, &reg_data);
				ddr3_tip_bus_read(dev_num, if_id,
						  ACCESS_TYPE_UNICAST,
						  dmin_phy_reg_table
						  [csindex * 5 + bus_id][0],
						  DDR_PHY_CONTROL,
						  dmin_phy_reg_table
						  [csindex * 5 + bus_id][1],
						  &reg_data1);
				printf("%d,%d,", 2 * (reg_data1 & 0xFF),
				       reg_data);
				/* DminRx, areaRX */
				ddr3_tip_bus_read(dev_num, if_id,
						  ACCESS_TYPE_UNICAST,
						  bus_id, DDR_PHY_DATA,
						  RESULT_DB_PHY_REG_ADDR +
						  csindex + 4, &reg_data);
				ddr3_tip_bus_read(dev_num, if_id,
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
				ddr3_tip_bus_read(dev_num, if_id,
						  ACCESS_TYPE_UNICAST,
						  bus_id, DDR_PHY_DATA,
						  RESULT_DB_PHY_REG_ADDR +
						  csindex, &reg_data);
				printf("%d,%d,", (reg_data & 0x1f),
				       ((reg_data & 0x3e0) >> 5));
#endif /* CONFIG_DDR4 */
				/* WL */
				ddr3_tip_bus_read(dev_num, if_id,
						  ACCESS_TYPE_UNICAST,
						  bus_id, DDR_PHY_DATA,
						  WL_PHY_REG(csindex),
						  &reg_data);
				printf("%d,%d,%d,",
				       (reg_data & 0x1f) +
				       ((reg_data & 0x1c0) >> 6) * 32,
				       (reg_data & 0x1f),
				       (reg_data & 0x1c0) >> 6);
				/* RL */
				CHECK_STATUS(ddr3_tip_if_read
					     (dev_num, ACCESS_TYPE_UNICAST,
					      if_id,
					      READ_DATA_SAMPLE_DELAY,
					      read_data, MASK_ALL_BITS));
				read_data[if_id] =
					(read_data[if_id] &
					 (0x1f << (8 * csindex))) >>
					(8 * csindex);
				ddr3_tip_bus_read(dev_num, if_id,
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
				/* Centralization */
				ddr3_tip_bus_read(dev_num, if_id,
						  ACCESS_TYPE_UNICAST, bus_id,
						  DDR_PHY_DATA,
						  WRITE_CENTRALIZATION_PHY_REG
						  + csindex * 4, &reg_data);
				printf("%d,", (reg_data & 0x3f));
				ddr3_tip_bus_read(dev_num, if_id,
						  ACCESS_TYPE_UNICAST, bus_id,
						  DDR_PHY_DATA,
						  READ_CENTRALIZATION_PHY_REG
						  + csindex * 4, &reg_data);
				printf("%d,", (reg_data & 0x1f));
				/* Vref */
				ddr3_tip_bus_read(dev_num, if_id,
						  ACCESS_TYPE_UNICAST, bus_id,
						  DDR_PHY_DATA,
						  PAD_CONFIG_PHY_REG,
						  &reg_data);
				printf("%d,", (reg_data & 0xF));
				/* DQVref */
				/* Need to add the Read Function from device */
				printf("%d,", dq_vref_vec[bus_id]);
#if defined(CONFIG_DDR4)
				
				for (idx = 0; idx < 12; idx++) {
					ddr3_tip_bus_read(dev_num, if_id,
							  ACCESS_TYPE_UNICAST,
							  bus_id, DDR_PHY_DATA,
							  0xd0 + 12 * csindex +
							  idx, &reg_data);
					printf("%d,", (reg_data & 0x3f));
				}
#endif /* CONFIG_DDR4 */
				for (idx = 0; idx < 12; idx++) {
					ddr3_tip_bus_read(dev_num, if_id,
							  ACCESS_TYPE_UNICAST,
							  bus_id, DDR_PHY_DATA,
							  0xd0 +
							  12 * csindex +
							  idx, &reg_data);
					printf("%d,", (reg_data & 0x3f));
				}
				for (idx = 0; idx < 12; idx++) {
					ddr3_tip_bus_read(dev_num, if_id,
							  ACCESS_TYPE_UNICAST,
							  bus_id, DDR_PHY_DATA,
							  0x10 +
							  16 * csindex +
							  idx, &reg_data);
					printf("%d,", (reg_data & 0x3f));
				}
				for (idx = 0; idx < 12; idx++) {
					ddr3_tip_bus_read(dev_num, if_id,
							  ACCESS_TYPE_UNICAST,
							  bus_id, DDR_PHY_DATA,
							  0x50 +
							  16 * csindex +
							  idx, &reg_data);
					printf("%d,", (reg_data & 0x3f));
				}
			} // End byte loop
		}//End CS loop
	}//End I/F loop
	printf("\n");

	return MV_OK;
} 
/******************************************************************************************** 
  ECCdisableEnable -
  This function jusy reset the ECC to continue have a operationla working state.
  It disabke the ECC --> zero the counters --> enable the ECC.
  This ECC support only the MC6                                                             
********************************************************************************************/
 
void EccDisableEnable() 
{
    reg_bit_clrset(MC6_REG_RAS_CTRL,0x0 << MC6_ECC_ENABLE_OFFS,MC6_ECC_ENABLE_MASK << MC6_ECC_ENABLE_OFFS);
    reg_write(MC6_REG_ECC_1bit_err_counter,0x0);
    reg_bit_clrset(MC6_REG_RAS_CTRL,ECC_ENABLE << MC6_ECC_ENABLE_OFFS,MC6_ECC_ENABLE_MASK << MC6_ECC_ENABLE_OFFS);
}

/******************************************************************************************** 
MemoryTestPerByte - 
This test support running on 32bit or 64 bit controller. 
It can be run on all width of the controller or per byte. 
does not support running on other constellation. 
Inputs: 
    start_addres_low - lowwer 32 bit of the start address
    start_addres_high - higher 16 bit of the start address
    End_addres_low - lowwer 32 bit of the start address
    End_addres_high - higher 16 bit of the start address
    Access_Numbers - number of access to the DRAM on each test
    ByteNum - which byte to access, options are:
        0,1,2,3,4,5,6,7,8(ecc),32,64,72
    printEn - print debug notes
    TestMap - each bit represent a test
    bit# - test name
    0 - Write2Read:Increament data:Increament address.Access data width at a time (32/64) and write increment number 0 - 255.
    1 - WW2RRR:Increament data:Increament address.second data is inverse. Access data width at a time (32/64) and write increment number 0 - 255.
 
 Output:
    The number of fail access.
 

*********************************************************************************************/
int AreaLengthConv[] = {8,16,32,64,128,256,512,1024,2048,4096,8192,16384,32768}; //in Mbytes
int MemoryTestPerByte(u32 start_addres_low, u32 start_addres_high, u32 End_addres_low, u32 End_addres_high,
                      u32 Access_Numbers, u32 ByteNum,u32 printEn,u32 TestMap)
{
    
    uint64_t data = 0x0,data2 = 0x0, data_read = 0x0,data_read2 = 0x0 ,ByteMask = 0;
    uintptr_t addr = 0x0,addr2 = 0,startAdd = 0, endAdd;
    
    unsigned long long Local_Area_length = Area_length;
    
    int MuxReg = mmio_read_32(0xf00116d8);
    
    if(MuxReg == 0x3CC) {
        Local_Area_length = 0;
    }
   
    uint64_t idx;
    int Write2Read = 0,Read2Write2Read = 0, Walkbit = 0,OferPat = 0;
    int shift2Byte = ByteNum*8;
   // int DataWidth = ((reg_read(MC6_BASE_ADDR + 0x44))&(0x7<<8))>>8; oferb to fix when the MC is ready
    int addressinc = (DataWidth == 0x3)?(4):(8) ;
    startAdd = ((uintptr_t)start_addres_low )| ((uintptr_t)(start_addres_high)<<32 );
    endAdd = ((uintptr_t)End_addres_low) | ((uintptr_t)(End_addres_high)<<32) ;
    ByteMask =  (uint64_t)(((uint64_t)0xFF) << shift2Byte);
    if(ByteNum == 32 || ByteNum == 64 || ByteNum == 72) {
        shift2Byte = 0;
        ByteMask = (ByteNum == 32)?0xFFFFFFFF:0xFFFFFFFFFFFFFFFF;
    }
	//ddr3_tip_reset_fifo_ptr(0);

	//printf("\n MemoryTest64PerByte-2: ByteMask 0x%16jx\n",ByteMask);
	
	/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/ 
    if((TestMap&0x1) == 0x1) {
        
        if(printEn) printf("\n");
        for (idx = 0; idx < Access_Numbers; idx += 1) {
            addr = idx * addressinc + startAdd;
            if(ByteNum == 64 || ByteNum == 72){
                data = ((idx%256) << 56) | ((idx%256) << 48) |((idx%256) << 40) |((idx%256) << 32) |((idx%256) << 24) |((idx%256) << 16) |((idx%256) << 8) |((idx%256));
            }
            else if(ByteNum == 32 ){
                data = (((idx%256) << 24) |((idx%256) << 16) |((idx%256) << 8) |((idx%256)));
            }else
                data = ((idx%256) << shift2Byte);
			//printf("1)addr: 0x%lx data: 0x%lx\n",addr,data);	
            cpu_write(Local_Area_length*effective_cs + addr, data);
            data_read = cpu_read(Local_Area_length*effective_cs + addr);
           
            if((data_read & ByteMask) != (data & ByteMask))
                {
                    if(printEn) printf("MemoryTest:W2R:Increament data:Increament address: 0x%16jx, read 0x%16jx, expected 0x%16jx\n",addr,data_read,data);
                    Write2Read++;
                }
        }
        
    }
    /*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
   if((TestMap&0x2) == 0x2){
       if(printEn) printf("\n");
       for (idx = 0; idx < Access_Numbers; idx += 1) {
            addr = idx * addressinc + startAdd;
            if(ByteNum == 64 || ByteNum == 72){
                data = ((idx%256) << 56) | ((idx%256) << 48) |((idx%256) << 40) |((idx%256) << 32) |((idx%256) << 24) |((idx%256) << 16) |((idx%256) << 8) |((idx%256));
            }
            else if(ByteNum == 32 ){
                data = (((idx%256) << 24) |((idx%256) << 16) |((idx%256) << 8) |((idx%256)));
            }else
                data = ((idx%256) << shift2Byte);
            
            cpu_write(Local_Area_length*effective_cs + addr, data);
            cpu_write((Local_Area_length*effective_cs + addr+addressinc), (~data));
            data_read = cpu_read(Local_Area_length*effective_cs + addr);
            data_read2 = cpu_read((Local_Area_length*effective_cs + addr+addressinc));
            
            if(((data_read & ByteMask) != (data & ByteMask) ) || ((data_read2 & ByteMask) != ((~data) & ByteMask) ))
                {
                    if(printEn) printf("MemoryTest:WW2RR:Increament data:Increament address:Inv data 0x%16jx, read 0x%16jx, expected 0x%16jx",addr,data_read,data);
                    if(printEn) printf(", read 0x%16jx, expected 0x%16jx\n",data_read2,(~data));
                    Write2Read++;
                }
        }
   }
    /*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
   if((TestMap&(1<<2)) == (1<<2)){
       if(printEn) printf("\n");
       for (idx = 0; idx < Access_Numbers; idx += 1) {
        addr = idx * addressinc*72;
        addr2 = (addr*addr)%(endAdd - startAdd);
        addr = addr + startAdd;
        addr2 = addr2 + startAdd;
        data = cpu_read(Local_Area_length*effective_cs + addr);
        cpu_write(Local_Area_length*effective_cs + addr2, data);
        data_read = cpu_read(Local_Area_length*effective_cs + addr2);
//        printf("MemoryTest:R2WR:Random data:Random address: 0x%16jx, read 0x%16jx, expected 0x%16jx\n",addr2,data_read,data);
        if((data_read & ByteMask) != (data & ByteMask))
            {
                if(printEn) printf("MemoryTest:R2WR:Random data:Random address: 0x%16jx, read 0x%16jx, expected 0x%16jx\n",addr2,data_read,data);
                Read2Write2Read++;
            }
       }
   }
    
    /*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
    /* Mix walking one to walking zero                                  */ 
	if((TestMap&(1<<3)) == (1<<3)){
        if(printEn) printf("\n");
        addr = endAdd;//The start address
        data = 0xFFFFFFFFFFFFFFFF; // init data
        data2 = 0x0;
        uint64_t pownum = 1;
        int counter = 0;
        
       for (idx = 0; idx < Access_Numbers; idx += 1) {
        addr2 = addr - (idx * addressinc);
        if(idx%2 == 0) {
            data2 = (~data2);
        }else if(idx%2 == 1){
            data2 = data - pownum ;
            pownum = pownum*(2);
            counter++;
            if(counter == 64) {
                pownum = 1;
            }
        }else{
            data2 = (~data2);
        }
        cpu_write(Local_Area_length*effective_cs + addr2, data2);
        data_read = cpu_read(Local_Area_length*effective_cs + addr2);
        
	//	printf("\nMemoryTest:W2R dect address: walking 1 stager 0: 0x%16jx, read 0x%16jx, expected 0x%16jx \n",addr2,data_read,data2);
        if((data_read & ByteMask) != (data2 & ByteMask))
            {
                if(printEn) printf("\nMemoryTest:W2R dect address: walking 1 stager 0: 0x%16jx, read 0x%16jx, expected 0x%16jx \n",addr2,data_read,data2);
                Walkbit++;
            }
       }
   }
   if((TestMap&(1<<4)) == (1<<4)){
       int NumOfByteInDevice = (DataWidth == 0x3)?(4):(8);
        int NumOfLoop = Access_Numbers;//(DataWidth == 0x3)?(ByteCount/4):(ByteCount/8);
        for(int NumOfLoopI = 0 ; NumOfLoopI<NumOfLoop; NumOfLoopI++) {
            if(1) {
                if(NumOfLoopI == 0) {
                    data = 127;
                }else{
                    data =data2&0xFF;
                    
                }
                if((NumOfLoopI%3 == 0)||(NumOfLoopI%3 == 1)) {
                    data = 255 - data;//inverse
                }else{
                    data = (data==0xFE)?127:(128+ (data/2));
                }
                     
             }
            if(0) {
                if(NumOfLoopI == 0) {
                    data = 128;
                }else{
                    data =data2&0xFF;
                    
                }
                if((NumOfLoopI%3 == 0)||(NumOfLoopI%3 == 1)) {
                    data = 255 - data;
                }else{
                    data = (data==0x1)?128:data/2;
                }
                     
             }
            data2 = 0x0;
            int shift = 0;
            for(int byteindex = 0; byteindex < NumOfByteInDevice; byteindex++){
                shift = shift + 8*byteindex;
                data2 |= ((data%256) << (shift));
             }
            cpu_write(addr,data2);
            data_read = 0;
            for(int Ir = 0 ; Ir<NumOfLoop;Ir++) {
                data_read |= cpu_read(addr);
            }
            if((data_read & ByteMask) != (data2 & ByteMask))
            {
                if(printEn) printf("\nMemoryTest:Ofer Special Pattern: 0: 0x%16jx, read 0x%16jx, expected 0x%16jx \n",addr2,data_read,data2);
                OferPat++;
            }
            
            
           // printf("cpu_write:\t\t0x%16lx\t0x%16lx\n",addr,data2);
            addr = addr + NumOfByteInDevice;
        }
   }
    if(ByteNum == 8) {
        return  (mmio_read_32(0xf0020364));//returning the ECC error
    }else
        return (Write2Read + Read2Write2Read +  Walkbit + OferPat);

}

/*========================================================================

 This function perform a sweep on the MC CL & CWl to check the 
 solution of the Mc PHY_RL_CYCLE_DLY. 
 The function return the CL & CWl to it original valur in teh end.
 ========================================================================*/

int MC_CL_CWL_impact(void)
{
    int ret,CWlindex,CLindex,MaxCWL = 21,MaxCL = 24,minCWL = 5,minCL = 9; 
    int reg = reg_read(0x20300);
    int NomCL = reg&0x3F;
    int NomCWL = (reg >> 8) & 0x3F;
    int ResultMatrix[32][32];
    unsigned long long Area_length = _1G;
         
    u32 start_addres_low = (reg_read(MC6_REG_MMAP_LOW_CH0(0))&(0x1FF<<23));
    u32 start_addres_high = reg_read(MC6_REG_MMAP_HIGH_CH0(0)); 
    
    unsigned long long End_addr_bytes = (((unsigned long long)start_addres_high<<32)| ((unsigned long long)start_addres_low) ) + Area_length ;
    u32 End_addres_low =  (u32)((End_addr_bytes)& 0x00000000FFFFFFFF);
    u32 End_addres_high =  (u32)((End_addr_bytes)>>32);
    int Access_Numbers = 1024;
    
    ret=0;
    printf("MC_CL_CWL_impact\n===========================\n");
    printf("start_addres_low\t0x%08x\n",start_addres_low);
    printf("start_addres_high\t0x%08x\n",start_addres_high);
    printf("End_addres_low\t0x%08x\n",End_addres_low);
    printf("End_addres_high\t0x%08x\n",End_addres_high);
    printf("=======================================================\n");
    
    for(CWlindex = MaxCWL ; CWlindex>=minCWL;CWlindex--) {
        reg_bit_clrset(0x20300, CWlindex << 8,0x3F << 8);
        for ( CLindex = MaxCL; CLindex >= minCL; CLindex--) {
            reg_bit_clrset(0x20300, CLindex ,0x3F);
            ret = MemoryTestPerByte(start_addres_low, start_addres_high, End_addres_low, End_addres_high,
                       Access_Numbers, 64, 0, 0xF);
            
            ddr3_tip_reset_fifo_ptr(0);
            ResultMatrix[CWlindex][CLindex] = ret;
         }
    }
    // return the original values
    reg_bit_clrset(0x20300, (NomCWL << 8) | NomCL ,(0x3F << 8)|0x3F);
    //print
    printf("Your CL - %d & CWl = %d\n",NomCL,NomCWL);

    for ( CLindex = MaxCL; CLindex >= minCL; CLindex--) {
            printf("========");//tab length is 8
    }
    printf("\nCWL\t");
    for ( CLindex = MaxCL; CLindex >= minCL; CLindex--) {
            printf("%d\t",CLindex);
    }
    printf("\n");
    for(CWlindex = MaxCWL ; CWlindex>=minCWL;CWlindex--) {
        printf("%d\t",CWlindex);
        for ( CLindex = MaxCL; CLindex >= minCL; CLindex--) {
            printf("%d\t",ResultMatrix[CWlindex][CLindex]);
        }
        printf("\n");
    }
    return 0;
}

/******************************************************************************************** 
 
 TIP BIST
 num_iter - repeat time of the test
********************************************************************************************/ 
int  TipIpBIST(int dev_num, int if_id, enum hws_dir direction,int init_value_used, int pattern, int cs_num)
{
    struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
    enum hws_training_result resultType = RESULT_PER_BYTE;
    enum hws_search_dir searchDirId = HWS_LOW2HIGH  ;
    //enum hws_dir direction = OPER_READ;
    enum hws_training_ip_stat trainingResult[MAX_INTERFACE_NUM]; 
    u16 *mask_results_pup_reg_map = ddr3_tip_get_mask_results_pup_reg_map();
    u32 octets_per_if_num = ddr3_tip_dev_attr_get(dev_num, MV_ATTR_OCTET_PER_INTERFACE);
    u32 start_pup = 0;
	u32 end_pup = octets_per_if_num - 1;
    //u32 effective_cs = cs_num;
    u32 Res = 0;
    u32 read_data[MAX_INTERFACE_NUM];

    ddr3_tip_ip_training(dev_num, ACCESS_TYPE_MULTICAST, if_id,ACCESS_TYPE_MULTICAST, PARAM_NOT_CARE, resultType,
				     HWS_CONTROL_ELEMENT_ADLL, searchDirId, direction,tm->if_act_mask, init_value_used, /*1*/31,
				     pattern, EDGE_FP, CS_SINGLE,cs_num, trainingResult);
    //printf("\nTipIpBIST::");
    for (int reg_offset = start_pup; reg_offset <= end_pup; reg_offset++) {
          ddr3_tip_if_read(dev_num,ACCESS_TYPE_UNICAST,if_id,mask_results_pup_reg_map[reg_offset],read_data,MASK_ALL_BITS);
          //printf("0x%x,\t",read_data[if_id]);
          if((read_data[if_id] & 0x02000000) == 0) {
            Res = Res + (1 << reg_offset);
            }
    }
							

    return Res;
}

        
/******************************************************************************************** 
Sweep validation 
----------------- 
repeat_num Number of repeat of the test 
testLength test length input for the memory test minimum is 0x10 
direction - 0 is Tx ; 1 is Rx
mode - optional for specific debug 0xFF means run on all pups other means mode  = pup number
 
********************************************************************************************/
int ddr4_cpu_sweep_test(int dev_num, u32 repeat_num, u32 testLength ,u32 direction,u32 mode)
{
    //testLength - 0 CPU only, 1 include Xor 128M@128K, 2 include partial write 1M@1K+1/2/3
    //             3 include Xor 1G@16M 
    
	//u8 Vctrl_sweepres[ADLL_LENGTH][ADLL_LENGTH][MAX_INTERFACE_NUM][MAX_BUS_NUM];
    u32 pup = 0, start_pup = 0, end_pup = 0, dutyCycle = 0 , rep = 0;
	u32 adll = 0;
    int printEn = 0;
	u32 res[MAX_INTERFACE_NUM] = { 0 };
    u32 VectorVtap[MAX_BUS_NUM] = {0};
	int if_id = 0,dutyCycleindex;
	u32 adll_value = 0;
	int reg = (direction == 0) ? WRITE_CENTRALIZATION_PHY_REG : READ_CENTRALIZATION_PHY_REG;
	int step = (direction == 0) ? 3 : 3;
    int MaxV = (direction == 0) ? 73 : 63;
    struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	enum hws_access_type pup_access;
	
	u32 octets_per_if_num = ddr3_tip_dev_attr_get(dev_num, MV_ATTR_OCTET_PER_INTERFACE);
    if (mode == 0xFF) {
		/* per pup */
		start_pup = 0;
		end_pup = octets_per_if_num - 1;
		pup_access = ACCESS_TYPE_UNICAST;
	} else if(mode == 0x1FF) {
		/* per pup */
		start_pup = 0;
		end_pup = octets_per_if_num;
		pup_access = ACCESS_TYPE_UNICAST;
	}  
	else {//need to add mode check for valid pup number
		start_pup = mode;
		end_pup = mode+1;
		pup_access = ACCESS_TYPE_UNICAST;
	}
    //start_pup = 0;
	//end_pup = 1;
	if((ddr3_if_ecc_enabled()==1)&& (start_pup != 8)){
        reg_bit_clrset(MC6_REG_RAS_CTRL,0x0 << MC6_ECC_ENABLE_OFFS,MC6_ECC_ENABLE_MASK << MC6_ECC_ENABLE_OFFS);
        reg_write(MC6_REG_ECC_1bit_err_counter,0x0);
    }	
   /* if(start_pup == 8) {
        printEn = 1;
        printf("set PrintEn\n");
    }*/
		/*
		 * Sweep ADLL  from 0:31 on all I/F on all Pup and perform
		 * BIST on each stage.
		 */

      //  cs = 0;//need to add CS support
		//effective_cs = cs;
        u32 reg_val_adll,reg_val_Vtap;

        ddr3_tip_if_write(0, ACCESS_TYPE_MULTICAST, PARAM_NOT_CARE, ODPG_DATA_CONTROL_REG,
					       effective_cs << ODPG_DATA_CS_OFFS,
					       ODPG_DATA_CS_MASK << ODPG_DATA_CS_OFFS);
       /* if((direction == 1)) // direction 0 is saved in dq_vref_vec array
            ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST,start_pup, DDR_PHY_DATA,CSN_IOB_VREF_REG(cs),&reg_val_Vtap);*/
        /*--------------------------------------------------------------------*/
        //ddr3_tip_reg_dump(0);

       // mmio_write_32(0xf00116d8, 0x3cc);
        int MuxReg = mmio_read_32(0xf00116d8);
		for (pup = start_pup; pup < end_pup; pup++) {
            //printf("\n");
            if((direction == 1)) {// direction 0 is saved in dq_vref_vec array
                ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST,pup, DDR_PHY_DATA,CSN_IOB_VREF_REG(effective_cs),&reg_val_Vtap);
                VectorVtap[pup] = reg_val_Vtap;
                //printf("====================>read reg_val_Vtap = %2d\n",reg_val_Vtap);
            }
            //Saving  the ADLl position & the Voltage 
            ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST,start_pup, DDR_PHY_DATA,reg + CS_BYTE_GAP(effective_cs),&reg_val_adll);
            printf("=\t=\t=\t=\t Byte %d Nom [ADLL,RC] = [%2d,%2d] =\t=\t=\t=\t",pup,reg_val_adll,reg_val_Vtap);
            printf("\n");
            for(dutyCycle = 0 ; dutyCycle <MaxV ; dutyCycle +=step ){
                WriteXORPat = 1; // 1 - write, 0 - dont
                XORS2D = 1; // 1 - Do, 0 - skip
#if 1
                    if((direction == 0)) {
#if 1
                       mmio_write_32(0xf00116d8, 0x3cc);
                        dutyCycleindex = dutyCycle;
                        //printf("Sweep:: DqVref %d , PUP %d \n", dutyCycleindex,pup);
                        /* Enter DRAM to VRef training mode */
                       mv_ddr4_vref_training_mode_ctrl(0, 0, ACCESS_TYPE_MULTICAST, MV_TRUE);
                        /* Set new VRef training value in DRAM */
                        mv_ddr4_vref_tap_set(0, 0, ACCESS_TYPE_MULTICAST, dutyCycleindex, MV_DDR4_VREF_TAP_START);
                        /*Close the VREF range*/
                        mv_ddr4_vref_tap_set(0, 0, ACCESS_TYPE_MULTICAST, dutyCycleindex,  MV_DDR4_VREF_TAP_END);
                        /* set mux to MC6 */
                       mmio_write_32(0xf00116d8, MuxReg);
#else
                           dutyCycleindex = dutyCycle;
                           /* Disable PDA ODPG HW Control */
                           CHECK_STATUS(ddr3_tip_if_write(0, ACCESS_TYPE_UNICAST, 0, CS_ENABLE_REG, (1 << 7), (1 << 7)));
                           /* Enable PDA mode */
                           ddr4PdaEnable(0, 0, effective_cs, MV_TRUE);
                           VALIDATE_BUS_ACTIVE(tm->bus_act_mask, pup);
                            /* Load pattern to ODPG - select which device is active for current MRS */
                            ddr4PdaLoadPatternToOdpg(0, ACCESS_TYPE_UNICAST, 0, (1 << pup), effective_cs);

                            /* Set final VRef training value in DRAM */
                            mv_ddr4_vref_tap_set(0, 0, ACCESS_TYPE_UNICAST, dutyCycleindex,VREF_TAP_SET_STATE_START);
                            /* Load pattern to ODPG*/
                            ddr4PdaLoadPatternToOdpg(0, ACCESS_TYPE_UNICAST, 0, tm->bus_act_mask, effective_cs);
                            /* Disable PDA mode */
                            ddr4PdaEnable(0, 0, effective_cs, MV_FALSE);
                            /* Enable PDA ODPG HW Control */
                            CHECK_STATUS(ddr3_tip_if_write(0, ACCESS_TYPE_UNICAST, 0, CS_ENABLE_REG, (0 << 7), (1 << 7)));
#endif

                    }else{
                        //printf("Sweep:: dutyCycle %d , PUP %d \n", dutyCycle,pup);
                        /* Set new Receiver DC training value in DRAM */
                        (ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, \
                        CSN_IOB_VREF_REG(effective_cs), dutyCycle));
                        (ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, \
                        CSN_IO_BASE_VREF_REG(effective_cs)+4, dutyCycle));
                        (ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, \
                        CSN_IO_BASE_VREF_REG(effective_cs)+5, dutyCycle));
                    }
#endif
                
				for (adll = 0; adll < ADLL_LENGTH; adll++) {
                    res[0] = 0;
                    adll_value =(direction == 0) ? (adll * 2) : adll;
					CHECK_STATUS(ddr3_tip_bus_write(dev_num, ACCESS_TYPE_MULTICAST, 0,pup_access, pup, DDR_PHY_DATA,reg + CS_BYTE_GAP(effective_cs),adll_value));
                    
                    for (rep = 0; rep < repeat_num; rep++) {
                        	res[0] = 0;
					}
                    
					//res[0] += bit_flip();
                    res[0] += MemoryTestPerByte(0x0, 0x0, _2M, 0,0x10, (pup), printEn, 0xF);
                    if((res[0] == 0)&& (testLength >= 0)) {
                        res[0] += MemoryTestPerByte(0x0, 0x0, _2M, 0,0x100, (pup), printEn, 0x10);

                        if((res[0] == 0)&& (testLength >= 1)) {//If the CPU test pass we can test it with the XOR engine
                           res[0] += XOR_Gradual_Test(testLength);
                           }//(testLength >= 1)
                        }//(testLength >= 0)
                   if(pup == 8) {
						printf("##########pup: %d #############\n",pup);
                        //need to recover the ECC value
                       if(direction == 1) {
                            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IOB_VREF_REG(effective_cs), VectorVtap[pup]);
                            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IO_BASE_VREF_REG(effective_cs)+4, VectorVtap[pup]);
                            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IO_BASE_VREF_REG(effective_cs)+5, VectorVtap[pup]);
                        }else{
                             mmio_write_32(0xf00116d8, 0x3cc);
                            /* Enter DRAM to VRef training mode */
                            mv_ddr4_vref_training_mode_ctrl(0, 0, ACCESS_TYPE_MULTICAST, MV_TRUE);
                            /* Set new VRef training value in DRAM */
                            mv_ddr4_vref_tap_set(0, 0, ACCESS_TYPE_MULTICAST, dq_vref_vec[pup], MV_DDR4_VREF_TAP_START);
                            /*Close the VREF range*/
                            mv_ddr4_vref_tap_set(0, 0, ACCESS_TYPE_MULTICAST, dq_vref_vec[pup],  MV_DDR4_VREF_TAP_END);
                            /* set mux to MC6 */
                            mmio_write_32(0xf00116d8, 0x38c);
                        }
                        ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, reg + CS_BYTE_GAP(effective_cs), reg_val_adll);
                        //rerun the tests
                        reg_bit_clrset(MC6_REG_RAS_CTRL,0x8,0x8);//Ignor Ecc
                        MemoryTestPerByte(0x0, 0x0, _1G, 0,0x10, (pup), printEn, 0xF);

                        if( (testLength >= 0)) {
                             MemoryTestPerByte(0x0, 0x0, _1G, 0,0x4000, (pup), printEn, 0xF);
                            if((testLength >= 1)) {//If the CPU test pass we can test it with the XOR engine
                                 XOR_Test(_1K,_128M, 0x0, _1G,0x0, _128K,0x0,0x0,0x0,NumOstdRd);
                                if( (testLength >= 2)) {
                                     XOR_Test(_1M,_1M, 0x0, _1G,0x0, _1K + 1,0x1,0x0,0x0,NumOstdRd);
                                     XOR_Test(_1M,_1M, 0x0, _1G,0x0, _1K + 2,0x2,0x0,0x0,NumOstdRd);
                                     XOR_Test(_1M,_1M, 0x0, _1G,0x0, _1K + 3,0x4,0x0,0x0,NumOstdRd);
                                    if( (testLength >= 3)) {
                                         XOR_Test(_256M,_1G, 0x0, _2G,0x0, _16M,0x1,0x0,0x0,NumOstdRd);
                                   }//(testLength >= 3)
                                }//(testLength >= 2)
                            }//(testLength >= 1)
                        }//(testLength >= 0)

                        reg_bit_clrset(MC6_REG_RAS_CTRL,0x0,0x8);//Ignor Ecc
                        // resetting the Vref value 
                        if(direction == 1) {
                            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IOB_VREF_REG(effective_cs), dutyCycle);
                            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IO_BASE_VREF_REG(effective_cs)+4, dutyCycle);
                            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IO_BASE_VREF_REG(effective_cs)+5, dutyCycle);
                        }else{
                              mmio_write_32(0xf00116d8, 0x3cc);
                                dutyCycleindex = dutyCycle;
                                //printf("Sweep:: DqVref %d , PUP %d \n", dutyCycleindex,pup);
                                /* Enter DRAM to VRef training mode */
                                mv_ddr4_vref_training_mode_ctrl(0, 0, ACCESS_TYPE_MULTICAST, MV_TRUE);
                                /* Set new VRef training value in DRAM */
                                mv_ddr4_vref_tap_set(0, 0, ACCESS_TYPE_MULTICAST, dutyCycleindex, MV_DDR4_VREF_TAP_START);
                                /*Close the VREF range*/
                                mv_ddr4_vref_tap_set(0, 0, ACCESS_TYPE_MULTICAST, dutyCycleindex,  MV_DDR4_VREF_TAP_END);
                                /* set mux to MC6 */
                               mmio_write_32(0xf00116d8, MuxReg);
                        }
                        reg_write(MC6_REG_ECC_1bit_err_counter,0x0);

                        } /// ECC re-fix   
                   
                    if((direction == 0)){
						if(((dq_vref_vec[pup] <=dutyCycle)&&((dq_vref_vec[pup] >= (dutyCycle-step))))&& (reg_val_adll >= adll_value)&& (reg_val_adll <= (adll_value+2))) {
                           res[0] = -1;
                        }
                    }else{
                        if(((reg_val_Vtap <=dutyCycle)&&(reg_val_Vtap >= (dutyCycle-step)))&& (reg_val_adll == adll)) {
                           res[0] = -1;
                        }
                    }
                   // WriteXORPat = 1; // 1 - write, 0 - dont
                    printf("%d\t,",res[0]);
				}
                printf("\n");
			}//end of dutye cycle
            if(direction == 1) {
                ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IOB_VREF_REG(effective_cs), VectorVtap[pup]);
                ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IO_BASE_VREF_REG(effective_cs)+4, VectorVtap[pup]);
                ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IO_BASE_VREF_REG(effective_cs)+5, VectorVtap[pup]);
            }else{
                // mmio_write_32(0xf00116d8, 0x3cc);
                 dutyCycleindex = dutyCycle;
                //printf("Sweep:: DqVref %d , PUP %d \n", dutyCycleindex,pup);
                /* Enter DRAM to VRef training mode */
                mv_ddr4_vref_training_mode_ctrl(0, 0, ACCESS_TYPE_MULTICAST, MV_TRUE);
                /* Set new VRef training value in DRAM */
                mv_ddr4_vref_tap_set(0, 0, ACCESS_TYPE_MULTICAST, dq_vref_vec[pup], MV_DDR4_VREF_TAP_START);
                /*Close the VREF range*/
                mv_ddr4_vref_tap_set(0, 0, ACCESS_TYPE_MULTICAST, dq_vref_vec[pup],  MV_DDR4_VREF_TAP_END);
                /* set mux to MC6 */
                // mmio_write_32(0xf00116d8, 0x38c);
            }
            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, reg + CS_BYTE_GAP(effective_cs), reg_val_adll);
		}
       // ddr3_tip_reg_dump(0);
		//printf("Final, CS %d,%s, Sweep, Result, Adll,", cs,((direction == 0) ? "TX" : "RX"));
        if(1) {
            for (if_id = 0; if_id <= MAX_INTERFACE_NUM - 1; if_id++) {
                VALIDATE_IF_ACTIVE(tm->if_act_mask, if_id);
                if (mode == 0xFF) {
                    for (pup = start_pup; pup <= end_pup; pup++) {
                        VALIDATE_BUS_ACTIVE(tm->bus_act_mask, pup);
                        printf("I/F%d-PHY%d , ", if_id, pup);
                    }
                } else {
                    printf("I/F%d , ", if_id);
                }
            }
        }
        printf("\n");
        //cs = 0;
        // Rewrite the ADLl and Voltage nominal values
        CHECK_STATUS(ddr3_tip_bus_write(dev_num, ACCESS_TYPE_MULTICAST, 0,pup_access, end_pup-1, DDR_PHY_DATA,reg + CS_BYTE_GAP(effective_cs),reg_val_adll));
        
    if((ddr3_if_ecc_enabled()==1)&& (start_pup != 8)) {
        reg_write(MC6_REG_ECC_1bit_err_counter,0x0);
        reg_bit_clrset(MC6_REG_RAS_CTRL,0x1 << MC6_ECC_ENABLE_OFFS,MC6_ECC_ENABLE_MASK << MC6_ECC_ENABLE_OFFS);
    }	
    ddr3_tip_reset_fifo_ptr(dev_num);

    //}

	return 0;
}

/********************************************************************************************/
int ddr4_VerticalAdjusment(int dev_num, u8 DepthStage,int EnMask,u8 byteNumber)
{
    u8 pup = 0, start_pup = 0, end_pup = 0;
    int printEn = 1;
    int step = (EnMask == 0)?2:4;
    int optRc = 0;
	enum hws_access_type pup_access;
    u32 reg_data;
    struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	//u32 cs;
	//u32 max_cs = ddr3_tip_max_cs_get(dev_num);
	u32 octets_per_if_num = 8;//ddr3_tip_dev_attr_get(dev_num, MV_ATTR_OCTET_PER_INTERFACE);
	
	printf(".");
   // struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
    if(byteNumber == 0xF) { // 0xF is all bytes
        start_pup = 0;
        end_pup = octets_per_if_num-1;
    }else
    {
        start_pup = byteNumber;
        end_pup = byteNumber+1;
    }
    pup_access = ACCESS_TYPE_UNICAST;
    if((ddr3_if_ecc_enabled()==1)&& (start_pup != 8)){
        reg_bit_clrset(MC6_REG_RAS_CTRL,0x0 << MC6_ECC_ENABLE_OFFS,MC6_ECC_ENABLE_MASK << MC6_ECC_ENABLE_OFFS);
        reg_write(MC6_REG_ECC_1bit_err_counter,0x0);
    }	
   
    /* does not need for MC6 recall to remove*/
    ddr3_tip_if_write(0, ACCESS_TYPE_MULTICAST, PARAM_NOT_CARE, ODPG_DATA_CONTROL_REG,
                           effective_cs << ODPG_DATA_CS_OFFS,
                           ODPG_DATA_CS_MASK << ODPG_DATA_CS_OFFS);
    //printf("\n");
    for (pup = start_pup; pup < end_pup; pup++) {
        VALIDATE_BUS_ACTIVE(tm->bus_act_mask, pup);

         
         if(printEn == 1) {
             ddr3_tip_bus_read(0, 0,ACCESS_TYPE_UNICAST, pup,DDR_PHY_DATA,CSN_IOB_VREF_REG(effective_cs),&reg_data);
            // printf("CS[%d]:byte%d RC\t=\t%2d[%2d,%2d]\t-->",effective_cs, pup,reg_data,rx_eye_lo_lvl[pup],rx_eye_hi_lvl[pup]);
         }
         if(EnMask == PerIF) {
             XorByteMask = 0;
             TypeOfMask = OR_Mask;
         }else if(EnMask == PerByte_res1){
             XorByteMask = ((unsigned long long)0x0 )| (((unsigned long long)0xFF)<<(pup*8));
             XorByteMask = ~XorByteMask;
             TypeOfMask = OR_Mask;
         }else if(EnMask == PerByte_res0){
             XorByteMask = ((unsigned long long)0x0 )| (((unsigned long long)0xFF)<<(pup*8));
             TypeOfMask = AND_Mask;
         }else if(EnMask == PerDQ0){//onlt DQ0 is togglimg
             XorByteMask = ((unsigned long long)0x0 )| (((unsigned long long)0x01)<<(pup*8));
             XorByteMask = ~XorByteMask;
             TypeOfMask = OR_Mask;
         }
         
       
       u8 VW[2];
       int result = XOR_Gradual_Test(DepthStage);
       int FPF;
       if(result == 0) {
           FPF = XORsearch_1D_2E(EDGE_PF,step, rx_eye_lo_lvl[pup],rx_eye_hi_lvl[pup],DepthStage,pup, RecCal,VW);
       }else
       {
           printf("ddr4_VerticalAdjusment - First try fail --> FP\n");
           FPF = XORsearch_1D_2E(EDGE_FP,step, rx_eye_lo_lvl[pup],rx_eye_hi_lvl[pup],DepthStage,pup, RecCal,VW);
       }
       
       if (FPF != 255)
       {
           optRc = 0.5*((int)VW[1] + (int)VW[0]);
       }else{
           if(printEn ==1) printf("No Lock-->ddr4_VerticalAdjusment Byte %d \n",pup);
           ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IOB_VREF_REG(effective_cs), reg_data);
           ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IO_BASE_VREF_REG(effective_cs)+4, reg_data);
           ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IO_BASE_VREF_REG(effective_cs)+5, reg_data);
           return 1;
       }
      // printf("\t%2d[%2d,%2d]\n", optRc,VW[0],VW[1]);
       ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IOB_VREF_REG(effective_cs), optRc);
       ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IO_BASE_VREF_REG(effective_cs)+4, optRc);
       ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IO_BASE_VREF_REG(effective_cs)+5, optRc);
    }//end of pup
        
              
        

    
    
    //printf("\n");   
    
    ddr3_tip_reset_fifo_ptr(dev_num);
    return 0;
}
/********************************************************************************************/
int ddr4_HorezintalAdjusment(int dev_num, u8 DepthStage,int EnMask, u8 byteNumber)
{
    
    u32 pup = 0, start_pup = 0, end_pup = 0;
    int printEn = 1;
    int step = (EnMask == 0)?1:2;
    int optCRx = 0;
	enum hws_access_type pup_access;
    u32 reg_data;
    struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	//u32 cs;
	//u32 max_cs = ddr3_tip_max_cs_get(dev_num);
	u32 octets_per_if_num = 8;//ddr3_tip_dev_attr_get(dev_num, MV_ATTR_OCTET_PER_INTERFACE);
	
	printf(".");
   // struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
    if(byteNumber == 0xF) { // 0xF is all bytes
        start_pup = 0;
        end_pup = octets_per_if_num-1;
    }else
    {
        start_pup = byteNumber;
        end_pup = byteNumber+1;
    }
	pup_access = ACCESS_TYPE_UNICAST;
    if((ddr3_if_ecc_enabled()==1)&& (start_pup != 8)){
        reg_bit_clrset(MC6_REG_RAS_CTRL,0x0 << MC6_ECC_ENABLE_OFFS,MC6_ECC_ENABLE_MASK << MC6_ECC_ENABLE_OFFS);
        reg_write(MC6_REG_ECC_1bit_err_counter,0x0);
    }	
   
    /* does not need for MC6 recall to remove*/
    ddr3_tip_if_write(0, ACCESS_TYPE_MULTICAST, PARAM_NOT_CARE, ODPG_DATA_CONTROL_REG,
					       effective_cs << ODPG_DATA_CS_OFFS,
					       ODPG_DATA_CS_MASK << ODPG_DATA_CS_OFFS);
    //printf("\n");
	for (pup = start_pup; pup < end_pup; pup++) {
        VALIDATE_BUS_ACTIVE(tm->bus_act_mask, pup);
        if(printEn == 1) {
             ddr3_tip_bus_read(0, 0,ACCESS_TYPE_UNICAST, pup,DDR_PHY_DATA,READ_CENTRALIZATION_PHY_REG + 4*effective_cs,&reg_data);
             //printf("CS[%d]:byte%d CRx\t=\t%2d\t-->", effective_cs,pup,reg_data);
         }
          if(EnMask == PerIF) {
             XorByteMask = 0;
             TypeOfMask = OR_Mask;
         }else if(EnMask == PerByte_res1){
             XorByteMask = ((unsigned long long)0x0 )| (((unsigned long long)0xFF)<<(pup*8));
             XorByteMask = ~XorByteMask;
             TypeOfMask = OR_Mask;
         }else if(EnMask == PerByte_res0){
             XorByteMask = ((unsigned long long)0x0 )| (((unsigned long long)0xFF)<<(pup*8));
             TypeOfMask = AND_Mask;
         }else if(EnMask == PerDQ0){//onlt DQ0 is togglimg
             XorByteMask = ((unsigned long long)0x0 )| (((unsigned long long)0x01)<<(pup*8));
             XorByteMask = ~XorByteMask;
             TypeOfMask = OR_Mask;
         }
         
       u8 VW[2];
       int result = XOR_Gradual_Test(DepthStage);
       int FPF;
       if(result == 0) {
           FPF = XORsearch_1D_2E(EDGE_PF,step, 0,31,DepthStage,pup, CRx,VW);
       }else
       {
           printf("ddr4_HorezintalAdjusment - First try fail --> FP\n");
           FPF = XORsearch_1D_2E(EDGE_FP,step, 0,31,DepthStage,pup, CRx,VW);
       }
       
       if (FPF != 255)
       {
           optCRx = 0.5*((int)VW[1] + (int)VW[0]);
       }else{
           if(printEn ==1) printf("No Lock-->ddr4_VerticalAdjusment Byte %d \n",pup);
           return 1;
       }
       //printf("\t%d[%2d,%2d]\n", optCRx,VW[0],VW[1]);
       ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, READ_CENTRALIZATION_PHY_REG + effective_cs*4, optCRx);
    }//end of pup
    
    //printf("\n");   
	
	ddr3_tip_reset_fifo_ptr(dev_num);
    return 0;

}

/********************************************************************************************/
int ddr4_DiagonalAdjusment(int dev_num, u8 DepthStage,int EnMask, u8 byteNumber)
{
    u32 pup = 0, start_pup = 0, end_pup = 0;
    //int printEn = 0;
	//u32 res[MAX_INTERFACE_NUM] = { 0 };
            
    
	enum hws_access_type pup_access;
	//u32 cs;
	//u32 max_cs = ddr3_tip_max_cs_get(dev_num);
	u32 octets_per_if_num = 8;//ddr3_tip_dev_attr_get(dev_num, MV_ATTR_OCTET_PER_INTERFACE);
	
	printf(".");
   // struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
    if(byteNumber == 0xF) { // 0xF is all bytes
        start_pup = 0;
        end_pup = octets_per_if_num-1;
    }else
    {
        start_pup = byteNumber;
        end_pup = byteNumber+1;
    }
	pup_access = ACCESS_TYPE_UNICAST;
    if((ddr3_if_ecc_enabled()==1)&& (start_pup != 8)){
        reg_bit_clrset(MC6_REG_RAS_CTRL,0x0 << MC6_ECC_ENABLE_OFFS,MC6_ECC_ENABLE_MASK << MC6_ECC_ENABLE_OFFS);
        reg_write(MC6_REG_ECC_1bit_err_counter,0x0);
    }	
   
    /* does not need for MC6 recall to remove*/
    ddr3_tip_if_write(0, ACCESS_TYPE_MULTICAST, PARAM_NOT_CARE, ODPG_DATA_CONTROL_REG,
		   		        effective_cs << ODPG_DATA_CS_OFFS,
                        ODPG_DATA_CS_MASK << ODPG_DATA_CS_OFFS);
    
	for (pup = start_pup; pup < end_pup; pup++) {
        u32 NomADLL = 0, NomRC = 0,tmpNomADLL = 0, tmpNomRC = 0;
        u32 Step1, init_val1,end_val1,Step2, init_val2,end_val2;
        int step;
        u8 VW[2];
        if(EnMask == PerIF) {
             XorByteMask = 0;
             TypeOfMask = OR_Mask;
         }else if(EnMask == PerByte_res1){
             XorByteMask = ((unsigned long long)0x0 )| (((unsigned long long)0xFF)<<(pup*8));
             XorByteMask = ~XorByteMask;
             TypeOfMask = OR_Mask;
         }else if(EnMask == PerByte_res0){
             XorByteMask = ((unsigned long long)0x0 )| (((unsigned long long)0xFF)<<(pup*8));
             TypeOfMask = AND_Mask;
         }else if(EnMask == PerDQ0){//onlt DQ0 is togglimg
             XorByteMask = ((unsigned long long)0x0 )| (((unsigned long long)0x01)<<(pup*8));
             XorByteMask = ~XorByteMask;
             TypeOfMask = OR_Mask;
         }
         if(pup == 8) {
            // printf("\n\tByte ECC:: XorByteMask = 0x%16llx\n",XorByteMask);
         }
        // Read Nominal Start point
        ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST,pup, DDR_PHY_DATA,READ_CENTRALIZATION_PHY_REG + effective_cs*4,&NomADLL);
        ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST,pup, DDR_PHY_DATA,CSN_IOB_VREF_REG(effective_cs),&NomRC);
        // This steps define the slope of the search
        Step1 = 1; // Adll step
        Step2 = 3; // RC step
        // The limit are unnown for this search not like the Vertical search so we take the limit at the end of the Adll-RC plane
        // and take the minimum between the two axis steps 
        step =((NomADLL/Step1)>(NomRC/Step2))?(NomRC/Step2):(NomADLL/Step1);
        init_val1 = NomADLL-step*Step1;
        init_val2 = NomRC-step*Step2;
        /* ************* */
        step =(((31-NomADLL)/Step1)>((63-NomRC)/Step2))?((63-NomRC)/Step2):((31-NomADLL)/Step1);
        end_val1 = NomADLL+step*Step1;
        end_val2 = NomRC+step*Step2;
        
         //printf("\n");
         if(EnMask == 0) {
             XorByteMask = 0;
         }else{
             XorByteMask = ((unsigned long long)0x0 )| (((unsigned long long)0xFF)<<(pup*8));
          //   printf("\n\tXorByteMask = 0x%16llx\n",XorByteMask);
             XorByteMask = ~XorByteMask;
            // printf("\tXorByteMask = 0x%16llx\n",XorByteMask);
         }
        

        int final = XORsearch_2D_2E(EDGE_FPF, CRx, HWS_LOW2HIGH , Step1, init_val1,end_val1,RecCal, HWS_LOW2HIGH , Step2, init_val2,end_val2,DepthStage,pup, VW);
        
        if(final != 255){   
            // VW[.] contain the number of step that have been done for each search
            tmpNomADLL = 0.5*((VW[0]*Step1 + init_val1)/* start*/ + (end_val1 - VW[1]*Step1)/*end*/); /*new Nom*/        
            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, READ_CENTRALIZATION_PHY_REG + effective_cs*4, tmpNomADLL);
            tmpNomRC = 0.5*((VW[0]*Step2 + init_val2)/* start*/ + (end_val2 - VW[1]*Step2)/*end*/); /*new Nom*/ 
            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IOB_VREF_REG(effective_cs), tmpNomRC);
            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IO_BASE_VREF_REG(effective_cs)+4, tmpNomRC);
            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IO_BASE_VREF_REG(effective_cs)+5, tmpNomRC);
           // printf("\nCS[%d]:byte%d:1st XORsearch_2D_2E [%2d,%2d]==>[%2d,%2d]\n",effective_cs,pup,NomADLL,NomRC,tmpNomADLL,tmpNomRC);
        }else{
            printf("ddr4_DiagonalAdjusment::cs[%d]:byte%d:1st XORsearch_2D_2E - Fail\n",effective_cs,pup);
        }
    }
    //printf("\n"); 
    for (pup = start_pup; pup < end_pup; pup++) {
        u32 NomADLL = 0, NomRC = 0,tmpNomADLL = 0, tmpNomRC = 0;
        u32 Step1, init_val1,end_val1,Step2, init_val2,end_val2;
        int step;
        ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST,pup, DDR_PHY_DATA,READ_CENTRALIZATION_PHY_REG + effective_cs*4,&NomADLL);
        ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST,pup, DDR_PHY_DATA,CSN_IOB_VREF_REG(effective_cs),&NomRC);
        Step1 = 1;
        Step2 = 3;
        step =((NomADLL/Step1)>((63-NomRC)/Step2))?((63-NomRC)/Step2):(NomADLL/Step1);
        init_val1 = NomADLL-step*Step1;
        init_val2 = NomRC+step*Step2;
        /* ************* */
        step =(((31-NomADLL)/Step1)>((NomRC)/Step2))?((NomRC)/Step2):((31-NomADLL)/Step1);
        end_val1 = NomADLL+step*Step1;
        end_val2 = NomRC-step*Step2;
        
         //printf("\n");
         if(EnMask == 0) {
             XorByteMask = 0;
         }else{
             XorByteMask = ((unsigned long long)0x0 )| (((unsigned long long)0xFF)<<(pup*8));
          //   printf("\n\tXorByteMask = 0x%16llx\n",XorByteMask);
             XorByteMask = ~XorByteMask;
            // printf("\tXorByteMask = 0x%16llx\n",XorByteMask);
         }
        u8 VW[2];
        int final = XORsearch_2D_2E(EDGE_FPF, CRx, HWS_LOW2HIGH , Step1, init_val1,end_val1,RecCal, HWS_HIGH2LOW , Step2, init_val2,end_val2,DepthStage,pup, VW);
        //printf("\nXORsearch_2D_2E Solution = %2d - [%2d,%2d]\n",final,VW[0],VW[1]);
        if(final != 255){   
            tmpNomADLL = 0.5*((VW[0]*Step1 + init_val1)/* start*/ + (end_val1 - VW[1]*Step1)/*end*/); /*new Nom*/        
            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, READ_CENTRALIZATION_PHY_REG + effective_cs*4, (tmpNomADLL - 0));
            tmpNomRC = 0.5*((init_val2 - VW[0]*Step2)/* start*/ + (end_val2 + VW[1]*Step2)/*end*/); /*new Nom*/ 
            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IOB_VREF_REG(effective_cs), (tmpNomRC - 0));
            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IO_BASE_VREF_REG(effective_cs)+4, (tmpNomRC - 0));
            ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0, pup_access, pup, DDR_PHY_DATA, CSN_IO_BASE_VREF_REG(effective_cs)+5, (tmpNomRC - 0));
            //printf("\ncs[%d]:byte%d:2nd XORsearch_2D_2E [%2d,%2d]==>[%2d,%2d]\n",effective_cs,pup,NomADLL,NomRC,tmpNomADLL,tmpNomRC);
        }else{
            printf("ddr4_DiagonalAdjusment::cs[%d]:byte%d:2nd XORsearch_2D_2E - Fail\n",effective_cs,pup);
        }
    }  
	
	ddr3_tip_reset_fifo_ptr(dev_num);

	//}

	return 0;
}

int RxAdjust()
{
    //effective_cs = 0;
    int debugflag = 1;
    DataWidth = 4;
    enum Maskdef EnMask = PerByte_res1;
    enum Maskdef startMask = PerIF;
    enum SearchType TypeOfSearch = Vertic;
    int Res = 1;
    int AlgoLoop = 0;
    u8 DepthStage = 2;
   // struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();

    u32 octets_per_if_num = 8;//ddr3_tip_dev_attr_get(0, MV_ATTR_OCTET_PER_INTERFACE);
	int byteNumStart = 0;
    int ByteNumberEnd = octets_per_if_num;


    ////////////////////////////////////////////
    // check severity of the I/F
    XorByteMask = 0;
    TypeOfMask = OR_Mask;
    Res = XOR_Gradual_Test(DepthStage);
    if(Res == 0){
        startMask = PerIF;
        // Check worst case severity
        DepthStage = 3;
        Res = XOR_Gradual_Test(DepthStage);
    }else
        startMask = PerByte_res1;
    ////////////////////////////////////////////  
    
    if(Res != 0) {//if DepthStage = 3 PerIF has pass no need for thi stage. 
       // printf("\nRxAdjust::Moving to V-H 1st level searc.");
       // printf("\n==============================.\n");
       // Counter(1);
        for(int byteNum = byteNumStart ; byteNum<ByteNumberEnd ; byteNum++) {
            if((ddr3_if_ecc_enabled()==1)&& (byteNum != 8)){
                reg_bit_clrset(MC6_REG_RAS_CTRL,0x0 << MC6_ECC_ENABLE_OFFS,MC6_ECC_ENABLE_MASK << MC6_ECC_ENABLE_OFFS);
                reg_write(MC6_REG_ECC_1bit_err_counter,0x0);
            }
            DepthStage = 2;
            TypeOfSearch = Vertic;
            EnMask = startMask;

            for(AlgoLoop = 0 ; AlgoLoop < 4; AlgoLoop++) {
                if(TypeOfSearch == Vertic) {
                    Res = ddr4_VerticalAdjusment(0,DepthStage,EnMask,byteNum);
                }else
                {
                    Res = ddr4_HorezintalAdjusment(0,DepthStage,EnMask,byteNum);
                }
                if(byteNum == 8) {
                    reg_bit_clrset(MC6_REG_RAS_CTRL,0x8,0x8);//Ignor Ecc
                    reg_bit_clrset(MC6_REG_RAS_CTRL,0x0,0x8);//Ignor Ecc
                    reg_write(MC6_REG_ECC_1bit_err_counter,0x0);

                }
                switch (AlgoLoop) { 
                case 0:
                    if(Res == 0) {  // pass moving to H search
                        TypeOfSearch = Horizon;
                        if(debugflag == 2) printf("\nRxAdjust::Byte %d - AlgoLoop %d pass.",byteNum,AlgoLoop);
                        AlgoLoop = 2;
                        continue;
                    }else{          // Fail try other mask
                        EnMask = PerByte_res0;
                        if(debugflag == 2) printf("\nRxAdjust::Byte %d - AlgoLoop %d Fail.",byteNum,AlgoLoop);
                    }
                    break;
                case 1:
                    if(Res == 0) { // pass moving to H search
                        TypeOfSearch = Horizon;
                        if(debugflag == 2) printf("\nRxAdjust::Byte %d - AlgoLoop %d pass.",byteNum,AlgoLoop);
                        AlgoLoop = 2;
                        continue;
                    }else{         // Still Fail try H search first
                        TypeOfSearch = Horizon;
                        EnMask = PerByte_res1;
                        if(debugflag == 2) printf("\nRxAdjust::Byte %d - AlgoLoop %d Fail.",byteNum,AlgoLoop);
                    }
                    break;
                case 2:
                    if(Res == 0) { // pass moving to V search
                        TypeOfSearch = Vertic;
                        if(debugflag == 2) printf("\nRxAdjust::Byte %d - AlgoLoop %d pass.",byteNum,AlgoLoop);
                        AlgoLoop = 2;
                        continue;
                    }else{         // FAIL!
                        if(debugflag == 2) printf("\nRxAdjust::Byte %d - AlgoLoop %d not converge.",byteNum,AlgoLoop);
                        return MV_FAIL;
                    }
                    break;
                case 3:
                    if(Res == 0) { // pass V & H search
                        //Pass moving to next Byte;
                        if(debugflag == 2) printf("\nRxAdjust::Byte %d - AlgoLoop %d pass.",byteNum,AlgoLoop);
                    }else{         // FAIL!
                        if(debugflag == 2) printf("\nRxAdjust::Byte %d - AlgoLoop %d not converge.",byteNum,AlgoLoop);
                        return MV_FAIL;
                    }
                    break;
                }//end switch
            }//end of Algoloop
            if((ddr3_if_ecc_enabled()==1)&& (byteNum != 8)) {
                reg_write(MC6_REG_ECC_1bit_err_counter,0x0);
                reg_bit_clrset(MC6_REG_RAS_CTRL,0x1 << MC6_ECC_ENABLE_OFFS,MC6_ECC_ENABLE_MASK << MC6_ECC_ENABLE_OFFS);
            }
        }//end of ByteNum
    }// 
     //
   // printf("\nRxAdjust::Moving to V-H 2nd level searc.");
   // printf("\n==============================.\n");
    for(int byteNum = byteNumStart ; byteNum<ByteNumberEnd ; byteNum++) {
        if((ddr3_if_ecc_enabled()==1)&& (byteNum != 8)){
            reg_bit_clrset(MC6_REG_RAS_CTRL,0x0 << MC6_ECC_ENABLE_OFFS,MC6_ECC_ENABLE_MASK << MC6_ECC_ENABLE_OFFS);
            reg_write(MC6_REG_ECC_1bit_err_counter,0x0);
        }
        DepthStage = 4;
        EnMask = PerIF;
        
        
        Res = ddr4_HorezintalAdjusment(0,DepthStage,EnMask,byteNum);
        if(Res != 0) {
            printf("\nRxAdjust::CS[%d]:Byte%d 2nd level Horezintal Adjust fail",effective_cs,byteNum);
        }

        Res = ddr4_VerticalAdjusment(0,DepthStage,EnMask,byteNum);
        if(Res != 0) {
            printf("\nRxAdjust::CS[%d]:Byte%d 2nd level Vertical Adjust fail",effective_cs,byteNum);
        }
        if(byteNum == 8) {
            reg_bit_clrset(MC6_REG_RAS_CTRL,0x8,0x8);//Ignor Ecc
            reg_bit_clrset(MC6_REG_RAS_CTRL,0x0,0x8);//Ignor Ecc
            reg_write(MC6_REG_ECC_1bit_err_counter,0x0);

        }
        
        if((ddr3_if_ecc_enabled()==1)&& (byteNum != 8)) {
            reg_write(MC6_REG_ECC_1bit_err_counter,0x0);
            reg_bit_clrset(MC6_REG_RAS_CTRL,0x1 << MC6_ECC_ENABLE_OFFS,MC6_ECC_ENABLE_MASK << MC6_ECC_ENABLE_OFFS);
        }
    }

    
    //Counter(2);
  //  printf("\n");
    EnMask = PerIF;
    DepthStage = 4;
   // printf("\nRxAdjust::Moving to diag searc.");
   // printf("\n==============================.\n");
    for(int byteNum = byteNumStart ; byteNum<ByteNumberEnd ; byteNum++) {
        if((ddr3_if_ecc_enabled()==1)&& (byteNum != 8)){
            reg_bit_clrset(MC6_REG_RAS_CTRL,0x0 << MC6_ECC_ENABLE_OFFS,MC6_ECC_ENABLE_MASK << MC6_ECC_ENABLE_OFFS);
            reg_write(MC6_REG_ECC_1bit_err_counter,0x0);
        }
        //Counter(1);
        ddr4_DiagonalAdjusment(0,DepthStage,EnMask,byteNum);
        //Counter(2);
        if(byteNum == 8) {
                reg_bit_clrset(MC6_REG_RAS_CTRL,0x8,0x8);//Ignor Ecc
                reg_bit_clrset(MC6_REG_RAS_CTRL,0x0,0x8);//Ignor Ecc
                reg_write(MC6_REG_ECC_1bit_err_counter,0x0);

        }
        if((ddr3_if_ecc_enabled()==1)&& (byteNum != 8)) {
            reg_write(MC6_REG_ECC_1bit_err_counter,0x0);
            reg_bit_clrset(MC6_REG_RAS_CTRL,0x1 << MC6_ECC_ENABLE_OFFS,MC6_ECC_ENABLE_MASK << MC6_ECC_ENABLE_OFFS);
        }
    }
   // printf("\n");
    return 0;
}



/********************************************************************************************
RunValidation 
Description - This is the main Validation Platform function. this function will be call from any where in the main algorithm code for Validation 
 
input: 
    topology - this should be all that is needed to understand in which platform are we.
    testNum - the test number will continue to grow as the Validation test will be extend. each new test need to be added in a new function.
        0 - XOR on CS0 one loop one pattern Type - 1M,Byte to copare 1BL 
        1 - XOR on CS0 3D sweep loop, 1) pattern Type, 2) transfer data 3) Byte count 
        2 - XOR on CS0 sweep loop, one pattern Type, on the TotalData2Test.
        3 - XOR on CS1 one loop one pattern Type - 1M,Byte to copare 1BL
        4 - XOR on CS0 Source CS1 Destantion one loop one pattern Type - 1M,Byte to copare 1BL
 
 
output: TBD


Desclamier - Supprot only 7040/8040 projects
*********************************************************************************************/

//int BankMap_conv[] = {1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8182,16384,32768}; //in kbyte
int printStabilityonce = MV_TRUE;

int XOT_Test_Wrapper(u32 TotalData2Test,enum CStest CSinvovle ,enum VP_pattern Pattern,int ByteCount,int index)
{
    int ret=0;
    int patternStart = (Pattern == VP_pattern_Last)? 0:Pattern;
    int patternEnd   = (Pattern == VP_pattern_Last)? Pattern:Pattern+1;
    u32 Source_AddressL = (reg_read(MC6_REG_MMAP_LOW_CH0(0))&(0x1FF<<23));
    u32 Source_AddressH = reg_read(MC6_REG_MMAP_HIGH_CH0(0)); 

    int SourceCS = (CSinvovle/4);
    int TargetCS = (CSinvovle%4);

    unsigned long long Source_Address = (((unsigned long long)Source_AddressH<<32)| ((unsigned long long)Source_AddressL) ) 
                                        + (unsigned long long)((SourceCS)* Area_length) 
                                        + (unsigned long long)(TotalData2Test)*(unsigned long long)(index);
                                                                    
    unsigned long long End_addr_bytes = (((unsigned long long)Source_AddressH<<32)| ((unsigned long long)Source_AddressL) ) 
                                        + (unsigned long long)((1+TargetCS)* Area_length) 
                                        - (unsigned long long)(TotalData2Test)*(unsigned long long)(index) ;
    u32 Destanation_AddressL =  (u32)((End_addr_bytes)& 0x00000000FFFFFFFF);
    u32 Destanation_AddressH =  (u32)((End_addr_bytes)>>32);

    /* printf("\n\t\t\tSource_Address = 0x%16llx\n",Source_Address);
     printf("\n\t\t\tPart1 = 0x%16llx\n",(((unsigned long long)Source_AddressH<<32)| ((unsigned long long)Source_AddressL) ));
     printf("\n\t\t\tPart2 = 0x%16llx\n",(unsigned long long)((CSindex)* Area_length));
     printf("\n\t\t\tPart3 = 0x%16llx\n",(unsigned long long)(TotalData2Test*(index)));*/

    Source_AddressL =  (u32)((Source_Address)& 0x00000000FFFFFFFF);
    Source_AddressH =  (u32)((Source_Address)>>32);

    
    ////////////////////////////////////////////////////
    int CheckFlag = XOT_Test_ConfigurationCheck(Source_Address, TotalData2Test, End_addr_bytes);
    if(CheckFlag == 1) {
        return 0;
    }
    ////////////////////////////////////////////////////////
    /////using the Source_Address parameter to calculate the descriptor position
    Source_Address = ((SourceCS)* Area_length) ;
        
    u32 DescBaseAddrL = (u32)((Source_Address)& 0x00000000FFFFFFFF);
    u32 DescBaseAddrH = (u32)((Source_Address)>>32);
    //////////////////////////////////////////////////////
    XorPrint(TotalData2Test,ByteCount,DescBaseAddrL,DescBaseAddrH,Source_AddressL,Source_AddressH,
                     Destanation_AddressL,Destanation_AddressH);
    
    for(int patI = patternStart ; patI < patternEnd ; patI++) {
        ret += XOR_Test(TotalData2Test,Source_AddressL, Source_AddressH, Destanation_AddressL,Destanation_AddressH, ByteCount,patI,DescBaseAddrL,DescBaseAddrH,NumOstdRd);
        if(printlog < 5) printf("\t\t- %d errors on pattern %d\t;\tRun Time[HCLK]: 0x%08x.%08x",ret,patI,Hclk_CounterHigh,Hclk_CounterLow);
        if(ret != 0) {
            printf("\n\tS[H,L] -> D[H,L]= [0x%08x,0x%08x]-> ", Source_AddressH,Source_AddressL);
            printf("[0x%08x,0x%08x] ", Destanation_AddressH,Destanation_AddressL);
        }
    }
    
    return ret;
   
}

int SetValidationGlobals()
{
    // struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
    BankMap = (reg_read(MC6_REG_MC_CONFIG(0))&(0x1F<<24))>>24;
    Area_length = 0;
    int Area_Length_dec = (reg_read(MC6_REG_MMAP_LOW_CH0(0))&(0x1F<<16))>>16;
    DataWidth = (reg_read(MC6_BASE_ADDR + 0x44)&(0x7<<8))>>8;
    switch ((reg_read(MC6_REG_MC_CONFIG(0))&(0x3<<16))>>16){
    case 0:
        PageSize = 0x200;
        break;
    case 1:
        PageSize = _1K;
        break;
    case 2:
        PageSize = _1K*2;
        break;
    }
    
    Area_length = AreaLengthConv[Area_Length_dec-7];//128
    //
    BankMap = BankMap_conv[BankMap-3];
    if(printlog == 1) {
     /*   printf("DDR Validation Platform\n");
        printf("--------------------------\n");
        printf("Bus width\t-\t%2d\n",((DataWidth == 0x3)?(4*8):(8*8)));
        printf("Area_length\t-\t%5lldMbyte\n",Area_length );
        printf("Bank Map\t-\t%5dKbyte\n",BankMap );
        printf("--------------------------\n");
        printf("--------------------------\n"); */
    }
    Area_length = Area_length*(unsigned long long)_1M;
    return 0;
}

int RunValidation(enum VP_tests testNum)
{
    // each time the valudation function is called the stability must be print
    if(printStabilityonce == MV_TRUE){
       // print_stability_log(0);
       // printStabilityonce = MV_FALSE;
        SetValidationGlobals();
    }
        
    mdelay(1000);
    int ret=0;
    switch (testNum) {
    case XOR_CS0_general_1M:
        {
            
            // This is a simple and short test on CS0 - 1 XOR test transaction of 1Mbyte in chunk of 1 BL from 1M
            // to End of memory deep less 3M            
            u32 TotalData2Test = _1M;
            int ByteCount = (DataWidth == 0x3)?(4*8):(8*8); // 1 BL
            ret=0;
            enum VP_pattern pattern = VP_pattern_Last;
            enum CStest CSinvovle = CS0ToCS0;
            TotalData2Test = 128*8;
            ByteCount = 128;
            ret = XOT_Test_Wrapper(TotalData2Test,CSinvovle,pattern,ByteCount,1);
           
            break;
        }
        case XOR_CS0_general_multiaccessSize:
        {
            // This is a more deeper test on CS0 - loop on XOR test transaction, change from 64K to 128M. The chunks of each transaction size is loop from divby8 t
            // o divby2. The destination of the transaction is the end of the memory deep minus 2 size of the data to transaction.
            // All is been done with three different patterns.
            
            u32 TotalData2Test = _1M;
            int NumOfError = 0;
            enum VP_pattern pattern = VP_pattern_Last;
            enum CStest CSinvovle = CS0ToCS0;
            for(TotalData2Test = 64*_1K ; TotalData2Test < _128M ; TotalData2Test = TotalData2Test*2) {
                ret=0;
                for(int ByteCount = TotalData2Test/64; ByteCount < TotalData2Test ; ByteCount = ByteCount*2) {
                    ret = XOT_Test_Wrapper(TotalData2Test,CSinvovle,pattern,ByteCount,1);
                    NumOfError += ret;
                }
            }
            if(printlog == 1) {
                printf("\n\nVP::XOR_CS0_general_multiaccessSize:: %d Error\n",NumOfError);
            }
            return NumOfError;
            break;
        }
        case XOR_CS0_FullAddressAccess:
        {
            // This is the deepest test on CS0. we test all over the memory depth, twise. 
            // we transaction a 64M from start to the end and back. Start + 64M x i ==> End - 64M x i. we stop when (Start + 64M x i) get to the end of the memory
            u32 TotalData2Test = _128M;
            int ByteCount = TotalData2Test/64;
            enum VP_pattern pattern = All_Aggresive;
            int NumOfError = 0;
            enum CStest CSinvovle = CS0ToCS0;
            int MaxIteration = (((Area_length-TotalData2Test)/TotalData2Test)-1);
            int IndexTH = MaxIteration/4;
           
            for(int index = 1 ; index < MaxIteration ; index++ ) {
                int ret=0;
                if(index%IndexTH==0) {
                    printf("\n----------------------------------");
                    printf(" XOR_CS0_FullAddressAccess index = %d (%d)",index,MaxIteration);
                    printf("----------------------------------\n");
                    printlog = 1;

                }
                for(ByteCount = TotalData2Test/64; ByteCount < (TotalData2Test/4) ; ByteCount = ByteCount*2) {
                    ret=0;
                    ret = XOT_Test_Wrapper(TotalData2Test,CSinvovle,pattern,ByteCount,index);
                    NumOfError += ret;
                    if((index==1) || (index%IndexTH==0)) {
                        printlog = 1;
                    }
                }
                
            }

            
            if(printlog == 1) {
                printf("RunValidation XOR_CS0_FullAddressAccess returned %d Errors\n",NumOfError);
            }
            return NumOfError;
            break;
        }
    case CPU_CS0_general_1M:
        {
            
            u32 start_addres_low = (reg_read(MC6_REG_MMAP_LOW_CH0(0))&(0x1FF<<23));
            u32 start_addres_high = reg_read(MC6_REG_MMAP_HIGH_CH0(0)); 
            Area_length = _1M;//_2G;
            unsigned long long End_addr_bytes = (((unsigned long long)start_addres_high<<32)| ((unsigned long long)start_addres_low) ) + Area_length ;
            u32 End_addres_low =  (u32)((End_addr_bytes)& 0x00000000FFFFFFFF);
            u32 End_addres_high =  (u32)((End_addr_bytes)>>32);
            int Access_Numbers = 1024; /* need to consider total memory size in CS*/
            int TestWidth = (DataWidth == 0x3)?(32):(64);
            
            
            ret = MemoryTestPerByte(start_addres_low, start_addres_high, End_addres_low, End_addres_high,
                       Access_Numbers, TestWidth, 1, 0xC);
            printf("CS0 MemoryTestPerByte: Number of access %d %d errors\n",Access_Numbers,ret);
            return ret;
           break;
        }
    case TIP2MC_equation:
        {
            MC_CL_CWL_impact();
            break;  
        }
    case Eye_sweep_AllCS:
        {//CS0&1
            u32 max_cs = ddr3_tip_max_cs_get(0);
            for(effective_cs = 0;effective_cs < max_cs;effective_cs++)
            {
                ddr3_tip_if_write(0, ACCESS_TYPE_MULTICAST, PARAM_NOT_CARE, ODPG_DATA_CONTROL_REG,
                               effective_cs << ODPG_DATA_CS_OFFS,
                               ODPG_DATA_CS_MASK << ODPG_DATA_CS_OFFS);
                int repeat = 1;
                DataWidth = 4;
                int Maxpup = (DataWidth == 0x3)?(4):(7);
                int testdepth = 2;
                //testLength - 0 CPU only, 1 include Xor 128M@128K, 2 include partial write 1M@1K+1/2/3
                //             3 include Xor 1G@16M
                for(int pup = 0 ; pup <= Maxpup ; pup++ ) {
                    ddr4_cpu_sweep_test(0, repeat, testdepth ,0x1,pup);
                    
                }
                for(int pup = 0 ; pup <= Maxpup ; pup++ ) {
                    ddr4_cpu_sweep_test(0, repeat, testdepth ,0x0,pup);
                    
                }
            }
              
            break;
        }
    case Eye_sweepTx_CS0://Tx Run Only can be run before the training
        {
            effective_cs = 0;
            int repeat = 1;
            DataWidth = 4;
            int Maxpup = (DataWidth == 0x3)?(4):(7);
            int testdepth = 2;
            for(int pup = 0 ; pup <= Maxpup ; pup++ ) {
               ddr4_cpu_sweep_test(0, repeat, testdepth ,0x0,pup);
            }
            
            break;
        }
    case Eye_sweepRx_CS0_woPBS://Rx Run Only but with removing all deskew elements.CS0 only
        {
            int repeat = 1;
            DataWidth = 4;
            int Maxpup = (DataWidth == 0x3)?(4):(7);
            int deskew[11];
            int cs = 0;
            u32 reg_val_deskew;
            int testdepth = 2;
            for(int pup = 0 ; pup <= Maxpup ; pup++ ) {
               
                for(int pad = 0 ; pad < 11 ; pad++) {
                    ddr3_tip_bus_read(0, 0, ACCESS_TYPE_UNICAST,pup, DDR_PHY_DATA,0x50 + CS_BYTE_GAP(cs),&reg_val_deskew);
                    deskew[pad] = reg_val_deskew;
                    ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0,ACCESS_TYPE_UNICAST, pup, DDR_PHY_DATA,0x50 + CS_BYTE_GAP(cs),0);
                }
                ddr4_cpu_sweep_test(0, repeat, testdepth ,0x1,pup);
                for(int pad = 0 ; pad < 11 ; pad++) {
                    ddr3_tip_bus_write(0, ACCESS_TYPE_UNICAST, 0,ACCESS_TYPE_UNICAST, pup, DDR_PHY_DATA,0x50 + CS_BYTE_GAP(cs),deskew[pad]);
                }
            }
            
            break;
        }
       case Eye_sweepRx_CS0:
        {
            effective_cs = 0;
            DataWidth = 4;
            //int Maxpup = (DataWidth == 0x3)?(4):(7);
            //printf("\nNew code\n");
             u32 max_cs = ddr3_tip_max_cs_get(0);
            //reg_write(0x6f8130, (0x10000005|(0x20<<4)|(0x20<<12)));
            //sleep(1);
            for(effective_cs = 0;effective_cs < max_cs;effective_cs++)
            {
                RxAdjust();
			}
            //reg_write(0x6f8130, (0x10000005|(0x24<<4)|(0x24<<12)));
            //sleep(1);
            
           // mmio_write_32(0xF0000030, 0x00000003);//set private ID
            
           // mmio_write_32(0xF000070C, 0x000000FF);//enable SMC for STX access
           
            //print_stability_log(0);
          
#if 0
              int repeat = 1;
			  int testdepth = 2;
			  XorByteMask = 0x0 ;
			  int pup = 0;
			  testdepth = 4;
            //for(int avs = 0x2F ; avs > 0xF ; avs = avs -5) {
               // reg_write(0x6f8130, (0x10000005|(avs<<4)|(avs<<12)));
                //printf("Changing AVS value to be 0x%x\n",avs);
                mdelay(10);
    			  for(effective_cs = 0;effective_cs < max_cs;effective_cs++)
    			  {
    				  for(pup = 0 ; pup <= 7 ; pup++ ) {
    					  /*for(int bit = 0 ; bit <= 7 ; bit++ ) {
    						unsigned long long bitmask= (0x1<<bit);
    						XorByteMask = ((unsigned long long)0x0 )| (bitmask<<(pup*8));
    						XorByteMask = ~XorByteMask;*/

    					  ddr4_cpu_sweep_test(0, repeat, testdepth ,0x1,pup);/*0=tx, 1=rx*/

    					  /*}*/
    				  }
    				  for(pup = 0 ; pup <= 7 ; pup++ ) {
    					  /*for(int bit = 0 ; bit <= 7 ; bit++ ) {
    						unsigned long long bitmask= (0x1<<bit);
    						XorByteMask = ((unsigned long long)0x0 )| (bitmask<<(pup*8));
    						XorByteMask = ~XorByteMask;*/

    					  ddr4_cpu_sweep_test(0, repeat, testdepth ,0x0,pup);/*0=tx, 1=rx*/

    					  /*}*/
				}
    			  }
           // }
#endif		
           reg_write(0x6f8130, nominal_avs);

            //reg_write(0x6f8130, (0x10000005|(0x24<<4)|(0x24<<12)));				
			break;
        }
		
		case Eye_sweepRx_CS0_perbit:
		{
            effective_cs = 0;
            int repeat = 1;
            DataWidth = 4;
            //int Maxpup = (DataWidth == 0x3)?(4):(7);
            int testdepth = 1;
            
            XorByteMask = 0x0 ;
            int pup = 7;
            //for(pup = 0 ; pup <= 0 ; pup++ ) {
            stamstam = 1024;
           // for(stamstam = 1 ; stamstam < 5000 ; stamstam = stamstam*2) {
            unsigned long long bitmask = 0;
            for(int bit = 0 ; bit <= 63 ; bit++ ) {
                
                  bitmask= bitmask|(((unsigned long long)0x1<<(63-bit)));
                  //  XorByteMask = ((unsigned long long)0x0 )| (bitmask<<(pup*8));
                  XorByteMask = bitmask;
                  //printf("\n\tXorByteMask = 0x%16llx\n",XorByteMask);
					XorByteMask = ~XorByteMask;
				  printf("\tXorByteMask = 0x%16llx\n",XorByteMask);
                  ddr4_cpu_sweep_test(0, repeat, testdepth ,0x1,pup);
                    
			}
            
            if(0) {
                TypeOfMask = AND_Mask ;
                for(pup = 0 ; pup <= 0 ; pup++ ) {
                    for(int bit = 0 ; bit <= 7 ; bit++ ) {
                        printf("\n run time: ");Counter(1);
                        unsigned long long bitmask= (0x1<<bit);
                        XorByteMask = ((unsigned long long)0x0 )| (bitmask<<(pup*8));
                    //   printf("\n\tXorByteMask = 0x%16llx\n",XorByteMask);
                        //XorByteMask = ~XorByteMask;
                     // printf("\tXorByteMask = 0x%16llx\n",XorByteMask);
                        ddr4_cpu_sweep_test(0, repeat, testdepth ,0x1,pup);
                        Counter(2);
                    }
                }            
                break;
            }
        }
        case XOR_CS0_PartialWrite:
        {
            // o	Total Data to transfer - 1024 x  Byte Count
            // o	Byte Count - 4byte:4byte: 64byte/32byte(depends if it is a 32/64 bit DRAM)            
            u32 TotalData2Test = _128M;
            int ByteCount = TotalData2Test/64;
            int NumOfError = 0;
            enum CStest CSinvovle = CS0ToCS0;
            int CSindex = CSinvovle/4;
            effective_cs = 0;
            int MaxIteration = 1000;
            //int IndexTH = MaxIteration/8;
            int MuxReg = mmio_read_32(0xf00116d8);
            if(MuxReg == 0x3CC) {//Tunit
                ddr3_tip_if_write(0, ACCESS_TYPE_MULTICAST, PARAM_NOT_CARE, ODPG_DATA_CONTROL_REG,
					       CSindex << ODPG_DATA_CS_OFFS,ODPG_DATA_CS_MASK << ODPG_DATA_CS_OFFS);
            }

            for(ByteCount = 5 ; ByteCount < 128 ; ByteCount = ByteCount + 4) {
                if(ByteCount%64 == 0) {
                    ByteCount = ByteCount + 4;
                }
                TotalData2Test = ByteCount * 1024;
                for(int index = 1 ; index < MaxIteration ; index++ ) {
                        int ret=0;
                        if(index==1) {
                        printf("\n----------------------------------");
                        printf(" XOR_CS0_PartialWrite ByteCount = %d (128)",ByteCount);
                        printf("----------------------------------\n");
                        printlog = 1;

                        }
               
                        ret=0;
                        ret = XOT_Test_Wrapper(TotalData2Test,CSinvovle,(index%4),ByteCount,index);
                        NumOfError += ret;
                        if((index==1) ) {
                            printlog = 5;
                        }
                }
                
            }
             if(printlog == 1) {
                printf("RunValidation XOR_CS1_FullAddressAccess returned %d Errors\n",NumOfError);
            }
            return NumOfError;
            break;
        }
        case XOR_CS1_FullAddressAccess:
        {
            u32 TotalData2Test = _128M;
            int ByteCount = TotalData2Test/64;
            int pattern = 1;
            int NumOfError = 0;
            enum CStest CSinvovle = CS1ToCS1;
            int CSindex = CSinvovle/4;
            effective_cs = 1;
            int MaxIteration = (((Area_length-TotalData2Test)/TotalData2Test)-1);
            int IndexTH = MaxIteration/4;
            int MuxReg = mmio_read_32(0xf00116d8);
            if(MuxReg == 0x3CC) {//Tunit
                ddr3_tip_if_write(0, ACCESS_TYPE_MULTICAST, PARAM_NOT_CARE, ODPG_DATA_CONTROL_REG,
					       CSindex << ODPG_DATA_CS_OFFS,ODPG_DATA_CS_MASK << ODPG_DATA_CS_OFFS);
            }


            for(int index = 1 ; index < MaxIteration ; index++ ) {
                int ret=0;
                if(index%IndexTH==0) {
                    printf("\n----------------------------------");
                    printf(" XOR_CS1_FullAddressAccess index = %d (%d)",index,MaxIteration);
                    printf("----------------------------------\n");
                    printlog = 1;

                }
                for(ByteCount = TotalData2Test/64; ByteCount < (TotalData2Test/4) ; ByteCount = ByteCount*2) {
                    ret=0;
                    ret = XOT_Test_Wrapper(TotalData2Test,CSinvovle,pattern,ByteCount,index);
                    NumOfError += ret;
                    if((index==1) || (index%IndexTH==0)) {
                        printlog = 5;
                    }
                }
                
            }
             if(printlog == 1) {
                printf("RunValidation XOR_CS1_FullAddressAccess returned %d Errors\n",NumOfError);
            }
            return NumOfError;
            break;
        }
        case XOR_CrossCs_FullAddressAccess:
        {
            // cross cs
            // we transaction a 64M from start to the end and back. Start + 64M x i ==> End - 64M x i. we stop when (Start + 64M x i) get to the end of the memory
            //int MuxReg = mmio_read_32(0xf00116d8);
            
            int TotalData2Test = 0x40;
            int ByteCount = 64;
            int VecTotalData2Test[3] = {_128K,_1M,_128M};
            int VecByteCount[3] = {0x40,0x100,_1M};
            int NumOfError = 0;
            enum CStest CSinvovle = CS0ToCS1;
            for(int testType = 0 ; testType < 3;testType++) {
                TotalData2Test = VecTotalData2Test[testType];
                ByteCount = VecByteCount[testType];
                int MaxIteration = (((Area_length- (unsigned long long)TotalData2Test)/(unsigned long long)TotalData2Test)-1);
                if(testType <2) {
                    MaxIteration = _1K;
                }
                int IndexTH = MaxIteration/4;
                /*if(testType == 2) {
                    IndexTH = 1;
                }*/
                 
                for(int index = 1 ; index < MaxIteration; index++ ) {
                    
                    int ret=0;
                    if(index%IndexTH==1) {
                        printf("\n----------------------------------");
                        printf(" XOR_CrossCs_FullAddressAccess index = %d (%d)",index,MaxIteration);
                        printf("----------------------------------\n");
                        printlog = 1;

                    }
                    ret=0;
                    ret = XOT_Test_Wrapper(TotalData2Test,CSinvovle ,(index%4),ByteCount,index);
                    NumOfError = NumOfError + ret;
                    if((index==1) || (index%IndexTH==1)) {
                        printlog = 5;
                    }
                    
                }
                if(printlog == 1) {
                    printf("RunValidation Cross-Cs FullAddressAccess have %d Errors\n",NumOfError);
                }
            }
            return NumOfError;
            break;
        }
    case XOR_CS0_general_1M_AVS_Sweep:
        {
            for(int avs = 0x2F ; avs > 0xF ; avs = avs -1) {
                reg_write(0x6f8130, (0x10000005|(avs<<4)|(avs<<12)));
                printf("Changing AVS value to be 0x%x\n",avs);
                mdelay(10);
                RunValidation(XOR_CS0_general_multiaccessSize);
            }
            break;
        }
	case CPU_RESET:
		{

#define MVEBU_REGS_BASE			0xF0000000
#define MVEBU_RFU_BASE			(MVEBU_REGS_BASE + 0x6F0000)
#define RFU_GLOBAL_SW_RST		MVEBU_RFU_BASE + 0x84
#define RFU_SW_RESET_OFFSET		0
		u32 reg;
		reg = readl(RFU_GLOBAL_SW_RST);
		reg &= ~(1 << RFU_SW_RESET_OFFSET);
		printf("CPU_Reset...\n");
		writel(reg, RFU_GLOBAL_SW_RST);
		break;
		}
    case XOR_CS1_PartialWrite:
        {
            u32 TotalData2Test = _128M;
            int ByteCount = TotalData2Test/64;
            int NumOfError = 0;
            enum CStest CSinvovle = CS1ToCS1;
            effective_cs = 1;
            int MaxIteration = 1000;
            //int IndexTH = MaxIteration/8;
            int MuxReg = mmio_read_32(0xf00116d8);
            if(MuxReg == 0x3CC) {//Tunit
                ddr3_tip_if_write(0, ACCESS_TYPE_MULTICAST, PARAM_NOT_CARE, ODPG_DATA_CONTROL_REG,
					       effective_cs << ODPG_DATA_CS_OFFS,ODPG_DATA_CS_MASK << ODPG_DATA_CS_OFFS);
            }
            for(int index = 25 ; index < MaxIteration ; index++ ) {
                int ret=0;
                if(index==1) {
                    printf("\n----------------------------------");
                    printf(" XOR_CS1_PartialWrite index = %d (%d)",index,MaxIteration);
                    printf("----------------------------------\n");
                    printlog = 4;

                }
                for(ByteCount = 4 ; ByteCount < 128 ; ByteCount = ByteCount + 4) {
                    TotalData2Test = ByteCount * 1024;
                    ret=0;
                    ret = XOT_Test_Wrapper(TotalData2Test,CSinvovle,(index%4),ByteCount,index);
                    NumOfError += ret;
                    if((index==1) ) {
                        printlog = 5;
                    }
                }
                
            }
             if(printlog == 1) {
                printf("RunValidation XOR_CS1_FullAddressAccess returned %d Errors\n",NumOfError);
            }
            return NumOfError;
            break;
        }
    case XOR_In_PageTest:
        {
            int NumOfError = 0;
            NumOfError = InnerPagetransactions();
             if(printlog == 1) {
                    printf("RunValidation XOR inside the Page test returned %d Errors\n",NumOfError);
             }
        
         return NumOfError;
         break;
        }
     case XOR_Page2PageTest:
        {
            int NumOfError = 0;
            //printlog = 1;
            NumOfError = Page2Pagetransactions();
             if(printlog == 1) {
                    printf("RunValidation XOR Page to Page test returned %d Errors\n",NumOfError);
             }
             printlog = 4;
         return NumOfError;
         break;
        }
     case XOR_In_BankTest:
        {
            int NumOfError = 0;
            //printlog = 1;
            NumOfError = InnerBanktransactions();
             if(printlog == 1) {
                    printf("RunValidation XOR inside the bank test returned %d Errors\n",NumOfError);
             }
             printlog = 4;
         return NumOfError;
         break;
        }
        
    case VP_full:
        {
            int Page_NumOfError = 0,PartialWrite_CS0_NumOfError = 0, CrossCs_NumOfError = 0,PartialWrite_CS1_NumOfError=0;
            int Page2Page_NumberOfError = 0,Bank_NumberOfError = 0;
            printf("#########################################################\n");
            printf("#########################################################\n");
            printf("Starting DDR3/4 - Validation Platfrom tests\n");
            mdelay(10);
            Page_NumOfError = RunValidation(XOR_In_PageTest);
            Page2Page_NumberOfError = RunValidation(XOR_Page2PageTest);
            Bank_NumberOfError = RunValidation(XOR_In_BankTest);
            PartialWrite_CS0_NumOfError = RunValidation(XOR_CS0_PartialWrite);
           //PartialWrite_CS1_NumOfError = RunValidation(XOR_CS1_PartialWrite);
           // CrossCs_NumOfError = RunValidation(XOR_CrossCs_FullAddressAccess);

            printf("#########################################################\n");
            printf("#########################################################\n");
            printf("DDR Validation Platform summary:\n");
            printf("\tInner Page test\t\t=\t%d\tError\n",Page_NumOfError);
            printf("\tPage2Page test\t\t=\t%d\tError\n",Page2Page_NumberOfError);
            printf("\tInner Bank test\t\t=\t%d\tError\n",Bank_NumberOfError);
            printf("\tPartial Write test@CS0\t=\t%d\tError\n",PartialWrite_CS0_NumOfError);
            printf("\tPartial Write test@CS1\t=\t%d\tError\n",PartialWrite_CS1_NumOfError);
            printf("\tCrros CS test\t\t=\t%d\tError\n",CrossCs_NumOfError);
            printf("#########################################################\n");
            printf("#########################################################\n");
            break;
        }
    case Funcional_WL:
        {
            int index;
            uint64_t data64;
            uintptr_t addr64 = 0;
            uint64_t readPattern_64[8] = {0};
            uint64_t patternTestPatternTable_64[8] = {
    		0xFFFFFFFFFFFFFFFF,
    		0xFFFFFFFFFFFFFFFF,
    		0x0000000000000000,
    		0x0000000000000000,
    		0x0000000000000000,
    		0x0000000000000000,
    		0xFFFFFFFFFFFFFFFF,
    		0xFFFFFFFFFFFFFFFF};

            for (index = 0; index < 8; index++) {
			data64 = patternTestPatternTable_64[index];
			mmio_write_64(addr64, data64);
			addr64 +=  sizeof(uint64_t);
            }
            for (index = 0; index < 8; index++) {
            data64 = mmio_read_64(addr64);
			addr64 +=  sizeof(uint64_t);
			readPattern_64[index] = data64;
            printf("0x%16jx\t",readPattern_64[index]);
		}
        }
    }//end of Switch

    return ret;
}


int bit_flip()
{
    unsigned long long* addr1 = (unsigned long long *) 0x8000000;
    unsigned long long* addr2 = (unsigned long long *) 0xa000000;
    unsigned int j, k;
    unsigned long long q;
	int cnt = 0x10000;
	int res = 0;

    int i;

    for (k = 0; k < 64; k++) {
        q = (unsigned long long)0x1 << k;
        for (j = 0; j < 8; j++) {
            q = ~q;

            for (i = 0; i < cnt; i++) {
                *(addr1+i)= *(addr2+i)= (i % 2) == 0 ? q : ~q;
            }
//			printf("k: %d j: %d adddress1: 0x%p data1: 0x%llx address2: 0x%p data2: 0x%llx\n", k,j,(unsigned long long*)addr1,(unsigned long long)*addr1,(unsigned long long*)addr2,(unsigned long long)*addr2 );
			for (i = 0; i < cnt; i++) {
				if (*(addr1+i) != *(addr2+i)){
					//printf("k: %d j: %d adddress1: 0x%p data1: 0x%llx address2: 0x%p data2: 0x%llx\n", k,j,(unsigned long long*)addr1,(unsigned long long)*addr1,(unsigned long long*)addr2,(unsigned long long)*addr2 );
					return (res + 100);
				}
			}            
        }
    }
    return res;
}

#if 1
//---------------------------------------------------------------------
void XOR_S2D_funcV(int NOD, int ByteCount, int Source_AddressL, int Source_AddressH, int Destanation_AddressL, 
                  int Destanation_AddressH, int Destnation_Jap, int WrBurstLen, int RdBurstLen)
{

   u32 TransferDesc[8];
   int indexNOD = 0;
   //u32 BCDesc[8];
   uint64_t GlobalRegBase = 0xf0000000;
   uint64_t GlobalRegBaseV = 0xf0000000;

   int XOREng = 4,IXOREng=0;
   uint64_t data = 0x0;
   uintptr_t addr = 0x0;

    // 1.Validate inputs.
    if(ByteCount > 0x1000) {
        ByteCount = ByteCount | 0x1000;// a know bug, need to enable bit 12.
    }
    
    // the S&D need to be inside the 512M
    Source_AddressH = 0x0;
    Destanation_AddressH = 0x0;
    Source_AddressL = Source_AddressL%_512M;
    Destanation_AddressL = Destanation_AddressL%_512M;

    WrBurstLen = WrBurstLen & 0x7;
    RdBurstLen = RdBurstLen & 0x7;
    if((NOD*0x40)> Source_AddressL) {
        NOD = (int)(Source_AddressL/0x40) - 1;
    }
    if(NOD> 0x7FFF) {
        NOD = 0x7FFF;
    }
    //------------------------------------------
    //printf("XOR_S2D_func:: 0x%08x --> (0x%08x + NOD x 0x%x ) .. bytecount 0x%x - NOD = %d\n"
            //,Source_AddressL,Destanation_AddressL,Destnation_Jap,ByteCount,NOD);
    // write all descriptions thru the CPU to the Memory
    for(IXOREng = 0; IXOREng < XOREng ; IXOREng++) {
       for(indexNOD = 0 ; indexNOD < NOD;indexNOD++ ) {
           //initilaize the descreptor array
           for (int i =0; i < 8; i++)
           {
              TransferDesc[i] = 0;
           }
           // configure the descreptor array
           TransferDesc[XOR_DESC_COMMAND_LINE] = Pure_DMA<<28;
           TransferDesc[XOR_DESC_BYTE_COUNTER_LINE] = ByteCount; //byte count
           TransferDesc[XOR_DESC_SRC_BUF_ADDR_LOW_LINE] = Source_AddressL + _512M*IXOREng; // source
           TransferDesc[XOR_DESC_SRC_BUF_ADDR_HIGH_LINE] = (Source_AddressH & 0x00000FFFF);
           TransferDesc[XOR_DESC_DEST_BUFF_ADDR_LOW_LINE] = (Destanation_AddressL + _512M*IXOREng + indexNOD*Destnation_Jap); // dest.
           TransferDesc[XOR_DESC_DEST_BUFF_ADDR_HIGH_LINE] = (Destanation_AddressH & 0xFFFF); // dest.

           // write desrciptor into the Memory
           addr = _512M*IXOREng + (uintptr_t)(0x20*indexNOD);
           for (int i =0; i < 4; i++)
           {
              
              data = (uint64_t)(TransferDesc[2*i]) | (uint64_t)((uint64_t)(TransferDesc[2*i+1])<<32);
              mmio_write_64(addr,data);
              addr += 8;
           }

           
       }
       //---------------------------------------------------------------
       GlobalRegBaseV = GlobalRegBase + 0x20000*IXOREng;
       //Q definition
       mmio_write_32(CV_XOR_DESQ_BASE_ADDR_LOW_REG(GlobalRegBaseV), _512M*IXOREng);
       mmio_write_32(CV_XOR_DESQ_BASE_ADDR_HIGH_REG(GlobalRegBaseV), 0);
       mmio_write_32(CV_XOR_DESQ_SIZE_REG(GlobalRegBaseV), NOD);
       mmio_write_32(CV_XOR_DESQ_AR_ATTRIBUTES_REG(GlobalRegBaseV), 0x3e3e);
       mmio_write_32(CV_XOR_DESQ_AW_ATTRIBUTES_REG(GlobalRegBaseV), 0x3e3e);
       mmio_write_32(CV_XOR_IMSG_THRESHOLD_REG(GlobalRegBaseV), 0);
       mmio_write_32(CV_XOR_DESQ_CTRL_REG(GlobalRegBaseV), 0);//0 - 32Byte
       mmio_write_32(CV_XOR_DESQ_STOP_REG(GlobalRegBaseV), 0);
       mmio_write_32(CV_XOR_BW_CTRL_REG(GlobalRegBaseV), (RdBurstLen<<CV_XOR_MUX_READ_BURST_OFFSET | WrBurstLen<<CV_XOR_MUX_WRITE_BURST_OFFSET));
    }
   //------------------------------------------------------------------
   // add descriptor to the XOR - "Start"
    for(IXOREng = 0; IXOREng < XOREng ; IXOREng++) {
        GlobalRegBaseV = GlobalRegBase + 0x20000*IXOREng;
        //printf("\nXOR_S2D_funcV : Start XOR Eng %d address = 0x%16jx\n", IXOREng,GlobalRegBaseV);
        mmio_write_32(CV_XOR_DESQ_ADD_REG(GlobalRegBaseV),NOD);
    }
    for(IXOREng = 0; IXOREng < XOREng ; IXOREng++) {
        GlobalRegBaseV = GlobalRegBase + 0x20000*IXOREng;
       int DESQ_DONE_index = 0;
       u32 ReadReg = 0;
       while(DESQ_DONE_index < NOD) {
           ReadReg = mmio_read_32(CV_XOR_DESQ_DONE_REG(GlobalRegBaseV));
           DESQ_DONE_index = (CV_XOR_DESQ_DONE_MASK & ReadReg)>>CV_XOR_DESQ_DONE_OFFSET;
           //printf("XOR_S2D_funcV : XOR Eng %d - DESQ_DONE_index %d(%d)\n",IXOREng,DESQ_DONE_index,NOD);
       }
       // send to the DMA massage that we read des.
       mmio_write_32(CV_XOR_DESQ_DEALLOC_REG(GlobalRegBaseV), NOD); 
    }

}
//-------------------------------------------------------------------------------------------------------
int XOR_ScmpD_funcV(int NOD, int ByteCount, int Source_AddressL, int Source_AddressH, int Destanation_AddressL, 
                  int Destanation_AddressH, int Destnation_Jap, int RdBurstLen)
{
    // each Q will be place at Q[IXOREng] = 0x0 + 512M*IXOREng
    //XOREng 0:: 0      - 512M
    //XOREng 0:: 512M   - 1G
    //XOREng 0:: 1G     - 1.5G
    //XOREng 0:: 1.5G   - 2G
   u32 TransferDesc[8];
   u32 ReadReg = 0,DESQ_SW_RdPtr = 0xFF/*,DESQ_SW_RdPtr_Last = 0xFF*/;
   int indexNOD = 0;
   int BCSIndex = 0;
   int printEn = 0;
   int XOREng = 4,IXOREng=0;
   //u32 BCDesc[8];
   uint64_t GlobalRegBase = 0xf0000000;
   uint64_t GlobalRegBaseV = 0xf0000000;
    
    uint64_t data = 0x0;
    uintptr_t addr = 0x0;

    // 1.Validate inputs.
    if(ByteCount > 0x1000) {
        ByteCount = ByteCount | 0x1000;// a know bug, need to enable bit 12.
    }
    // the S&D need to be inside the 512M
    Source_AddressH = 0x0;
    Destanation_AddressH = 0x0;
    Source_AddressL = Source_AddressL%_512M;
    Destanation_AddressL = Destanation_AddressL%_512M;
    
    RdBurstLen = RdBurstLen & 0x7;
    if((NOD*0x40)> Source_AddressL) {
        NOD = (int)(Source_AddressL/0x40) - 1;
    }
    if(NOD> 0x7FFF) {
        NOD = 0x7FFF;
    }
    //------------------------------------------
    //printf("XOR_ScmpD_funcV:: 0x%08x --> (0x%08x + NOD x 0x%x ) .. bytecount 0x%x - NOD = %d\n"
//            ,Source_AddressL,Destanation_AddressL,Destnation_Jap,ByteCount,NOD);
    // write all descriptions thru the CPU to the Memory
   for(IXOREng = 0; IXOREng < XOREng ; IXOREng++) {
       for(indexNOD = 0 ; indexNOD < NOD;indexNOD++ ) {
           //initilaize the descreptor array
           for (int i =0; i < 8; i++)
           {
              TransferDesc[i] = 0;
           }
           // configure the descreptor array
           TransferDesc[XOR_DESC_COMMAND_LINE] = ((Byte_Compare<<XOR_DESC_OPERATION_MODE_OFFSET) | (0x3 << 19));;
           TransferDesc[XOR_DESC_BYTE_COUNTER_LINE] = ByteCount; //byte count
           TransferDesc[XOR_DESC_SRC_BUF_ADDR_LOW_LINE] = Source_AddressL + _512M*IXOREng; // source
           TransferDesc[XOR_DESC_SRC_BUF_ADDR_HIGH_LINE] = (Source_AddressH & 0x00000FFFF);
           TransferDesc[XOR_DESC_DEST_BUFF_ADDR_LOW_LINE] = (Destanation_AddressL + _512M*IXOREng + indexNOD*Destnation_Jap); // dest.
           TransferDesc[XOR_DESC_DEST_BUFF_ADDR_HIGH_LINE] = (Destanation_AddressH & 0xFFFF); // dest.

           // write desrciptor into the Memory
           addr = _512M*IXOREng + (uintptr_t)(0x20*indexNOD);
           for (int i =0; i < 4; i++)
           {
              
              data = (uint64_t)(TransferDesc[2*i]) | (uint64_t)((uint64_t)(TransferDesc[2*i+1])<<32);
              mmio_write_64(addr,data);
              addr += 8;
           }

           
       }
       //---------------------------------------------------------------
   
       GlobalRegBaseV = GlobalRegBase + 0x20000*IXOREng;
       //Q definition
       mmio_write_32(CV_XOR_DESQ_BASE_ADDR_LOW_REG(GlobalRegBaseV), _512M*IXOREng);
       mmio_write_32(CV_XOR_DESQ_BASE_ADDR_HIGH_REG(GlobalRegBaseV), 0);
       mmio_write_32(CV_XOR_DESQ_SIZE_REG(GlobalRegBaseV), NOD);
       mmio_write_32(CV_XOR_DESQ_AR_ATTRIBUTES_REG(GlobalRegBaseV), 0x3e3e);
       mmio_write_32(CV_XOR_DESQ_AW_ATTRIBUTES_REG(GlobalRegBaseV), 0x3e3e);
       mmio_write_32(CV_XOR_IMSG_THRESHOLD_REG(GlobalRegBaseV), 0);
       mmio_write_32(CV_XOR_DESQ_CTRL_REG(GlobalRegBaseV), 0);//0 - 32Byte
       mmio_write_32(CV_XOR_DESQ_STOP_REG(GlobalRegBaseV), 0);
       mmio_write_32(CV_XOR_BW_CTRL_REG(GlobalRegBaseV), (RdBurstLen<<CV_XOR_MUX_READ_BURST_OFFSET ));
   }
   //------------------------------------------------------------------
   // add descriptor to the XOR - "Start"
   for(IXOREng = 0; IXOREng < XOREng ; IXOREng++) {
       GlobalRegBaseV = GlobalRegBase + 0x20000*IXOREng;
       mmio_write_32(CV_XOR_DESQ_ADD_REG(GlobalRegBaseV),NOD);
   }

   // polling result from the descreptor when it is ready.
   //----------------------------------------------------
   //poll on SW pointer untill the descriptor is ready 
   for(IXOREng = 0; IXOREng < XOREng ; IXOREng++) {
       GlobalRegBaseV = GlobalRegBase + 0x20000*IXOREng;
       indexNOD = 0;
       int DESQ_DONE_index=0;
       while(indexNOD < NOD) {
           ReadReg = mmio_read_32(CV_XOR_DESQ_DONE_REG(GlobalRegBaseV));
           DESQ_DONE_index = (CV_XOR_DESQ_DONE_MASK & ReadReg)>>CV_XOR_DESQ_DONE_OFFSET;
           //while(DESQ_SW_RdPtr == DESQ_SW_RdPtr_Last) {
           if(DESQ_DONE_index>0) {
                   DESQ_SW_RdPtr = (ReadReg & CV_XOR_DESQ_SW_READ_PTR_MASK) >> CV_XOR_DESQ_SW_READ_PTR_OFFSET;
                   // send to the DMA massage that we read des.
                   mmio_write_32(CV_XOR_DESQ_DEALLOC_REG(GlobalRegBaseV), 1); 
                   //DESQ_SW_RdPtr_Last = DESQ_SW_RdPtr;
                   data = mmio_read_64((uintptr_t)((uintptr_t)(0x20*indexNOD + (uintptr_t)(_512M*IXOREng)) + (uintptr_t)XOR_DESC_STATUS_LINE));
                   addr =  ((uintptr_t)((uintptr_t)(0x20*indexNOD + (uintptr_t)(_512M*IXOREng)) + (uintptr_t)XOR_DESC_STATUS_LINE));
                   //printf("XOR_ScmpD_func:: descriptor data 0x%16jx --- CV_XOR_DESQ_DONE_REG- 0x%08x",data,ReadReg );
                   data =( data >> XOR_DESC_BCS_OFFSET) & 0x1;
                  // printf("XOR_ScmpD_funcV:: IXOREng %d , 0x%16jx  Descriptor %d(DONE - %d), offset 0x%08x , BCS %s\n",IXOREng,addr,indexNOD,DESQ_DONE_index,DESQ_SW_RdPtr,((data == 1)?"OK":"FAIL"));
           
               if((data == 0) & (indexNOD>0) ) {//fail
                   BCSIndex++;
                   if(printEn == 1) printf("XOR_ScmpD_func:: Descriptor %d(DONE - %d), offset 0x%08x , BCS %s\n",indexNOD,DESQ_DONE_index,DESQ_SW_RdPtr,((data == 1)?"OK":"FAIL"));
               }
               indexNOD++;
           }
       }
   }


   return BCSIndex;
}
//-------------------------------------------------------------------------------------------------------

int XOR_TestV(int TotalData2Test,int Source_AddressL, int Source_AddressH, int Destanation_AddressL, 
                  int Destanation_AddressH, int ByteCount,int PatternType, int nNumOstdRd)
{
    int Destnation_Jap = ByteCount*2 + 8; // The 8 is not to be cas align, for 32bit DRAM nee to be 4.
    int WrBurstLen = 7;
    int RdBurstLen = 7;
    int NOD,pattern_length_cnt,data32 = 0;
    int XOREng = 4,IXOREng=0;
    int res = 0;
	uint64_t data = 0x0,data2 = 0x0;
    uintptr_t addr = 0x0;
    NOD = TotalData2Test/ByteCount;
    struct pattern_info *pattern_table = ddr3_tip_get_pattern_table();
    //NOD = NOD - 1;
   // printf("XOR_Test:: 0x%08x --> (0x%08x + NOD x 0x%x ) .. bytecount 0x%x - NOD = %d\n"
            //,Source_AddressL,Destanation_AddressL,Destnation_Jap,ByteCount,NOD);
    // the S&D need to be inside the 512M
    Source_AddressH = 0x0;
    Destanation_AddressH = 0x0;
    Source_AddressL = Source_AddressL%_512M;
    Destanation_AddressL = Destanation_AddressL%_512M;

    for(IXOREng = 0; IXOREng < XOREng ; IXOREng++) {
        addr = Source_AddressL + _512M*IXOREng;
        int NumOfLoop = ByteCount/(256*2*8);//The ODPG is 256
        NumOfLoop = (NumOfLoop == 0)?1:NumOfLoop;
		if(0)
		{
			for(int NumOfLoopI = 0 ; NumOfLoopI<NumOfLoop; NumOfLoopI++) {
				if(PatternType == 0) {// 0 - is killer
					for(int pattern = PATTERN_KILLER_DQ0; pattern <= PATTERN_KILLER_DQ7;pattern++) {
						for (pattern_length_cnt = 0;pattern_length_cnt < pattern_table[pattern].pattern_len;
							  pattern_length_cnt++) {
							//64 bit support only
							data32 = pattern_table_get_word(0, pattern,(u8) (pattern_length_cnt));
							data = (uint64_t)(data32) & 0x00000000FFFFFFFF;
							data = data | (uint64_t)((uint64_t)(data32)<<32);
							mmio_write_64(addr,data);
							addr += 8;
						}//end pattern_length_cnt
					}//end int pattern
				}
				if(PatternType == 1) {// 1 - is killer inv
					for(int pattern = PATTERN_KILLER_DQ0_INV; pattern <= PATTERN_KILLER_DQ7_INV;pattern++) {
						for (pattern_length_cnt = 0;pattern_length_cnt < pattern_table[pattern].pattern_len;
							  pattern_length_cnt++) {
							//64 bit support only
							data = 0;
							data32 = pattern_table_get_word(0, pattern,(u8) (pattern_length_cnt));
							data = (uint64_t)(data32) & 0x00000000FFFFFFFF;
							data = data | (uint64_t)((uint64_t)(data32)<<32);
							//printf("\nXOR TEst : address = 0x%16jx, data32 = 0x%16jx, data32<<32 = 0x%16jx --> data = 0x%16jx\n",addr,(uint64_t)(data32),
							//       (uint64_t)((uint64_t)(data32)<<32),data);
							mmio_write_64(addr,data);
							addr += 8;
						}//end pattern_length_cnt
					}
				}
				if(PatternType == 2) {// 2 - resonance
					NumOfLoop = ByteCount/(256*8*8);//The ODPG is 256
					for(int pattern = PATTERN_RESONANCE_2T; pattern <= PATTERN_RESONANCE_9T;pattern++) {
						for(int index = 0 ; index < 8 ; index++) {
							for (pattern_length_cnt = 0;pattern_length_cnt < pattern_table[pattern].pattern_len;
								  pattern_length_cnt++) {
								//64 bit support only
								data32 = pattern_table_get_word(0, pattern,(u8) (pattern_length_cnt));
								data = (uint64_t)(data32) & 0x00000000FFFFFFFF;
								data = data | (uint64_t)((uint64_t)(data32)<<32);
								mmio_write_64(addr,data);
								addr += 8;
							}//end pattern_length_cnt
						}
					}
				}

			}
		}
		else{
            int NumOfByteInDevice = (DataWidth == 0x3)?(4):(8);
            NumOfLoop = (DataWidth == 0x3)?(ByteCount/4):(ByteCount/8);
            for(int NumOfLoopI = 0 ; NumOfLoopI<NumOfLoop; NumOfLoopI++) {
                if(PatternType == Moving_one_croosand_zero) {
                    if(NumOfLoopI == 0) {
                        data = 127;
                    }else{
                        data =data2&0xFF;
                        
                    }
                    if((NumOfLoopI%3 == 0)||(NumOfLoopI%3 == 1)) {
                        data = 255 - data;//inverse
                    }else{
                        data = (data==0xFE)?127:(128+ (data/2));
                    }
                         
                 }
                if(PatternType == Moving_zero_croosand_one) {
                    if(NumOfLoopI == 0) {
                        data = 128;
                    }else{
                        data =data2&0xFF;
                        
                    }
                    if((NumOfLoopI%3 == 0)||(NumOfLoopI%3 == 1)) {
                        data = 255 - data;
                    }else{
                        data = (data==0x1)?128:data/2;
                    }
                         
                 }
                if(PatternType == 5) {
                    data = pattern_table_get_word(0, PATTERN_KILLER_DQ0,
                                                  (u8) (NumOfLoopI%pattern_table[PATTERN_KILLER_DQ0].pattern_len));
                 }
                if(PatternType == 6) {
                    data = pattern_table_get_word(0, PATTERN_VREF,
                                                  (u8) (NumOfLoopI%pattern_table[PATTERN_KILLER_DQ0].pattern_len));
                 }
                if(PatternType == 7) {
                    data = pattern_table_get_word(0, PATTERN_RESONANCE_9T,
                                                  (u8) (NumOfLoopI%pattern_table[PATTERN_KILLER_DQ0].pattern_len));
                 }
                if(PatternType == 8) {
                    if(NumOfLoopI%2 == 0) {
                        data = 0x0;
                    }else{
                        data =0xFFFFFFFFFFFFFFFF;
                        
                    }
                                            
                 }
                data2 = 0x0;
                int shift = 0;
                for(int byteindex = 0; byteindex < NumOfByteInDevice; byteindex++){
                    shift = shift + 8*byteindex;
                    data2 |= ((data%256) << (shift));
                 }
                
                if(TypeOfMask == OR_Mask) {
                    cpu_write(addr,(data2|XorByteMask));
                }else{
                    cpu_write(addr,(data2 & XorByteMask));
                }
               // printf("cpu_write:\t\t0x%16lx\t0x%16lx\n",addr,data2);
                addr = addr + NumOfByteInDevice;
            }
        }
    }
    //mmio_write_32(0xf00116d8, 0x38c);
    XOR_S2D_funcV(NOD, ByteCount, Source_AddressL, Source_AddressH, Destanation_AddressL, 
                  Destanation_AddressH, Destnation_Jap, WrBurstLen, RdBurstLen);
    res = XOR_ScmpD_funcV(NOD, ByteCount, Source_AddressL, Source_AddressH, Destanation_AddressL, 
                  Destanation_AddressH, Destnation_Jap, RdBurstLen);
    return res;
}

#endif

int mv_ddr_validate(void) {

	RunValidation(Eye_sweepRx_CS0);

	printf("\n");

	return 0;
}
