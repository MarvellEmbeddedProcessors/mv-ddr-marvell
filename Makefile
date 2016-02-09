#*******************************************************************************
# Copyright (C) 2016 Marvell International Ltd.
#
# This software file (the "File") is owned and distributed by Marvell
# International Ltd. and/or its affiliates ("Marvell") under the following
# alternative licensing terms.  Once you have made an election to distribute the
# File under one of the following license alternatives, please (i) delete this
# introductory statement regarding license alternatives, (ii) delete the three
# license alternatives that you have not elected to use and (iii) preserve the
# Marvell copyright notice above.
#
#********************************************************************************
# Marvell Commercial License Option
#
# If you received this File from Marvell and you have entered into a commercial
# license agreement (a "Commercial License") with Marvell, the File is licensed
# to you under the terms of the applicable Commercial License.
#
#********************************************************************************
# Marvell GPL License Option
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#********************************************************************************
# Marvell GNU General Public License FreeRTOS Exception
#
# If you received this File from Marvell, you may opt to use, redistribute and/or
# modify this File in accordance with the terms and conditions of the Lesser
# General Public License Version 2.1 plus the following FreeRTOS exception.
# An independent module is a module which is not derived from or based on
# FreeRTOS.
# Clause 1:
# Linking FreeRTOS statically or dynamically with other modules is making a
# combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
# General Public License cover the whole combination.
# As a special exception, the copyright holder of FreeRTOS gives you permission
# to link FreeRTOS with independent modules that communicate with FreeRTOS solely
# through the FreeRTOS API interface, regardless of the license terms of these
# independent modules, and to copy and distribute the resulting combined work
# under terms of your choice, provided that:
# 1. Every copy of the combined work is accompanied by a written statement that
# details to the recipient the version of FreeRTOS used and an offer by yourself
# to provide the FreeRTOS source code (including any modifications you may have
# made) should the recipient request it.
# 2. The combined work is not itself an RTOS, scheduler, kernel or related
# product.
# 3. The independent modules add significant and primary functionality to
# FreeRTOS and do not merely extend the existing functionality already present in
# FreeRTOS.
# Clause 2:
# FreeRTOS may not be used for any competitive or comparative purpose, including
# the publication of any form of run time or compile time metric, without the
# express permission of Real Time Engineers Ltd. (this is the norm within the
# industry and is intended to ensure information accuracy).
#
#********************************************************************************
# Marvell BSD License Option
#
# If you received this File from Marvell, you may opt to use, redistribute and/or
# modify this File under the following licensing terms.
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
#      * Redistributions of source code must retain the above copyright notice,
#        this list of conditions and the following disclaimer.
#
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in the
#        documentation and/or other materials provided with the distribution.
#
#      * Neither the name of Marvell nor the names of its contributors may be
#        used to endorse or promote products derived from this software without
#        specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#*******************************************************************************

ifdef CONFIG_SPL_BUILD
obj-$(CONFIG_SPL_BUILD) += ddr3_a38x.o
obj-$(CONFIG_SPL_BUILD) += ddr3_a38x_training.o
obj-$(CONFIG_SPL_BUILD) += ddr3_debug.o
obj-$(CONFIG_SPL_BUILD) += ddr3_hws_hw_training.o
obj-$(CONFIG_SPL_BUILD) += ddr3_init.o
obj-$(CONFIG_SPL_BUILD) += ddr3_training.o
obj-$(CONFIG_SPL_BUILD) += ddr3_training_bist.o
obj-$(CONFIG_SPL_BUILD) += ddr3_training_centralization.o
obj-$(CONFIG_SPL_BUILD) += ddr3_training_db.o
obj-$(CONFIG_SPL_BUILD) += ddr3_training_hw_algo.o
obj-$(CONFIG_SPL_BUILD) += ddr3_training_ip_engine.o
obj-$(CONFIG_SPL_BUILD) += ddr3_training_leveling.o
obj-$(CONFIG_SPL_BUILD) += ddr3_training_pbs.o
obj-$(CONFIG_SPL_BUILD) += ddr3_training_static.o
obj-$(CONFIG_SPL_BUILD) += xor.o
else # CONFIG_SPL_BUILD
include ../../base.mk

#TIP_INC = $(BH_ROOT_DIR)/src_ddr/ddr3libv2/h

#INCLUDE = -I$(TIP_INC)/Os -I$(TIP_INC)/Os/gtOs -I$(TIP_INC)/Os/common/siliconIf \
	  -I$(TIP_INC)/SoC -I$(TIP_INC)/Silicon -I$(TIP_INC)/Os/common/configElementDb \
	  -I$(TIP_INC)/Driver -I$(TIP_INC)/Driver/ddr3  -I$(BH_ROOT_DIR)/inc/common -I$(BH_ROOT_DIR)/inc/common/gtOs\
	  -I$(BH_ROOT_DIR)/inc/ddr3_soc/$(BOARD) -I$(BH_ROOT_DIR)/inc/ddr3_soc/$(INCNAME) \
	  -I$(BH_ROOT_DIR)/src_ddr  -I$(BH_ROOT_DIR)/platform/sysEnv/$(BOARD)

ifeq ($(DDRTYPE),ddr4)
	INCLUDE += -I$(BH_ROOT_DIR)/src_ddr/mv_ddr4/
endif

#DDRTYPE = ddr3
#LIBNAME = a38x
#CFLAGS += -DMV_DDR

#TGT = ddr_$(LIBNAME).a
#TGT_UART = ddr_$(LIBNAME).uart.a

TLIB = ./$(DDRTYPE)_training_$(LIBNAME).lib

TDDR4SUBLIB = ./$(DDRTYPE)_training_$(LIBNAME)sub.lib

#TSRC = $(wildcard ./src/Driver/ddr3/*.c)
#TSRC += ./src/Silicon/mvHwsDdr3$(SILNAME).c
#ifeq "$(BOARD)"  "msys"
#	TSRC += ./src/Silicon/mvHwsDdr3Msys.c
#endif
#TSRC += ./src/Soc/ddr3_$(BOARDNAME)_training.c
#TSRC += ./src/Soc/ddr3_hws_hw_training.c
#TSRC += ./src/Os/gtOs/mvXor.c


TSRCDDR4 = $(wildcard ../mv_ddr4/*.c)


TSRC = $(wildcard ./*.c)
TOBJ = $(TSRC:.c=.o)
TOBJDDR4 = $(TSRCDDR4:.c=.o)

ifeq ($(DDR4SUBLIB),yes)
	TARGETS = $(TLIB) $(TDDR4SUBLIB)
else
	TARGETS = $(TLIB)
endif

#TARGETS = $(TLIB)


#############global flags enable/disable features to save footprint###########
# exclude debug function not relevent for SoC( set by default)
#CFLAGS += -DEXCLUDE_SWITCH_DEBUG -DDDR_VIEWER_TOOL
#ifeq "$(CONFIG_BOBCAT2)"  "y"
#CFLAGS += -DMV_HWS_EXCLUDE_DEBUG_PRINTS
#endif
#remove all debug prints from training( unset by default)
#CFLAGS += -DSILENT_LIB
#CFLAGS += -DLIB_FUNCTIONAL_DEBUG_ONLY

#Flag to support static training algo functions( unset by default)
#ifeq ($(BOARD),a38x)
#CFLAGS += -DSTATIC_ALGO_SUPPORT
#else
#CFLAGS += -DSTATIC_ALGO_SUPPORT
#endif
#flag to support ODT test debug function( unset by default)
#CFLAGS += -DODT_TEST_SUPPORT

# Flag to enable RX IO BIST Test
# CFLAGS += -DMV_HWS_RX_IO_BIST
# Flag enable IO BIST of CMD/ADDR Test (in addition to MV_HWS_RX_IO_BIST)
# CFLAGS += -DMV_HWS_RX_IO_BIST_ETP

#############end of global flags #############################################


all:   $(TARGETS)

%.o: %.c
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

#%.uart.o: %.c
#	$(CC) $(CFLAGS) -DNOT_USE_UART -DMV_NO_INPUT -DMV_NO_PRINT  $(CPPFLAGS) -c -o  $@ $<

$(TDDR4SUBLIB): $(TOBJDDR4)
	$(RM) ./$@
	ar rcs $(TDDR4SUBLIB) $(TOBJDDR4)
	$(CP) ./$@ ../lib

$(TLIB): $(TOBJ)
	$(RM) ./$@
	ar rcs $(TLIB) $(TOBJ)
	$(CP) ./$@ ../lib

clean:
	$(RM) ./*.o  ./*.a
endif # CONFIG_SPL_BUILD
