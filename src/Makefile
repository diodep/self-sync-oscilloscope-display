##
## This file is part of anlogic-usbjtag project
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
## Copyright (C) 2017 Zhiyuan Wan <h@iloli.bid>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

BINARY = dac

OBJS = 

LDSCRIPT = stm32f103x8.ld

DFU_UTIL = dfu-util

LIBNAME		?= opencm3_gd32f150
DEFS		?= -DGD32F150 -Ofast
FP_FLAGS    ?= -msoft-float
ARCH_FLAGS	?= -mthumb -mcpu=cortex-m3 $(FP_FLAGS) -mfix-cortex-m3-ldrd

include ../libopencm3.rules.mk


size: $(OBJS) $(BINARY).elf
	@$(PREFIX)-size $(OBJS) $(BINARY).elf
	

swd:  $(OBJS) $(BINARY).bin
	st-flash write $(BINARY).bin 0x08000000
