##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
## Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
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

LDSCRIPT	:= $(ROOT)/makefiles/common/opencm3_common/STM32F103CBT6.ld
LIBNAME		= opencm3_stm32f1

FP_FLAGS	?= -msoft-float
DEVICE_FLAGS	= -mthumb -mcpu=cortex-m3 $(FP_FLAGS) -mfix-cortex-m3-ldrd -DSTM32F1

################################################################################
# OpenOCD specific variables

OOCD		?= openocd
OOCD_INTERFACE	?= flossjtag
OOCD_TARGET	?= stm32f1x

################################################################################
# Black Magic Probe specific variables
# Set the BMP_PORT to a serial port and then BMP is used for flashing
BMP_PORT	?=

################################################################################
# texane/stlink specific variables
#STLINK_PORT	?= :4242
