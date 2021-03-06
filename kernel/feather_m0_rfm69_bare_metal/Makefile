#
#   This file is part of IcedCoffeeOS
#   (https://github.com/rromanotero/IcedCoffeeOS).
#
#   Copyright (c) 2020 Rafael Roman Otero.
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <https://www.gnu.org/licenses/>.
#

AS=arm-none-eabi-gcc
CC=arm-none-eabi-gcc
LD=arm-none-eabi-ld
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
ELF=kernel.elf
HEX=kernel.hex
BIN=kernel.bin

TARGET_EXEC ?= kernel.hex
SRC_DIRS ?= ./src
BUILD_DIR ?= ./build
OUTPUT ?= ./output

SRCS := $(shell find $(SRC_DIRS) -name *.c -or -name *.S)
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)

INC_DIRS := $(shell find $(SRC_DIRS) -type d)
INC_FLAGS := $(addprefix -I,$(INC_DIRS))

CFLAGS = $(INC_FLAGS) -D__SAMD21G18A__ -mthumb -mcpu=cortex-m0plus -Os -ffreestanding
LDFLAGS = -L/gnu_toolchain/arm-none-eabi/lib -lm -L /gnu_toolchain/arm-none-eabi/lib -lc

$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS)
	mkdir -p ./build_temp
	cp -r ../common/* ./build_temp
	$(CC) $(OBJS) $(LDFLAGS) -T linker.ld -o $(OUTPUT)/$(ELF) $(CFLAGS)
	$(OBJCOPY) -O ihex -R .eeprom -R .fuse -R .lock -R .signature $(OUTPUT)/$(ELF) $(OUTPUT)/$(HEX)
	$(OBJCOPY) -O binary $(OUTPUT)/$(ELF) $(OUTPUT)/$(BIN)
	$(OBJDUMP) -D $(OUTPUT)/$(ELF) > $(OUTPUT)/kernel.lss
	$(OBJDUMP) -s $(OUTPUT)/$(ELF) > $(OUTPUT)/kernel.dump

# assembly
$(BUILD_DIR)/%.S.o: %.S
	$(MKDIR_P) $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# c source
$(BUILD_DIR)/%.c.o: %.c
	$(MKDIR_P) $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean

clean:
	$(RM) -r $(BUILD_DIR)


MKDIR_P ?= mkdir -p
