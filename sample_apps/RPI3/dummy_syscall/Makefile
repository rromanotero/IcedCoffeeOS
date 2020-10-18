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

AS=aarch64-elf-gcc
CC=aarch64-elf-gcc
LD=aarch64-elf-ld
OBJCOPY=aarch64-elf-objcopy
OBJDUMP=aarch64-elf-objdump

SRC_DIRS ?= src
BUILD_DIR ?= build
OUTPUT ?= output
ELF=app.elf
BINARY=app.bin

SRCS := $(shell find $(SRC_DIRS) -name *.c -or -name *.S)
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)

INC_DIRS := $(shell find $(SRC_DIRS) -type d)
INC_FLAGS := $(addprefix -I,$(INC_DIRS))

CFLAGS = $(INC_FLAGS) -mcpu=cortex-a53 -O0 -nostdlib -ffreestanding -fpic -fpie

$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS)
	$(LD) $(OBJS) -T linker.ld -o $(OUTPUT)/$(ELF)
	$(OBJCOPY) -O binary $(OUTPUT)/$(ELF) $(OUTPUT)/$(BINARY)
	$(OBJDUMP) -D $(OUTPUT)/$(ELF) > $(OUTPUT)/app.lss
	$(OBJDUMP) -s $(OUTPUT)/$(ELF) > $(OUTPUT)/app.dump

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
