# ===================================================================================
# Project:  CH32X035/X034/X033 Example
# Author:   Stefan Wagner
# Year:     2023
# URL:      https://github.com/wagiminator    
# ===================================================================================
# Type "make help" in the command line.
# ===================================================================================

# Files and Folders
MAINFILE = pd_pps_variable.c
TARGET   = pd_pps_variable
TARGET_V1V2 = pd_pps_variable_V1V2
INCLUDE  = include
TOOLS    = tools

# Microcontroller Settings
F_CPU    = 48000000
LDSCRIPT = linker/ch32x035.ld
CPUARCH  = -march=rv32imac -mabi=ilp32

# Toolchain
PREFIX   = riscv64-unknown-elf
CC       = $(PREFIX)-gcc
OBJCOPY  = $(PREFIX)-objcopy
OBJDUMP  = $(PREFIX)-objdump
OBJSIZE  = $(PREFIX)-size
NEWLIB   = /usr/include/newlib
CLEAN    = rm -f *.lst *.obj *.cof *.list *.map *.eep.hex *.o *.d

# Compiler Flags
CFLAGS   = -g -Os -flto -ffunction-sections -fno-builtin -nostdlib -Wall
CFLAGS  += $(CPUARCH) -DF_CPU=$(F_CPU) -I$(NEWLIB) -I$(INCLUDE) -I.
CFLAGS_V1V2 = $(CFLAGS) -DV1V2
LDFLAGS  = -T$(LDSCRIPT) -L$(INCLUDE)/libgcc -lgcc -Wl,--gc-sections
COMMON_CFILES = $(MAINFILE) $(wildcard $(INCLUDE)/*.c) $(wildcard $(INCLUDE)/*.S)
CFILES   = $(COMMON_CFILES) 7seg_button.c
CFILES_V1V2 = $(COMMON_CFILES) 7seg_button_V1V2.c

.DEFAULT_GOAL := all
.PHONY: all elf bin hex asm clean size removetemp removeelf help

# Symbolic Targets
help:
	@echo "Use the following commands:"
	@echo "make all        compile and build normal and V1V2 firmware"
	@echo "make hex        compile and build normal and V1V2 hex files"
	@echo "make asm        compile and disassemble normal and V1V2 firmware"
	@echo "make bin        compile and build normal and V1V2 bin files"
	@echo "make clean      remove all build files"

$(TARGET).elf: $(CFILES)
	@echo "Building $(TARGET).elf ..."
	@$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS)

$(TARGET_V1V2).elf: $(CFILES_V1V2)
	@echo "Building $(TARGET_V1V2).elf ..."
	@$(CC) -o $@ $^ $(CFLAGS_V1V2) $(LDFLAGS)

$(TARGET).lst: $(TARGET).elf
	@echo "Building $(TARGET).lst ..."
	@$(OBJDUMP) -S $^ > $(TARGET).lst

$(TARGET_V1V2).lst: $(TARGET_V1V2).elf
	@echo "Building $(TARGET_V1V2).lst ..."
	@$(OBJDUMP) -S $^ > $(TARGET_V1V2).lst

$(TARGET).map: $(TARGET).elf
	@echo "Building $(TARGET).map ..."
	@$(OBJDUMP) -t $^ > $(TARGET).map

$(TARGET_V1V2).map: $(TARGET_V1V2).elf
	@echo "Building $(TARGET_V1V2).map ..."
	@$(OBJDUMP) -t $^ > $(TARGET_V1V2).map

$(TARGET).bin: $(TARGET).elf
	@echo "Building $(TARGET).bin ..."
	@$(OBJCOPY) -O binary $< $(TARGET).bin

$(TARGET_V1V2).bin: $(TARGET_V1V2).elf
	@echo "Building $(TARGET_V1V2).bin ..."
	@$(OBJCOPY) -O binary $< $(TARGET_V1V2).bin

$(TARGET).hex: $(TARGET).elf
	@echo "Building $(TARGET).hex ..."
	@$(OBJCOPY) -O ihex $< $(TARGET).hex

$(TARGET_V1V2).hex: $(TARGET_V1V2).elf
	@echo "Building $(TARGET_V1V2).hex ..."
	@$(OBJCOPY) -O ihex $< $(TARGET_V1V2).hex

$(TARGET).asm: $(TARGET).elf
	@echo "Disassembling to $(TARGET).asm ..."
	@$(OBJDUMP) -d $(TARGET).elf > $(TARGET).asm

$(TARGET_V1V2).asm: $(TARGET_V1V2).elf
	@echo "Disassembling to $(TARGET_V1V2).asm ..."
	@$(OBJDUMP) -d $(TARGET_V1V2).elf > $(TARGET_V1V2).asm

all: $(TARGET).lst $(TARGET).map $(TARGET).bin $(TARGET).hex $(TARGET).asm $(TARGET_V1V2).lst $(TARGET_V1V2).map $(TARGET_V1V2).bin $(TARGET_V1V2).hex $(TARGET_V1V2).asm size

elf: $(TARGET).elf $(TARGET_V1V2).elf removetemp size

bin: $(TARGET).bin $(TARGET_V1V2).bin removetemp size removeelf

hex: $(TARGET).hex $(TARGET_V1V2).hex removetemp size removeelf

asm: $(TARGET).asm $(TARGET_V1V2).asm removetemp size removeelf

clean:
	@echo "Cleaning all up ..."
	@$(CLEAN)
	@rm -f $(TARGET).elf $(TARGET).bin $(TARGET).hex $(TARGET).asm
	@rm -f $(TARGET_V1V2).elf $(TARGET_V1V2).bin $(TARGET_V1V2).hex $(TARGET_V1V2).asm

size: $(TARGET).elf $(TARGET_V1V2).elf
	@echo "------------------"
	@echo "$(TARGET)"
	@printf "FLASH: "; $(OBJSIZE) -d $(TARGET).elf | awk '/[0-9]/ {print $$1 + $$2 " bytes"}'
	@printf "SRAM:  "; $(OBJSIZE) -d $(TARGET).elf | awk '/[0-9]/ {print $$2 + $$3 " bytes"}'
	@echo "$(TARGET_V1V2)"
	@printf "FLASH: "; $(OBJSIZE) -d $(TARGET_V1V2).elf | awk '/[0-9]/ {print $$1 + $$2 " bytes"}'
	@printf "SRAM:  "; $(OBJSIZE) -d $(TARGET_V1V2).elf | awk '/[0-9]/ {print $$2 + $$3 " bytes"}'
	@echo "------------------"

removetemp:
	@echo "Removing temporary files ..."
	@$(CLEAN)

removeelf:
	@echo "Removing $(TARGET).elf ..."
	@rm -f $(TARGET).elf
	@echo "Removing $(TARGET_V1V2).elf ..."
	@rm -f $(TARGET_V1V2).elf
