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
TARGET_DISABLE_MOSFET = pd_pps_variable_disable_mosfet
TARGET_DISABLE_CURRENT = pd_pps_variable_disable_current
TARGET_DISABLE_MOSFET_DISABLE_CURRENT = pd_pps_variable_disable_mosfet_disable_current
TARGET_V1V2 = pd_pps_variable_V1V2
TARGET_V1V2_DISABLE_MOSFET = pd_pps_variable_V1V2_disable_mosfet
TARGET_V1V2_DISABLE_CURRENT = pd_pps_variable_V1V2_disable_current
TARGET_V1V2_DISABLE_MOSFET_DISABLE_CURRENT = pd_pps_variable_V1V2_disable_mosfet_disable_current
TARGETS = $(TARGET) $(TARGET_DISABLE_MOSFET) $(TARGET_DISABLE_CURRENT) $(TARGET_DISABLE_MOSFET_DISABLE_CURRENT) $(TARGET_V1V2) $(TARGET_V1V2_DISABLE_MOSFET) $(TARGET_V1V2_DISABLE_CURRENT) $(TARGET_V1V2_DISABLE_MOSFET_DISABLE_CURRENT)
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

$(TARGET)_CFILES = $(CFILES)
$(TARGET_DISABLE_MOSFET)_CFILES = $(CFILES)
$(TARGET_DISABLE_CURRENT)_CFILES = $(CFILES)
$(TARGET_DISABLE_MOSFET_DISABLE_CURRENT)_CFILES = $(CFILES)
$(TARGET_V1V2)_CFILES = $(CFILES_V1V2)
$(TARGET_V1V2_DISABLE_MOSFET)_CFILES = $(CFILES_V1V2)
$(TARGET_V1V2_DISABLE_CURRENT)_CFILES = $(CFILES_V1V2)
$(TARGET_V1V2_DISABLE_MOSFET_DISABLE_CURRENT)_CFILES = $(CFILES_V1V2)

$(TARGET)_CFLAGS = $(CFLAGS)
$(TARGET_DISABLE_MOSFET)_CFLAGS = $(CFLAGS) -DDISABLE_MOSFET
$(TARGET_DISABLE_CURRENT)_CFLAGS = $(CFLAGS) -DDISABLE_CURRENT
$(TARGET_DISABLE_MOSFET_DISABLE_CURRENT)_CFLAGS = $(CFLAGS) -DDISABLE_MOSFET -DDISABLE_CURRENT
$(TARGET_V1V2)_CFLAGS = $(CFLAGS_V1V2)
$(TARGET_V1V2_DISABLE_MOSFET)_CFLAGS = $(CFLAGS_V1V2) -DDISABLE_MOSFET
$(TARGET_V1V2_DISABLE_CURRENT)_CFLAGS = $(CFLAGS_V1V2) -DDISABLE_CURRENT
$(TARGET_V1V2_DISABLE_MOSFET_DISABLE_CURRENT)_CFLAGS = $(CFLAGS_V1V2) -DDISABLE_MOSFET -DDISABLE_CURRENT

ELF_FILES = $(addsuffix .elf,$(TARGETS))
LST_FILES = $(addsuffix .lst,$(TARGETS))
MAP_FILES = $(addsuffix .map,$(TARGETS))
BIN_FILES = $(addsuffix .bin,$(TARGETS))
HEX_FILES = $(addsuffix .hex,$(TARGETS))
ASM_FILES = $(addsuffix .asm,$(TARGETS))

.DEFAULT_GOAL := all
.PHONY: all elf bin hex asm clean size removetemp removeelf help

# Symbolic Targets
help:
	@echo "Use the following commands:"
	@echo "make all        compile and build all board and option variants"
	@echo "make hex        compile and build all hex files"
	@echo "make asm        compile and disassemble all firmware variants"
	@echo "make bin        compile and build all bin files"
	@echo "make clean      remove all build files"

define BUILD_ELF
$(1).elf: $$($(1)_CFILES)
	@echo "Building $(1).elf ..."
	@$$(CC) -o $$@ $$^ $$($(1)_CFLAGS) $$(LDFLAGS)
endef

$(foreach target,$(TARGETS),$(eval $(call BUILD_ELF,$(target))))

%.lst: %.elf
	@echo "Building $@ ..."
	@$(OBJDUMP) -S $^ > $@

%.map: %.elf
	@echo "Building $@ ..."
	@$(OBJDUMP) -t $^ > $@

%.bin: %.elf
	@echo "Building $@ ..."
	@$(OBJCOPY) -O binary $< $@

%.hex: %.elf
	@echo "Building $@ ..."
	@$(OBJCOPY) -O ihex $< $@

%.asm: %.elf
	@echo "Disassembling to $@ ..."
	@$(OBJDUMP) -d $< > $@

all: $(LST_FILES) $(MAP_FILES) $(BIN_FILES) $(HEX_FILES) $(ASM_FILES) size

elf: $(ELF_FILES) size

bin: $(BIN_FILES) size removeelf

hex: $(HEX_FILES) size removeelf

asm: $(ASM_FILES) size removeelf

clean:
	@echo "Cleaning all up ..."
	@$(CLEAN)
	@rm -f $(ELF_FILES) $(BIN_FILES) $(HEX_FILES) $(ASM_FILES)

size: $(ELF_FILES)
	@echo "------------------"
	@for target in $(TARGETS); do \
		echo "$$target"; \
		printf "FLASH: "; $(OBJSIZE) -d "$$target.elf" | awk '/[0-9]/ {print $$1 + $$2 " bytes"}'; \
		printf "SRAM:  "; $(OBJSIZE) -d "$$target.elf" | awk '/[0-9]/ {print $$2 + $$3 " bytes"}'; \
	done
	@echo "------------------"

removetemp:
	@echo "Removing temporary files ..."
	@$(CLEAN)

removeelf:
	@echo "Removing elf files ..."
	@rm -f $(ELF_FILES)
