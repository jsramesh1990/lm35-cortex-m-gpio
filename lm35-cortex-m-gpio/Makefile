# Makefile for LM35 Cortex-M GPIO Project
#
# Usage:
#   make              - Build the project
#   make clean        - Clean build files
#   make flash        - Flash to board
#   make debug        - Build with debug symbols
#   make test         - Run tests
#   make size         - Show memory usage

# Project name
PROJECT = lm35_cortex_m_gpio

# Board configuration (override with BOARD=stm32f7 etc.)
BOARD ?= stm32f4
BUILD_TYPE ?= release

# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AS = $(PREFIX)gcc
LD = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE = $(PREFIX)size
GDB = $(PREFIX)gdb

# Directories
SRC_DIR = src
INC_DIR = inc
DRIVERS_DIR = $(SRC_DIR)/drivers
STARTUP_DIR = $(SRC_DIR)/startup
BUILD_DIR = build/$(BOARD)/$(BUILD_TYPE)
BIN_DIR = bin

# Source files
SRCS = \
    $(SRC_DIR)/main.c \
    $(SRC_DIR)/system_init.c \
    $(DRIVERS_DIR)/lm35_driver.c \
    $(DRIVERS_DIR)/gpio_interface.c

# Startup file (board-specific)
ifeq ($(BOARD),stm32f4)
    STARTUP = $(STARTUP_DIR)/startup_stm32f4xx.s
    LDSCRIPT = $(STARTUP_DIR)/STM32F407VGTx_FLASH.ld
    DEFINES = -DSTM32F4 -DARM_MATH_CM4
    MCU = cortex-m4
    FPU = fpv4-sp-d16
else ifeq ($(BOARD),stm32f7)
    STARTUP = $(STARTUP_DIR)/startup_stm32f7xx.s
    LDSCRIPT = $(STARTUP_DIR)/STM32F746NGHx_FLASH.ld
    DEFINES = -DSTM32F7 -DARM_MATH_CM7
    MCU = cortex-m7
    FPU = fpv5-sp-d16
else ifeq ($(BOARD),nucleo-f401re)
    STARTUP = $(STARTUP_DIR)/startup_stm32f4xx.s
    LDSCRIPT = $(STARTUP_DIR)/STM32F401RETx_FLASH.ld
    DEFINES = -DSTM32F4 -DARM_MATH_CM4
    MCU = cortex-m4
    FPU = fpv4-sp-d16
else
    $(error Unknown board: $(BOARD))
endif

# Include configuration
-include Makefile.config

# Object files
OBJS = $(SRCS:%.c=$(BUILD_DIR)/%.o) $(STARTUP:%.s=$(BUILD_DIR)/%.o)

# Include directories
INCLUDES = -I$(INC_DIR) -I$(SRC_DIR) -I$(DRIVERS_DIR)

# Compiler flags based on build type
ifeq ($(BUILD_TYPE),debug)
    CFLAGS = -O0 -g3 -DDEBUG
else
    CFLAGS = -O2 -DNDEBUG
endif

# Common compiler flags
CFLAGS += \
    -mcpu=$(MCU) \
    -mthumb \
    -mfpu=$(FPU) \
    -mfloat-abi=hard \
    -Wall \
    -Wextra \
    -Werror \
    -ffunction-sections \
    -fdata-sections \
    $(DEFINES) \
    $(INCLUDES)

# ASM flags
ASFLAGS = -mcpu=$(MCU) -mthumb -mfpu=$(FPU) -mfloat-abi=hard -x assembler-with-cpp

# Linker flags
LDFLAGS = \
    -mcpu=$(MCU) \
    -mthumb \
    -mfpu=$(FPU) \
    -mfloat-abi=hard \
    -T$(LDSCRIPT) \
    -Wl,--gc-sections \
    -Wl,-Map=$(BUILD_DIR)/$(PROJECT).map \
    -specs=nosys.specs \
    -specs=nano.specs \
    -lm

# Targets
.PHONY: all clean flash debug test size

all: $(BIN_DIR)/$(PROJECT).elf $(BIN_DIR)/$(PROJECT).bin $(BIN_DIR)/$(PROJECT).hex

# Create directories
$(BUILD_DIR) $(BIN_DIR):
	@mkdir -p $@

# Compile C sources
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	@echo "CC  $<"
	@$(CC) $(CFLAGS) -c $< -o $@

# Compile ASM sources
$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	@echo "AS  $<"
	@$(AS) $(ASFLAGS) -c $< -o $@

# Link ELF file
$(BIN_DIR)/$(PROJECT).elf: $(OBJS) | $(BIN_DIR)
	@echo "LD  $@"
	@$(LD) $(OBJS) $(LDFLAGS) -o $@
	@echo "Build complete: $@"

# Create binary file
$(BIN_DIR)/$(PROJECT).bin: $(BIN_DIR)/$(PROJECT).elf
	@echo "OBJCOPY $@"
	@$(OBJCOPY) -O binary $< $@

# Create hex file
$(BIN_DIR)/$(PROJECT).hex: $(BIN_DIR)/$(PROJECT).elf
	@echo "OBJCOPY $@"
	@$(OBJCOPY) -O ihex $< $@

# Clean build files
clean:
	@echo "Cleaning build files..."
	@rm -rf build bin

# Flash to board using st-link
flash: $(BIN_DIR)/$(PROJECT).bin
	@echo "Flashing to board..."
	@st-flash write $< 0x8000000

# Flash using openocd
flash-openocd: $(BIN_DIR)/$(PROJECT).elf
	@echo "Flashing with OpenOCD..."
	@openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg \
		-c "program $< verify reset exit"

# Debug build
debug:
	@$(MAKE) BUILD_TYPE=debug

# Run tests
test: $(BIN_DIR)/test_lm35.elf $(BIN_DIR)/test_gpio.elf
	@echo "Running tests..."
	@echo "Note: Tests require hardware connection"

# Show memory usage
size: $(BIN_DIR)/$(PROJECT).elf
	@$(SIZE) $<

# Build test executables
$(BIN_DIR)/test_lm35.elf: tests/test_lm35.c $(SRCS) $(STARTUP)
	@mkdir -p $(BUILD_DIR) $(BIN_DIR)
	@echo "Building test_lm35..."
	@$(CC) $(CFLAGS) -DTEST_MODE=1 tests/test_lm35.c $(SRCS) $(STARTUP) $(LDFLAGS) -o $@

$(BIN_DIR)/test_gpio.elf: tests/test_gpio.c $(SRCS) $(STARTUP)
	@mkdir -p $(BUILD_DIR) $(BIN_DIR)
	@echo "Building test_gpio..."
	@$(CC) $(CFLAGS) -DGPIO_TEST=1 tests/test_gpio.c $(SRCS) $(STARTUP) $(LDFLAGS) -o $@

# Dependency generation (optional)
-include $(OBJS:.o=.d)

# Help target
help:
	@echo "Available targets:"
	@echo "  make              - Build the project"
	@echo "  make clean        - Clean build files"
	@echo "  make flash        - Flash to board using st-link"
	@echo "  make flash-openocd - Flash using OpenOCD"
	@echo "  make debug        - Build with debug symbols"
	@echo "  make test         - Build test executables"
	@echo "  make size         - Show memory usage"
	@echo "  make help         - Show this help"
	@echo ""
	@echo "Board selection (default: stm32f4):"
	@echo "  make BOARD=stm32f4"
	@echo "  make BOARD=stm32f7"
	@echo "  make BOARD=nucleo-f401re"
	@echo ""
	@echo "Build type (default: release):"
	@echo "  make BUILD_TYPE=debug"
	@echo "  make BUILD_TYPE=release"
