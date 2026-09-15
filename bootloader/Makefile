# UART bootloader for the mower STM32F411CE board.
#
#   make            -> build/bootloader.elf / .bin / .hex
#   make flash      -> program the bootloader with st-flash (sectors 0-1 only)
#   make clean
#
# Toolchain: defaults to the GNU tools bundled with STM32CubeIDE on macOS.
# Override with  make TOOLCHAIN_BIN=/path/to/bin  or put arm-none-eabi-* on PATH
# and pass TOOLCHAIN_BIN= (empty).

CUBE_TOOLS := $(wildcard /Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin)
TOOLCHAIN_BIN ?= $(firstword $(CUBE_TOOLS))
PREFIX := $(if $(TOOLCHAIN_BIN),$(TOOLCHAIN_BIN)/,)arm-none-eabi-

CC      := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size

ROOT   := ..
BUILD  := build
TARGET := bootloader

C_SOURCES := \
  Src/bl_main.c \
  Src/bl_protocol.c \
  Src/bl_flash.c \
  $(ROOT)/Core/Src/system_stm32f4xx.c \
  $(ROOT)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
  $(ROOT)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
  $(ROOT)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
  $(ROOT)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
  $(ROOT)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
  $(ROOT)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
  $(ROOT)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
  $(ROOT)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c

ASM_SOURCES := $(ROOT)/Core/Startup/startup_stm32f411ceux.s

INCLUDES := \
  -IInc \
  -I$(ROOT)/Module/Inc \
  -I$(ROOT)/Drivers/STM32F4xx_HAL_Driver/Inc \
  -I$(ROOT)/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy \
  -I$(ROOT)/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
  -I$(ROOT)/Drivers/CMSIS/Include

# The app's system_stm32f4xx.c relocates the vector table to the app offset by
# default; the bootloader keeps the table at the flash base.
DEFINES := -DUSE_HAL_DRIVER -DSTM32F411xE -DVECT_TAB_OFFSET=0x0U -DBOOTLOADER_BUILD

MCU_FLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS  := $(MCU_FLAGS) -std=gnu11 -Os -g3 -Wall -Wextra -ffunction-sections -fdata-sections \
           -fno-common $(DEFINES) $(INCLUDES) --specs=nano.specs
ASFLAGS := $(MCU_FLAGS) -x assembler-with-cpp
LDFLAGS := $(MCU_FLAGS) -T STM32F411CEUX_BOOT.ld --specs=nano.specs --specs=nosys.specs \
           -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map -static -Wl,--start-group -lc -lm -Wl,--end-group

OBJECTS := $(addprefix $(BUILD)/,$(notdir $(C_SOURCES:.c=.o))) \
           $(addprefix $(BUILD)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

.PHONY: all clean flash size

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex size

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/%.o: %.s | $(BUILD)
	$(CC) $(ASFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJECTS) STM32F411CEUX_BOOT.ld
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/%.hex: $(BUILD)/%.elf
	$(OBJCOPY) -O ihex $< $@

size: $(BUILD)/$(TARGET).elf
	$(SIZE) $<

flash: $(BUILD)/$(TARGET).bin
	st-flash --reset write $< 0x08000000

clean:
	rm -rf $(BUILD)

-include $(OBJECTS:.o=.d)
