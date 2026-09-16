# STM32F411CE application firmware for the mower base (FreeRTOS + HAL).
#
#   make            -> build/mower_robot_firmware.elf / .bin / .hex / .json
#   make flash      -> program over the host UART with tools/mower_flash.py
#                      (PORT=/dev/ttyS3 on the LubanCat, PORT=/dev/cu.usb* on a Mac)
#   make bootloader -> build bootloader/ (see bootloader/Makefile)
#   make clean
#
# Same sources, defines and target flags as the STM32CubeIDE project
# (.cproject); this file exists so CI and the Docker image can build the
# firmware without CubeIDE. The .ioc / CubeIDE project stays usable for
# debugging.
#
# Toolchain: defaults to the GNU tools bundled with STM32CubeIDE on macOS, or
# arm-none-eabi-* on PATH (Linux / CI: apt install gcc-arm-none-eabi).
# Override with  make TOOLCHAIN_BIN=/path/to/bin
#
# Build identity (reported in the 0x87 FIRMWARE_INFO frame, firmware_version.h):
#   MOWER_VERSION      e.g. 0.6.0 or 0.6.0-3-gabcdef12-dirty   (default: git describe)
#   MOWER_GIT_SHA      commit hash                              (default: git rev-parse)
#   MOWER_BUILD_DIRTY  0/1                                      (default: git status)
#   MOWER_BUILD_UNIX   unix seconds                             (default: commit time,
#                                                                now if dirty)
# The Docker build passes these in because the build context has no .git.

CUBE_TOOLS := $(wildcard /Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin)
TOOLCHAIN_BIN ?= $(firstword $(CUBE_TOOLS))
PREFIX := $(if $(TOOLCHAIN_BIN),$(TOOLCHAIN_BIN)/,)arm-none-eabi-

CC      := $(PREFIX)gcc
CXX     := $(PREFIX)g++
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size

BUILD  := build
TARGET := mower_robot_firmware
PORT   ?= /dev/ttyS3

# ---- build identity ---------------------------------------------------------
GIT_OK := $(shell git rev-parse --is-inside-work-tree 2>/dev/null)
ifeq ($(GIT_OK),true)
  MOWER_VERSION     ?= $(shell git describe --tags --match 'v*' --always --dirty 2>/dev/null)
  MOWER_GIT_SHA     ?= $(shell git rev-parse HEAD 2>/dev/null)
  MOWER_BUILD_DIRTY ?= $(shell test -z "$$(git status --porcelain -- . 2>/dev/null)" && echo 0 || echo 1)
  ifeq ($(MOWER_BUILD_DIRTY),0)
    MOWER_BUILD_UNIX ?= $(shell git log -1 --format=%ct 2>/dev/null)
  endif
endif
# Defaults; an empty value passed on the command line (Docker build args
# that were not set) also falls back here.
ifeq ($(strip $(MOWER_VERSION)),)
  override MOWER_VERSION := 0.0.0
endif
ifeq ($(strip $(MOWER_GIT_SHA)),)
  override MOWER_GIT_SHA := 0000000000000000
endif
ifeq ($(strip $(MOWER_BUILD_DIRTY)),)
  override MOWER_BUILD_DIRTY := 0
endif
ifeq ($(strip $(MOWER_BUILD_UNIX)),)
  override MOWER_BUILD_UNIX := $(shell date +%s)
endif

# "v0.6.0-3-gabc-dirty" / "0.6.0" -> 0 6 0 ; anything without a leading
# version number -> 0 0 0 (a bare commit hash from git describe --always).
VERSION_CORE := $(shell echo "$(MOWER_VERSION)" | sed -nE 's/^v?([0-9]+)\.([0-9]+)\.([0-9]+).*/\1 \2 \3/p')
ifeq ($(VERSION_CORE),)
  VERSION_CORE := 0 0 0
endif
FW_MAJOR := $(word 1,$(VERSION_CORE))
FW_MINOR := $(word 2,$(VERSION_CORE))
FW_PATCH := $(word 3,$(VERSION_CORE))
FW_SHA32 := $(shell echo "$(MOWER_GIT_SHA)" | sed -nE 's/^([0-9a-fA-F]{8}).*/\1/p')
ifeq ($(FW_SHA32),)
  FW_SHA32 := 00000000
endif
FW_FLAGS := $(if $(filter 1,$(MOWER_BUILD_DIRTY)),0x01,0x00)
ifneq ($(findstring -dirty,$(MOWER_VERSION)),)
  FW_FLAGS := 0x01
endif

VERSION_DEFINES := \
  -DFW_VERSION_MAJOR=$(FW_MAJOR)U \
  -DFW_VERSION_MINOR=$(FW_MINOR)U \
  -DFW_VERSION_PATCH=$(FW_PATCH)U \
  -DFW_GIT_SHA=0x$(FW_SHA32)UL \
  -DFW_BUILD_UNIX=$(MOWER_BUILD_UNIX)UL \
  -DFW_BUILD_FLAGS=$(FW_FLAGS)U

# ---- sources -----------------------------------------------------------------
C_SOURCES := \
  $(wildcard Core/Src/*.c) \
  $(wildcard Module/Src/*.c) \
  $(wildcard Drivers/STM32F4xx_HAL_Driver/Src/*.c) \
  Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
  Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
  Middlewares/Third_Party/FreeRTOS/Source/list.c \
  Middlewares/Third_Party/FreeRTOS/Source/queue.c \
  Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
  Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
  Middlewares/Third_Party/FreeRTOS/Source/timers.c \
  Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
  Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
  Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c

CXX_SOURCES := $(wildcard Module/Src/*.cpp)

ASM_SOURCES := Core/Startup/startup_stm32f411ceux.s

INCLUDES := \
  -ICore/Inc \
  -IModule/Inc \
  -IDrivers/STM32F4xx_HAL_Driver/Inc \
  -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
  -IDrivers/CMSIS/Include \
  -IMiddlewares/Third_Party/FreeRTOS/Source/include \
  -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
  -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F

DEFINES := -DUSE_HAL_DRIVER -DSTM32F411xE

MCU_FLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
COMMON_FLAGS := $(MCU_FLAGS) -Os -g3 -Wall -ffunction-sections -fdata-sections -fstack-usage \
                $(DEFINES) $(INCLUDES) --specs=nano.specs
CFLAGS   := $(COMMON_FLAGS) -std=gnu11
CXXFLAGS := $(COMMON_FLAGS) -std=gnu++17 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics
ASFLAGS  := $(MCU_FLAGS) -x assembler-with-cpp
LDFLAGS  := $(MCU_FLAGS) -T STM32F411CEUX_FLASH.ld --specs=nano.specs --specs=nosys.specs \
            -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map -static \
            -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group

OBJECTS := $(addprefix $(BUILD)/,$(C_SOURCES:.c=.o)) \
           $(addprefix $(BUILD)/,$(CXX_SOURCES:.cpp=.o)) \
           $(addprefix $(BUILD)/,$(ASM_SOURCES:.s=.o))

.PHONY: all clean flash size bootloader version FORCE

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).json size

version:
	@echo "MOWER_VERSION=$(MOWER_VERSION) -> $(FW_MAJOR).$(FW_MINOR).$(FW_PATCH) sha=$(FW_SHA32) build=$(MOWER_BUILD_UNIX) flags=$(FW_FLAGS)"

# Only the file that embeds the build identity depends on it, so a version
# bump does not rebuild the HAL.
$(BUILD)/version.txt: FORCE
	@mkdir -p $(dir $@)
	@echo "$(VERSION_DEFINES)" | cmp -s - $@ || echo "$(VERSION_DEFINES)" > $@

$(BUILD)/Module/Src/uart_interface.o: $(BUILD)/version.txt
$(BUILD)/Module/Src/uart_interface.o: CXXFLAGS += $(VERSION_DEFINES)

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(ASFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJECTS) STM32F411CEUX_FLASH.ld
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/%.hex: $(BUILD)/%.elf
	$(OBJCOPY) -O ihex $< $@

# Manifest next to the .bin: what firmware-sync compares against the 0x87 frame.
$(BUILD)/$(TARGET).json: $(BUILD)/$(TARGET).bin tools/fw_manifest.py
	python3 tools/fw_manifest.py $< --version "$(MOWER_VERSION)" --semver $(FW_MAJOR).$(FW_MINOR).$(FW_PATCH) \
	  --git-sha $(MOWER_GIT_SHA) --build-unix $(MOWER_BUILD_UNIX) --dirty $(MOWER_BUILD_DIRTY) -o $@

size: $(BUILD)/$(TARGET).elf
	$(SIZE) $<

flash: $(BUILD)/$(TARGET).bin
	python3 tools/mower_flash.py -p $(PORT) flash $<

bootloader:
	$(MAKE) -C bootloader TOOLCHAIN_BIN=$(TOOLCHAIN_BIN)

clean:
	rm -rf $(BUILD)

-include $(OBJECTS:.o=.d)
