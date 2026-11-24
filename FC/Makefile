TARGET = fc401
BUILD_DIR = build

CXX_SRCS = src/main.cpp
ASM_SRCS = startup/startup_stm32f401cc.s
LDSCRIPT = linker/STM32F401CC_FLASH.ld

PREFIX = arm-none-eabi-
CC      = $(PREFIX)gcc
CXX     = $(PREFIX)g++
AS      = $(PREFIX)as
CP      = $(PREFIX)objcopy
SZ      = $(PREFIX)size

MCU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

CXX_FLAGS = $(MCU) -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter \
            -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti

AS_FLAGS = $(MCU) -x assembler-with-cpp

LDFLAGS = $(MCU) -T$(LDSCRIPT) --specs=nano.specs --specs=nosys.specs \
          -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -lm

OBJS  = $(addprefix $(BUILD_DIR)/,$(notdir $(CXX_SRCS:.cpp=.o)))
OBJS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SRCS:.s=.o)))

all: $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/%.o: src/%.cpp | $(BUILD_DIR)
	$(CXX) -c $(CXX_FLAGS) $< -o $@

$(BUILD_DIR)/%.o: startup/%.s | $(BUILD_DIR)
	$(CC) -c $(AS_FLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O binary $< $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O ihex $< $@

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean
