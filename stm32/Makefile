# Minimal arm-none-eabi-gcc Makefile for task4_serial (STM32F103C8T6)
# Usage: make -f Makefile
# Requires: arm-none-eabi-gcc toolchain in PATH

TARGET   = task4_serial
BUILD    = build

MCU      = cortex-m3
DEFS     = -DSTM32F103xB -DUSE_HAL_DRIVER
OPT      = -Os -g3

# HAL library location (CubeMX-generated projects place it under Drivers/).
# If you regenerate from STM32CubeIDE, point HAL_DIR at the generated Drivers folder.
HAL_DIR  = Drivers/STM32F1xx_HAL_Driver
CMSIS_DIR = Drivers/CMSIS

INC      = -ICore/Inc \
           -I$(HAL_DIR)/Inc \
           -I$(CMSIS_DIR)/Device/ST/STM32F1xx/Include \
           -I$(CMSIS_DIR)/Include

SRCS     = Core/Src/main.c \
           Core/Src/usart.c \
           Core/Src/gpio.c \
           Core/Src/i2c.c \
           Core/Src/stm32f1xx_it.c \
           Core/Src/system_stm32f1xx.c \
           Core/Src/uart_ring.c \
           Core/Src/frame_parser.c \
           Core/Src/ssd1306.c

# HAL sources (only modules enabled in stm32f1xx_hal_conf.h)
HAL_SRCS = $(HAL_DIR)/Src/stm32f1xx_hal.c \
           $(HAL_DIR)/Src/stm32f1xx_hal_rcc.c \
           $(HAL_DIR)/Src/stm32f1xx_hal_rcc_ex.c \
           $(HAL_DIR)/Src/stm32f1xx_hal_gpio.c \
           $(HAL_DIR)/Src/stm32f1xx_hal_dma.c \
           $(HAL_DIR)/Src/stm32f1xx_hal_cortex.c \
           $(HAL_DIR)/Src/stm32f1xx_hal_flash.c \
           $(HAL_DIR)/Src/stm32f1xx_hal_flash_ex.c \
           $(HAL_DIR)/Src/stm32f1xx_hal_pwr.c \
           $(HAL_DIR)/Src/stm32f1xx_hal_i2c.c \
           $(HAL_DIR)/Src/stm32f1xx_hal_uart.c \
           $(HAL_DIR)/Src/stm32f1xx_hal_exti.c

STARTUP  = $(CMSIS_DIR)/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s
LINKER   = $(CMSIS_DIR)/Device/ST/STM32F1xx/Source/Templates/gcc/linker/stm32f103xb_flash.ld

CFLAGS   = $(MCU) $(DEFS) $(INC) $(OPT) -Wall -ffunction-sections -fdata-sections -std=c99
LDFLAGS  = $(MCU) -T$(LINKER) -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map

OBJS     = $(patsubst %.c,$(BUILD)/%.o,$(SRCS) $(HAL_SRCS))
OBJS    += $(BUILD)/startup.o

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	arm-none-eabi-gcc $(CFLAGS) -c $< -o $@

$(BUILD)/startup.o: $(STARTUP)
	@mkdir -p $(BUILD)
	arm-none-eabi-gcc $(MCU) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS)
	arm-none-eabi-gcc $(LDFLAGS) $^ -o $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	arm-none-eabi-objcopy -O binary $< $@

clean:
	rm -rf $(BUILD)

.PHONY: all clean