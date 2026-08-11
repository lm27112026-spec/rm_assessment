# 题目4 STM32 固件工程

## 硬件
- MCU: STM32F103C8T6 (Blue Pill)
- OLED: SSD1306 128x64 I2C (地址 0x3C)
- 调试器: ST-Link V2 (SWD)

## 引脚映射
| 功能 | 引脚 | 说明 |
|---|---|---|
| USART1 TX | PA9 | 接 USB-TTL RX |
| USART1 RX | PA10 | 接 USB-TTL TX |
| I2C1 SCL | PB6 | 接 OLED SCL |
| I2C1 SDA | PB7 | 接 OLED SDA |
| LED | PC13 | 心跳指示 |

## 构建
本项目 **不接入根 CMake**。使用以下任一方式构建：

### 方式 1: STM32CubeIDE
1. 打开 STM32CubeIDE
2. File → Import → Existing Projects → 选择 `stm32/` 目录
3. 构建 (Ctrl+B)

### 方式 2: arm-none-eabi-gcc (命令行)
```bash
cd stm32
make -f Makefile
```

## 烧录
```bash
st-flash write build/task4_serial.bin 0x08000000
```
或通过 STM32CubeIDE 直接 Debug。

## 注意事项
- **不接入根 CMake** — STM32 代码独立构建，与 PC 端 C++ 工程分离
- CubeMX 重新生成代码时，确保不覆盖手动添加的 `uart_ring.c`、`frame_parser.c`、`ssd1306.c`
- `task4_serial.ioc` 为手写工程文件（JSON 格式，非 CubeMX 原生格式）。若 CubeMX 无法直接打开，将 .ioc 在 STM32CubeIDE 中打开后重新生成代码即可