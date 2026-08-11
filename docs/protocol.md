# 题目4 串口通信协议 (PC ↔ STM32)

> **Version**: 1.0 | **Date**: 2026-08-11 | **Status**: Frozen

## 物理层
- 接口: USB-TTL (UART)
- 波特率: 115200 8N1 (8 data bits, no parity, 1 stop bit)
- 电平: 3.3V TTL
- 共地: USB-TTL GND ↔ STM32 GND ↔ OLED GND (必须)
- 接线: USB-TTL TX → STM32 PA10(RX), USB-TTL RX → STM32 PA9(TX)

## 帧格式
| 字节 | 0 | 1..12 | 13 |
|---|---|---|---|
| 内容 | `0xAA` (HEADER) | Payload (12 bytes) | `0xBB` (FOOTER) |

- 总帧长: 14 字节
- Payload 固定 12 字节，小端字节序
- 定界方式: 固定长度 (非搜索 `0xBB`)
- 帧头/帧尾不在 payload 中定界

## Payload 字段表

| Offset | Type | Name | Unit | Range | Description |
|---|---|---|---|---|---|
| 0-1 | u16 LE | frame_counter | - | 0-65535 | 帧计数，每帧 +1 |
| 2 | u8 | target_present | - | 0/1 | 0=无, 1=有 |
| 3-4 | i16 LE | target_x | pixels | -32768..32767 | 目标中心 X |
| 5-6 | i16 LE | target_y | pixels | -32768..32767 | 目标中心 Y |
| 7-8 | i16 LE | distance | mm | -32768..32767 | 距离，占位 0 |
| 9 | u8 | tracker_state | - | 0-255 | 跟踪器状态 |
| 10 | u8 | digit | - | 0-9, 255=未识别 | 识别数字 |
| 11 | u8 | xor_checksum | - | 0-255 | XOR 校验值 (XOR of bytes 0-10) |

## XOR 校验
- 校验范围: Payload 前 11 字节 (offset 0..10)
- 校验值: Payload 第 12 字节 (offset 11)
- 算法: `xor = byte[0] ^ byte[1] ^ ... ^ byte[10]`

## 时序
- PC 端: 20 Hz (50 ms interval)
- STM32 端: UART RX ISR → ring buffer → main loop parse → OLED refresh

## 示例帧
- 目标存在 (frame=1, x=200, y=100, tracker=2, digit=5): `AA 0100 01 C800 6400 0000 02 05 E3 BB`
  - Payload 字节: `01 00 01 C8 00 64 00 00 00 02 05` → XOR = `E3`
- 目标丢失 (frame=0, digit=255): `AA 0000 00 0000 0000 0000 00 FF FF BB`
  - Payload 字节: `00 00 00 00 00 00 00 00 00 00 FF` → XOR = `FF`
- 边界值 (frame=65535, x=32767, y=32767, tracker=3, digit=9): `AA FFFF 01 FF7F FF7F FFFF 03 09 0B BB`
  - Payload 字节: `FF FF 01 FF 7F FF 7F FF FF 03 09` → XOR = `0B`

## 错误处理
- STM32 端: 连续 250ms 无 READY 帧 → OLED 显示 `LINK LOST`
- 错帧计数: error_count
- 帧头/帧尾错误 → 拒绝帧, error_count++
- 截断帧 → 拒绝, error_count++