# 题目4 串口通信协议 (PC ↔ STM32)

## 物理层
- 接口: USB-TTL (UART)
- 波特率: 115200 8N1 (8 data bits, no parity, 1 stop bit)
- 电平: 3.3V TTL
- 共地: USB-TTL GND ↔ STM32 GND ↔ OLED GND (必须)

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
| 11 | u8 | confidence | - | 0-255 | 置信度 |

## XOR 校验
- 校验范围: Payload 前 11 字节 (offset 0..10)
- 校验值: Payload 第 12 字节 (offset 11)
- 算法: `xor = byte[0] ^ byte[1] ^ ... ^ byte[10]`

## 时序
- PC 端: 20 Hz (50 ms interval)
- STM32 端: UART RX ISR → ring buffer → main loop parse → OLED refresh

## 示例帧
- 目标存在: `AA 0100 01 2800 C800 0000 02 05 C0 FF BB`
- 目标丢失: `AA 0000 00 0000 0000 0000 00 FF 00 00 BB`
- 边界值: `AA FFFF 01 FF7F FF7F FFFF 03 09 FF XX BB`

## 错误处理
- STM32 端: 连续 250ms 无 READY 帧 → OLED 显示 `LINK LOST`
- 错帧计数: error_count
- 帧头/帧尾错误 → 拒绝帧, error_count++
- 截断帧 → 拒绝, error_count++