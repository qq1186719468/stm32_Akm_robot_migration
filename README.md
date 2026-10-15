# STM32 阿克曼机器人下位机底盘

这是面向 C30D 1.0 阿克曼底盘的 STM32F407 机器人控制工程。项目基于原有 WHEELTEC 控制代码完成通信、安全、状态监控和现场诊断能力的迁移与增强。

## 硬件与开发环境

- MCU：STM32F407VET6，168 MHz
- 实时系统：FreeRTOS V9
- 工具链：Keil MDK、ARMCC5、STM32F4 标准外设库
- 底盘：后轮双电机、前轮舵机的阿克曼结构
- 传感器：GMR 编码器、MPU6050
- 显示：原 OLED 与 ST7735S 128×160 TFT

## 主要功能

- 阿克曼运动学与双后轮速度闭环控制
- USART3 DMA、IDLE 中断和循环缓冲接收
- 兼容原来的串口中断旧协议与新增的带 CRC16 的 AA55 协议
- 通信超时停机、PI 状态复位和舵机回中保护
- 运行状态快照、任务心跳和资源监控
- USART3 DMA 非阻塞状态上报
- 低优先级异步事件日志
- 独立看门狗监控框架
- TFT 现场诊断界面

## 目录结构

```text
BALANCE/    控制、状态与监控模块
CORE/       Cortex-M4 内核与启动文件
FWLIB/      STM32F4 标准外设库
FreeRTOS/   FreeRTOS 内核与移植层
HARDWARE/   电机、编码器、通信、显示和传感器驱动
SYSTEM/     时钟、延时和基础串口模块
USER/       主程序、Keil 工程和中断处理
OBJ/        编译输出与烧录文件
tools/      协议帧和串口监控工具
```

## 编译与烧录

1. 使用 Keil MDK 打开 `USER/WHEELTEC.uvprojx`。
2. 选择 ARMCC5 工具链并执行 Rebuild All。
3. 编译生成的烧录文件位于 `OBJ/WHEELTEC.hex`。
4. 下载前确认目标器件为 STM32F407VET6，Flash 大小为 512 KB。

## 使用提示

- 上电后等待 IMU 校准完成，再发送运动控制指令。
- 首次调试应架空车轮，并先验证停机电平、舵机中位和通信超时保护。
- USART3 默认用于控制协议和状态上报；USART2 用于低优先级文本日志。
- 看门狗默认保持关闭，完成专项硬件验证后再启用。

