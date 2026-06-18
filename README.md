# IDC Robocon 小车

2026 IDC Robocon 校赛参赛作品。基于 STM32F407 的两驱小车，装配有丝杆夹爪结构，支持遥控、巡线、抓球和自动爬坡。

## 硬件

- **主控**: STM32F407VET6 (Cortex-M4)
- **电机**: VESC 无刷电机，CAN 总线通信
- **传感器**: ICM20948 / MPU6050 陀螺仪，ccd视觉摄像头
- **输入**: 航模六通道遥控器、按键
- **通信**: UART、I2C、SPI、CAN、USB

## 功能

- 航模遥控器手动控制
- 巡线行驶
- 自动爬坡：检测到斜坡后自动加速通过，到平地后停止

## 怎么用

用 Keil MDK 打开工程，编译后烧录到 STM32F407 即可。

## 文件结构

```
├── USER/main.c          主程序
├── HARDWARE/            外设驱动
│   ├── VESC_CAN/        CAN 电机控制
│   ├── IMU_TEST/        陀螺仪
│   ├── LINE_FOLLOW/     巡线
│   ├── REMOTE/          遥控器
│   └── ACTUATOR/        执行器
└── SYSTEM/              系统（延时、串口）
```
