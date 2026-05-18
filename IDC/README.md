# IDCv3.0 - Robocon 巡线抓球小车

基于 STM32F407VET6 的 Robocon 比赛小车，实现 CCD 巡线、VESC 电机驱动、原地 180° 转弯、反向巡线返回等功能。

## 硬件配置

| 模块 | 型号/规格 | 接口 |
|------|-----------|------|
| MCU | STM32F407VET6 (C30D 底板) | HSE 8MHz → PLL → 168MHz |
| 驱动电机 | 2× VESC 无刷电调 | CAN1 (PD0/PD1, 500Kbps) |
| 巡线传感器 | TSL1401CL 线性 CCD (128像素) | PA4(ADC), PA5(CLK), PC4(SI) |
| 升降电机 | ZDT EMM5 闭环步进 | CAN1 扩展帧 (ZDT协议) |
| 夹爪舵机 | PWM 舵机 | TIM3_CH1 (PC6) |
| 遥控接收 | 6通道 PWM 接收机 | EXTI (PA2/PA3/PA6/PA7/PA8/PC0) |
| 启动按键 | 微动开关 (低有效) | PE0 |
| 调试串口 | USART1 | PA9(TX), PA10(RX), 115200bps |

## 目录结构

```
IDC/
├── USER/
│   ├── main.c              # 主循环：按键扫描 + 状态机
│   ├── stm32f4xx_it.c      # 中断服务
│   └── system_stm32f4xx.c  # 系统时钟配置
├── HARDWARE/
│   ├── LINE_FOLLOW/
│   │   ├── line_follow.c   # CCD 巡线：采集、边缘检测、PD控制
│   │   └── line_follow.h
│   ├── ACTUATOR/
│   │   ├── actuator.c      # 执行机构：遥控解析、电机/舵机/升降控制
│   │   └── actuator.h
│   ├── VESC_CAN/
│   │   ├── vesc_can.c      # VESC CAN 通信：RPM 控制、状态接收
│   │   └── vesc_can.h
│   ├── REMOTE/
│   │   ├── remote.c        # 遥控器：EXTI 边沿捕获、微秒计时器
│   │   └── remote.h
│   └── ...                 # 其他硬件模块 (OLED/IMU/编码器等)
├── FWLIB/                  # STM32F4 标准外设库
├── CORE/                   # Cortex-M4 内核文件
├── SYSTEM/                 # delay/sys/usart 基础设施
├── Project/                # Keil MDK 工程文件
└── docs/
    └── 遥控与执行机构接口说明.md
```

## 功能说明

### 自动巡线流程（4状态机）

按下 PE0 启动按键后，小车自动执行以下流程：

```
┌──────────┐  按键  ┌──────────────┐  15s到  ┌──────────────┐  1.5s到  ┌──────────────┐  15s到
│ 0: 待机  │──────→│ 1: 正向巡线  │──────→│ 2: 原地转180°│──────→│ 3: 反向巡线  │──────→ 回到待机
└──────────┘       └──────────────┘       └──────────────┘       └──────────────┘
                    Reverse=0               覆盖左右轮ERPM         Reverse=1
                    BaseERPM>0              左正转/右反转           BaseERPM<0
                                            = 原地右转              像素翻转+转向交换
```

- **State 1**：正向巡线 15 秒，沿 CCD 检测到的黑线前进
- **State 2**：停车后原地右转 180°，耗时约 1.5 秒
- **State 3**：反向巡线 15 秒，CCD 像素翻转 + 倒退行驶 + 转向方向交换，沿原路返回

### 反向巡线实现要点

车身转 180° 后，CCD 视角左右镜像，需要三处特殊处理：

1. **CCD 像素翻转**：ProcessFrame 前将像素数组左右反转，使边缘检测仍能正确找到线的左右边界
2. **基础速度取反**：`base_erpm = -base_erpm`，倒退行驶
3. **转向方向交换**：`left = base_erpm - steer_erpm; right = base_erpm + steer_erpm`（正负号与正向相反）

### CCD 巡线算法

- TSL1401CL 线性 CCD，128 像素，有效范围 [5, 122]，中心像素 63
- 动态阈值 = (max + min) / 2
- 边缘检测：连续 3 高→3 低 找左边缘，连续 3 低→3 高 找右边缘
- 位置误差 = (center - 63) × 1000 / 63，范围 [-1000, +1000]
- PD 控制：`steer = Position × Kp + (Position - last_Position) × Kd`
  - Kp = 4, Kd = 2
  - 转向限幅 ±5000 ERPM
  - 丢线时按上次偏差方向转 ±3500 ERPM
- 十字线检测：线宽 > 30 像素

### 遥控手动控制

6 通道 PWM 接收机映射：

| 通道 | 引脚 | 功能 | 范围 |
|------|------|------|------|
| CH0 | PA2 | 转向 | 1000~2000μs → [-1000, +1000] |
| CH1 | PA3 | 前后 | 1000~2000μs → [-1000, +1000] |
| CH2 | PA6 | 油门/速度比例 | 1000→0%, 2000→100% |
| CH3 | PA7 | - | - |
| CH4 | PA8 | 夹爪三档 | 低=高尔夫球, 中=张开, 高=网球 |
| CH5 | PC0 | 升降三档 | 低=下降, 中=停, 高=上升 |

**安全逻辑**：摇杆回中时驱动锁定（需超过重启死区才解锁，防止误触）。巡线启用时，电机指令由 PD 控制器接管，遥控器转向/前后无效。

### VESC 电机控制

- CAN1 通信，500Kbps（Prescaler=6, BS1=10tq, BS2=3tq, SJW=1tq）
- 扩展帧格式：ExtId = (命令码 << 8) | 控制器ID
- 左电机 ID = 1，右电机 ID = 2
- RPM 控制模式，10ms 周期发送
- 接收 VESC 状态帧（erpm/电流/duty），存储到调试变量
- **符号约定**：VESC_LEFT_SIGN = -1, VESC_RIGHT_SIGN = +1（因电机安装方向）

### EMM5 升降步进电机

- 与 VESC 共用 CAN1 总线，使用 ZDT 扩展帧协议
- 默认地址 3，校验字 0x6B
- 位置控制：速度 300RPM，加速度 50，脉冲数 5000
- 支持使能/清零/上降控制

## 关键参数（需根据实车调整）

| 参数 | 默认值 | 位置 | 说明 |
|------|--------|------|------|
| `LINEFOLLOW_BASE_ERPM_MAX` | 7000 | line_follow.c | 巡线基础速度 |
| `LINEFOLLOW_STEER_MAX_ERPM` | 5000 | line_follow.c | 转向最大 ERPM |
| `LINEFOLLOW_LOST_STEER_ERPM` | 3500 | line_follow.c | 丢线时转向 ERPM |
| `LINEFOLLOW_KP_NUM / _DEN` | 4 / 1 | line_follow.c | PD 控制 Kp |
| `LINEFOLLOW_KD_NUM / _DEN` | 2 / 1 | line_follow.c | PD 控制 Kd |
| `TURN_ERPM` | 3000 | main.c | 原地转弯速度 |
| `LINEFOLLOW_TURN_TIME_US` | 1.5s | main.c | 原地转弯时长 |
| `LINEFOLLOW_FORWARD_TIME_US` | 15s | main.c | 正向巡线时长 |
| `LINEFOLLOW_REVERSE_TIME_US` | 15s | main.c | 反向巡线时长 |
| `Act_LiftPulseCount` | 5000 | actuator.c | 升降行程脉冲数 |

## 开发环境

- **IDE**：Keil MDK-ARM 5
- **芯片支持包**：Keil.STM32F407VETx
- **调试器**：J-Link
- **外设库**：STM32F4 标准外设库 (StdPeriph)
- **烧录方式**：J-Link SWD 或串口 ISP

## 串口调试输出

USART1 (PA9/PA10) 每 100ms 输出状态信息：

```
ST:1 SYS:2 HSE:1 CORE:168000000
│    │     │     └── SystemCoreClock (Hz)
│    │     └── HSE 就绪标志
│    └── SYSCLK 来源 (0=HSI, 2=PLL)
└── 当前状态机状态 (0-3)
```

## CAN 总线拓扑

```
STM32F407 (CAN1, PD0/PD1, 500Kbps)
 ├── VESC 左电机 (ID=1) — 标准VESC协议, 扩展帧
 ├── VESC 右电机 (ID=2) — 标准VESC协议, 扩展帧
 └── EMM5 步进电机 (ID=3) — ZDT协议, 扩展帧
```

所有设备共用同一条 CAN 总线，通过帧 ID 区分。VESC 使用 `(命令码<<8)|控制器ID` 作为 ExtId；EMM5 使用 `(地址<<8)|包序号` 作为 ExtId。
