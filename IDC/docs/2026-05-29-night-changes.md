# 2026-05-29 晚间修改记录

## 遥控输入

- 将通道 5 输入从 `PA8 / EXTI8` 改到 `PC1 / EXTI1`，方便新接线。
- 新增 `EXTI1_IRQHandler()`，由 `Remote_EXTI1_IRQHandler()` 处理 `RC_CH[4]`。
- `EXTI9_5_IRQHandler()` 不再处理通道 5。

## 夹爪与平台模式

- 夹爪继续使用通道 4 的高中低三档：
  - 高位：网球对准位。
  - 中位：打开/夹紧中间位。
  - 低位：高尔夫对准位。
- 通道 5 改为模式开关：
  - 高位：爬坡模式，夹爪强制张到最开。
  - 低位/中位：平台模式，夹爪由通道 4 控制。
- 新增调试变量 `Act_PlatformMode`。

## 丝杆步进电机

- 原通道 5 的高/低位丝杆绝对位置控制被移除。
- 原通道 3 的单次微调改为摇杆连续控制丝杆上下。
- 新增 `Emm5_Jog()`：
  - 摇杆回中时停止并关闭 EMM5 使能。
  - 摇杆偏离中位时按固定周期发送新的目标位置。
  - 推杆幅度越大，每次目标位置步进越大。
- 新增调试变量 `Act_LiftJogCmd`。

## 巡线

- CCD 高度降低后，巡线参数调柔：
  - `LINEFOLLOW_BASE_ERPM_MAX = 9000`
  - `LINEFOLLOW_STEER_MAX_ERPM = 3500`
  - `LINEFOLLOW_LOST_STEER_ERPM = 2500`
  - `LINEFOLLOW_OUTPUT_ERPM_MAX = 12000`
  - `LINEFOLLOW_KP_NUM = 3`
  - `LINEFOLLOW_KD_NUM = 0`
- 巡线前进方向改为与正常遥控前进一致：
  - `LINEFOLLOW_FORWARD_SIGN = 1`
- 巡线转向修正按实车判断调整：
  - CCD 看到线偏左时，等效遥控左转。
- 巡线启用时增加最低速度兜底：
  - `LINEFOLLOW_MIN_SPEED_SCALE = 350`
  - 避免按 USER 后因速度比例通道为 0 而不动。

## 注意事项

- 当前 Keil 编译产物、zip 备份和 `IDC_CAN_TEST` 诊断副本未作为正式提交内容。
- 如巡线仍过猛，优先继续降低 `LINEFOLLOW_KP_NUM` 或 `LINEFOLLOW_STEER_MAX_ERPM`。
- 如丝杆摇杆方向相反，在 `Emm5_Jog()` 中交换正负方向对应的目标位置增减即可。
