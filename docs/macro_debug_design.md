# 宏定义编译与调试体系说明

本工程大量使用宏定义来选择硬件形态、机器人类型、模块启停、调试链路和参数映射。核心入口集中在：

```text
application/robot_def.h
```

这套设计的价值是：同一套业务代码可以在不同机器人、不同板卡、不同调试阶段之间快速切换；不需要频繁删除代码，只需要改宏后重新编译。

## 总体分层

当前宏定义大致分为五层：

1. 板卡形态选择：决定当前固件跑整车、底盘板还是云台板。
2. 机器人型号选择：决定使用 R1 还是 R2 的参数和机构配置。
3. 模块任务开关：决定某些 FreeRTOS 任务和应用是否创建/运行。
4. 外设与通信开关：决定视觉、光流、屏幕、VOFA 等链路是否启用。
5. 车种参数映射：把 `R1/R2` 的差异参数映射成业务代码统一使用的宏。

这种设计把“编译时确定的结构差异”和“运行时变化的控制逻辑”分开。硬件是否存在、任务是否创建、某类代码是否参与编译，适合用宏；遥控器模式、视觉是否在线、机械臂当前动作，适合运行时状态机。

## 板卡形态选择

位置：`application/robot_def.h`

```c
#define ONE_BOARD
// #define CHASSIS_BOARD
// #define GIMBAL_BOARD
```

三者只能开一个。文件中有冲突检测：

```c
#error Conflict board definition! You can only define one board type.
```

当前使用 `ONE_BOARD`，表示单板控制整车。底盘、CMD、Delta 等应用会在同一块板上初始化和运行。

使用建议：

| 宏 | 用途 |
|---|---|
| `ONE_BOARD` | 当前 R2 整车调试主路径 |
| `CHASSIS_BOARD` | 双板架构下只跑底盘侧 |
| `GIMBAL_BOARD` | 双板架构下只跑云台/上层控制侧 |

调试时不要同时打开多个板卡宏，否则编译会直接报错，这是刻意设计的防误烧保护。

## 机器人型号选择

位置：`application/robot_def.h`

```c
// #define ROBOT_R1
#define ROBOT_R2
```

R1 和 R2 只能开一个。当前工程编译的是 `ROBOT_R2`。

这个宏影响非常大，主要包括：

| 差异 | R1 | R2 |
|---|---|---|
| 击球电机数量 | 3 个 Delta 电机 | 2 个 Delta 电机 |
| 发球拨杆 | 有 `Serve` | 无 `Serve` |
| 底盘参数 | R1 独立参数区 | R2 独立参数区 |
| 光流方向/比例 | R1 独立参数区 | R2 独立参数区 |
| 视觉 PID | R1 独立参数区 | R2 独立参数区 |
| pitch 遥控映射 | R1 独立参数区 | R2 独立参数区 |

代码中通过：

```c
#if defined(ROBOT_R1)
...
#elif defined(ROBOT_R2)
...
#else
#error Robot type undefined!
#endif
```

把不同车的参数映射到统一业务宏。业务代码通常不直接使用 `CHASSIS_R2_SPEED_PID_KP`，而是使用：

```c
CHASSIS_SPEED_PID_KP
VISION_PID_X_KP
PITCH_MOTOR_ID
DELTA_MOTOR_NUM
```

这样 `chassis.c`、`robot_cmd.c`、`delta.c` 不需要到处写 R1/R2 分支。

## 模块任务开关

这些宏用于控制任务是否创建、应用是否初始化，主要服务于硬件调试阶段。

```c
#define R2_DEBUG_ENABLE_CMD_APP 1
#define R2_DEBUG_ENABLE_INS_TASK 1
#define R2_DEBUG_ENABLE_MOTOR_TASK 1
#define R2_DEBUG_ENABLE_DAEMON_TASK 1
#define R2_DEBUG_ENABLE_CHASSIS_APP 1
#define R2_DEBUG_ENABLE_DELTA_APP 1
```

典型使用位置：

```text
application/robot.c
application/robot_task.h
```

含义如下：

| 宏 | 作用 |
|---|---|
| `R2_DEBUG_ENABLE_CMD_APP` | 是否初始化并运行 CMD，CMD 负责遥控器、视觉、底盘和机械臂的统一指令下发 |
| `R2_DEBUG_ENABLE_INS_TASK` | 是否创建 INS 姿态解算任务 |
| `R2_DEBUG_ENABLE_MOTOR_TASK` | 是否创建全局电机控制任务 |
| `R2_DEBUG_ENABLE_DAEMON_TASK` | 是否创建守护任务和蜂鸣器任务 |
| `R2_DEBUG_ENABLE_CHASSIS_APP` | 是否初始化并周期运行底盘应用 |
| `R2_DEBUG_ENABLE_DELTA_APP` | 是否初始化并周期运行 Delta/机械臂应用 |

这组宏适合用来排查“某个模块影响另一个模块”的问题。例如机械臂单独正常、整车运行异常时，可以先保留基础任务，逐步打开 CMD、底盘、Delta、视觉等链路，定位是谁引入了阻塞、栈溢出、CAN 无输出或外设冲突。

注意：这类宏改完必须重新编译烧录；它们不是运行时开关。

## 外设和通信开关

### 视觉通信通道

```c
#define VISION_USE_VCP
// #define VISION_USE_UART
```

当前使用 USB 虚拟串口。`VISION_USE_UART` 是硬件串口路径。两个不要同时开。

### 视觉功能启停

```c
#define ROBOT_ENABLE_VISION 1
```

置 `1` 时 CMD 会初始化视觉通信，并周期发送/接收视觉协议。置 `0` 时会跳过视觉链路，适合相机未接入或排查 USB/协议导致卡死时使用。

### 光流功能启停

```c
#define ROBOT_ENABLE_OPTICAL_FLOW 0
```

置 `1` 时初始化光流并读取位移。置 `0` 时跳过光流初始化和周期读取。当前策略已经取消使用光流计，视觉 `cmd=1` 误差模式不依赖光流。

### VOFA 底盘调试输出

```c
#define ROBOT_ENABLE_VOFA_CHASSIS_DEBUG 1
#define ROBOT_VOFA_CHASSIS_DEBUG_DIVIDER 20u
```

用于把底盘命令、电机目标速度、反馈速度、电流、视觉目标等数据转发到 VOFA 看曲线。

`ROBOT_VOFA_CHASSIS_DEBUG_DIVIDER` 是分频值。当前 RobotTask/ChassisTask 大约 2ms 调一次，分频 20 后大约 40ms 发一次，即约 25Hz。

### 屏幕任务

```c
#define ROBOT_ENABLE_SCREEN_TASK 0
```

置 `1` 创建 LCD 屏幕任务；置 `0` 只保留代码但不运行。机械臂、底盘、视觉联调时，如果怀疑屏幕/SPI/ADC 干扰，可以先关掉。

## 临时调试宏

这类宏是实车联调期间非常有用的“保险丝”。

```c
#define R2_AUTO_MODE_CHASSIS_LOCK_TEST 0u
#define R2_AUTO_MODE_ARM_DISABLE_TEST 1u
```

通过 R2 映射后，业务代码使用：

```c
AUTO_MODE_CHASSIS_LOCK_TEST
AUTO_MODE_ARM_DISABLE_TEST
```

含义：

| 宏 | 作用 |
|---|---|
| `R2_AUTO_MODE_CHASSIS_LOCK_TEST` | 自动模式下临时锁定底盘，用于原地测试视觉/机械臂 |
| `R2_AUTO_MODE_ARM_DISABLE_TEST` | 自动模式下禁用视觉触发机械臂，用于先联调视觉和底盘 |

这些宏非常适合分阶段联调：

1. 先锁底盘，只看视觉通信和上位机数据是否正常。
2. 再放开底盘，测试自动/遥控混控。
3. 最后放开机械臂，让 `flag` 触发接球动作。

## 参数宏与业务宏映射

本工程一个重要设计是：R1/R2 各自维护完整参数区，然后在文件后半段映射成统一业务宏。

例如 R2 底盘参数：

```c
#define CHASSIS_R2_SPEED_PID_KP ...
#define CHASSIS_R2_SPEED_PID_KI ...
#define CHASSIS_R2_SPEED_PID_MAX_OUT ...
```

在 `ROBOT_R2` 分支中映射为：

```c
#define CHASSIS_SPEED_PID_KP CHASSIS_R2_SPEED_PID_KP
#define CHASSIS_SPEED_PID_KI CHASSIS_R2_SPEED_PID_KI
#define CHASSIS_SPEED_PID_MAX_OUT CHASSIS_R2_SPEED_PID_MAX_OUT
```

业务代码只看 `CHASSIS_SPEED_PID_KP`。这样换车时，不需要改 `chassis.c`，只需要切换 `ROBOT_R1/ROBOT_R2`。

同样的映射也用于：

| 参数族 | 业务宏示例 |
|---|---|
| 底盘几何和电机 ID | `WHEEL_BASE`, `CHASSIS_MOTOR_LF_ID` |
| 底盘 PID 和前馈 | `CHASSIS_SPEED_PID_KP`, `CHASSIS_SPEED_FEEDFORWARD_KV` |
| 车头锁定 | `CHASSIS_HEADING_PID_KP`, `CHASSIS_HEADING_PID_MAX_OUT` |
| 遥控器比例和死区 | `CMD_REMOTE_MOVE_SCALE`, `CMD_REMOTE_DEADBAND` |
| 视觉 PID | `VISION_PID_X_KP`, `VISION_PID_Y_MAXOUT` |
| 光流方向和比例 | `OPTICAL_FLOW_SWAP_XY`, `OPTICAL_FLOW_X_DIRECTION` |
| Delta/机械臂 | `DELTA_MOTOR_NUM`, `PITCH_MOTOR_ID`, `PITCH_REMOTE_BACK_POS` |

## 什么时候该用宏，什么时候不该用宏

适合用宏的场景：

- 某个硬件是否存在。
- 某个 FreeRTOS 任务是否创建。
- 当前编译 R1 还是 R2。
- 当前使用 USB 视觉还是 UART 视觉。
- 调试阶段临时屏蔽某个模块。
- 不同车种的固定参数差异。

不适合用宏的场景：

- 遥控器实时模式切换。
- 视觉是否在线。
- 机械臂当前动作状态。
- 底盘当前是否正在运动。
- 比赛过程中需要动态变化的策略。

简单判断：如果改完需要重新烧录才能生效，且这个选择本来就不该在比赛中变化，那适合用宏。否则更适合用状态变量、消息或状态机。

## 推荐调试流程

整车联调时，建议按下面顺序逐步打开：

1. 保留基础任务：`INS`、`MOTOR`、`DAEMON`。
2. 单独打开 `DELTA_APP`，确认机械臂 CAN 和电机反馈正常。
3. 打开 `CHASSIS_APP`，确认底盘四轮反馈和混控方向正常。
4. 打开 `CMD_APP`，确认遥控器输入、底盘命令、机械臂命令正常。
5. 打开 `ROBOT_ENABLE_VISION`，确认上位机通信和协议解析。
6. 如需恢复 `cmd=0` 坐标模式或右二固定距离测试，再打开 `ROBOT_ENABLE_OPTICAL_FLOW` 并确认位移方向、比例和在线状态。
7. 必要时打开 `ROBOT_ENABLE_VOFA_CHASSIS_DEBUG`，用曲线观察命令、反馈、电流和视觉误差。
8. 最后再关闭临时保险宏，例如放开自动模式底盘锁定和机械臂触发。

如果出现“一开某个模块 CAN 就没输出/系统卡死”的问题，优先回到最近一个正常组合，然后只改变一个宏重新编译验证。不要一次打开多个变量，否则很难定位。

## 当前 R2 联调状态速查

以当前 `application/robot_def.h` 为准，最近读取到的主要开关为：

| 宏 | 当前值 | 含义 |
|---|---|---|
| `ONE_BOARD` | 开启 | 单板整车 |
| `ROBOT_R2` | 开启 | 编译 R2 |
| `VISION_USE_VCP` | 开启 | 视觉走 USB 虚拟串口 |
| `ROBOT_ENABLE_VISION` | `1` | 视觉链路启用 |
| `ROBOT_ENABLE_OPTICAL_FLOW` | `0` | 光流链路关闭 |
| `ROBOT_ENABLE_VOFA_CHASSIS_DEBUG` | `1` | VOFA 底盘曲线启用 |
| `ROBOT_ENABLE_SCREEN_TASK` | `0` | 屏幕任务关闭 |
| `R2_DEBUG_ENABLE_CMD_APP` | `1` | CMD 整车控制启用 |
| `R2_DEBUG_ENABLE_CHASSIS_APP` | `1` | 底盘应用启用 |
| `R2_DEBUG_ENABLE_DELTA_APP` | `1` | 机械臂应用启用 |
| `R2_AUTO_MODE_CHASSIS_LOCK_TEST` | `0u` | 自动模式不锁底盘 |
| `R2_AUTO_MODE_ARM_DISABLE_TEST` | `1u` | 自动模式临时禁用视觉触发机械臂 |

这张表只是记录写本文档时的状态。实车调试时请以 `application/robot_def.h` 当前内容为准。
