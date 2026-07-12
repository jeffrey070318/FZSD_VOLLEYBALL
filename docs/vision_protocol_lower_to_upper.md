# 下位机 -> 上位机 视觉协议

本文件记录当前下位机主动发送给视觉上位机的数据帧。注意：这套帧和 `docs/vision_protocol.md` 中的“上位机 -> 下位机”帧不是同一个方向，帧长和字段含义不同。

当前代码位置：

- `modules/master_machine/master_process.h`
- `modules/master_machine/master_process.c`
- `application/cmd/robot_cmd.c`

## 基本格式

帧长：17 字节

帧头：`0xAA`

帧尾：`0x55`

校验：XOR，计算范围为数组索引 `1..14`

发送方式：当前 R2 工程使用 USB 虚拟串口，即 `VISION_USE_VCP`

## 字节布局

这里的索引是 C 代码数组下标，从 0 开始。

| 索引 | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0 | header | `uint8_t` | 固定 `0xAA` |
| 1 | mode | `uint8_t` | 机器人当前控制模式 |
| 2 | state | `uint8_t` | 机器人当前业务状态 |
| 3-6 | robot_x | `float32` | 下位机上报的车体 X 位置 |
| 7-10 | robot_y | `float32` | 下位机上报的车体 Y 位置 |
| 11-14 | robot_yaw | `float32` | 下位机上报的车体/IMU yaw |
| 15 | xor_val | `uint8_t` | 索引 `1..14` 逐字节异或 |
| 16 | tail | `uint8_t` | 固定 `0x55` |

## 字段说明

### mode

定义位置：`modules/master_machine/master_process.h`

| 值 | 含义 |
|---|---|
| 0 | `MODE_IDLE`，遥控器不在线/空闲 |
| 1 | `MODE_REMOTE`，手动遥控模式 |
| 2 | `MODE_SELF`，自动模式 |

### state

定义位置：`modules/master_machine/master_process.h`

| 值 | 含义 |
|---|---|
| 0 | `STATE_WAITING_PLAN` |
| 1 | `STATE_RECEIVED_PLAN` |
| 2 | `STATE_CATCHING` |
| 3 | `STATE_OVER` |

### robot_x / robot_y

当前由光流计全局位置填充：

```c
vision_send_data.robot_x = flow_data->position_x_global;
vision_send_data.robot_y = flow_data->position_y_global;
```

如果光流计未接入或数据指针为空，则上报 `0.0f`。

### robot_yaw

当前最后一个 `float32` 字段为 `robot_yaw`，与旧版上位机协议保持一致。

数据来源链路：

```text
DM_IMU_GetData()->yaw 或 ins_imu_data->Yaw
    -> vision_send_data.robot_yaw
    -> send_buff[11..14]
```

也就是说，上位机解析下位机回传帧时，索引 `11..14` 应按 `float32 robot_yaw` 解析。

## 校验计算

```python
def calc_lower_to_upper_xor(frame: bytes) -> int:
    result = 0
    for b in frame[1:15]:  # 索引 1..14
        result ^= b
    return result
```

## Python 解析示例

```python
import struct

def parse_lower_to_upper(frame: bytes):
    if len(frame) != 17:
        raise ValueError("invalid frame length")
    if frame[0] != 0xAA or frame[16] != 0x55:
        raise ValueError("invalid frame head/tail")

    xor_val = 0
    for b in frame[1:15]:
        xor_val ^= b
    if xor_val != frame[15]:
        raise ValueError("invalid xor")

    mode = frame[1]
    state = frame[2]
    robot_x = struct.unpack("<f", frame[3:7])[0]
    robot_y = struct.unpack("<f", frame[7:11])[0]
    robot_yaw = struct.unpack("<f", frame[11:15])[0]

    return {
        "mode": mode,
        "state": state,
        "robot_x": robot_x,
        "robot_y": robot_y,
        "robot_yaw": robot_yaw,
    }
```

## 和上位机下发帧的区别

上位机下发帧是 19 字节，字段中有 `cmd / target_x / target_y / flag / target_time`。

下位机上发帧是 17 字节，字段中有 `mode / state / robot_x / robot_y / robot_yaw`。

两者不要混用。尤其注意：上位机下发帧中的索引 `13..16` 是 `target_time`；下位机上发帧中的 yaw 在索引 `11..14`。
